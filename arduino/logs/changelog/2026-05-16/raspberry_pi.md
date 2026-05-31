================================================================================
  树莓派4B 视觉跟随系统 - 每日更新日志
================================================================================

项目名称: 树莓派4B 视觉跟随系统 (raspberry_pi)
版本号:   v1.0.0
更新日期: 2026-05-16

================================================================================
一、主要更新内容
================================================================================

【新增功能】

  1. 串口通信模块 (hardware/serial_comm.py)
     - SerialComm类: 封装pyserial，提供JSON指令的发送/接收/验证
     - SerialConfig数据类: 可配置串口路径、波特率、超时参数
     - 指令验证白名单: 仅允许move/status/mode/estop四种cmd
     - 紧凑JSON序列化: separators=(",",":")减少传输字节数
     - 发送前自动验证: _validate_message()确保指令格式正确

  2. 云台控制模块 (hardware/gimbal.py)
     - GimbalHardware类: 通过RPi.GPIO软件PWM控制双轴舵机
     - GimbalConfig数据类: 可配置引脚(BCM17/27)、PWM频率(50Hz)、角度范围
     - 角度→占空比映射: 2.5% + normalized × 10.0 (标准舵机信号)
     - 资源清理: cleanup()释放GPIO，防止程序退出后舵机锁定

  3. 规划模块 (planning/)
     - config.py: PlanningConfig数据类，包含P控制增益、速度限制、平滑系数等
     - planner.py: Planner协调器，视觉丢失分级处理(<0.2s缓存/0.2~0.8s保持/>0.8s停车)
     - follow_controller.py: FollowController，基于云台偏角计算v/w
     - gimbal_controller.py: GimbalController，增量式云台角度控制
     - types.py: VisionTarget/MoveCommand/GimbalOutput数据类型

  4. 主程序 (main.py)
     - 双线程架构: 视觉线程(主线程) + 控制线程(daemon, 10Hz)
     - SharedVisionState: 线程安全的状态共享(Lock保护)
     - 启动流程: 串口打开→停车→切换自动模式→启动控制线程
     - 退出流程(finally): 停车→切换遥控器模式→释放GPIO→关闭串口
     - 命令行参数: --port/--baudrate/--camera-backend/--control-hz/--debug-*

  5. 视觉检测模块 (vision/pose_landmarker.py)
     - MediaPipe Pose人体姿态检测
     - 支持opencv(USB摄像头)和rpicam(CSI摄像头)两种后端
     - 检测回调on_detected()更新SharedVisionState

【新增指令支持】

  serial_comm.py的_validate_message()新增mode和estop指令验证:
    - mode指令: 验证mode字段为字符串类型
    - estop指令: 无额外参数验证
    - move指令: 验证v和w为数值类型
    - status指令: 无额外参数验证

【修改内容】

  1. main.py启动流程增加模式切换指令
     - comm.send_message({"cmd":"mode","mode":"rpi_auto"})
     - 确保Arduino在控制线程启动前进入自动模式

  2. main.py退出流程增加模式恢复指令
     - comm.send_message({"cmd":"mode","mode":"remote"})
     - 确保Arduino在程序退出后回到遥控器模式
     - 所有退出操作用try/except包裹，防止串口已断开时阻塞

================================================================================
二、影响范围
================================================================================

  系统架构影响:
    - 树莓派从"可选外设"升级为"核心控制节点"
    - Arduino与树莓派通过USB串口建立主从控制关系
    - 树莓派负责视觉感知和运动规划，Arduino负责底层驱动

  通信协议影响:
    - 新增JSON over Serial通信协议
    - 波特率115200bps，匹配Arduino端配置
    - 指令以'\n'为分隔符

  安全影响:
    - 树莓派异常退出时自动交还控制权给遥控器
    - 视觉丢失0.8秒后自动停车
    - 多层急停保护确保系统安全

================================================================================
三、兼容性说明
================================================================================

  与Arduino UNO主控兼容: 需配套v2.4.0使用
    - Arduino需支持CTRL_MODE_RPI_AUTO模式和JSON指令解析
    - USB串口波特率必须一致(115200bps)

  硬件要求:
    - 树莓派4B (2GB+ RAM推荐)
    - USB摄像头或CSI摄像头
    - 双轴云台(BCM17水平舵机, BCM27垂直舵机)
    - USB A-to-B数据线连接Arduino UNO

  软件依赖:
    - Python 3.9+
    - pyserial
    - mediapipe
    - opencv-python (或picamera2 for CSI)
    - RPi.GPIO

================================================================================
四、已知问题
================================================================================

  1. RPi.GPIO软件PWM精度有限，舵机可能存在微小抖动
     建议: 对精度要求高的场景可改用PCA9685硬件PWM驱动

  2. MediaPipe Pose在树莓派4B上推理耗时约30~80ms
     实际控制频率可能低于10Hz目标值
     建议: 可降低模型复杂度或使用TFLite加速

  3. USB串口设备路径(/dev/ttyACM0)可能因插入顺序变化
     建议: 使用udev规则绑定固定设备名

  4. 控制线程为daemon线程，主线程退出时可能未完成当前控制周期
     已通过finally块确保停车指令发出

================================================================================
五、关键代码变更清单
================================================================================

  新增文件:
    raspberry_pi/main.py                    主程序入口
    raspberry_pi/hardware/serial_comm.py    串口通信模块
    raspberry_pi/hardware/gimbal.py         云台控制模块
    raspberry_pi/planning/config.py         规划参数配置
    raspberry_pi/planning/planner.py        规划协调器
    raspberry_pi/planning/follow_controller.py  跟随控制器
    raspberry_pi/planning/gimbal_controller.py  云台控制器
    raspberry_pi/planning/types.py          数据类型定义
    raspberry_pi/vision/pose_landmarker.py  视觉检测模块

  修改文件:
    raspberry_pi/hardware/serial_comm.py    _validate_message()新增mode/estop支持
    raspberry_pi/main.py                    启动/退出流程增加模式切换指令

================================================================================
