# Week-2 Hardware & Main

## 1. 整体架构

```
┌────────────────────────────────────────────────────────────────┐
│                           main.py                              │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│   ┌──────────────────┐          ┌──────────────────┐           │
│   │   Vision Thread  │          │  Control Thread  │           │
│   │   (Main Thread)  │          │  (Background)    │           │
│   │                  │          │                  │           │
│   │ run_pose_land-   │          │ _run_control_    │           │
│   │ marker_on_camera │          │     _loop()      │           │
│   │         │        │          │                  │           │
│   │         ▼        │          │ SharedVisionState│           │
│   │ on_detected()    │─────▶    │        │         │           │
│   │         │        │  target  │        ▼         │           │
│   │         ▼        │          │ Planner.update() │           │
│   │ SharedVisionState│◄─────────│        │         │           │
│   │                  │          │ SerialComm       │           │
│   └──────────────────┘          │   .send()        │           │
│                                 └────────┬─────────┘           │
│                                          │                     │
│                                          ▼                     │
│                                  ┌──────────────┐              │
│                                  │   Arduino    │              │
│                                  │  (UART/JSON) │              │
│                                  └──────┬───────┘              │
│                                         │                      │
│                              ┌──────────┴──────────┐           │
│                              ▼                     ▼           │
│                        ┌──────────┐        ┌──────────┐        │
│                        │ Chassis  │        │  Gimbal  │        │
│                        │  Motors  │        │ Servos   │        │
│                        └──────────┘        └──────────┘        │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

## 2. 调用流程

### 2.1 启动流程

```
1. _parse_args()          → 解析命令行参数，创建 RuntimeConfig
2. run()                  → 主入口函数
   ├── Planner()          → 创建决策规划器（使用 PlanningConfig）
   ├── SharedVisionState() → 创建线程安全共享状态
   ├── SerialComm()       → 创建串口通信对象
   ├── comm.open()        → 打开串口连接
   ├── time.sleep(1.5)    → 等待 Arduino 初始化
   ├── comm.send_stop()   → 发送初始停止指令
   ├── control_thread.start() → 启动控制线程
   └── 根据 camera_backend 选择视觉函数（阻塞）
```

### 2.2 运行时数据流

**视觉检测循环**（主线程）：
1. `run_pose_landmarker_on_camera()` 每帧调用 `on_detected(pose_info)`
2. `on_detected()` 提取 `x_error_norm`, `y_error_norm`, `height_norm`, `width_norm`
3. `_to_vision_target()` 验证并创建 `VisionTarget` 对象
4. `shared.update()` 写入共享状态（带锁保护）

**控制循环**（后台线程）：
1. 以 `control_hz` 频率循环（默认 10Hz）
2. `shared.get()` 读取最新目标（检查是否过期）
3. `planner.update(target, dt_s)` 计算控制指令
4. `comm.send_message()` 发送给 Arduino
5. 可选：`comm.request_status()` 查询 Arduino 状态

这里把视觉检测作为主线程，因为在 mac 上 OpenCV 的 `cv2.imshow()` 需要在主线程而不能在子线程执行

### 2.3 关闭流程

```
1. 用户按 'q' 或异常发生
2. finally 块执行：
   ├── stop_event.set()          → 通知控制线程停止
   ├── control_thread.join(1.0)  → 等待控制线程结束
   ├── comm.send_stop()          → 发送停止指令
   └── comm.close()              → 关闭串口
```

## 3 RuntimeConfig

**来源**: 命令行参数  
**用途**: 控制程序整体行为

| 参数                  | 类型      | 默认值    | 说明                                                               |
| ------------------- | ------- | ------ | ---------------------------------------------------------------- |
| `port`              | `str`   | **必需** | 串口设备路径，如 `/dev/cu.usbmodem1101` (macOS) 或 `/dev/ttyACM0` (Linux) |
| `baudrate`          | `int`   | `9600` | UART 波特率，必须与 Arduino 代码匹配                                        |
| `camera_backend`    | `str`   | `"opencv"` | 摄像头后端：`"opencv"` (USB/webcam) 或 `"rpicam"` (CSI via rpicam-vid) |
| `camera_id`         | `int`   | `0`    | 摄像头设备 ID                                                     |
| `show_window`       | `bool`  | `False` | 是否显示 OpenCV 窗口                                               |
| `frame_width`       | `int`   | `640`  | 摄像头画面宽度                                                     |
| `frame_height`      | `int`   | `480`  | 摄像头画面高度                                                     |
| `camera_fps`        | `int`   | `15`   | 摄像头帧率                                                        |
| `num_poses`         | `int`   | `1`    | MediaPipe 同时检测人数                                             |
| `control_hz`        | `float` | `10.0` | 控制循环频率（Hz），即每秒发送指令次数                                             |
| `detection_stale_s` | `float` | `0.2`  | 目标"过期"时间（秒）。超过此时间未检测到人体，视为目标丢失                                   |
| `status_interval_s` | `float` | `0.0`  | 状态查询间隔（秒）。`<=0` 表示不查询 Arduino 状态，>0 则定期查询                        |

## 4. 命令行用法

### 完整参数示例

USB 摄像头 (opencv)：
```bash
python -m raspberry_pi.main \
    --port /dev/ttyACM0 \
    --baudrate 115200 \
    --camera-backend opencv \
    --camera-id 0 \
    --control-hz 20.0 \
    --detection-stale-s 0.3 \
    --status-interval-s 5.0
```

树莓派 CSI 摄像头 (rpicam)：
```bash
python -m raspberry_pi.main \
    --port /dev/serial0 \
    --baudrate 115200 \
    --camera-backend rpicam \
    --frame-width 640 \
    --frame-height 480 \
    --camera-fps 15 \
    --num-poses 1
```

### 参数说明

```
usage: main.py [-h] --port PORT [--baudrate BAUDRATE] [--camera-backend {opencv,rpicam}]
               [--camera-id CAMERA_ID] [--show_window] [--frame-width FRAME_WIDTH]
               [--frame-height FRAME_HEIGHT] [--camera-fps CAMERA_FPS] [--num-poses NUM_POSES]
               [--control-hz CONTROL_HZ] [--detection-stale-s DETECTION_STALE_S]
               [--status-interval-s STATUS_INTERVAL_S]

Pose follow main pipeline

options:
  -h, --help            show this help message and exit
  --port PORT           Serial port (required)
  --baudrate BAUDRATE   UART baudrate
  --camera-backend {opencv,rpicam}
                        Camera capture backend
  --camera-id CAMERA_ID Camera device ID
  --show_window         Show OpenCV visualization window
  --frame-width FRAME_WIDTH
                        Camera frame width
  --frame-height FRAME_HEIGHT
                        Camera frame height
  --camera-fps CAMERA_FPS
                        Camera frame rate
  --num-poses NUM_POSES
                        Number of poses for MediaPipe
  --control-hz CONTROL_HZ
                        Control loop frequency
  --detection-stale-s DETECTION_STALE_S
                        Treat vision target as lost after this age
  --status-interval-s STATUS_INTERVAL_S
                        Optional status polling period; <=0 disables
```

