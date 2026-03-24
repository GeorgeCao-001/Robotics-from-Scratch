# 机器人小车仿真与底层控制 (Robot Car Sim & Control)

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange)](https://platformio.org)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-green)](https://gazebosim.org/)

本项目是一个综合性的机器人开发工程，实现了从 **Gazebo 物理仿真环境** 到 **PlatformIO 嵌入式底层逻辑** 的全链路控制。主要用于研究机器人小车的运动学控制、传感器数据融合以及 ROS2 环境下的仿真调试。

---

## 🛠️ 开发环境 (Environment)

为了确保项目正常运行，请确认你的开发环境满足以下要求：

* **操作系统**: Ubuntu 22.04 LTS (推荐) / Windows 10+
* **代码编辑器**: Visual Studio Code
* **嵌入式工具**: PlatformIO IDE 扩展
* **仿真框架**: ROS2 (Humble) & Gazebo
* **版本管理**: Git

---

## 📂 项目结构 (Folder Structure)

```text
.arduino_uno_car
├── src/ 
    └── main.cpp              # 嵌入式 C++ 源代码 (电机/舵机控制逻辑)
├── include/            # 项目头文件 (.h)   
├── platformio.ini      # PlatformIO 配置文件
└── README.md           # 本项目说明书
```
```text
my_robot_car/
├── launch/      # 启动文件：一键打开 Gazebo、加载 URDF 和控制器 
├── urdf/                 # 机器人描述文件：定义底盘、轮子、云台及插件 [cite: 1, 3]
├── scripts/           # 键盘控制脚本：监听按键并发布 ROS 2 话题 
├── meshes/                        # (可选) 如果你以后使用复杂的 3D 模型 (.stl/.dae)
├── CMakeLists.txt                 # 编译配置文件
└── package.xml                    # 项目依赖定义 (如 rclpy, geometry_msgs)
```
## 🚀 快速上手 (Quick Start)

### 1. 编译嵌入式端 (PlatformIO)
硬件接线预设
```text
常用的 L298N 驱动模块
Arduino UNO 的引脚带不动电机，必须经过驱动板：

左轮 (电机 A): ENA 接 5，IN1 接 2，IN2 接 3

右轮 (电机 B): ENB 接 6，IN3 接 4，IN4 接 7

云台 SG90: Pan (左右偏航) 接 9，Tilt (上下俯仰) 接 10
```
先确认自己有没有安装platformIO插件，如果已安装，在platformIO上打开一个新的项目，开发板选用Arduino UNO，编译架构选择Arduino，将scr/main.cpp中代码全部改为所给文件中的代码，点击左下角的√，如果已经连接了开发板，可以直接点右箭头进行烧录。然后打开串口监视器，可以通过按键来检查是否正常运行。
### ❗注意事项:

#### 1.外部供电（核心痛点）：
千万不要只靠 Arduino 的 5V 引脚带动两个舵机和四个电机。
必须使用外部电池组（如 7.4V 锂电池）给驱动板供电，并确保电池负极与 Arduino 的 GND 连在一起（共地）。
#### 2.端口冲突：
检查你的 Servo 端口（9, 10）是否和电机驱动端口（通常也是 9, 10）重叠了 。如果重叠，请改用 11, 12 。
#### 3.库安装：
在 PlatformIO 的 platformio.ini 文件里，确保有一行：lib_deps = Servo 。

### 2.Gazebo 物理仿真环境
请确保你已有Ubuntu 22.04系统且安装了ROS2 Humble以及关于Gazebo的相关插件
打开vs code，在终端运行
```bash
mkdir -p ~/my_robot_car/{launch,urdf,scripts,meshes} && touch ~/my_robot_car/{CMakeLists.txt,package.xml}
```
此时已经自动生成了一个项目包，根据给出的代码将所有文件修改好。
之后打开一个终端，输入
```bash
 ros2 launch ~/my_robot_car/launch/display.launch.py
```
此时会自动打开Gazebo，里面理应有一个小车
然后另开一个终端窗口，输入
```bash
python3 ~/my_robot_car/scripts/sim_control.py
```
此时应该可以通过按键使小车运动（请确保在终端内键入）
#### ❗仿真文件主文件夹只能在Ubuntu主目录里，不然需要手动改改路径，非常麻烦！！！