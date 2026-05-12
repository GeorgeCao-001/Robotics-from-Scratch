# Week-2 CSI Camera rpicam Backend

## 1. 问题背景

树莓派 CSI 摄像头（Camera Module v1/v2/v3）在新版 Raspberry Pi OS / Debian Trixie 上使用 libcamera 驱动栈，不能通过 `cv2.VideoCapture(0)` 直接采集图像。尝试后 `cap.read()` 返回空帧，打印 `Failed to grab frame`。

### 为什么不用 Picamera2

Raspberry Pi 官方提供 `python3-picamera2`（`apt` 包），但项目当前 Python 环境是 `pyenv` 安装的 Python 3.11.9，`apt` 的系统包路径不会被 `pyenv` 虚拟环境加载：

```text
apt: /usr/lib/python3/dist-packages/picamera2
pip: ~/my_robot_env/lib/python3.11/site-packages/  (mediapipe 在这里)

两个路径隔离，无法共存。
```

`mediapipe` 对 Python 3.13 不兼容（树莓派系统 Python），安装到系统 Python 也不可行。因此放弃 Python API `Picamera2`。

### 解决方案

通过系统工具 `rpicam-vid`（命令行 libcamera 前端）采集 CSI 摄像头，Python 从子进程 stdout 读取 MJPEG 帧，解码后交给 OpenCV/MediaPipe。

```
┌────────────────────────────────────────────────────────────┐
│                        Python 进程                         │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────────┐ │
│  │ rpicam-  │───▶│ MJPEG    │───▶│  OpenCV / MediaPipe  │ │
│  │ vid 子进程│    │ 帧解析   │    │  pose landmarker    │ │
│  │ (libcam) │    │ JPEG→BGR │    │        │             │ │
│  └──────────┘    └──────────┘    │        ▼             │ │
│                      ▲           │  planning + serial   │ │
│                      │           └──────────────────────┘ │
│               stdout (MJEPG)                              │
│               stderr (日志采集)                             │
└────────────────────────────────────────────────────────────┘
         ▲
         │  CSI
    ┌────┴────┐
    │ OV5647  │
    │ (Rev1.3)│
    └─────────┘
```

---

## 2. 依赖

| 组件 | 安装方式 | 说明 |
|---|---|---|
| `rpicam-vid` | `sudo apt install rpicam-apps` | 命令行 libcamera 前端 |
| `mediapipe` | `pip install mediapipe` | pyenv venv 内 |
| `opencv-contrib-python` | `pip install opencv-contrib-python` | pyenv venv 内，用于 JPEG 解码和处理 |
| `pyserial` | `pip install pyserial` | pyenv venv 内 |

**不需要安装**：`python3-picamera2`、`libcamera` Python 绑定。

---

## 3. MJPEG 帧解析

`rpicam-vid` 配置：

```bash
rpicam-vid \
  --camera 0 \
  --timeout 0 \
  --nopreview \
  --codec mjpeg \
  --width 640 --height 480 \
  --framerate 15 \
  --verbose 0 \
  -o -
```

- `--codec mjpeg`：每个视频帧编码为独立 JPEG
- `-o -`：输出到 stdout
- `--timeout 0`：永不自动退出

Python 端解析逻辑：

1. 从 stdout 读取字节块追加到缓冲区 `buf`
2. 在 `buf` 中查找 JPEG 边界标记 `FF D8`（SOI）和 `FF D9`（EOI）
3. 取出完整 JPEG 字节序列
4. `cv2.imdecode()` 解码为 BGR numpy 数组
5. 交给 `_process_pose_frame()` 做 MediaPipe 推理

---

## 4. 超时与安全机制

### 帧超时

如果摄像头卡住、排线松动、libcamera 内部重试，stdout 可能长时间无 MJPEG 数据。通过 `select.select()` 每 0.2 秒检查可读性。超过 `frame_timeout_s`（默认 5 秒，或 `3/fps`）无完整帧时抛异常退出。

### 缓冲区上限

如果 `rpicam-vid` 输出非 MJPEG 格式数据（如日志混入 stdout、codec 不匹配），`buf` 会无限增长。设置 `max_buffer_bytes = max(w*h*4, 10MB)`，超限后抛异常。

### 子进程退出检测

每轮循环检查 `proc.poll()`。若 `rpicam-vid` 异常退出（返回码非 0），提取 stderr 尾部日志，抛异常。

### 错误信息收集

`rpicam-vid` 的 stderr 被一个 daemon 线程持续读取，保留最近 20 行。任何异常退出都会附带 stderr 日志，便于定位：

- 摄像头超时：`Camera frontend has timed out`
- 权限问题：`Failed to open ... Permission denied`
- 编码格式：`Unsupported codec`

### 子进程清理

`finally` 块：
1. `proc.terminate()` 发 SIGTERM
2. 等 2 秒，若未退出则 `proc.kill()` 发 SIGKILL
3. 确保摄像头资源释放

---

## 5. 命令行用法

### 树莓派 CSI 摄像头

```bash
source ~/my_robot_env/bin/activate
cd ~/Robotics-from-Scratch

python -m raspberry_pi.main \
  --port /dev/serial0 \
  --baudrate 115200 \
  --camera-backend rpicam \
  --frame-width 640 \
  --frame-height 480 \
  --camera-fps 15 \
  --num-poses 1
```

### 低性能模式（帧率不足时）

```bash
python -m raspberry_pi.main \
  --port /dev/serial0 \
  --baudrate 115200 \
  --camera-backend rpicam \
  --frame-width 320 \
  --frame-height 240 \
  --camera-fps 10 \
  --num-poses 1 \
  --control-hz 5.0
```

### USB 摄像头（macOS 或 Linux UVC）

```bash
python -m raspberry_pi.main \
  --port /dev/ttyACM0 \
  --baudrate 115200 \
  --camera-backend opencv \
  --camera-id 0
```

---

## 6. 故障排查

| 症状 | 原因 | 检查 |
|---|---|---|
| `rpicam-vid ended before a complete frame was received` | rpicam-vid 启动后立即退出 | 检查摄像头连接、排线方向、`sudo apt install rpicam-apps` |
| `timed out waiting for rpicam-vid frame` | 摄像头卡住、libcamera 重试 | 先跑 `rpicam-hello --timeout 5000 --nopreview` 确认硬件正常 |
| `rpicam-vid MJPEG buffer exceeded limit` | stdout 输出不是 MJPEG | 检查 `--codec mjpeg` 是否支持、输出是否混入日志 |
| `rpicam-vid exited with code ...` | 子进程异常退出 | 看附带的 stderr 日志，常见原因：权限不足（加 `video` 组）、摄像头被占用 |
| `[MAIN] vision runtime failed: ...` | 视觉后端抛出 RuntimeError | 错误信息包含 rpicam-vid stderr 上下文 |

### 串口权限

```bash
sudo usermod -a -G dialout $USER
```

重新登录后生效。

### 摄像头权限

```bash
sudo usermod -a -G video $USER
```

---

## 7. 代码结构

```
raspberry_pi/vision/pose_landmarker.py
├── _create_pose_landmarker_options(num_poses)     ← MediaPipe 配置
├── _process_pose_frame(frame_bgr, ...)            ← 共用处理逻辑
├── run_pose_landmarker_on_camera(...)             ← OpenCV 后端
└── run_pose_landmarker_on_rpicam(...)             ← rpicam 后端
        ├── subprocess.Popen("rpicam-vid ...")
        ├── drain_stderr() daemon 线程
        ├── select.select() 超时等待
        ├── MJPEG 帧解析 (FF DE → FF D9)
        ├── cv2.imdecode() → _process_pose_frame()
        └── finally: terminate/kill + cv2.destroyAllWindows()

raspberry_pi/main.py
├── RuntimeConfig                           ← camera_backend / frame_width / ...
├── run()                                   ← 按 backend 选择视觉函数
└── _parse_args()                           ← --camera-backend rpicam|opencv
```
