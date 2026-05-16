# Week-3 Gimbal & Camera Debug Log

## Background

目标是在 Raspberry Pi 上运行主程序，使用 `rpicam` 摄像头输入、MediaPipe 姿态检测、GPIO PWM 直控云台，并通过 UART 控制 Arduino 小车底盘。

运行命令：

```bash
python3 -m raspberry_pi.main \
  --port /dev/serial0 \
  --camera-backend rpicam \
  --camera-id 0 \
  --debug-vision \
  --debug-control \
  --debug-gimbal
```

## Problem 1: GPIO Module Missing

### Symptom

程序启动后出现：

```text
[MAIN] gimbal write failed: No module named 'RPi'
[MAIN] vision runtime failed: control loop stopped
```

### Diagnosis

`raspberry_pi/hardware/gimbal.py` 使用：

```python
import RPi.GPIO as GPIO
```

但当前 Python 虚拟环境中没有 `RPi.GPIO`。

### Resolution

在树莓派对应 Python 环境中安装 GPIO 依赖。安装后该错误消失。

### Conclusion

该问题是 Python 环境依赖缺失，不是摄像头或舵机硬件问题。

## Problem 2: MediaPipe / TensorFlow Lite Warnings

### Symptom

运行时出现：

```text
Error in cpuinfo: prctl(PR_SVE_GET_VL) failed
INFO: Created TensorFlow Lite XNNPACK delegate for CPU.
WARNING: All log messages before absl::InitializeLog() is called are written to STDERR
W... inference_feedback_manager.cc:114] Feedback manager requires a model with a single signature inference.
```

### Diagnosis

这些是 TensorFlow Lite / MediaPipe 在树莓派上的常见 warning：

- `SVE_GET_VL failed`：CPU 指令集探测失败，不影响普通推理。
- `XNNPACK delegate`：TFLite 启用 CPU 加速。
- `feedback tensors`：当前模型不支持反馈张量，MediaPipe 会自动禁用。

### Conclusion

这些日志不是程序失败原因，可以暂时忽略。

## Problem 3: Gimbal Does Not Move Because Control Output Is Stop

### Symptom

调试日志显示：

```text
[GIMBAL] pan_delta=0.000 speed=0.000 servo_pan=90.0 tilt=90.0
[CONTROL] target=no move(v=0.000,w=0.000) gimbal(pan_delta=0.000,tilt_abs=90.000,pan_abs=0.000)
```

### Diagnosis

当前控制循环没有拿到视觉目标：

```text
target=no
```

所以云台输出：

```text
pan_delta=0
servo_pan=90.0
```

对于水平连续旋转舵机，`servo_pan=90.0` 是停止信号。

### Conclusion

当时舵机不转是符合程序输出的，不是 GPIO 写入失败。

## Problem 4: Camera Hardware Suspected Broken

### Verification Command

使用系统命令绕过 Python 程序，直接测试 `rpicam`：

```bash
timeout 5 rpicam-vid --camera 0 --timeout 0 --nopreview --codec mjpeg --width 640 --height 480 --framerate 15 --verbose 0 -o - > test.mjpeg
ls -lh test.mjpeg
```

### Result

```text
-rw-rw-r-- 1 xzmeng xzmeng 732K May 14 13:17 test.mjpeg
```

### Diagnosis

`test.mjpeg` 有 732K，说明：

- 摄像头不是完全坏的。
- `rpicam-vid` 能从摄像头输出 MJPEG 数据。
- 问题更可能在 Python 的 `rpicam` 读取/解码链路，或 MediaPipe 检测链路。

## Problem 5: Python rpicam Backend Initially Had No Vision Stats

### Symptom

运行 10 秒后只看到：

```text
[VISION] camera opening backend=rpicam id=0 width=640 height=480 fps=15
[CONTROL] target=no ...
[GIMBAL] pan_delta=0.000 ...
```

没有看到：

```text
[VISION] stats ...
```

### Static Diagnosis

原始代码使用：

```python
proc.stdout.read(4096)
```

即使 `select()` 判断 stdout 可读，buffered `read(4096)` 仍可能阻塞等待更多字节，导致视觉循环卡住。

### Code Change

将读取方式改为非缓冲：

```python
os.read(proc.stdout.fileno(), 4096)
```

并在 `Popen` 中设置：

```python
bufsize=0
```

同时加入底层 rpicam stream debug：

```text
[VISION] rpicam stream bytes=... chunks=... buffer=... soi=... eoi=... decoded=... decode_failed=...
[VISION] rpicam first frame decoded width=640 height=480
```

## Problem 6: rpicam Decode Pipeline Confirmed Working

### Symptom

修改后运行日志出现：

```text
[VISION] rpicam first frame decoded width=640 height=480
[VISION] rpicam stream bytes=36864 chunks=9 buffer=3893 soi=4 eoi=3 decoded=3 decode_failed=0
[VISION] stats frames=1 fps=0.5 detections=0
```

### Diagnosis

这说明：

- `rpicam-vid` stdout 有数据。
- MJPEG 边界能被识别。
- `cv2.imdecode()` 能成功解码。
- 摄像头链路已经打通。

但是：

```text
detections=0
```

说明 MediaPipe Pose 当时没有检测到人体目标。

## Problem 7: Servo Power Caused System Performance Issues

### Symptom

舵机 VCC 接在树莓派上时，视觉 FPS 很低：

```text
[VISION] stats frames=1 fps=0.5 detections=0
```

断开舵机供电后，FPS 提升到：

```text
fps=10.5
```

### Diagnosis

舵机启动/堵转瞬间电流较大，直接从树莓派 5V 取电会导致供电不稳，影响摄像头和 CPU 推理性能。

### Conclusion

该问题不是摄像头坏，也不是 MediaPipe 本身异常，而是舵机供电拖垮树莓派电源。

## Correct Servo Wiring

舵机必须独立供电：

```text
外部 5V/6V 舵机电源 -> 舵机 VCC
外部电源 GND        -> 舵机 GND
树莓派 GND          -> 外部电源 GND
GPIO17              -> 水平舵机信号线
GPIO27              -> 垂直舵机信号线
```

不要这样接：

```text
树莓派 5V -> 舵机 VCC
```

必须共地：

```text
树莓派 GND <-> 外部舵机电源 GND
```

## Current Conclusions

1. `RPi.GPIO` 依赖问题已解决。
2. `rpicam` 摄像头不是坏的，能输出 MJPEG。
3. Python rpicam backend 已能解码帧。
4. 舵机不转时，程序输出是 `pan_delta=0`，原因是 `target=no`。
5. 早期 FPS 极低的主要原因是舵机从树莓派取电导致供电不稳。
6. 断开舵机供电后 FPS 提升到约 `10.5`，说明视觉链路本身可运行。
7. 下一步应使用独立舵机电源，并继续观察 `detections` 是否大于 0。

## Follow-up Checklist

检查树莓派是否发生过欠压：

```bash
vcgencmd get_throttled
```

理想输出：

```text
throttled=0x0
```

重新接入独立舵机电源后运行：

```bash
python3 -m raspberry_pi.main \
  --port /dev/serial0 \
  --camera-backend rpicam \
  --camera-id 0 \
  --debug-vision \
  --debug-control \
  --debug-gimbal
```

重点观察：

```text
[VISION] stats fps=...
[VISION] stats detections=...
[CONTROL] target=yes/no
[GIMBAL] pan_delta=... servo_pan=...
```

如果：

```text
detections>0
pan_delta!=0
servo_pan!=90
```

但舵机仍不转，则继续检查：

- 外部舵机电源电压。
- 舵机 GND 是否与树莓派 GND 共地。
- GPIO17 / GPIO27 是否接到信号线。
- 水平连续旋转舵机停止点是否真为 90。
- 舵机信号方向是否需要反转。
