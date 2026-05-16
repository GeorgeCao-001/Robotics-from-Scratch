# AGENTS.md (Raspberry Pi)

## 角色

负责：
- 视觉处理
- 决策制定
- 指令生成

---

## 模块

### 视觉模块
- 输入：摄像头帧
- 输出：结构化数据

示例：

```json
{
  "target_x": 120,
  "target_y": 80,
  "width": 150,
  "target_x_norm": 0.50,
  "target_y_norm": 0.42,
  "x_error_norm": 0.00,
  "y_error_norm": -0.16,
  "width_norm": 0.23
}
```

- `target_x`, `target_y`：目标中心在画面中的像素坐标
- `width`：目标宽度（用于距离估计）
- `target_x_norm`, `target_y_norm`：目标中心归一化坐标（`[0,1]`）
- `x_error_norm`, `y_error_norm`：相对画面中心的归一化误差（`[-1,1]`）
- `width_norm`：目标宽度归一化结果（`[0,1]`）

---

### 云台模块
- 输入：目标位置偏移
- 输出：`GimbalOutput(pan_delta, tilt_delta, pan_abs, tilt_abs)` — 树莓派内部数据类

```python
GimbalOutput(
    pan_delta=-12.5,   # 水平本轮角度变化量，仅供调试
    tilt_delta=3.0,    # 垂直本轮角度变化量，仅供调试
    pan_abs=77.5,      # 水平绝对角度（传给 GPIO 硬件层）
    tilt_abs=30.0,     # 垂直绝对角度（传给 GPIO 硬件层）
)
```

角度定义：
- `pan_abs` 为水平绝对角，范围 `[-135, 135]`，传给 270° 水平舵机
- `tilt_abs` 为垂直绝对角，范围 `[-90, 90]`，传给 180° 垂直舵机
- `pan_delta` 为本轮水平角度变化量，仅供调试
- `tilt_delta` 为本轮垂直角度变化量，仅供调试

**云台舵机由树莓派 GPIO PWM 直接控制**，不经过 UART/Arduino。

**核心逻辑**：如果人脸在画面左侧，增加偏转角（pan），使人脸回到画面中心。

**控制策略**：
- 水平舵机（pan）：绝对位置舵机，控制水平角度
- 垂直舵机（tilt）：绝对位置舵机，控制上下角度
- 采用简单的比例控制（P控制）
- 目标进入死区/居中后，云台保持当前绝对角度，不自动回到 0°

实现约定：
- 云台控制状态与增量计算统一在 `planning/gimbal_controller.py`
- `hardware/gimbal.py` 负责 GPIO PWM 输出，不维护云台姿态状态

---

### 决策模块
- 输入：视觉输出（目标位置、宽度）
- 输出：运动指令

示例：

```json
{
  "cmd": "move",
  "v": 0.3,
  "w": -0.2
}
```

**PID 控制策略**：

线速度控制（基于距离）：
```
v = Kp_distance × (target_distance - actual_distance)
```

角速度控制（基于水平舵机相对 0° 的绝对偏角）：
```
w = Kp_angle × (pan_abs / pan_half_span)
```

建议：优先在 planning 中使用归一化量（`x_error_norm`, `y_error_norm`, `width_norm` 等）做控制，
减少分辨率变化对参数的影响。

参数说明：
- `target_distance`：期望跟随距离（例如：1.0米）
- `actual_distance`：根据目标宽度估算的实际距离
- `image_center_x`：画面中心 x 坐标
- `Kp_distance`, `Kp_angle`：比例系数（需要调参）

运动控制语义：
- `v`：线速度绝对值命令，允许正负（`+` 前进，`-` 后退）
- `w`：角速度绝对值命令，允许正负（方向由符号决定）

---

## 约束

- 禁止直接控制电机
- 禁止包含 ESP32 特定逻辑

---

## 性能

- 目标帧率 >= 15 FPS
- 最小化延迟

---

## 故障处理

- 如果视觉模块故障 → 发送 STOP 指令
