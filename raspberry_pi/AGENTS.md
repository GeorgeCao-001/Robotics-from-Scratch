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
- 输出：舵机角度指令（float）

示例：

```json
{
  "pan": -12.5,
  "tilt": 3.0
}
```

角度定义：
- `pan` 为本次调用的相对增量角，范围 `[-180, 180]`（内部保留累计绝对角）
- `tilt` 为本次调用的相对增量角（内部保留累计绝对角并限制在 `[0, 180]`）

**核心逻辑**：如果人脸在画面左侧，增加偏转角（pan），使人脸回到画面中心。

**控制策略**：
- 水平舵机（pan）：控制左右转动
- 垂直舵机（tilt）：控制上下转动
- 采用简单的比例控制（P控制）

实现约定：
- 云台控制状态与增量计算统一在 `planning/gimbal_controller.py`
- `hardware/` 只负责通信接口（如 `serial_comm.py`），不重复维护云台姿态状态

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

角速度控制（基于水平归一化误差）：
```
w = Kp_angle × x_error_norm
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
