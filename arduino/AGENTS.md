# AGENTS.md (Arduino)

## 角色

负责：
- 电机控制（L298N 驱动 4WD 底盘）
- 传感器读取（电池电压、编码器可选）
- 实时执行
- 接收树莓派指令并通过串口反馈状态

---

## 职责

### 控制循环
- 从串口读取指令
- 更新电机输出

---

### 安全机制

**1. 心跳超时保护**
- 如果在 500ms 内未收到任何指令，自动停止电机
- 防止树莓派程序崩溃导致车辆失控

**2. 指令验证**
- 检查 JSON 格式有效性
- 检查速度值范围（-1.0 ~ 1.0）
- 无效指令返回错误状态

**3. 低电压保护**
- 监测电池电压
- 低于 7.0V 时自动停止并上报

---

### 状态反馈

定期（建议 100ms）向树莓派发送状态：

```json
{
  "status": "ok",
  "battery": 87,
  "left_speed": 0.3,
  "right_speed": 0.35,
  "error": null
}
```

错误状态示例：

```json
{
  "status": "error",
  "battery": 87,
  "error": "LOW_BATTERY"
}
```

---

## 约束

- 禁止繁重的浮点运算
- 禁止阻塞式延时
- 需要确定性循环
- 注意内存限制（2KB SRAM）

---

## 接口

输入（来自树莓派）：

```json
{
  "v": 0.5,
  "w": 0.1
}
```

---

## 硬件配置

- **电机驱动**：L298N（双路 H 桥）
- **底盘**：4WD 四轮驱动
  - 左前轮 + 左后轮（并联）→ 左电机组
  - 右前轮 + 右后轮（并联）→ 右电机组
- **电源**：电池（需监测电压）
- **通信**：UART 串口（连接树莓派）

---

## 控制策略

### 轮速转换

对于 4WD 差速底盘，线速度 `v` 和角速度 `w` 转换为左右轮速：

```cpp
// 轮距（左右轮中心距离，单位：米）
const float WHEEL_BASE = 0.25;

// 左轮速度
left_speed = v - w * WHEEL_BASE / 2.0;

// 右轮速度
right_speed = v + w * WHEEL_BASE / 2.0;
```

### PWM 映射

将轮速转换为 PWM 占空比（假设速度范围 ±1.0 对应 PWM 0-255）：

```cpp
int speed_to_pwm(float speed) {
    // 限制速度范围
    speed = constrain(speed, -1.0, 1.0);
    // 映射到 PWM 值
    return (int)(speed * 255);
}
```

### 电机控制

L298N 控制逻辑：

| ENA  | IN1 | IN2 | 左电机状态 |
|------|-----|-----|------------|
| HIGH | HIGH| LOW | 正转       |
| HIGH | LOW | HIGH| 反转       |
| LOW  | X   | X   | 停止       |

（右电机 ENB/IN3/IN4 同理）

---

## 传感器

### 电池电压监测

通过分压电阻读取电池电压：

```cpp
// 分压比（根据实际电路调整）
const float VOLTAGE_DIVIDER = 0.5;
const float ADC_REFERENCE = 3.3;
const int ADC_RESOLUTION = 1023;

float read_battery_voltage() {
    int adc_value = analogRead(BATTERY_PIN);
    float voltage = (adc_value * ADC_REFERENCE / ADC_RESOLUTION) / VOLTAGE_DIVIDER;
    return voltage;
}
```

**低电压保护**：当电池电压低于阈值（如 7.0V 对于 2S 锂电池），自动停止并上报。

---

## 参数说明

- **v**：线速度（m/s），正为前进，负为后退
- **w**：角速度（rad/s），正为左转，负为右转
- **WHEEL_BASE**：轮距（左右轮间距）
- **PWM 频率**：建议 20kHz（降低电机噪音）
