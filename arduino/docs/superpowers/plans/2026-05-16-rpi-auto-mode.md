# Raspberry Pi 4B 自动模式集成 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将树莓派4B与Arduino UNO通过UART直连，实现树莓派完全自动控制小车运动（自动模式），不受遥控器信号干扰，保留失控急停功能，确保云台/摄像头正常配合。

**Architecture:** 树莓派4B通过USB串口（/dev/ttyACM0）与Arduino UNO的硬件串口（Serial, 115200bps）直连。Arduino端新增 `CTRL_MODE_RPI_AUTO` 模式，在该模式下只接受树莓派通过Serial发送的JSON指令，忽略遥控器命令。树莓派端 `SerialComm` 的波特率从115200匹配Arduino的硬件串口。云台由树莓派GPIO直接控制，不经过Arduino。

**Tech Stack:** Arduino C++ (ATmega328P), Python 3 (RPi), pyserial, RPi.GPIO, MediaPipe Pose

---

## 现有系统架构分析

### Arduino UNO 引脚占用表

| 引脚 | 功能 | 备注 |
|------|------|------|
| D2 | 编码器左A相 (INT0) | 外部中断 |
| D3 | 编码器右A相 (INT1) | 外部中断 |
| D4 | TB6612 STBY | 待机控制 |
| D5 | SoftSerial RX | ESP32-C3通信 |
| D6 | 电机A IN2 | 方向控制 |
| D7 | 电机A IN1 | 方向控制 |
| D8 | 电机B IN1 | 方向控制 |
| D9 | 电机A PWM | 调速 |
| D10 | 电机B PWM | 调速 |
| D11 | SoftSerial TX | ESP32-C3通信 |
| D12 | 电机B IN2 | 方向控制 |
| A0 | 电池电压检测 | 模拟输入 |
| A4 | 编码器左B相 | 方向检测 |
| A5 | 编码器右B相 | 方向检测 |

**关键发现：** Arduino UNO的硬件串口（D0 RX, D1 TX）当前未被占用！`Serial` 对象仅用于USB调试输出（115200bps）。树莓派可通过USB线连接Arduino，使用 `/dev/ttyACM0` 与Arduino的硬件串口通信。

### 树莓派4B GPIO占用表

| GPIO (BCM) | 功能 | 备注 |
|------------|------|------|
| GPIO17 | 云台水平舵机 (Pan) | PWM 50Hz |
| GPIO27 | 云台垂直舵机 (Tilt) | PWM 50Hz |

**关键发现：** 树莓派4B有大量空闲GPIO，USB口也充足。

### 当前通信架构

```
遥控器(ESP32-S3) --ESP-NOW--> 接收器(ESP32-C3) --UART 9600bps--> Arduino UNO (SoftSerial D5/D11)
树莓派4B --未连接--> Arduino UNO
```

### 目标通信架构

```
遥控器(ESP32-S3) --ESP-NOW--> 接收器(ESP32-C3) --UART 9600bps--> Arduino UNO (SoftSerial D5/D11)
树莓派4B --USB Serial 115200bps--> Arduino UNO (Hardware Serial D0/D1)
```

**核心思路：** Arduino的硬件串口 `Serial` 当前仅用于USB调试打印。我们将复用这个串口，让树莓派通过USB线连接Arduino，发送JSON运动指令。Arduino在自动模式下从 `Serial` 读取树莓派指令，同时仍可通过USB串口输出调试信息。

---

## Task 1: Arduino端 - 新增树莓派自动控制模式

**Files:**
- Modify: `d:\A_George_Projects\Arduino_TB6612_TwoWheel_Car\Arduino_TB6612_TwoWheel_Car.ino`

**设计要点：**
- 新增 `CTRL_MODE_RPI_AUTO = 3` 控制模式
- 在该模式下，Arduino从 `Serial`（硬件串口）读取树莓派发来的JSON指令
- 忽略遥控器（SoftSerial）的摇杆控制命令
- 保留遥控器超时急停作为安全后备
- 保留USB串口调试输出能力

- [ ] **Step 1: 添加控制模式定义和树莓派相关常量**

在控制模式定义区域添加：

```cpp
#define CTRL_MODE_RPI_AUTO  3   // 树莓派自动模式 (视觉跟随)
```

在系统参数配置区域添加：

```cpp
#define RPI_CMD_TIMEOUT  1000   // 树莓派指令超时 (ms)
#define RPI_CMD_MAX_LEN  64     // 树莓派指令最大长度
```

在全局变量区域添加：

```cpp
unsigned long lastRpiCmdTime = 0;
bool rpiAutoActive = false;
char rpiBuf[RPI_CMD_MAX_LEN];
uint8_t rpiBufIdx = 0;
```

- [ ] **Step 2: 实现树莓派JSON指令解析函数**

在 `parseRemoteCommand()` 函数之后添加：

```cpp
void parseRpiCommand(const char *json) {
    const char *cmdPtr = strstr(json, "\"cmd\"");
    if (!cmdPtr) return;

    if (strstr(cmdPtr, "\"stop\"")) {
        moveStop();
        lastRpiCmdTime = millis();
        return;
    }

    if (!strstr(cmdPtr, "\"move\"")) return;

    const char *vPtr = strstr(json, "\"v\"");
    const char *wPtr = strstr(json, "\"w\"");
    if (!vPtr || !wPtr) return;

    float v = atof(vPtr + 3);
    float w = atof(wPtr + 3);

    v = constrain(v, -1.0, 1.0);
    w = constrain(w, -1.0, 1.0);

    int velPulse = (int)(v * REMOTE_SPEED_MAX);
    int turnPulse = (int)(w * DEFAULT_TURN);

    directPwmControl = false;
    velocity = velPulse;
    turn = turnPulse;
    flagStop = false;

    lastRpiCmdTime = millis();

    static unsigned long lastRpiPrint = 0;
    unsigned long now = millis();
    if (now - lastRpiPrint > 200) {
        Serial.print(F("[RPI] v="));
        Serial.print(v, 2);
        Serial.print(F(" w="));
        Serial.print(w, 2);
        Serial.print(F(" -> vel="));
        Serial.print(velPulse);
        Serial.print(F(" turn="));
        Serial.println(turnPulse);
        lastRpiPrint = now;
    }
}
```

- [ ] **Step 3: 实现树莓派串口读取函数**

在 `parseRpiCommand()` 之后添加：

```cpp
void handleRpiCommand() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (rpiBufIdx > 0) {
                rpiBuf[rpiBufIdx] = '\0';
                if (ctrlMode == CTRL_MODE_RPI_AUTO) {
                    parseRpiCommand(rpiBuf);
                }
                rpiBufIdx = 0;
            }
            continue;
        }

        if (rpiBufIdx < RPI_CMD_MAX_LEN - 1) {
            rpiBuf[rpiBufIdx++] = c;
        } else {
            rpiBufIdx = 0;
        }
    }
}
```

- [ ] **Step 4: 实现树莓派超时急停检测**

在 `handleRpiCommand()` 之后添加：

```cpp
void checkRpiTimeout() {
    if (!rpiAutoActive) return;
    if (millis() - lastRpiCmdTime > RPI_CMD_TIMEOUT) {
        moveStop();
        Serial.println(F("[RPI] 树莓派指令超时! 自动停车"));
    }
}
```

- [ ] **Step 5: 修改 handleSerialCommand() - 区分USB串口命令和树莓派JSON指令**

当前 `handleSerialCommand()` 处理的是单字符命令（w/a/s/d等）。需要让它能区分单字符命令和JSON指令。修改逻辑：如果收到 `{` 开头的数据，说明是JSON指令（来自树莓派），走 `rpiBuf` 解析路径；否则走原有的单字符命令路径。

替换 `handleSerialCommand()` 函数：

```cpp
void handleSerialCommand() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '{') {
            rpiBufIdx = 0;
            rpiBuf[rpiBufIdx++] = c;
            continue;
        }

        if (rpiBufIdx > 0) {
            if (c == '\n' || c == '\r') {
                rpiBuf[rpiBufIdx] = '\0';
                if (ctrlMode == CTRL_MODE_RPI_AUTO) {
                    parseRpiCommand(rpiBuf);
                }
                rpiBufIdx = 0;
            } else if (rpiBufIdx < RPI_CMD_MAX_LEN - 1) {
                rpiBuf[rpiBufIdx++] = c;
            } else {
                rpiBufIdx = 0;
            }
            continue;
        }

        if (c == '\n' || c == '\r') continue;

        switch (c) {
            case 'w': case 'W':
                demoMode = false;
                moveForward(DEFAULT_SPEED);
                break;
            case 's': case 'S':
                demoMode = false;
                moveBackward(DEFAULT_SPEED);
                break;
            case 'a': case 'A':
                demoMode = false;
                turnLeft(DEFAULT_TURN);
                break;
            case 'd': case 'D':
                demoMode = false;
                turnRight(DEFAULT_TURN);
                break;
            case 'q': case 'Q':
                demoMode = false;
                moveForwardLeft(DEFAULT_SPEED, DEFAULT_TURN);
                break;
            case 'e': case 'E':
                demoMode = false;
                moveForwardRight(DEFAULT_SPEED, DEFAULT_TURN);
                break;
            case ' ':
                demoMode = false;
                moveStop();
                break;
            case 'r': case 'R':
                ctrlMode = CTRL_MODE_DEMO;
                demoMode = true;
                Serial.println(F("[模式] 切换到演示模式"));
                break;
            case 'p': case 'P':
                ctrlMode = CTRL_MODE_RPI_AUTO;
                rpiAutoActive = true;
                demoMode = false;
                lastRpiCmdTime = millis();
                Serial.println(F("[模式] 切换到树莓派自动模式"));
                break;
            case 'm': case 'M':
                ctrlMode = (ctrlMode + 1) % 4;
                demoMode = (ctrlMode == CTRL_MODE_DEMO);
                rpiAutoActive = (ctrlMode == CTRL_MODE_RPI_AUTO);
                Serial.print(F("[模式] 切换到: "));
                switch (ctrlMode) {
                    case CTRL_MODE_DEMO:    Serial.println(F("演示模式")); break;
                    case CTRL_MODE_SERIAL:  Serial.println(F("串口命令模式")); break;
                    case CTRL_MODE_REMOTE:  Serial.println(F("遥控器模式")); break;
                    case CTRL_MODE_RPI_AUTO: Serial.println(F("树莓派自动模式")); break;
                }
                break;
            case '+': case '=':
                if (!flagStop) {
                    float newVel = velocity + 5;
                    velocity = constrain(newVel, -100, 100);
                    Serial.print(F("[速度] 当前速度="));
                    Serial.println(velocity);
                }
                break;
            case '-': case '_':
                if (!flagStop) {
                    float newVel = velocity - 5;
                    velocity = constrain(newVel, -100, 100);
                    Serial.print(F("[速度] 当前速度="));
                    Serial.println(velocity);
                }
                break;
            case 'i': case 'I':
                printStatus();
                break;
            default:
                Serial.print(F("[提示] 未知命令: "));
                Serial.println(c);
                break;
        }
    }
}
```

- [ ] **Step 6: 修改 parseRemoteCommand() - 自动模式下忽略遥控器摇杆数据**

在 `parseRemoteCommand()` 中，当前已有 `if (ctrlMode != CTRL_MODE_REMOTE) return;` 的过滤。需要确保 `CTRL_MODE_RPI_AUTO` 模式下也被过滤。当前逻辑已经满足（因为 `ctrlMode != CTRL_MODE_REMOTE` 时直接return），无需额外修改。但需要在模式切换请求中处理自动模式：

修改 `parseRemoteCommand()` 中的模式切换逻辑，增加树莓派自动模式的处理：

```cpp
  if (flags & FLAG_MODE_SWITCH_REQUEST) {
    moveStop();
    if (flags & FLAG_AUTO_MODE) {
      ctrlMode = CTRL_MODE_RPI_AUTO;
      rpiAutoActive = true;
      demoMode = false;
      Serial.println(F("[遥控] 用户触发: 切换到自动模式(树莓派控制)"));
      return;
    }
    ctrlMode = CTRL_MODE_REMOTE;
    rpiAutoActive = false;
    demoMode = false;
    Serial.println(F("[遥控] 用户触发: 切换到手动遥控模式"));
  }
```

- [ ] **Step 7: 修改 printStatus() - 增加树莓派自动模式显示**

在 `printStatus()` 的 switch 中添加：

```cpp
    case CTRL_MODE_RPI_AUTO: Serial.println(F("树莓派自动")); break;
```

- [ ] **Step 8: 修改 setup() - 更新启动提示信息**

在 `setup()` 的串口命令帮助信息中添加：

```cpp
  Serial.println(F("  p/P - 切换到树莓派自动模式"));
```

在控制模式列表中添加：

```cpp
  Serial.println(F("  3=树莓派自动"));
```

修改初始化日志中的模式列表：

```cpp
  Serial.println(F("  控制: 增量式PI速度闭环 + 树莓派视觉跟随"));
```

- [ ] **Step 9: 修改 loop() - 增加树莓派超时检测**

在 `loop()` 中，在遥控器超时检测之后添加：

```cpp
  if (rpiAutoActive) {
    checkRpiTimeout();
  }
```

- [ ] **Step 10: 修改 loop() - 移除 handleRpiCommand() 的独立调用**

由于 Step 5 已经将JSON指令解析整合到 `handleSerialCommand()` 中，不需要单独调用 `handleRpiCommand()`。但需要确保 `handleSerialCommand()` 在 `loop()` 中被正确调用（当前已有）。

删除之前添加的 `handleRpiCommand()` 函数（Step 3），因为其逻辑已合并到 Step 5 的 `handleSerialCommand()` 中。

- [ ] **Step 11: 编译验证**

上传代码到Arduino UNO，通过串口监视器测试：
1. 发送 `p` 切换到树莓派自动模式
2. 发送 `{"cmd":"move","v":0.3,"w":0.0}` 应让小车前进
3. 发送 `{"cmd":"move","v":0.0,"w":0.5}` 应让小车转向
4. 发送 `{"cmd":"move","v":0.0,"w":0.0}` 应让小车停止
5. 发送 `m` 循环切换模式，确认4种模式都能正确切换

---

## Task 2: 树莓派端 - 适配Arduino串口通信

**Files:**
- Modify: `d:\A_George_Projects\Arduino_TB6612_TwoWheel_Car\raspberry_pi\hardware\serial_comm.py`
- Modify: `d:\A_George_Projects\Arduino_TB6612_TwoWheel_Car\raspberry_pi\main.py`

**设计要点：**
- 树莓派 `SerialComm` 当前波特率为115200，与Arduino硬件串口匹配，无需修改波特率
- `SerialComm.send_message()` 已发送JSON格式，与Arduino端 `parseRpiCommand()` 兼容
- 需要确保树莓派启动时发送模式切换信号给Arduino
- 需要确保树莓派退出时发送停车指令

- [ ] **Step 1: 验证 SerialComm 协议兼容性**

当前 `SerialComm.send_move(v, w)` 发送格式为：
```json
{"cmd":"move","v":0.3,"w":-0.2}
```

这与Arduino端 `parseRpiCommand()` 期望的格式完全匹配。`send_stop()` 发送 `{"cmd":"move","v":0.0,"w":0.0}` 也兼容。

无需修改 `serial_comm.py`。

- [ ] **Step 2: 修改 main.py - 启动时通知Arduino进入自动模式**

在 `run()` 函数中，`comm.send_stop()` 之后，添加模式切换指令：

修改 `run()` 函数中 `comm.open()` 后的初始化序列：

```python
    try:
        from raspberry_pi.vision.pose_landmarker import (
            run_pose_landmarker_on_camera,
            run_pose_landmarker_on_rpicam,
        )

        comm.open()
        time.sleep(1.5)
        comm.send_stop()
        comm.send_message({"cmd": "mode", "mode": "rpi_auto"})
        time.sleep(0.5)
        control_thread.start()
        control_started = True
```

- [ ] **Step 3: 修改 Arduino 端 parseRpiCommand() - 支持模式切换指令**

在 `parseRpiCommand()` 中添加模式切换处理：

```cpp
void parseRpiCommand(const char *json) {
    const char *cmdPtr = strstr(json, "\"cmd\"");
    if (!cmdPtr) return;

    if (strstr(cmdPtr, "\"mode\"")) {
        const char *modePtr = strstr(json, "\"mode\"");
        if (!modePtr) return;
        if (strstr(modePtr, "\"rpi_auto\"")) {
            ctrlMode = CTRL_MODE_RPI_AUTO;
            rpiAutoActive = true;
            demoMode = false;
            lastRpiCmdTime = millis();
            Serial.println(F("[RPI] 树莓派请求: 切换到自动模式"));
        }
        return;
    }

    if (strstr(cmdPtr, "\"stop\"")) {
        moveStop();
        lastRpiCmdTime = millis();
        return;
    }

    if (!strstr(cmdPtr, "\"move\"")) return;

    const char *vPtr = strstr(json, "\"v\"");
    const char *wPtr = strstr(json, "\"w\"");
    if (!vPtr || !wPtr) return;

    float v = atof(vPtr + 3);
    float w = atof(wPtr + 3);

    v = constrain(v, -1.0, 1.0);
    w = constrain(w, -1.0, 1.0);

    int velPulse = (int)(v * REMOTE_SPEED_MAX);
    int turnPulse = (int)(w * DEFAULT_TURN);

    directPwmControl = false;
    velocity = velPulse;
    turn = turnPulse;
    flagStop = false;

    lastRpiCmdTime = millis();

    static unsigned long lastRpiPrint = 0;
    unsigned long now = millis();
    if (now - lastRpiPrint > 200) {
        Serial.print(F("[RPI] v="));
        Serial.print(v, 2);
        Serial.print(F(" w="));
        Serial.print(w, 2);
        Serial.print(F(" -> vel="));
        Serial.print(velPulse);
        Serial.print(F(" turn="));
        Serial.println(turnPulse);
        lastRpiPrint = now;
    }
}
```

- [ ] **Step 4: 修改 main.py - 退出时确保停车**

当前 `run()` 的 `finally` 块已有 `comm.send_stop()`。需要额外确保退出自动模式：

```python
    finally:
        stop_event.set()
        if control_started:
            control_thread.join(timeout=1.0)
        try:
            comm.send_stop()
        except Exception:
            pass
        try:
            comm.send_message({"cmd": "mode", "mode": "remote"})
        except Exception:
            pass
        try:
            gimbal_hw.cleanup()
        except Exception:
            pass
        comm.close()
```

- [ ] **Step 5: 修改 SerialComm - 支持 mode 命令**

当前 `_validate_message()` 只允许 `move` 和 `status` 命令。需要添加 `mode`：

```python
    def _validate_message(self, message: JsonMessage) -> None:
        cmd = message.get("cmd")
        if cmd not in {"move", "status", "mode"}:
            raise ValueError("Unsupported cmd")

        if cmd == "status":
            return

        if cmd == "mode":
            mode = message.get("mode")
            if not isinstance(mode, str):
                raise ValueError("mode command requires string mode")
            return

        if cmd == "move":
            v = message.get("v")
            w = message.get("w")
            if not _is_number(v) or not _is_number(w):
                raise ValueError("move command requires numeric v and w")
            return
```

- [ ] **Step 6: 端到端测试**

在树莓派上运行：
```bash
python -m raspberry_pi.main --port /dev/ttyACM0 --debug-control --debug-vision
```

验证：
1. Arduino串口监视器显示 `[RPI] 树莓派请求: 切换到自动模式`
2. 摄像头检测到人体后，小车跟随运动
3. Ctrl+C 退出后，Arduino显示回到遥控器模式

---

## Task 3: 硬件连接方案文档化

**Files:**
- Modify: `d:\A_George_Projects\Arduino_TB6612_TwoWheel_Car\技术文档.md`

- [ ] **Step 1: 在技术文档中添加硬件连接章节**

在技术文档的适当位置添加树莓派4B与Arduino UNO的硬件连接说明，包括：
1. 连接方式：USB A-to-B 数据线（树莓派USB口 → Arduino UNO USB口）
2. 通信参数：115200bps, 8N1
3. 设备路径：`/dev/ttyACM0`（Linux）或 `/dev/cu.usbmodemxxxx`（macOS）
4. 引脚冲突分析：无冲突（使用Arduino硬件串口D0/D1，不占用任何新引脚）
5. 云台连接：树莓派GPIO17(Pan) + GPIO27(Tilt)，通过杜邦线连接舵机
6. 供电方案：树莓派独立供电（5V 3A USB-C），Arduino由12V电池组供电
7. 共地要求：树莓派GND与Arduino GND必须相连（USB线已包含GND）

---

## Task 4: 失控急停功能完善

**Files:**
- Modify: `d:\A_George_Projects\Arduino_TB6612_TwoWheel_Car\Arduino_TB6612_TwoWheel_Car.ino`

- [ ] **Step 1: 完善急停逻辑 - 多层保护**

当前已有：
- 遥控器超时急停（500ms）
- 树莓派超时急停（1000ms）

需要确保在 `CTRL_MODE_RPI_AUTO` 模式下：
1. 树莓派指令超时 → 自动停车（已实现）
2. 遥控器仍可发送急停信号（模式切换请求）→ 切换到遥控器模式并停车
3. USB串口发送空格 → 立即停车（已实现）

验证 `parseRemoteCommand()` 中的模式切换逻辑：当遥控器发送 `FLAG_MODE_SWITCH_REQUEST` 时，无论当前是什么模式，都会先 `moveStop()`。这确保了遥控器急停功能在自动模式下仍然有效。

无需额外代码修改，仅需验证测试。

- [ ] **Step 2: 添加紧急停止命令支持**

在 `parseRpiCommand()` 中添加 `estop` 命令支持：

```cpp
    if (strstr(cmdPtr, "\"estop\"")) {
        moveStop();
        ctrlMode = CTRL_MODE_REMOTE;
        rpiAutoActive = false;
        Serial.println(F("[RPI] 紧急停止! 切换到遥控器模式"));
        lastRpiCmdTime = millis();
        return;
    }
```

同时更新 `SerialComm._validate_message()` 允许 `estop` 命令。

---

## 硬件连接方案（详细）

### 方案：USB串口直连（推荐）

**连接方式：** 树莓派4B USB口 → USB A-to-B 数据线 → Arduino UNO USB口

**优点：**
- 无需占用任何新的Arduino引脚
- USB线自带GND共地
- 通信稳定，115200bps可靠传输
- 即插即用，无需电平转换

**接线图：**
```
树莓派4B                    Arduino UNO
┌──────────┐               ┌──────────┐
│ USB口    ├──USB A-to-B──→│ USB口    │
│ (主机)   │               │ (设备)   │
└──────────┘               └──────────┘

树莓派4B                    云台舵机
┌──────────┐               ┌──────────┐
│ GPIO17   ├───信号线─────→│ Pan舵机  │
│ GPIO27   ├───信号线─────→│ Tilt舵机 │
│ GND      ├───地线───────→│ GND      │
│ 5V       ├───电源───────→│ VCC(5V)  │
└──────────┘               └──────────┘
```

**注意事项：**
1. 树莓派和Arduino必须**共地**（USB线已包含GND连接）
2. 舵机供电建议使用独立5V电源，不要仅依赖树莓派5V引脚（舵机峰值电流可达500mA+）
3. Arduino由12V电池组通过DC圆孔供电
4. 树莓派由独立5V 3A USB-C电源供电

### 通信协议

**树莓派 → Arduino (JSON格式, 115200bps, 换行符结束)**

| 指令 | 格式 | 说明 |
|------|------|------|
| 运动控制 | `{"cmd":"move","v":0.3,"w":-0.2}\n` | v:线速度[-1,1], w:角速度[-1,1] |
| 停车 | `{"cmd":"move","v":0.0,"w":0.0}\n` | 等同于stop |
| 模式切换 | `{"cmd":"mode","mode":"rpi_auto"}\n` | 请求切换到树莓派自动模式 |
| 模式切换 | `{"cmd":"mode","mode":"remote"}\n` | 请求切换回遥控器模式 |
| 紧急停止 | `{"cmd":"estop"}\n` | 立即停车并切换到遥控器模式 |

**v/w 到 Arduino 速度的映射：**
- `v` 范围 [-1.0, 1.0] → `velocity` 范围 [-REMOTE_SPEED_MAX, REMOTE_SPEED_MAX] = [-40, 40] 脉冲/5ms
- `w` 范围 [-1.0, 1.0] → `turn` 范围 [-DEFAULT_TURN, DEFAULT_TURN] = [-18, 18] 脉冲/5ms

### 云台连接

**树莓派 GPIO → 舵机信号线：**
- GPIO17 (BCM) → 水平舵机(Pan) 信号线（橙色/黄色）
- GPIO27 (BCM) → 垂直舵机(Tilt) 信号线（橙色/黄色）
- GND → 舵机地线（棕色/黑色）
- 5V → 舵机电源线（红色）— 建议外部供电

**舵机参数：**
- PWM频率：50Hz（周期20ms）
- 水平舵机：270° 范围，脉宽 500~2500μs
- 垂直舵机：180° 范围，脉宽 500~2500μs
