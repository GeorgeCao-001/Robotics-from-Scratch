# 编码器 A/B 双相升级 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将编码器从单A相计数升级为A+B双相方向检测，使编码器能自动判断正转/反转方向

**Architecture:** 在现有A相外部中断(INT0/INT1)的ISR中增加B相digitalRead()判断方向，正转+1反转-1。B相引脚选用A4/A5(模拟口)，无需PCINT，改动最小化。同时修正ENCODER_PPR注释与实际计数方式的不一致。

**Tech Stack:** Arduino UNO (ATmega328P), C++, MG513XP28_12V 霍尔编码器

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|------|------|
| `arduino/Arduino_TB6612_TwoWheel_Car/Arduino_TB6612_TwoWheel_Car.ino` | 修改 | 新增B相引脚定义、修改ISR方向判断、修正ENCODER_PPR注释 |
| `arduino/Arduino_TB6612_TwoWheel_Car/MG513XP28_12V_接线指南.md` | 修改 | 更新电机接口说明(5线→6线)、接线表、检查清单、示意图 |
| `arduino/Arduino_TB6612_TwoWheel_Car/遥控器系统接线指南.md` | 修改 | 更新Arduino引脚分配汇总表 |
| `arduino/Arduino_TB6612_TwoWheel_Car/logs/changelog/2026-05-14/main_controller.md` | 修改 | 追加v2.3.0变更记录 |
| `arduino/Arduino_TB6612_TwoWheel_Car/logs/changelog/2026-05-14/wiring_guide.md` | 修改 | 追加v2.3.0变更记录 |

---

## 引脚分配方案

当前Arduino UNO引脚占用：

| 引脚 | 当前用途 | 状态 |
|------|---------|------|
| D0/D1 | Hardware Serial | 占用 |
| D2 | 左编码器A相 (INT0) | 占用 |
| D3 | 右编码器A相 (INT1) | 占用 |
| D4 | TB6612 STBY | 占用 |
| D5 | SoftSerial RX | 占用 |
| D6 | MOTOR_A_IN2 | 占用 |
| D7 | MOTOR_A_IN1 | 占用 |
| D8 | MOTOR_B_IN1 | 占用 |
| D9 | MOTOR_A_PWM | 占用 |
| D10 | MOTOR_B_PWM | 占用 |
| D11 | SoftSerial TX | 占用 |
| D12 | MOTOR_B_IN2 | 占用 |
| A0 | 电池电压ADC | 占用 |
| **A1** | **空闲** | 可用 |
| **A2** | **空闲** | 可用 |
| **A3** | **空闲** | 可用 |
| **A4** | **空闲** | 可用 |
| **A5** | **空闲** | 可用 |

**B相引脚选择：A4 (左编码器B相)、A5 (右编码器B相)**

选择理由：
- A1~A5全部空闲，A4/A5位于端口C末端，与其他功能无冲突
- B相仅需在A相ISR中digitalRead()读取，不需要独立中断
- A4/A5若将来需要I2C，可灵活调整到A1/A2

---

## A/B相方向判断原理

```
正转时:                    反转时:
  A: ┌─┐ ┌─┐               A: ┌─┐ ┌─┐
     │ │ │ │                  │ │ │ │
  ───┘ └─┘ └──          ────┘ └─┘ └──
  B:  ┌─┐ ┌─┐            B:┌─┐ ┌─┐
      │ │ │ │               │ │ │ │
  ────┘ └─┘ └─         ────┘ └─┘ └──

A相上升沿时:
  B=LOW  → 正转 → count++
  B=HIGH → 反转 → count--
```

---

### Task 1: 修改主控代码 - 新增B相引脚定义和ISR方向判断

**Files:**
- Modify: `arduino/Arduino_TB6612_TwoWheel_Car/Arduino_TB6612_TwoWheel_Car.ino`

- [ ] **Step 1: 新增B相引脚定义**

在 `ENCODER_RIGHT_A` 定义之后（约第50行后），新增B相引脚定义：

```cpp
// 左电机编码器 (使用外部中断 INT0)
#define ENCODER_LEFT_A   2   // 编码器A相脉冲 (INT0)
#define ENCODER_LEFT_B   A4  // 编码器B相方向 (A4)

// 右电机编码器 (使用外部中断 INT1)
#define ENCODER_RIGHT_A  3   // 编码器A相脉冲 (INT1)
#define ENCODER_RIGHT_B  A5  // 编码器B相方向 (A5)
```

- [ ] **Step 2: 修正ENCODER_PPR注释**

当前代码注释写"4倍频后"但实际只用了A相上升沿(1倍频)，需修正：

```cpp
// 编码器参数 (MG513XP28_12V典型值)
// 霍尔编码器: 11线, A相上升沿计数 = 11脉冲/电机轴圈
// 减速比1:28/1:30时, 输出轴每圈脉冲数 = 11 × 减速比
#define ENCODER_PPR     11      // 编码器每圈脉冲数 (电机轴, A相上升沿)
```

> 注意：ENCODER_PPR从44改为11。但此宏当前未在控制循环中使用（控制循环直接用原始脉冲计数），所以不影响PI控制行为。如果将来用于RPM计算，需使用正确值。

- [ ] **Step 3: 修改编码器ISR - 增加方向判断**

将ISR从单纯计数改为方向判断计数：

```cpp
// 左电机编码器中断 (INT0, Pin 2)
// A相上升沿时读取B相: B=LOW→正转(+1), B=HIGH→反转(-1)
void encoderLeftISR() {
  if (digitalRead(ENCODER_LEFT_B) == LOW) {
    encoderLeftCount++;
  } else {
    encoderLeftCount--;
  }
}

// 右电机编码器中断 (INT1, Pin 3)
// A相上升沿时读取B相: B=LOW→正转(+1), B=HIGH→反转(-1)
void encoderRightISR() {
  if (digitalRead(ENCODER_RIGHT_B) == LOW) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}
```

- [ ] **Step 4: 在setup()中初始化B相引脚**

在编码器中断初始化部分，增加B相引脚的pinMode设置：

```cpp
  // --- 编码器中断初始化 ---
  pinMode(ENCODER_LEFT_A,  INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B,  INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  // 绑定外部中断 (RISING沿触发)
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A),
                  encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A),
                  encoderRightISR, RISING);
```

- [ ] **Step 5: 更新文件头部注释中的编码器描述**

将文件头部的编码器相关描述更新：

```
 * 电机接口: 6线集成接口 (电机电源×2 + 编码器×4)
```

- [ ] **Step 6: 验证编译通过**

Run: `pio run -d "d:\文档\PlatformIO\Projects\Robotics-from-Scratch"` 或在Arduino IDE中验证编译
Expected: 编译成功，无错误

---

### Task 2: 修改接线指南文档 - 更新编码器接口说明

**Files:**
- Modify: `arduino/Arduino_TB6612_TwoWheel_Car/MG513XP28_12V_接线指南.md`

- [ ] **Step 1: 更新电机接口说明（第一节）**

将5线接口更新为6线接口：

```markdown
 * MG513XP28_12V 电机为6线集成接口，包含：
 * 
 *  接口序号 | 功能说明       | 典型线色       | 电压等级
 *  --------|---------------|---------------|----------
 *  1       | 电机电源+     | 红色          | 12V
 *  2       | 电机电源-     | 黑色          | GND
 *  3       | 编码器VCC     | 白色/红色     | 3.3V~5V
 *  4       | 编码器GND     | 白色/黑色     | GND
 *  5       | 编码器信号A   | 黄色          | 开漏输出
 *  6       | 编码器信号B   | 绿色/蓝色     | 开漏输出
 *
 * 注意:
 *  - 6线接口为集成设计，无法分离，需整体连接
 *  - 编码器为双端输出(A相+B相)，支持方向检测
 *  - A相用于脉冲计数(外部中断)，B相用于方向判断
 *  - A/B相输出相差90°，A相上升沿时B相电平决定转向:
 *    B=LOW → 正转, B=HIGH → 反转
```

- [ ] **Step 2: 更新接线表（第三节）**

左电机接线表增加B相行：

```markdown
 *  左电机(L) → 模块A口 (TB6612):
 *
 *  左电机接口    | 模块A口引脚   | Arduino引脚   | 说明
 *  -------------|--------------|--------------|-------------------
 *  电机电源+     | AO1          | -            | 左电机电源正极
 *  电机电源-     | AO2          | -            | 左电机电源负极
 *  编码器VCC     | -            | 5V           | 编码器供电
 *  编码器GND     | -            | GND          | 编码器地
 *  编码器信号A   | -            | D2 (INT0)    | 左编码器A相脉冲
 *  编码器信号B   | -            | A4           | 左编码器B相方向
```

右电机接线表增加B相行：

```markdown
 *  右电机(R) → 模块B口 (TB6612):
 *
 *  右电机接口    | 模块B口引脚   | Arduino引脚   | 说明
 *  -------------|--------------|--------------|-------------------
 *  电机电源+     | BO1          | -            | 右电机电源正极
 *  电机电源-     | BO2          | -            | 右电机电源负极
 *  编码器VCC     | -            | 5V           | 编码器供电
 *  编码器GND     | -            | GND          | 编码器地
 *  编码器信号A   | -            | D3 (INT1)    | 右编码器A相脉冲
 *  编码器信号B   | -            | A5           | 右编码器B相方向
```

- [ ] **Step 3: 更新电路检查清单（第四节）**

编码器检查部分增加B相检查项：

```markdown
 * 3. 编码器检查:
 *  [ ] 左编码器VCC → Arduino 5V
 *  [ ] 左编码器GND → Arduino GND
 *  [ ] 左编码器信号A → Arduino D2
 *  [ ] 左编码器信号B → Arduino A4
 *  [ ] 右编码器VCC → Arduino 5V
 *  [ ] 右编码器GND → Arduino GND
 *  [ ] 右编码器信号A → Arduino D3
 *  [ ] 右编码器信号B → Arduino A5
```

- [ ] **Step 4: 更新接线验证步骤（第五节）**

步骤2编码器信号验证增加B相验证：

```markdown
 * 步骤2: 编码器信号验证 (接编码器, 手转电机)
 *   - 将编码器A相信号引脚连接到Arduino D2/D3
 *   - 将编码器B相信号引脚连接到Arduino A4/A5
 *   - 手转电机轴, 用示波器或串口打印观察脉冲
 *   - 确认A相有脉冲输出
 *   - 确认A/B相相位差约90° (正转时A超前B, 反转时B超前A)
```

- [ ] **Step 5: 更新驱动程序配置建议（第六节）**

修正编码器脉冲数说明：

```markdown
 *  编码器脉冲/圈 | 11 (A相上升沿) | 单倍频计数
```

以及配置建议：

```markdown
 * 1. 修改编码器脉冲数:
 *    #define ENCODER_PPR     11    // A相上升沿计数 (单倍频)
```

- [ ] **Step 6: 更新接线示意图（第八节）**

左电机示意图增加B相：

```markdown
 * 左电机 → 模块A口:
 *   [左电机6线接口]
 *     ├─ 电机+ ──→ [模块A口 AO1]
 *     ├─ 电机- ──→ [模块A口 AO2]
 *     ├─ 编码器VCC ──→ [Arduino 5V]
 *     ├─ 编码器GND ──→ [Arduino GND]
 *     ├─ 编码器A相 ──→ [Arduino D2]
 *     └─ 编码器B相 ──→ [Arduino A4]
```

右电机示意图增加B相：

```markdown
 * 右电机 → 模块B口:
 *   [右电机6线接口]
 *     ├─ 电机+ ──→ [模块B口 BO1]
 *     ├─ 电机- ──→ [模块B口 BO2]
 *     ├─ 编码器VCC ──→ [Arduino 5V]
 *     ├─ 编码器GND ──→ [Arduino GND]
 *     ├─ 编码器A相 ──→ [Arduino D3]
 *     └─ 编码器B相 ──→ [Arduino A5]
```

---

### Task 3: 修改遥控器系统接线指南 - 更新引脚分配表

**Files:**
- Modify: `arduino/Arduino_TB6612_TwoWheel_Car/遥控器系统接线指南.md`

- [ ] **Step 1: 更新Arduino UNO引脚分配汇总表**

在D2/D3行增加"A相"标注，并新增A4/A5行：

```markdown
 *  ┌──────────┬──────────────────────┬───────────────────────────┐
 *  │ 引脚     │ 功能                 │ 说明                      │
 *  ├──────────┼──────────────────────┼───────────────────────────┤
 *  │ D0       │ Hardware Serial RX   │ USB调试                   │
 *  │ D1       │ Hardware Serial TX   │ USB调试                   │
 *  │ D2       │ 编码器左A相 (INT0)   │ 霍尔编码器A相脉冲         │
 *  │ D3       │ 编码器右A相 (INT1)   │ 霍尔编码器A相脉冲         │
 *  │ D4       │ TB6612 STBY          │ 待机控制                  │
 *  │ D5       │ SoftSerial RX        │ ← ESP32-C3 UART1 TX      │
 *  │ D6       │ MOTOR_A_IN2          │ 左电机方向2               │
 *  │ D7       │ MOTOR_A_IN1          │ 左电机方向1               │
 *  │ D8       │ MOTOR_B_IN1          │ 右电机方向1               │
 *  │ D9       │ MOTOR_A_PWM          │ 左电机PWM                 │
 *  │ D10      │ MOTOR_B_PWM          │ 右电机PWM                 │
 *  │ D11      │ SoftSerial TX        │ → ESP32-C3 UART1 RX      │
 *  │ D12      │ MOTOR_B_IN2          │ 右电机方向2               │
 *  │ A0       │ 电池电压ADC          │ 分压检测                  │
 *  │ A4       │ 编码器左B相          │ 霍尔编码器B相方向         │
 *  │ A5       │ 编码器右B相          │ 霍尔编码器B相方向         │
 *  └──────────┴──────────────────────┴───────────────────────────┘
```

---

### Task 4: 更新changelog日志

**Files:**
- Modify: `arduino/Arduino_TB6612_TwoWheel_Car/logs/changelog/2026-05-14/main_controller.md`
- Modify: `arduino/Arduino_TB6612_TwoWheel_Car/logs/changelog/2026-05-14/wiring_guide.md`

- [ ] **Step 1: 更新主控changelog**

在 `main_controller.md` 末尾追加v2.3.0变更记录：

```markdown
================================================================================
六、v2.3.0 追加更新 (2026-05-14)
================================================================================

【新增功能】
  - 编码器升级为A/B双相方向检测
    新增ENCODER_LEFT_B(A4)和ENCODER_RIGHT_B(A5)引脚定义，
    ISR中在A相上升沿读取B相电平判断转向:
      B=LOW → 正转(count++)
      B=HIGH → 反转(count--)

【修复的问题】
  - 修正ENCODER_PPR注释与实际计数方式不一致
    原注释"4倍频后=44"但代码仅用A相上升沿(1倍频)，
    现修正为ENCODER_PPR=11 (A相上升沿计数)
    注: 此宏当前未参与控制循环计算，不影响PI行为

【优化项】
  - 文件头电机接口描述从"5线"更新为"6线"
  - setup()中新增B相引脚INPUT_PULLUP初始化

================================================================================
```

- [ ] **Step 2: 更新接线指南changelog**

在 `wiring_guide.md` 末尾追加v2.3.0变更记录：

```markdown
================================================================================
六、v2.3.0 追加更新 (2026-05-14)
================================================================================

【新增内容】
  - 接线指南全面更新为6线编码器接口
    新增编码器B相信号线(序号6, 绿色/蓝色)
  - 接线表新增B相行: 左编码器B→A4, 右编码器B→A5
  - 电路检查清单新增B相连接检查项
  - 接线验证步骤新增A/B相相位差验证
  - 接线示意图从5线更新为6线

【修复的问题】
  - 修正编码器脉冲/圈说明
    原说明"44 (11×4) 4倍频后脉冲数"与实际1倍频计数不一致
    现修正为"11 (A相上升沿) 单倍频计数"

【优化项】
  - 遥控器系统接线指南Arduino引脚分配表新增A4/A5编码器B相行
  - D2/D3功能描述从"编码器左/右"更精确为"编码器左A相/右A相"

================================================================================
```

---

### Task 5: 整体验证

- [ ] **Step 1: 全局搜索确认无遗漏**

搜索所有文件中的以下关键词，确认无残留旧描述：
- "5线" (应为6线)
- "单端输出" (应为双端输出)
- "无B相" (应已删除)
- "4倍频后" (应为A相上升沿)
- "44" 在编码器上下文中 (应为11)

- [ ] **Step 2: 编译验证**

确认主控代码编译通过，无错误无警告

- [ ] **Step 3: 逻辑一致性检查**

确认以下逻辑链完整一致：
1. 引脚定义: ENCODER_LEFT_B=A4, ENCODER_RIGHT_B=A5 ✓
2. ISR方向判断: A相上升沿读B相 ✓
3. setup()初始化: B相INPUT_PULLUP ✓
4. 接线文档: 6线接口，B相→A4/A5 ✓
5. 引脚分配表: A4/A5编码器B相 ✓
6. ENCODER_PPR: 11 (1倍频) ✓

---

## 兼容性说明

1. **硬件兼容**: 需确认MG513XP28_12V电机实际引出了B相线(6线版本)。若为5线版本(无B相引出)，B相引脚A4/A5悬空(INPUT_PULLUP默认HIGH)，ISR中digitalRead()恒为HIGH，编码器计数将始终递减。此时需将ISR改回纯递增模式或断开B相连接。

2. **PI控制兼容**: 编码器计数现在有正负方向，但PI控制器已通过`velocity + turn` / `velocity - turn`计算目标速度，正转时encoder为正、反转时为负，与targetSpeed的符号一致，PI控制逻辑无需修改。

3. **速度值兼容**: DEFAULT_SPEED=25等参数基于原始脉冲计数设定。A相上升沿1倍频下，同转速的脉冲数与之前一致（之前也是1倍频，只是注释误标为4倍频），所以速度参数无需调整。

4. **编码器回传**: sendEncoderFeedback()回传的encoderLeft/encoderRight现在可能为负值，constrain(-127, 127)仍可正常工作，ESP32-C3接收端int8_t可正确表示负数。
