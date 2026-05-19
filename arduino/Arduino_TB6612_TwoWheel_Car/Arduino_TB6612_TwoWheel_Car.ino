/*
 * TB6612 两驱小车控制程序 - Arduino UNO
 * ==========================================
 * 基于 L130_模块化小车_S28A_HAL库 的控制逻辑移植
 *
 * 硬件平台: Arduino UNO (ATmega328P, 16MHz)
 * 驱动模块: TB6612FNG 双路H桥电机驱动 (A口+B口)
 * 电机类型: MG513XP28_12V (WHEELTEC带霍尔编码器直流减速电机)
 * 电机接口: 6线集成接口 (电机电源×2 + 编码器×4)
 * 电机连接: 左电机→A口, 右电机→B口
 * 供电方式: 12V锂电池组 (3S, 标称11.1V~12.6V)
 * 小车结构: 三轮结构 (两驱动轮 + 前部万向轮)
 *
 * 控制方式: 差速驱动 (Diff_Car模式)
 * 速度控制: 增量式PI闭环控制
 * 控制周期: 5ms (200Hz)
 * 遥控方式: ESP-NOW (ESP32-S3→ESP32-C3→UART→Arduino)
 */

// ============================================================
// 引脚定义 - TB6612 双路电机驱动模块 (A口+B口)
// ============================================================
// 模块集成一颗TB6612FNG芯片, 提供2个独立电机端口:
//   A口: 控制左电机
//   B口: 控制右电机
//
// 每个端口有3个控制引脚: PWM + IN1 + IN2

// 左电机 → 模块A口
#define MOTOR_A_PWM   9     // PWMA - PWM调速
#define MOTOR_A_IN1   7     // AIN1 - 方向控制1
#define MOTOR_A_IN2   6     // AIN2 - 方向控制2

// 右电机 → 模块B口
#define MOTOR_B_PWM   10    // PWMB - PWM调速
#define MOTOR_B_IN1   8     // BIN1 - 方向控制1
#define MOTOR_B_IN2   12    // BIN2 - 方向控制2

// TB6612 待机控制 (STBY=HIGH正常工作, LOW=待机)
#define STBY_PIN      4     // STBY

// ============================================================
// 引脚定义 - 霍尔编码器
// ============================================================

// 左电机编码器 (使用外部中断 INT0)
#define ENCODER_LEFT_A   2   // 编码器A相脉冲 (INT0)
#define ENCODER_LEFT_B   A4  // 编码器B相方向 (A4)

// 右电机编码器 (使用外部中断 INT1)
#define ENCODER_RIGHT_A  3   // 编码器A相脉冲 (INT1)
#define ENCODER_RIGHT_B  A5  // 编码器B相方向 (A5)

// ============================================================
// 引脚定义 - ESP32-C3 通信 (SoftwareSerial)
// ============================================================
// ESP32-C3 作为ESP-NOW接收器, 通过UART与Arduino UNO通信
// ESP32-C3 UART1 TX (GPIO5) → Arduino D5 (SoftSerial RX)
// ESP32-C3 UART1 RX (GPIO6) ← Arduino D11 (SoftSerial TX)

#define SOFTSERIAL_RX  5     // D5 - 接收来自ESP32-C3
#define SOFTSERIAL_TX  11    // D11 - 发送至ESP32-C3

// ============================================================
// 控制模式定义
// ============================================================
#define CTRL_MODE_DEMO    0   // 演示模式 (自动运行)
#define CTRL_MODE_SERIAL  1   // 串口命令模式 (USB调试)
#define CTRL_MODE_REMOTE  2   // 遥控器模式 (ESP-NOW)
#define CTRL_MODE_RPI_AUTO 3  // 树莓派自动模式 (视觉跟随)

// 遥控器超时 (ms), 超过此时间未收到指令则自动停车
#define REMOTE_TIMEOUT   500

// 树莓派指令超时 (ms), 超过此时间未收到指令则自动停车
#define RPI_CMD_TIMEOUT  1000
#define RPI_CMD_MAX_LEN  64

// ============================================================
// 系统参数配置
// ============================================================

// PWM参数
#define PWM_MAX         255     // Arduino PWM最大值 (8位)
#define PWM_LIMIT       240     // PWM限幅值 (预留余量)

// ============================================================
// MG513XP28_12V 电机参数配置
// ============================================================
// 编码器参数 (MG513XP28_12V典型值)
// 霍尔编码器: 11线, A相上升沿计数 = 11脉冲/电机轴圈
// 减速比1:28/1:30时, 输出轴每圈脉冲数 = 11 × 减速比
#define ENCODER_PPR     11      // 编码器每圈脉冲数 (电机轴, A相上升沿)
#define GEAR_RATIO      28      // 减速比 (根据实际型号调整: 10/20/28/30/60)

// PID控制参数 (MG513XP28电机优化值)
// MG513电机扭矩较大, 响应较慢, KP/KI需适当增大
#define VELOCITY_KP     18.0f   // 速度环比例系数 (建议范围: 15-25)
#define VELOCITY_KI     8.0f    // 速度环积分系数 (建议范围: 5-12)
#define VELOCITY_KD     0.0f    // 速度环微分系数 (通常为0)

// 默认运动速度 (编码器脉冲/5ms控制周期)
// MG513电机建议初始值: 20-30脉冲/5ms
#define DEFAULT_SPEED   25      // 默认前进速度
#define DEFAULT_TURN    18      // 默认转向差速

// PID输出限幅
#define PID_OUTPUT_MIN  -240    // PID最小输出
#define PID_OUTPUT_MAX  240     // PID最大输出

// 电池电压阈值
#define BATTERY_LOW     10.0f   // 低压保护阈值 (12V锂电池组)

// 控制周期
#define CONTROL_PERIOD  5       // 控制周期 (ms)

// ============================================================
// 库引入
// ============================================================
#include <SoftwareSerial.h>

// ============================================================
// SoftwareSerial 实例 (与ESP32-C3通信)
// ============================================================
SoftwareSerial remoteSerial(SOFTSERIAL_RX, SOFTSERIAL_TX);

// ============================================================
// 全局变量
// ============================================================

// 编码器脉冲计数 (在中断服务程序中累加)
volatile long encoderLeftCount  = 0;
volatile long encoderRightCount = 0;

// 编码器速度值 (每控制周期的脉冲数)
int encoderLeft  = 0;
int encoderRight = 0;

// 目标速度 (编码器脉冲/控制周期)
int targetSpeedA = 0;
int targetSpeedB = 0;

// 运动控制变量
float velocity = 0;     // 线速度
float turn     = 0;     // 转向角速度(差速值)

// 电机PWM输出值
int motorPwmA = 0;
int motorPwmB = 0;

// 系统状态标志
bool flagStop    = true;    // 停止标志 (默认停止状态)
bool flagRunning = false;   // 运行标志

// 电池电压
float batteryVoltage = 12.0f;

// 控制循环计数器
unsigned long lastControlTime = 0;

// 演示模式: 自动运行演示序列
// 设置为false可通过串口命令控制
bool demoMode = false;

// 控制模式 (DEMO/SERIAL/REMOTE)
uint8_t ctrlMode = CTRL_MODE_REMOTE;

// 遥控器相关变量
unsigned long lastRemoteCmdTime = 0;   // 上次收到遥控指令的时间
bool remoteConnected = false;          // 遥控器连接状态
#define REMOTE_SPEED_MAX 40            // 遥控器摇杆满偏对应的最大速度 (脉冲/5ms)
#define REMOTE_PWM_MAX 220             // 遥控模式下的最大PWM输出
#define REMOTE_TURN_PWM_MAX 80        // 遥控模式下的最大转向PWM
#define REMOTE_INPUT_DEADZONE 10       // 遥控输入死区 (%)
#define FLAG_AUTO_MODE 0x01
#define FLAG_MODE_SWITCH_REQUEST 0x02

// 遥控器串口接收缓冲区
char remoteBuf[32];
uint8_t remoteBufIdx = 0;

// 树莓派串口接收缓冲区
char rpiBuf[RPI_CMD_MAX_LEN];
uint8_t rpiBufIdx = 0;
unsigned long lastRpiCmdTime = 0;
bool rpiAutoActive = false;

// 遥控模式下直接PWM混控输出
bool directPwmControl = false;
int directPwmLeft = 0;
int directPwmRight = 0;

// PI控制器内部状态
int piBiasA = 0;
int piLastBiasA = 0;
int piPwmA = 0;
int piBiasB = 0;
int piLastBiasB = 0;
int piPwmB = 0;

// ============================================================
// 初始化函数
// ============================================================

void setup() {
  // --- 串口初始化 ---
  Serial.begin(115200);
  Serial.println(F("========================================"));
  Serial.println(F("  TB6612 两驱小车控制程序"));
  Serial.println(F("  平台: Arduino UNO"));
  Serial.println(F("  驱动: TB6612 双路H桥 (A口+B口)"));
  Serial.println(F("  控制: 增量式PI速度闭环"));
  Serial.println(F("  遥控: ESP-NOW (ESP32-S3→ESP32-C3)"));
  Serial.println(F("========================================"));
  Serial.println();

  // --- SoftwareSerial 初始化 (与ESP32-C3通信) ---
  remoteSerial.begin(9600);
  Serial.println(F("[初始化] SoftwareSerial 已启动 (9600bps)"));
  Serial.print(F("[初始化] RX=D"));
  Serial.print(SOFTSERIAL_RX);
  Serial.print(F(" TX=D"));
  Serial.println(SOFTSERIAL_TX);

  // --- TB6612 引脚初始化 ---
  pinMode(MOTOR_A_PWM,  OUTPUT);
  pinMode(MOTOR_A_IN1,  OUTPUT);
  pinMode(MOTOR_A_IN2,  OUTPUT);
  pinMode(MOTOR_B_PWM,  OUTPUT);
  pinMode(MOTOR_B_IN1,  OUTPUT);
  pinMode(MOTOR_B_IN2,  OUTPUT);
  pinMode(STBY_PIN,     OUTPUT);

  // 初始状态: 电机停止
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);

  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);

  // 使能TB6612 (STBY = HIGH)
  digitalWrite(STBY_PIN, HIGH);

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

  // --- Timer2 初始化 (5ms控制周期) ---
  // Timer2为8位定时器, 时钟16MHz
  // 预分频1024: 16MHz/1024 = 15625Hz
  // OCR2A = 78: 15625/78 ≈ 200.3Hz ≈ 4.99ms
  initTimer2();

  // --- 初始化完成 ---
  Serial.println(F("[初始化] 系统就绪"));
  Serial.println(F("[初始化] 默认状态: 停止"));
  Serial.println(F("[初始化] 控制模式: 遥控器模式"));
  Serial.println(F("----------------------------------------"));
  Serial.println(F("串口命令:"));
  Serial.println(F("  w/W - 前进    s/S - 后退"));
  Serial.println(F("  a/A - 左转    d/D - 右转"));
  Serial.println(F("  空格 - 停止   r/R - 运行演示"));
  Serial.println(F("  p/P - 切换到树莓派自动模式"));
  Serial.println(F("  m/M - 切换控制模式"));
  Serial.println(F("  +/- - 加速/减速"));
  Serial.println(F("----------------------------------------"));
  Serial.println(F("控制模式:"));
  Serial.println(F("  0=演示  1=串口  2=遥控器  3=树莓派自动"));
  Serial.println(F("----------------------------------------"));

  // 短暂延时确保系统稳定
  delay(100);
}

// ============================================================
// Timer2 初始化 (5ms控制周期)
// ============================================================

void initTimer2() {
  cli();  // 关闭全局中断

  TCCR2A = 0;  // 普通模式
  TCCR2B = 0;  // 停止定时器

  TCNT2  = 0;  // 计数器清零

  // CTC模式, 预分频1024
  // WGM21=1 (CTC), CS22=1 CS21=1 CS20=1 (1024预分频)
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);

  OCR2A  = 77;  // 比较值 (0-77 = 78个计数)
                // 15625Hz / 78 = 200.32Hz ≈ 4.99ms

  TIMSK2 = (1 << OCIE2A);  // 使能输出比较A中断

  sei();  // 开启全局中断
}

// ============================================================
// 编码器中断服务程序
// ============================================================

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

// ============================================================
// Timer2 中断服务程序 (5ms控制周期)
// 对应原项目 HAL_TIM_PeriodElapsedCallback
// ============================================================

ISR(TIMER2_COMPA_vect) {
  // --- 读取编码器值并清零 ---
  // 对应原项目 Read_Encoder()
  cli();  // 短暂关中断保护原子操作
  encoderLeft  = encoderLeftCount;
  encoderRight = encoderRightCount;
  encoderLeftCount  = 0;
  encoderRightCount = 0;
  sei();

  // --- 运动学分析 ---
  // 对应原项目 Kinematic_Analysis()
  // Diff_Car模式: Target_A = velocity + turn, Target_B = velocity - turn
  targetSpeedA = (int)(velocity + turn);
  targetSpeedB = (int)(velocity - turn);

  // --- 速度PI闭环控制 ---
  // 对应原项目 Incremental_PI_A / Incremental_PI_B
  if (!flagStop) {
    if (directPwmControl) {
      setMotorPwm(directPwmLeft, directPwmRight);
      return;
    }

    motorPwmA = incrementalPI_A(encoderLeft,  targetSpeedA);
    motorPwmB = incrementalPI_B(encoderRight, targetSpeedB);

    // PWM限幅
    // 对应原项目 Xianfu_Pwm()
    motorPwmA = constrain(motorPwmA, -PWM_LIMIT, PWM_LIMIT);
    motorPwmB = constrain(motorPwmB, -PWM_LIMIT, PWM_LIMIT);

    // 设置电机PWM
    // 对应原项目 Set_Pwm()
    setMotorPwm(motorPwmA, motorPwmB);
  } else {
    // 停止状态: 清零PI积分并停止电机
    resetPIController();
    setMotorPwm(0, 0);
  }
}

// ============================================================
// 增量式PI控制器 (MG513XP28_12V优化)
// 对应原项目 Incremental_PI_A / Incremental_PI_B
// 公式: Pwm += Kp * (Bias - LastBias) + Ki * Bias
// ============================================================

int incrementalPI_A(int encoder, int target) {
  piBiasA = encoder - target;
  
  // 增量式PI计算
  piPwmA += (int)(VELOCITY_KP * (piBiasA - piLastBiasA) + VELOCITY_KI * piBiasA);
  
  // 输出限幅 (防止积分饱和)
  if (piPwmA > PID_OUTPUT_MAX) piPwmA = PID_OUTPUT_MAX;
  if (piPwmA < PID_OUTPUT_MIN) piPwmA = PID_OUTPUT_MIN;
  
  piLastBiasA = piBiasA;

  return piPwmA;
}

int incrementalPI_B(int encoder, int target) {
  piBiasB = encoder - target;
  
  // 增量式PI计算
  piPwmB += (int)(VELOCITY_KP * (piBiasB - piLastBiasB) + VELOCITY_KI * piBiasB);
  
  // 输出限幅 (防止积分饱和)
  if (piPwmB > PID_OUTPUT_MAX) piPwmB = PID_OUTPUT_MAX;
  if (piPwmB < PID_OUTPUT_MIN) piPwmB = PID_OUTPUT_MIN;
  
  piLastBiasB = piBiasB;

  return piPwmB;
}

// ============================================================
// 重置PI控制器 (停止时清零积分)
// ============================================================

void resetPIController() {
  piBiasA = 0;
  piLastBiasA = 0;
  piPwmA = 0;
  piBiasB = 0;
  piLastBiasB = 0;
  piPwmB = 0;
  motorPwmA = 0;
  motorPwmB = 0;
}

// ============================================================
// 设置电机PWM和方向
// 对应原项目 Set_Pwm()
// pwmA > 0: 左电机(A口)正转, pwmA < 0: 左电机反转
// pwmB > 0: 右电机(B口)正转, pwmB < 0: 右电机反转
// ============================================================

void setMotorPwm(int pwmA, int pwmB) {
  // --- 电机A (左侧, 模块A口) ---
  if (pwmA > 0) {
    // 正转 (前进方向)
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
  } else if (pwmA < 0) {
    // 反转 (后退方向)
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else {
    // 停止 (短制动)
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  }
  analogWrite(MOTOR_A_PWM, abs(pwmA));

  // --- 电机B (右侧, 模块B口) ---
  if (pwmB > 0) {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
  } else if (pwmB < 0) {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
  }
  analogWrite(MOTOR_B_PWM, abs(pwmB));
}

// ============================================================
// 运动控制函数
// ============================================================

int scalePercentToPwm(int percent, int maxPwm) {
  return (int)((long)percent * maxPwm / 100L);
}

void setDirectPwmDrive(int leftPwm, int rightPwm) {
  directPwmLeft = constrain(leftPwm, -REMOTE_PWM_MAX, REMOTE_PWM_MAX);
  directPwmRight = constrain(rightPwm, -REMOTE_PWM_MAX, REMOTE_PWM_MAX);
  motorPwmA = directPwmLeft;
  motorPwmB = directPwmRight;
  directPwmControl = true;
  velocity = 0;
  turn = 0;
  flagStop = false;
}

void applyRemoteDrive(int velPercent, int turnPercent) {
  int drivePercent = (abs(velPercent) <= REMOTE_INPUT_DEADZONE) ? 0 : velPercent;
  int steerPercent = (abs(turnPercent) <= REMOTE_INPUT_DEADZONE) ? 0 : turnPercent;

  steerPercent = -steerPercent;

  if (drivePercent == 0 && steerPercent == 0) {
    moveStop();
    return;
  }

  int drivePwm = scalePercentToPwm(drivePercent, REMOTE_PWM_MAX);
  int steerPwm = scalePercentToPwm(steerPercent, REMOTE_TURN_PWM_MAX);

  int leftPwm = drivePwm + steerPwm;
  int rightPwm = drivePwm - steerPwm;

  if (drivePercent == 0 && steerPercent != 0) {
    leftPwm = steerPwm;
    rightPwm = -steerPwm;
  }

  int maxMagnitude = max(abs(leftPwm), abs(rightPwm));
  if (maxMagnitude > REMOTE_PWM_MAX) {
    leftPwm = (int)((long)leftPwm * REMOTE_PWM_MAX / maxMagnitude);
    rightPwm = (int)((long)rightPwm * REMOTE_PWM_MAX / maxMagnitude);
  }

  setDirectPwmDrive(leftPwm, rightPwm);

  static unsigned long lastRemotePrint = 0;
  unsigned long now = millis();
  if (now - lastRemotePrint > 200) {
    Serial.print(F("[遥控混控] V="));
    Serial.print(velPercent);
    Serial.print(F(" T="));
    Serial.print(turnPercent);
    Serial.print(F(" -> L="));
    Serial.print(directPwmLeft);
    Serial.print(F(" R="));
    Serial.println(directPwmRight);
    lastRemotePrint = now;
  }
}

// 前进
void moveForward(int speedVal) {
  directPwmControl = false;
  velocity = speedVal;
  turn     = 0;
  flagStop = false;
  Serial.print(F("[运动] 前进  速度="));
  Serial.println(speedVal);
}

// 后退
void moveBackward(int speedVal) {
  directPwmControl = false;
  velocity = -speedVal;
  turn     = 0;
  flagStop = false;
  Serial.print(F("[运动] 后退  速度="));
  Serial.println(speedVal);
}

// 左转 (原地左转: 左轮后退, 右轮前进)
void turnLeft(int speedVal) {
  directPwmControl = false;
  velocity = 0;
  turn     = -speedVal;
  flagStop = false;
  Serial.print(F("[运动] 左转  差速="));
  Serial.println(speedVal);
}

// 右转 (原地右转: 左轮前进, 右轮后退)
void turnRight(int speedVal) {
  directPwmControl = false;
  velocity = 0;
  turn     = speedVal;
  flagStop = false;
  Serial.print(F("[运动] 右转  差速="));
  Serial.println(speedVal);
}

// 左前转 (前进+左转)
void moveForwardLeft(int speedVal, int turnVal) {
  directPwmControl = false;
  velocity = speedVal;
  turn     = -turnVal;
  flagStop = false;
  Serial.print(F("[运动] 左前转  速度="));
  Serial.print(speedVal);
  Serial.print(F(" 差速="));
  Serial.println(turnVal);
}

// 右前转 (前进+右转)
void moveForwardRight(int speedVal, int turnVal) {
  directPwmControl = false;
  velocity = speedVal;
  turn     = turnVal;
  flagStop = false;
  Serial.print(F("[运动] 右前转  速度="));
  Serial.print(speedVal);
  Serial.print(F(" 差速="));
  Serial.println(turnVal);
}

// 停止
void moveStop() {
  directPwmControl = false;
  directPwmLeft = 0;
  directPwmRight = 0;
  velocity = 0;
  turn     = 0;
  flagStop = true;
  Serial.println(F("[运动] 停止"));
}

// ============================================================
// 电池电压检测 (模拟输入)
// 对应原项目 Turn_Off() 中的电压检测逻辑
// ============================================================

float readBatteryVoltage() {
  // 使用电阻分压: 12V -> 约4.5V (适合Arduino 5V ADC)
  // 分压比: R1=10kΩ, R2=4.7kΩ → Vout = Vin * 4.7/(10+4.7) ≈ Vin * 0.32
  // 12V * 0.32 = 3.84V (在ADC范围内)
  // 如需使用, 请将分压输出连接到A0引脚

  int adcValue = analogRead(A0);
  // Arduino ADC: 10位, 0-1023对应0-5V
  float measuredVoltage = adcValue * (5.0f / 1023.0f);
  // 根据分压比反算实际电池电压
  float actualVoltage = measuredVoltage / 0.32f;

  return actualVoltage;
}

// 电池低压保护检查
// 对应原项目 Turn_Off()
bool checkBatteryLow() {
  batteryVoltage = readBatteryVoltage();

  if (batteryVoltage < BATTERY_LOW) {
    Serial.print(F("[警告] 电池电压过低: "));
    Serial.print(batteryVoltage);
    Serial.println(F("V, 电机已停止"));
    return true;
  }
  return false;
}

// ============================================================
// 演示序列
// 对应原项目 Get_RC() 中的蓝牙控制逻辑
// ============================================================

void runDemoSequence() {
  static int demoStep = 0;
  static unsigned long demoLastTime = 0;
  unsigned long currentTime = millis();

  // 每步持续2秒
  if (currentTime - demoLastTime < 2000) return;
  demoLastTime = currentTime;

  // 检查电池电压
  if (checkBatteryLow()) {
    moveStop();
    return;
  }

  switch (demoStep) {
    case 0:
      Serial.println(F("\n=== 演示: 前进 ==="));
      moveForward(DEFAULT_SPEED);
      break;
    case 1:
      Serial.println(F("\n=== 演示: 停止 ==="));
      moveStop();
      break;
    case 2:
      Serial.println(F("\n=== 演示: 后退 ==="));
      moveBackward(DEFAULT_SPEED);
      break;
    case 3:
      Serial.println(F("\n=== 演示: 停止 ==="));
      moveStop();
      break;
    case 4:
      Serial.println(F("\n=== 演示: 原地左转 ==="));
      turnLeft(DEFAULT_TURN);
      break;
    case 5:
      Serial.println(F("\n=== 演示: 停止 ==="));
      moveStop();
      break;
    case 6:
      Serial.println(F("\n=== 演示: 原地右转 ==="));
      turnRight(DEFAULT_TURN);
      break;
    case 7:
      Serial.println(F("\n=== 演示: 停止 ==="));
      moveStop();
      break;
    case 8:
      Serial.println(F("\n=== 演示: 左前转 ==="));
      moveForwardLeft(DEFAULT_SPEED, DEFAULT_TURN);
      break;
    case 9:
      Serial.println(F("\n=== 演示: 停止 ==="));
      moveStop();
      break;
    case 10:
      Serial.println(F("\n=== 演示: 右前转 ==="));
      moveForwardRight(DEFAULT_SPEED, DEFAULT_TURN);
      break;
    case 11:
      Serial.println(F("\n=== 演示: 停止 ==="));
      moveStop();
      break;
    case 12:
      Serial.println(F("\n=== 演示序列完成, 重新开始 ==="));
      demoStep = -1;  // 循环
      break;
  }
  demoStep++;
}

// ============================================================
// 串口命令处理
// ============================================================

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
        parseRpiCommand(rpiBuf);
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
        rpiAutoActive = false;
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
          case CTRL_MODE_DEMO:     Serial.println(F("演示模式")); break;
          case CTRL_MODE_SERIAL:   Serial.println(F("串口命令模式")); break;
          case CTRL_MODE_REMOTE:   Serial.println(F("遥控器模式")); break;
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

// ============================================================
// 遥控器命令处理 (来自ESP32-C3的SoftwareSerial)
// ============================================================
// 协议格式: "$V<vel>,T<turn>,F<flags>*\n"
// 示例: "$V50,T0,F0*\n"
//   vel:   -100~100 (速度百分比, 左摇杆Y轴)
//   turn:  -100~100 (转向百分比, 右摇杆X轴)
//   flags: bit0=当前模式 (0=手动, 1=自动), bit1=模式切换请求

void handleRemoteCommand() {
  while (remoteSerial.available()) {
    char c = remoteSerial.read();

    if (c == '\n' || c == '\r') {
      if (remoteBufIdx > 0) {
        remoteBuf[remoteBufIdx] = '\0';
        parseRemoteCommand(remoteBuf);
        remoteBufIdx = 0;
      }
      continue;
    }

    if (remoteBufIdx < 31) {
      remoteBuf[remoteBufIdx++] = c;
    } else {
      remoteBufIdx = 0;
    }
  }
}

void parseRemoteCommand(const char *cmd) {
  if (cmd[0] != '$' || cmd[1] != 'V') return;

  const char *p = cmd;

  int velInput = 0;
  int turnInput = 0;
  int flags = 0;

  p += 2;
  velInput = atoi(p);

  p = strstr(p, ",T");
  if (!p) return;
  p += 2;
  turnInput = atoi(p);

  p = strstr(p, ",F");
  if (!p) return;
  p += 2;
  flags = atoi(p);

  lastRemoteCmdTime = millis();
  remoteConnected = true;

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

  if (ctrlMode != CTRL_MODE_REMOTE) {
    return;
  }

  demoMode = false;
  applyRemoteDrive(velInput, turnInput);
}

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
    } else if (strstr(modePtr, "\"remote\"")) {
      ctrlMode = CTRL_MODE_REMOTE;
      rpiAutoActive = false;
      demoMode = false;
      Serial.println(F("[RPI] 树莓派请求: 切换到遥控器模式"));
    }
    return;
  }

  if (strstr(cmdPtr, "\"estop\"")) {
    moveStop();
    ctrlMode = CTRL_MODE_REMOTE;
    rpiAutoActive = false;
    Serial.println(F("[RPI] 紧急停止! 切换到遥控器模式"));
    lastRpiCmdTime = millis();
    return;
  }

  if (strstr(cmdPtr, "\"stop\"")) {
    moveStop();
    lastRpiCmdTime = millis();
    return;
  }

  if (!strstr(cmdPtr, "\"move\"")) return;

  if (ctrlMode != CTRL_MODE_RPI_AUTO) return;

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

void checkRpiTimeout() {
  if (!rpiAutoActive) return;
  if (millis() - lastRpiCmdTime > RPI_CMD_TIMEOUT) {
    moveStop();
    Serial.println(F("[RPI] 树莓派指令超时! 自动停车"));
  }
}

// ============================================================
// 遥控器超时检测
// ============================================================

void checkRemoteTimeout() {
  if (!remoteConnected) return;

  if (millis() - lastRemoteCmdTime > REMOTE_TIMEOUT) {
    remoteConnected = false;
    Serial.println(F("[遥控] 遥控器超时! 自动停车"));
    moveStop();
  }
}

// ============================================================
// 向ESP32-C3回传编码器数据
// ============================================================

void sendEncoderFeedback() {
  remoteSerial.print(F("#E"));
  remoteSerial.print(constrain(encoderLeft, -127, 127));
  remoteSerial.print(F(","));
  remoteSerial.print(constrain(encoderRight, -127, 127));
  remoteSerial.print(F(","));
  remoteSerial.print(motorPwmA);
  remoteSerial.print(F(","));
  remoteSerial.print(motorPwmB);
  remoteSerial.println(F(","));
}

// ============================================================
// 状态打印
// ============================================================

void printStatus() {
  Serial.println(F("----------------------------------------"));
  Serial.print(F("系统状态: "));
  Serial.println(flagStop ? F("停止") : F("运行中"));
  Serial.print(F("控制模式: "));
  switch (ctrlMode) {
    case CTRL_MODE_DEMO:     Serial.println(F("演示")); break;
    case CTRL_MODE_SERIAL:   Serial.println(F("串口")); break;
    case CTRL_MODE_REMOTE:   Serial.println(F("遥控器")); break;
    case CTRL_MODE_RPI_AUTO: Serial.println(F("树莓派自动")); break;
  }
  Serial.print(F("遥控器: "));
  Serial.print(remoteConnected ? F("在线") : F("离线"));
  Serial.println();
  Serial.print(F("目标速度A: "));
  Serial.print(targetSpeedA);
  Serial.print(F("  目标速度B: "));
  Serial.println(targetSpeedB);
  Serial.print(F("编码器A: "));
  Serial.print(encoderLeft);
  Serial.print(F("  编码器B: "));
  Serial.println(encoderRight);
  Serial.print(F("PWM输出A: "));
  Serial.print(motorPwmA);
  Serial.print(F("  PWM输出B: "));
  Serial.println(motorPwmB);
  Serial.print(F("电池电压: "));
  Serial.print(batteryVoltage);
  Serial.println(F("V"));
  Serial.println(F("----------------------------------------"));
}

// ============================================================
// 主循环
// ============================================================

void loop() {
  // 处理USB串口命令
  handleSerialCommand();

  // 处理遥控器命令 (来自ESP32-C3)
  handleRemoteCommand();

  // 遥控器超时检测
  if (remoteConnected) {
    checkRemoteTimeout();
  }

  // 树莓派指令超时检测
  if (rpiAutoActive) {
    checkRpiTimeout();
  }

  // 演示模式
  if (ctrlMode == CTRL_MODE_DEMO && demoMode) {
    runDemoSequence();
  }

  // 向ESP32-C3回传编码器数据 (每200ms)
  static unsigned long lastFeedbackTime = 0;
  if (millis() - lastFeedbackTime >= 200) {
    lastFeedbackTime = millis();
    sendEncoderFeedback();
  }

  // 定期打印状态 (每2秒)
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 2000) {
    lastPrintTime = millis();
    if (!flagStop) {
      Serial.print(F("[状态] 编码器L="));
      Serial.print(encoderLeft);
      Serial.print(F(" R="));
      Serial.print(encoderRight);
      Serial.print(F(" PWM_A="));
      Serial.print(motorPwmA);
      Serial.print(F(" PWM_B="));
      Serial.println(motorPwmB);
    }
  }
}
