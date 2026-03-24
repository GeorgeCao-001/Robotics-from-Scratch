#include <Arduino.h>
#include <Servo.h>

// ========== 1. 引脚定义 ==========
// 左侧电机引脚
const int ENA = 5; // 左轮调速 (必须是PWM引脚，带~号的)
const int IN1 = 2; // 左轮前进
const int IN2 = 3; // 左轮后退

// 右侧电机引脚
const int ENB = 6; // 右轮调速 (必须是PWM引脚)
const int IN3 = 4; // 右轮前进
const int IN4 = 7; // 右轮后退

// 云台舵机引脚
const int SERVO_PAN_PIN = 9;  // 左右转头
const int SERVO_TILT_PIN = 10; // 上下点头

// 实例化舵机对象
Servo panServo;
Servo tiltServo;

// 当前角度记录（用于诊断输出）
int currentPan = 90;
int currentTilt = 90;

// ========== 2. 初始化设置 ==========
void setup() {
  // 开启串口通信，波特率设置为 9600（这是我们检错的核心）
  Serial.begin(9600);
  Serial.println("\n--- 系统启动：小车与云台自检程序已加载 ---");

  // 设置电机引脚为输出模式
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // 绑定舵机引脚，并初始化到 90 度（正前方）
  panServo.attach(SERVO_PAN_PIN);
  tiltServo.attach(SERVO_TILT_PIN);
  panServo.write(currentPan);
  tiltServo.write(currentTilt);
  
  // 设置默认速度 (0-255)
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);

  Serial.println("状态：舵机已归位到90度，电机已准备就绪。");
  Serial.println("等待指令...\n");
  Serial.println("【控制说明】:");
  Serial.println(" 移动: w(前) | s(后) | a(左) | d(右) | space(停)");
  Serial.println(" 云台(连续): j/l(左右) | i/k(上下) | r(复位)");
  Serial.println(" 云台(绝对): pXXX(例 p90) | tXXX(例 t45)");
}

// ========== 3. 电机控制动作函数 ==========
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  Serial.println("[诊断] 动作: 前进 -> IN1:1, IN2:0 | IN3:1, IN4:0");
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  Serial.println("[诊断] 动作: 后退 -> IN1:0, IN2:1 | IN3:0, IN4:1");
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // 左轮后退
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // 右轮前进
  Serial.println("[诊断] 动作: 左转 -> IN1:0, IN2:1 | IN3:1, IN4:0");
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // 左轮前进
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // 右轮后退
  Serial.println("[诊断] 动作: 右转 -> IN1:1, IN2:0 | IN3:0, IN4:1");
}

void stopCar() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  Serial.println("[诊断] 动作: 停止 -> 所有 IN 引脚拉低");
}

// ========== 4. 主循环：监听和执行 ==========
void loop() {
  // 如果收到电脑发来的数据
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // 读取一行指令直到回车
    input.trim(); // 去除多余的空格或回车符

    if (input.length() == 0) return; // 忽略空消息

    char command = input.charAt(0); // 获取第一个字母作为主命令

    // 检错日志：打印收到了什么原始数据
    Serial.print(">> 收到原始指令: "); 
    Serial.println(input);

    switch (command) {
      // --- 底盘控制 ---
      case 'w': moveForward(); break;
      case 's': moveBackward(); break;
      case 'a': turnLeft(); break;
      case 'd': turnRight(); break;
      case ' ': stopCar(); break; // 空格键停止

      // --- 新增：对接 Python 脚本的增量云台控制 ---
      case 'j': // 左转
        currentPan = constrain(currentPan + 5, 0, 180);
        panServo.write(currentPan);
        Serial.print("[诊断] 云台 Pan (左移) -> "); Serial.println(currentPan);
        break;
      case 'l': // 右转
        currentPan = constrain(currentPan - 5, 0, 180);
        panServo.write(currentPan);
        Serial.print("[诊断] 云台 Pan (右移) -> "); Serial.println(currentPan);
        break;
      case 'i': // 抬头
        currentTilt = constrain(currentTilt + 5, 0, 180);
        tiltServo.write(currentTilt);
        Serial.print("[诊断] 云台 Tilt (上移) -> "); Serial.println(currentTilt);
        break;
      case 'k': // 低头
        currentTilt = constrain(currentTilt - 5, 0, 180);
        tiltServo.write(currentTilt);
        Serial.print("[诊断] 云台 Tilt (下移) -> "); Serial.println(currentTilt);
        break;
      case 'r': // 一键复位
        currentPan = 90; currentTilt = 90;
        panServo.write(currentPan); tiltServo.write(currentTilt);
        Serial.println("[诊断] 云台一键复位至 90 度");
        break;

      // --- 原版保留：绝对角度控制 (例如发 p120) ---
      case 'p': 
      case 'P':
        currentPan = input.substring(1).toInt(); // 截取 'p' 后面的数字
        currentPan = constrain(currentPan, 0, 180); // 限制在 0-180 度防止卡死舵机
        panServo.write(currentPan);
        Serial.print("[诊断] 云台 Pan (绝对) 设为: "); 
        Serial.print(currentPan); Serial.println(" 度");
        break;

      case 't': 
      case 'T':
        currentTilt = input.substring(1).toInt();
        currentTilt = constrain(currentTilt, 0, 180);
        tiltServo.write(currentTilt);
        Serial.print("[诊断] 云台 Tilt (绝对) 设为: "); 
        Serial.print(currentTilt); Serial.println(" 度");
        break;

      default:
        Serial.println("[错误] 未知指令，请检查输入格式！");
        break;
    }
    Serial.println("-----------------------------------");
  }
}