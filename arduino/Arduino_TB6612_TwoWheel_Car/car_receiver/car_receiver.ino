/*
 * ============================================================
 * ESP32-C3 小车端接收器固件
 * ============================================================
 *
 * 硬件平台: ESP32-C3 (SuperMini / DevKitM)
 * 通信协议: ESP-NOW (接收) + UART (转发至Arduino UNO)
 *
 * 功能:
 *   1. 接收ESP32-S3遥控器的ESP-NOW控制数据
 *   2. 解析控制数据并通过UART转发给Arduino UNO
 *   3. 驱动二轴云台舵机 (SG90/MG90S)
 *   4. 回传小车状态 (电池电压/编码器速度) 给遥控器
 *   5. 心跳监测 - 遥控器断连时自动停车
 *
 * 连接关系:
 *   ESP32-C3 UART1 TX (GPIO5) → Arduino UNO D5 (SoftwareSerial RX)
 *   ESP32-C3 UART1 RX (GPIO6) ← Arduino UNO D11 (SoftwareSerial TX)
 *   ESP32-C3 GPIO7 → 云台Pan舵机
 *   ESP32-C3 GPIO8 → 云台Tilt舵机
 *   ESP32-C3 GPIO2 → 状态LED (内置)
 *   ESP32-C3 GPIO0 → 电池电压ADC (ADC1_CH0)
 *
 * UART协议 (ESP32-C3 → Arduino UNO):
 *   格式: "$V<vel>,T<turn>,F<flags>,S<speed>*\n"
 *   示例: "$V50,T0,F0,S5*\n"
 *   vel:   -100~100 (速度)
 *   turn:  -100~100 (转向)
 *   flags: 0=正常, 1=急停
 *   speed: 0~10 (速度档位)
 */

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// ============================================================
// 引脚定义 - ESP32-C3 小车端接收器
// ============================================================

#define UART_TX_PIN    5     // GPIO5  - UART1 TX → Arduino D5
#define UART_RX_PIN    6     // GPIO6  - UART1 RX ← Arduino D11
#define PTZ_PAN_PIN    7     // GPIO7  - 云台Pan舵机 (水平)
#define PTZ_TILT_PIN   8     // GPIO8  - 云台Tilt舵机 (垂直)
#define LED_PIN        2     // GPIO2  - 状态LED (内置)
#define BATT_ADC_PIN   0     // GPIO0  - 电池电压 (ADC1_CH0)

// ============================================================
// ESP-NOW 配置
// ============================================================

// 遥控器 ESP32-S3 的 MAC 地址 (需要修改为实际地址)
uint8_t REMOTE_MAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define ESP_NOW_CHANNEL 1

// ============================================================
// 系统参数
// ============================================================

#define HEARTBEAT_TIMEOUT   1000    // 心跳超时 (ms), 超过则停车
#define STATUS_SEND_INTERVAL 200    // 状态回传间隔 (ms)
#define UART_BAUD           9600    // UART波特率 (与Arduino端一致)
#define SERVO_CENTER        90      // 舵机中位角度
#define SERVO_RANGE         45      // 舵机偏转范围 (±45°)
#define BATT_DIVIDER_RATIO  0.32f   // 分压比 (与Arduino端一致)
#define ADC_RANGE           4095    // ESP32-C3 ADC 12位

// ============================================================
// 数据包定义 (与遥控器端一致)
// ============================================================

#define PKT_HEADER     0xAA
#define PKT_CTRL       0x01
#define PKT_HEARTBEAT  0x03

#define FLAG_ESTOP     0x01
#define FLAG_MODE      0x02
#define FLAG_PTZ_EN    0x04

typedef struct {
    uint8_t header;
    uint8_t type;
    int8_t  velocity;
    int8_t  turn;
    int8_t  ptzPan;
    int8_t  ptzTilt;
    uint8_t flags;
    uint8_t speedLevel;
    uint8_t checksum;
} __attribute__((packed)) ControlPacket;

typedef struct {
    uint8_t header;       // 0xBB
    uint8_t type;         // 0x01 = 状态
    uint8_t batteryVx10;  // 电池电压×10 (124 = 12.4V)
    int8_t  encSpeedL;    // 左编码器速度
    int8_t  encSpeedR;    // 右编码器速度
    uint8_t statusFlags;  // 状态标志
    uint8_t checksum;
} __attribute__((packed)) StatusPacket;

// ============================================================
// 全局变量
// ============================================================

bool espNowReady = false;
bool remoteOnline = false;
unsigned long lastRemoteTime = 0;
unsigned long lastStatusSend = 0;

ControlPacket lastCmd = {};
bool hasNewCmd = false;

Servo panServo;
Servo tiltServo;
bool servoAttached = false;
int panAngle  = SERVO_CENTER;
int tiltAngle = SERVO_CENTER;

float batteryVoltage = 12.0f;

// Arduino UNO 回传的编码器数据
int8_t arduinoEncL = 0;
int8_t arduinoEncR = 0;

// ============================================================
// 校验和计算
// ============================================================

uint8_t calcChecksum(const uint8_t *data, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

// ============================================================
// ESP-NOW 接收回调
// ============================================================

void onRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    if (len != sizeof(ControlPacket)) return;

    ControlPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.header != PKT_HEADER) return;

    uint8_t cs = calcChecksum((uint8_t *)&pkt, 8);
    if (cs != pkt.checksum) {
        Serial.println("[ESP-NOW] 校验失败");
        return;
    }

    lastRemoteTime = millis();
    remoteOnline = true;

    if (pkt.type == PKT_CTRL) {
        memcpy(&lastCmd, &pkt, sizeof(pkt));
        hasNewCmd = true;
    }
}

// ============================================================
// ESP-NOW 发送回调
// ============================================================

void onSend(const uint8_t *macAddr, esp_now_send_status_t status) {
}

// ============================================================
// ESP-NOW 初始化
// ============================================================

bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setChannel(ESP_NOW_CHANNEL);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] 初始化失败!");
        return false;
    }

    esp_now_register_recv_cb(onRecv);
    esp_now_register_send_cb(onSend);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, REMOTE_MAC, 6);
    peerInfo.channel = ESP_NOW_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] 添加对端失败!");
        return false;
    }

    Serial.println("[ESP-NOW] 初始化成功");
    Serial.print("[ESP-NOW] 本机MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.printf("[ESP-NOW] 目标MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  REMOTE_MAC[0], REMOTE_MAC[1], REMOTE_MAC[2],
                  REMOTE_MAC[3], REMOTE_MAC[4], REMOTE_MAC[5]);

    return true;
}

// ============================================================
// UART 初始化
// ============================================================

void initUart() {
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.println("[UART1] 初始化完成");
    Serial.printf("[UART1] TX=GPIO%d, RX=GPIO%d, Baud=%d\n",
                  UART_TX_PIN, UART_RX_PIN, UART_BAUD);
}

// ============================================================
// 向 Arduino UNO 发送控制命令
// ============================================================

void sendToArduino(int8_t vel, int8_t turn, uint8_t flags, uint8_t speed) {
    Serial1.printf("$V%d,T%d,F%d,S%d*\n", vel, turn, flags, speed);
}

// ============================================================
// 从 Arduino UNO 读取状态数据
// ============================================================

void readFromArduino() {
    while (Serial1.available()) {
        String line = Serial1.readStringUntil('\n');
        line.trim();

        if (line.startsWith("#E")) {
            int commaL = line.indexOf(',', 2);
            int commaR = line.indexOf(',', commaL + 1);
            if (commaL > 0 && commaR > 0) {
                arduinoEncL = (int8_t)line.substring(2, commaL).toInt();
                arduinoEncR = (int8_t)line.substring(commaL + 1, commaR).toInt();
            }
        }
    }
}

// ============================================================
// 云台舵机控制
// ============================================================

void initServos() {
    panServo.attach(PTZ_PAN_PIN, 500, 2500);
    tiltServo.attach(PTZ_TILT_PIN, 500, 2500);
    panServo.write(SERVO_CENTER);
    tiltServo.write(SERVO_CENTER);
    servoAttached = true;
    Serial.println("[舵机] 云台初始化完成");
}

void updateServos(int8_t pan, int8_t tilt) {
    if (!servoAttached) return;

    int targetPan  = SERVO_CENTER + (int)map(pan,  -100, 100, -SERVO_RANGE, SERVO_RANGE);
    int targetTilt = SERVO_CENTER + (int)map(tilt, -100, 100, -SERVO_RANGE, SERVO_RANGE);

    targetPan  = constrain(targetPan,  SERVO_CENTER - SERVO_RANGE, SERVO_CENTER + SERVO_RANGE);
    targetTilt = constrain(targetTilt, SERVO_CENTER - SERVO_RANGE, SERVO_CENTER + SERVO_RANGE);

    if (abs(targetPan - panAngle) > 1) {
        panAngle += (targetPan - panAngle) / 4;
        panServo.write(panAngle);
    }

    if (abs(targetTilt - tiltAngle) > 1) {
        tiltAngle += (targetTilt - tiltAngle) / 4;
        tiltServo.write(tiltAngle);
    }
}

// ============================================================
// 电池电压检测
// ============================================================

void checkBattery() {
    int raw = analogRead(BATT_ADC_PIN);
    float adcVoltage = raw * (3.3f / ADC_RANGE);
    batteryVoltage = adcVoltage / BATT_DIVIDER_RATIO;
}

// ============================================================
// 向遥控器回传状态
// ============================================================

void sendStatusToRemote() {
    if (!espNowReady) return;

    StatusPacket pkt = {};
    pkt.header      = 0xBB;
    pkt.type        = 0x01;
    pkt.batteryVx10 = (uint8_t)constrain((int)(batteryVoltage * 10), 0, 255);
    pkt.encSpeedL   = arduinoEncL;
    pkt.encSpeedR   = arduinoEncR;
    pkt.statusFlags = remoteOnline ? 0x01 : 0x00;
    pkt.checksum    = calcChecksum((uint8_t *)&pkt, 6);

    esp_now_send(REMOTE_MAC, (uint8_t *)&pkt, sizeof(pkt));
}

// ============================================================
// 状态LED控制
// ============================================================

void updateStatusLed() {
    static unsigned long lastToggle = 0;
    static bool ledState = false;

    if (!remoteOnline) {
        if (millis() - lastToggle > 300) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
            lastToggle = millis();
        }
    } else if (lastCmd.flags & FLAG_ESTOP) {
        if (millis() - lastToggle > 150) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
            lastToggle = millis();
        }
    } else {
        digitalWrite(LED_PIN, HIGH);
    }
}

// ============================================================
// 初始化
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("========================================"));
    Serial.println(F("  ESP32-C3 小车端接收器"));
    Serial.println(F("  通信: ESP-NOW + UART"));
    Serial.println(F("========================================"));

    // ADC配置
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // UART初始化
    initUart();

    // ESP-NOW初始化
    espNowReady = initEspNow();

    if (!espNowReady) {
        Serial.println("[错误] ESP-NOW 初始化失败!");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    // 云台舵机初始化
    initServos();

    Serial.println("[初始化] 系统就绪");
    Serial.printf("[重要] 请修改 REMOTE_MAC 为遥控器ESP32-S3的MAC地址!\n");
    Serial.printf("  本机MAC: %s\n", WiFi.macAddress().c_str());

    // 启动指示
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }

    delay(200);
}

// ============================================================
// 主循环
// ============================================================

void loop() {
    unsigned long now = millis();

    // 检查遥控器在线状态
    if (remoteOnline && (now - lastRemoteTime > HEARTBEAT_TIMEOUT)) {
        remoteOnline = false;
        Serial.println("[状态] 遥控器离线! 自动停车");
        sendToArduino(0, 0, FLAG_ESTOP, 0);
    }

    // 处理新的控制命令
    if (hasNewCmd) {
        hasNewCmd = false;

        int8_t vel  = lastCmd.velocity;
        int8_t turn = lastCmd.turn;
        uint8_t flags = lastCmd.flags;
        uint8_t speed = lastCmd.speedLevel;

        sendToArduino(vel, turn, flags, speed);

        if (lastCmd.flags & FLAG_PTZ_EN) {
            updateServos(lastCmd.ptzPan, lastCmd.ptzTilt);
        }

        static unsigned long lastCmdPrint = 0;
        if (now - lastCmdPrint > 500) {
            Serial.printf("[CMD] V=%d T=%d Pan=%d Tilt=%d F=0x%02X S=%d\n",
                          vel, turn, lastCmd.ptzPan, lastCmd.ptzTilt,
                          flags, speed);
            lastCmdPrint = now;
        }
    }

    // 从Arduino读取状态
    readFromArduino();

    // 定时检查电池
    static unsigned long lastBattCheck = 0;
    if (now - lastBattCheck > 2000) {
        checkBattery();
        lastBattCheck = now;
    }

    // 定时回传状态给遥控器
    if (now - lastStatusSend >= STATUS_SEND_INTERVAL) {
        sendStatusToRemote();
        lastStatusSend = now;
    }

    // 更新状态LED
    updateStatusLed();
}
