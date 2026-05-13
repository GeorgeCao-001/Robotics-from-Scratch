/*
 * ============================================================
 * ESP32-S3 遥控器固件 - 二驱小车控制系统
 * ============================================================
 *
 * 硬件平台: ESP32-S3 (WROOM-1)
 * 通信协议: ESP-NOW (2.4GHz, 低延迟点对点)
 * 接收端:   ESP32-C3 (小车端接收器)
 *
 * 输入设备:
 *   左摇杆 (KY-023): Y轴=速度控制 (前进/后退)
 *   右摇杆 (KY-023): X轴=方向控制 (左转/右转)
 *   模式按键: 手动控制/自动运行 切换
 *
 * 供电: 3.7V 锂电池 (18650/锂聚合物) + 升压模块
 *
 * ESP-NOW 数据包格式 (6字节):
 *   [0] Header  (0xAA)
 *   [1] Type    (0x01=控制, 0x03=心跳)
 *   [2] Velocity (int8, -100~100, 左摇杆Y轴)
 *   [3] Turn    (int8, -100~100, 右摇杆X轴)
 *   [4] Flags   (bit0=模式: 0=手动, 1=自动)
 *   [5] Checksum (XOR of [0]~[4])
 */

#include <esp_now.h>
#include <WiFi.h>

// ============================================================
// 引脚定义 - ESP32-S3 遥控器
// ============================================================

// 左摇杆 (速度控制)
#define JOY_L_VRY    2     // GPIO2  (ADC1_CH1) - 速度 (前后)

// 右摇杆 (方向控制)
#define JOY_R_VRX    3     // GPIO3  (ADC1_CH2) - 方向 (左右)

// 模式按键
#define BTN_MODE     8     // GPIO8  - 手动/自动切换

// 输出
#define LED_STATUS   47    // GPIO47 - 状态指示LED

// 电池检测
#define BATT_ADC     5     // GPIO5  (ADC1_CH4) - 电池电压

// ============================================================
// ESP-NOW 配置
// ============================================================

uint8_t CAR_MAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define ESP_NOW_CHANNEL 1

// ============================================================
// 系统参数
// ============================================================

#define SEND_INTERVAL_MS     50
#define HEARTBEAT_INTERVAL   500
#define JOYSTICK_DEADZONE    8
#define ADC_SAMPLES          4
#define ADC_RANGE            4095
#define BATT_LOW_THRESHOLD   3.3f
#define BATT_FULL_VOLTAGE    4.2f
#define BATT_DIVIDER_RATIO   2.0f

// ============================================================
// 数据包定义
// ============================================================

#define PKT_HEADER     0xAA
#define PKT_CTRL       0x01
#define PKT_HEARTBEAT  0x03

#define FLAG_AUTO_MODE 0x01

typedef struct {
    uint8_t header;
    uint8_t type;
    int8_t  velocity;
    int8_t  turn;
    uint8_t flags;
    uint8_t checksum;
} __attribute__((packed)) ControlPacket;

typedef struct {
    uint8_t header;
    uint8_t type;
    uint8_t batteryVx10;
    int8_t  encSpeedL;
    int8_t  encSpeedR;
    uint8_t statusFlags;
    uint8_t checksum;
} __attribute__((packed)) StatusPacket;

// ============================================================
// 全局变量
// ============================================================

ControlPacket txPacket;

bool espNowReady    = false;
bool carOnline      = false;
bool autoMode       = false;

unsigned long lastSendTime      = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastCarResponse   = 0;

int joyLVy = 0;
int joyRVx = 0;

float batteryVoltage = 4.0f;

typedef struct {
    uint8_t pin;
    bool    lastState;
    bool    currentState;
    unsigned long lastDebounceTime;
} Button;

Button btnMode = {BTN_MODE, HIGH, HIGH, 0};

#define DEBOUNCE_DELAY 50

// ============================================================
// ADC 读取 (带均值滤波)
// ============================================================

int readAdcAvg(int pin) {
    long sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += analogRead(pin);
    }
    return (int)(sum / ADC_SAMPLES);
}

// ============================================================
// 摇杆读取与映射
// ============================================================

int mapJoystick(int rawValue, int deadzone) {
    int centered = rawValue - 2048;
    int mapped = map(centered, -2048, 2047, -100, 100);

    if (abs(mapped) < deadzone) {
        mapped = 0;
    } else if (mapped > 0) {
        mapped = map(mapped, deadzone, 100, 0, 100);
    } else {
        mapped = map(mapped, -deadzone, -100, 0, -100);
    }

    return constrain(mapped, -100, 100);
}

void readJoysticks() {
    joyLVy = mapJoystick(readAdcAvg(JOY_L_VRY), JOYSTICK_DEADZONE);
    joyRVx = mapJoystick(readAdcAvg(JOY_R_VRX), JOYSTICK_DEADZONE);
}

// ============================================================
// 按键处理 (去抖)
// ============================================================

bool updateButton(Button *btn) {
    bool reading = digitalRead(btn->pin);

    if (reading != btn->lastState) {
        btn->lastDebounceTime = millis();
    }

    if ((millis() - btn->lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != btn->currentState) {
            btn->currentState = reading;
            if (btn->currentState == LOW) {
                btn->lastState = reading;
                return true;
            }
        }
    }

    btn->lastState = reading;
    return false;
}

void handleButtons() {
    if (updateButton(&btnMode)) {
        autoMode = !autoMode;
        Serial.print("[按键] 模式切换: ");
        Serial.println(autoMode ? "自动运行" : "手动控制");
    }
}

// ============================================================
// 电池电压检测
// ============================================================

void checkBattery() {
    int raw = readAdcAvg(BATT_ADC);
    float adcVoltage = raw * (3.3f / ADC_RANGE);
    batteryVoltage = adcVoltage * BATT_DIVIDER_RATIO;

    if (batteryVoltage < BATT_LOW_THRESHOLD) {
        static unsigned long lastWarn = 0;
        if (millis() - lastWarn > 3000) {
            Serial.print("[警告] 遥控器电池低压: ");
            Serial.print(batteryVoltage);
            Serial.println("V");
            lastWarn = millis();
        }
    }
}

// ============================================================
// ESP-NOW 回调
// ============================================================

void onSend(const uint8_t *macAddr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[ESP-NOW] 发送失败");
    }
}

void onRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    if (len < 2) return;

    if (data[0] == 0xBB && data[1] == 0x01 && len >= sizeof(StatusPacket)) {
        lastCarResponse = millis();
        carOnline = true;

        StatusPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));

        uint8_t cs = calcChecksum((uint8_t *)&pkt, sizeof(pkt) - 1);
        if (cs != pkt.checksum) return;

        float carBatt = pkt.batteryVx10 / 10.0f;

        if (carBatt < 10.0f) {
            static unsigned long lastCarWarn = 0;
            if (millis() - lastCarWarn > 5000) {
                Serial.print("[警告] 小车电池低压: ");
                Serial.print(carBatt);
                Serial.println("V");
                lastCarWarn = millis();
            }
        }
    }
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

    esp_now_register_send_cb(onSend);
    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, CAR_MAC, 6);
    peerInfo.channel = ESP_NOW_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] 添加对端失败!");
        return false;
    }

    Serial.println("[ESP-NOW] 初始化成功");
    Serial.print("[ESP-NOW] 本机MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.print("[ESP-NOW] 目标MAC: ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                  CAR_MAC[0], CAR_MAC[1], CAR_MAC[2],
                  CAR_MAC[3], CAR_MAC[4], CAR_MAC[5]);

    return true;
}

// ============================================================
// 数据包构建与发送
// ============================================================

uint8_t calcChecksum(const uint8_t *data, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

void buildControlPacket() {
    int8_t vel = (int8_t)joyLVy;
    int8_t trn = (int8_t)joyRVx;

    if (autoMode) {
        vel = 0;
        trn = 0;
    }

    txPacket.header   = PKT_HEADER;
    txPacket.type     = PKT_CTRL;
    txPacket.velocity = vel;
    txPacket.turn     = trn;
    txPacket.flags    = 0;
    if (autoMode) txPacket.flags |= FLAG_AUTO_MODE;
    txPacket.checksum = calcChecksum((uint8_t *)&txPacket, sizeof(txPacket) - 1);
}

void sendControlPacket() {
    if (!espNowReady) return;

    buildControlPacket();

    esp_err_t result = esp_now_send(CAR_MAC, (uint8_t *)&txPacket, sizeof(txPacket));

    if (result != ESP_OK) {
        Serial.println("[ESP-NOW] 发送错误");
    }
}

void sendHeartbeat() {
    if (!espNowReady) return;

    ControlPacket hb = {};
    hb.header   = PKT_HEADER;
    hb.type     = PKT_HEARTBEAT;
    hb.flags    = autoMode ? FLAG_AUTO_MODE : 0;
    hb.checksum = calcChecksum((uint8_t *)&hb, sizeof(hb) - 1);

    esp_now_send(CAR_MAC, (uint8_t *)&hb, sizeof(hb));
}

// ============================================================
// 状态 LED 控制
// ============================================================

void updateStatusLed() {
    static unsigned long lastToggle = 0;
    static bool ledState = false;

    if (!carOnline) {
        if (millis() - lastToggle > 500) {
            ledState = !ledState;
            digitalWrite(LED_STATUS, ledState);
            lastToggle = millis();
        }
    } else if (autoMode) {
        if (millis() - lastToggle > 800) {
            ledState = !ledState;
            digitalWrite(LED_STATUS, ledState);
            lastToggle = millis();
        }
    } else if (batteryVoltage < BATT_LOW_THRESHOLD) {
        if (millis() - lastToggle > 1000) {
            ledState = !ledState;
            digitalWrite(LED_STATUS, ledState);
            lastToggle = millis();
        }
    } else {
        digitalWrite(LED_STATUS, HIGH);
    }
}

// ============================================================
// 串口状态打印
// ============================================================

void printStatus() {
    Serial.println(F("========== 遥控器状态 =========="));
    Serial.printf("  小车在线: %s\n", carOnline ? "是" : "否");
    Serial.printf("  控制模式: %s\n", autoMode ? "自动运行" : "手动控制");
    Serial.printf("  左摇杆Y(速度): %d\n", joyLVy);
    Serial.printf("  右摇杆X(方向): %d\n", joyRVx);
    Serial.printf("  电池: %.2fV\n", batteryVoltage);
    Serial.printf("  发送: Vel=%d Trn=%d Flags=0x%02X\n",
                  txPacket.velocity, txPacket.turn, txPacket.flags);
    Serial.println(F("================================"));
}

// ============================================================
// 初始化
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("========================================"));
    Serial.println(F("  ESP32-S3 遥控器"));
    Serial.println(F("  通信: ESP-NOW"));
    Serial.println(F("  左摇杆: 速度控制"));
    Serial.println(F("  右摇杆: 方向控制"));
    Serial.println(F("  按键: 手动/自动切换"));
    Serial.println(F("========================================"));

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    pinMode(BTN_MODE, INPUT_PULLUP);

    pinMode(LED_STATUS, OUTPUT);
    digitalWrite(LED_STATUS, LOW);

    espNowReady = initEspNow();

    if (!espNowReady) {
        Serial.println("[错误] ESP-NOW 初始化失败, 请检查WiFi配置");
        while (1) {
            digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
            delay(100);
        }
    }

    Serial.println("[初始化] 系统就绪");
    Serial.println("----------------------------------------");
    Serial.println("操作说明:");
    Serial.println("  左摇杆Y轴: 控制速度 (前推=前进, 后拉=后退)");
    Serial.println("  右摇杆X轴: 控制方向 (左推=左转, 右推=右转)");
    Serial.println("  模式按键: 切换手动控制/自动运行");
    Serial.println("----------------------------------------");
    Serial.println();
    Serial.print("[重要] 请修改 CAR_MAC 为小车端ESP32-C3的MAC地址!");
    Serial.printf("  本机MAC: %s\n", WiFi.macAddress().c_str());

    delay(200);
}

// ============================================================
// 主循环
// ============================================================

void loop() {
    unsigned long now = millis();

    readJoysticks();

    handleButtons();

    static unsigned long lastBattCheck = 0;
    if (now - lastBattCheck > 5000) {
        checkBattery();
        lastBattCheck = now;
    }

    if (carOnline && (now - lastCarResponse > 2000)) {
        carOnline = false;
        Serial.println("[状态] 小车离线");
    }

    if (now - lastSendTime >= SEND_INTERVAL_MS) {
        sendControlPacket();
        lastSendTime = now;
    }

    if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeatTime = now;
    }

    updateStatusLed();

    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 3000) {
        printStatus();
        lastPrint = now;
    }
}
