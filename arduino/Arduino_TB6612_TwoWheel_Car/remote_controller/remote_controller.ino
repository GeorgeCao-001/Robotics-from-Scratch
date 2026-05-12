/*
 * ============================================================
 * ESP32-S3 遥控器固件 - 二驱小车 + 二轴云台控制系统
 * ============================================================
 *
 * 硬件平台: ESP32-S3 (WROOM-1)
 * 通信协议: ESP-NOW (2.4GHz, 低延迟点对点)
 * 接收端:   ESP32-C3 (小车端接收器)
 *
 * 输入设备:
 *   左摇杆 (KY-023): 小车速度/方向控制
 *   右摇杆 (KY-023): 二轴云台 Pan/Tilt 控制 (预留)
 *   按键组: 急停/模式/加速/减速
 *
 * 供电: 3.7V 锂电池 (18650/锂聚合物) + 升压模块
 *
 * ESP-NOW 数据包格式 (9字节):
 *   [0] Header (0xAA)
 *   [1] Type (0x01=控制, 0x03=心跳)
 *   [2] Velocity (int8, -100~100)
 *   [3] Turn (int8, -100~100)
 *   [4] PTZ Pan (int8, -100~100)
 *   [5] PTZ Tilt (int8, -100~100)
 *   [6] Flags (bit0=急停, bit1=模式, bit2=云台使能)
 *   [7] Speed Level (0~10)
 *   [8] Checksum (XOR of [0]~[7])
 */

#include <esp_now.h>
#include <WiFi.h>

// ============================================================
// 引脚定义 - ESP32-S3 遥控器
// ============================================================

// 左摇杆 (小车控制)
#define JOY_L_VRX    1     // GPIO1  (ADC1_CH0) - 速度 (前后)
#define JOY_L_VRY    2     // GPIO2  (ADC1_CH1) - 方向 (左右)
#define JOY_L_SW     6     // GPIO6  - 按下

// 右摇杆 (云台控制)
#define JOY_R_VRX    3     // GPIO3  (ADC1_CH2) - Pan (水平)
#define JOY_R_VRY    4     // GPIO4  (ADC1_CH3) - Tilt (垂直)
#define JOY_R_SW     7     // GPIO7  - 按下

// 按键
#define BTN_ESTOP    8     // GPIO8  - 急停
#define BTN_MODE     9     // GPIO9  - 模式切换
#define BTN_SPEED_UP 10    // GPIO10 - 加速
#define BTN_SPEED_DN 21    // GPIO21 - 减速

// 输出
#define LED_STATUS   47    // GPIO47 - 状态指示LED
#define BUZZER_PIN   48    // GPIO48 - 蜂鸣器 (可选)

// 电池检测
#define BATT_ADC     5     // GPIO5  (ADC1_CH4) - 电池电压

// ============================================================
// ESP-NOW 配置
// ============================================================

// 小车端 ESP32-C3 的 MAC 地址 (需要修改为实际地址)
// 查看方法: 在ESP32-C3上运行 Serial.println(WiFi.macAddress());
uint8_t CAR_MAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 通信通道 (必须与接收端一致)
#define ESP_NOW_CHANNEL 1

// ============================================================
// 系统参数
// ============================================================

#define SEND_INTERVAL_MS     50      // 发送间隔 (ms), 20Hz
#define HEARTBEAT_INTERVAL   500     // 心跳间隔 (ms)
#define JOYSTICK_DEADZONE    8       // 摇杆死区 (0~100范围)
#define ADC_SAMPLES          4       // ADC采样次数 (滤波)
#define ADC_RANGE            4095    // ESP32 ADC 12位
#define SPEED_LEVEL_MAX      10      // 最大速度档位
#define SPEED_LEVEL_DEFAULT  5       // 默认速度档位
#define BATT_LOW_THRESHOLD   3.3f    // 低压报警阈值 (V)
#define BATT_FULL_VOLTAGE    4.2f    // 满电电压 (V)
#define BATT_DIVIDER_RATIO   2.0f    // 分压比 (R1=R2时为2.0)

// ============================================================
// 数据包定义
// ============================================================

#define PKT_HEADER     0xAA
#define PKT_CTRL       0x01
#define PKT_HEARTBEAT  0x03

#define FLAG_ESTOP     0x01
#define FLAG_MODE      0x02
#define FLAG_PTZ_EN    0x04

// ============================================================
// 全局变量
// ============================================================

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

ControlPacket txPacket;
ControlPacket rxPacket;

bool espNowReady    = false;
bool carOnline      = false;
bool estopActive    = false;
uint8_t speedLevel  = SPEED_LEVEL_DEFAULT;
uint8_t modeFlag    = 0;
bool ptzEnabled     = false;

unsigned long lastSendTime     = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastCarResponse  = 0;

int joyLVx = 0, joyLVy = 0;
int joyRVx = 0, joyRVy = 0;

float batteryVoltage = 4.0f;

// 按键去抖
typedef struct {
    uint8_t pin;
    bool    lastState;
    bool    currentState;
    unsigned long lastDebounceTime;
} Button;

Button btnEstop   = {BTN_ESTOP,    HIGH, HIGH, 0};
Button btnMode    = {BTN_MODE,     HIGH, HIGH, 0};
Button btnSpeedUp = {BTN_SPEED_UP, HIGH, HIGH, 0};
Button btnSpeedDn = {BTN_SPEED_DN, HIGH, HIGH, 0};

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
    joyLVx = mapJoystick(readAdcAvg(JOY_L_VRX), JOYSTICK_DEADZONE);
    joyLVy = mapJoystick(readAdcAvg(JOY_L_VRY), JOYSTICK_DEADZONE);
    joyRVx = mapJoystick(readAdcAvg(JOY_R_VRX), JOYSTICK_DEADZONE);
    joyRVy = mapJoystick(readAdcAvg(JOY_R_VRY), JOYSTICK_DEADZONE);
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
    if (updateButton(&btnEstop)) {
        estopActive = !estopActive;
        if (estopActive) {
            Serial.println("[按键] 急停激活!");
        } else {
            Serial.println("[按键] 急停解除");
        }
    }

    if (updateButton(&btnMode)) {
        modeFlag = (modeFlag + 1) % 2;
        Serial.print("[按键] 模式切换: ");
        Serial.println(modeFlag == 0 ? "小车控制" : "云台控制");
    }

    if (updateButton(&btnSpeedUp)) {
        if (speedLevel < SPEED_LEVEL_MAX) {
            speedLevel++;
            Serial.print("[按键] 加速 → 档位 ");
            Serial.println(speedLevel);
        }
    }

    if (updateButton(&btnSpeedDn)) {
        if (speedLevel > 0) {
            speedLevel--;
            Serial.print("[按键] 减速 → 档位 ");
            Serial.println(speedLevel);
        }
    }

    if (digitalRead(JOY_L_SW) == LOW) {
        ptzEnabled = !ptzEnabled;
        delay(300);
        Serial.print("[摇杆] 云台 ");
        Serial.println(ptzEnabled ? "启用" : "禁用");
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

    if (data[0] == 0xBB && data[1] == 0x01 && len >= 7) {
        lastCarResponse = millis();
        carOnline = true;

        uint8_t carBattRaw = data[2];
        float carBatt = carBattRaw / 10.0f;
        int8_t encL = (int8_t)data[3];
        int8_t encR = (int8_t)data[4];
        uint8_t statusFlags = data[5];

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
    float scaleFactor = (float)speedLevel / SPEED_LEVEL_MAX;

    int8_t vel = (int8_t)(joyLVy * scaleFactor);
    int8_t trn = (int8_t)(joyLVx * scaleFactor);
    int8_t pan = ptzEnabled ? (int8_t)joyRVx : 0;
    int8_t tilt = ptzEnabled ? (int8_t)joyRVy : 0;

    if (estopActive) {
        vel = 0;
        trn = 0;
        pan = 0;
        tilt = 0;
    }

    txPacket.header     = PKT_HEADER;
    txPacket.type       = PKT_CTRL;
    txPacket.velocity   = vel;
    txPacket.turn       = trn;
    txPacket.ptzPan     = pan;
    txPacket.ptzTilt    = tilt;
    txPacket.flags      = 0;
    if (estopActive)  txPacket.flags |= FLAG_ESTOP;
    if (modeFlag)     txPacket.flags |= FLAG_MODE;
    if (ptzEnabled)   txPacket.flags |= FLAG_PTZ_EN;
    txPacket.speedLevel = speedLevel;
    txPacket.checksum  = calcChecksum((uint8_t *)&txPacket, 8);
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
    hb.header     = PKT_HEADER;
    hb.type       = PKT_HEARTBEAT;
    hb.speedLevel = speedLevel;
    hb.flags      = estopActive ? FLAG_ESTOP : 0;
    hb.checksum   = calcChecksum((uint8_t *)&hb, 8);

    esp_now_send(CAR_MAC, (uint8_t *)&hb, sizeof(hb));
}

// ============================================================
// 状态 LED 控制
// ============================================================

void updateStatusLed() {
    static unsigned long lastToggle = 0;
    static bool ledState = false;

    if (estopActive) {
        if (millis() - lastToggle > 200) {
            ledState = !ledState;
            digitalWrite(LED_STATUS, ledState);
            lastToggle = millis();
        }
    } else if (!carOnline) {
        if (millis() - lastToggle > 500) {
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
    Serial.printf("  急停状态: %s\n", estopActive ? "激活" : "正常");
    Serial.printf("  速度档位: %d / %d\n", speedLevel, SPEED_LEVEL_MAX);
    Serial.printf("  控制模式: %s\n", modeFlag == 0 ? "小车" : "云台");
    Serial.printf("  云台使能: %s\n", ptzEnabled ? "是" : "否");
    Serial.printf("  左摇杆: X=%d Y=%d\n", joyLVx, joyLVy);
    Serial.printf("  右摇杆: X=%d Y=%d\n", joyRVx, joyRVy);
    Serial.printf("  电池: %.2fV\n", batteryVoltage);
    Serial.printf("  发送: Vel=%d Trn=%d Pan=%d Tilt=%d\n",
                  txPacket.velocity, txPacket.turn,
                  txPacket.ptzPan, txPacket.ptzTilt);
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
    Serial.println(F("  控制: 双摇杆 + 按键"));
    Serial.println(F("========================================"));

    // 摇杆引脚 (ADC, 无需pinMode)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // 按键引脚
    pinMode(JOY_L_SW,    INPUT_PULLUP);
    pinMode(JOY_R_SW,    INPUT_PULLUP);
    pinMode(BTN_ESTOP,   INPUT_PULLUP);
    pinMode(BTN_MODE,    INPUT_PULLUP);
    pinMode(BTN_SPEED_UP, INPUT_PULLUP);
    pinMode(BTN_SPEED_DN, INPUT_PULLUP);

    // 输出引脚
    pinMode(LED_STATUS, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LED_STATUS, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    // ESP-NOW 初始化
    espNowReady = initEspNow();

    if (!espNowReady) {
        Serial.println("[错误] ESP-NOW 初始化失败, 请检查WiFi配置");
        while (1) {
            digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
            delay(100);
        }
    }

    // 启动提示音
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);

    Serial.println("[初始化] 系统就绪");
    Serial.println("----------------------------------------");
    Serial.println("操作说明:");
    Serial.println("  左摇杆: 控制小车前进/后退/左转/右转");
    Serial.println("  右摇杆: 控制云台水平/垂直 (需先启用)");
    Serial.println("  左摇杆按下: 启用/禁用云台");
    Serial.println("  急停键: 紧急停止/解除");
    Serial.println("  模式键: 切换小车/云台模式");
    Serial.println("  加速/减速键: 调整速度档位");
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

    // 读取摇杆
    readJoysticks();

    // 处理按键
    handleButtons();

    // 检查电池
    static unsigned long lastBattCheck = 0;
    if (now - lastBattCheck > 5000) {
        checkBattery();
        lastBattCheck = now;
    }

    // 检查小车在线状态
    if (carOnline && (now - lastCarResponse > 2000)) {
        carOnline = false;
        Serial.println("[状态] 小车离线");
    }

    // 定时发送控制数据
    if (now - lastSendTime >= SEND_INTERVAL_MS) {
        sendControlPacket();
        lastSendTime = now;
    }

    // 定时发送心跳
    if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeatTime = now;
    }

    // 更新状态LED
    updateStatusLed();

    // 定时打印状态
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 3000) {
        printStatus();
        lastPrint = now;
    }
}
