#include <Arduino.h>
#include <Servo.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint8_t PIN_PAN = 9;
constexpr uint8_t PIN_TILT = 10;

constexpr unsigned long SERIAL_BAUD = 9600;
constexpr unsigned long SERVO_UPDATE_MS = 20;
constexpr unsigned long STATUS_UPDATE_MS = 1000;

constexpr float PAN_MIN = -180.0f;
constexpr float PAN_MAX = 180.0f;
constexpr float TILT_MIN = 0.0f;
constexpr float TILT_MAX = 180.0f;

constexpr float PAN_CENTER = 0.0f;
constexpr float TILT_CENTER = 90.0f;
constexpr float MOVE_STEP_DEG = 2.0f;
constexpr float TARGET_REACHED_EPSILON_DEG = 1.0f;

constexpr int PAN_SERVO_STOP_DEG = 90;
constexpr int PAN_SPEED_MAX_OFFSET = 45;
constexpr float PAN_DEADBAND = 0.03f;
constexpr unsigned long PAN_CMD_TIMEOUT_MS = 400;

constexpr float SELFTEST_PAN_MIN = -90.0f;
constexpr float SELFTEST_PAN_MAX = 90.0f;
constexpr float SELFTEST_TILT_MIN = 5.0f;
constexpr float SELFTEST_TILT_MAX = 85.0f;
constexpr unsigned long SELFTEST_HOLD_MS = 350;
constexpr unsigned long SELFTEST_PAN_SPIN_MS = 600;
constexpr float SELFTEST_PAN_SPIN_STRENGTH = 0.5f;
constexpr unsigned long SELFTEST_STATE_TIMEOUT_MS = 4000;

constexpr size_t SERIAL_BUF_SIZE = 128;

Servo panServo;
Servo tiltServo;

char serialBuf[SERIAL_BUF_SIZE];
size_t serialLen = 0;

float panTotalOffset = PAN_CENTER;
float targetPanAbs = PAN_CENTER;
float targetTiltAbs = TILT_CENTER;
float currentPanAbs = PAN_CENTER;
float currentTiltAbs = TILT_CENTER;
float panStrengthCmd = 0.0f;

int panServoWritten = -1;
int tiltServoWritten = -1;

const __FlashStringHelper *lastError = nullptr;

unsigned long lastServoUpdateMs = 0;
unsigned long lastStatusUpdateMs = 0;
unsigned long lastPanCmdMs = 0;

enum class SelfTestState : uint8_t {
  Center,
  PanMin,
  PanMax,
  TiltMin,
  TiltMax,
  ReturnCenter,
  Done,
};

SelfTestState selfTestState = SelfTestState::Center;
unsigned long selfTestStateEnterMs = 0;

float clampf(float value, float minV, float maxV) {
  if (value < minV)
    return minV;
  if (value > maxV)
    return maxV;
  return value;
}

float absf(float value) { return (value < 0.0f) ? -value : value; }

float stepToward(float current, float target, float step) {
  const float delta = target - current;
  if (absf(delta) <= step)
    return target;
  return current + ((delta > 0.0f) ? step : -step);
}

void print2Digits(unsigned long value) {
  if (value < 10) {
    Serial.print('0');
  }
  Serial.print(value);
}

void print3Digits(unsigned long value) {
  if (value < 100) {
    Serial.print('0');
  }
  if (value < 10) {
    Serial.print('0');
  }
  Serial.print(value);
}

void printLogPrefix(const char *direction) {
  const unsigned long now = millis();
  const unsigned long totalSeconds = now / 1000;
  const unsigned long ms = now % 1000;
  const unsigned long sec = totalSeconds % 60;
  const unsigned long min = (totalSeconds / 60) % 60;
  const unsigned long hour = (totalSeconds / 3600) % 24;

  Serial.print('[');
  print2Digits(hour);
  Serial.print(':');
  print2Digits(min);
  Serial.print(':');
  print2Digits(sec);
  Serial.print('.');
  print3Digits(ms);
  Serial.print(F("] ["));
  Serial.print(direction);
  Serial.print(F("] "));
}

void sendError(const __FlashStringHelper *errorCode) {
  lastError = errorCode;
  printLogPrefix("TX");
  Serial.print(F("{\"status\":\"error\",\"error\":\""));
  Serial.print(errorCode);
  Serial.println(F("\"}"));
}

void sendStatus() {
  printLogPrefix("TX");
  Serial.print(F("{\"status\":\"ok\",\"pan_abs\":"));
  Serial.print(currentPanAbs, 1);
  Serial.print(F(",\"tilt_abs\":"));
  Serial.print(currentTiltAbs, 1);
  Serial.print(F(",\"target_pan_abs\":"));
  Serial.print(targetPanAbs, 1);
  Serial.print(F(",\"target_tilt_abs\":"));
  Serial.print(targetTiltAbs, 1);
  Serial.print(F(",\"error\":"));
  if (lastError == nullptr) {
    Serial.print(F("null"));
  } else {
    Serial.print('"');
    Serial.print(lastError);
    Serial.print('"');
  }
  Serial.println('}');
}

bool extractCmd(const char *json, char *outCmd, size_t outSize) {
  const char *cmdKey = strstr(json, "\"cmd\"");
  if (cmdKey == nullptr)
    return false;

  const char *colon = strchr(cmdKey, ':');
  if (colon == nullptr)
    return false;

  const char *quoteStart = strchr(colon, '"');
  if (quoteStart == nullptr)
    return false;
  ++quoteStart;

  const char *quoteEnd = strchr(quoteStart, '"');
  if (quoteEnd == nullptr || quoteEnd <= quoteStart)
    return false;

  const size_t len = static_cast<size_t>(quoteEnd - quoteStart);
  if (len + 1 > outSize)
    return false;

  memcpy(outCmd, quoteStart, len);
  outCmd[len] = '\0';
  return true;
}

const char *findKey(const char *json, const char *key) {
  const size_t keyLen = strlen(key);
  const char *cursor = json;

  while (true) {
    const char *keyPos = strstr(cursor, key);
    if (keyPos == nullptr) {
      return nullptr;
    }

    const char prev = (keyPos == json) ? '\0' : *(keyPos - 1);
    const char next = *(keyPos + keyLen);
    const bool prevValid = (prev == '{' || prev == ',' ||
                            isspace(static_cast<unsigned char>(prev)));
    const bool nextValid = (next == ' ' || next == '\t' || next == '\r' ||
                            next == '\n' || next == ':');

    if (prevValid && nextValid) {
      return keyPos;
    }

    cursor = keyPos + 1;
  }
}

bool extractNumber(const char *json, const char *key, float &outValue) {
  const char *keyPos = findKey(json, key);
  if (keyPos == nullptr)
    return false;

  const char *colon = strchr(keyPos, ':');
  if (colon == nullptr)
    return false;

  char *endPtr = nullptr;
  const double parsed = strtod(colon + 1, &endPtr);
  if (endPtr == colon + 1)
    return false;
  if (!isfinite(parsed))
    return false;

  while (*endPtr == ' ' || *endPtr == '\t' || *endPtr == '\r' ||
         *endPtr == '\n') {
    ++endPtr;
  }

  if (*endPtr != ',' && *endPtr != '}')
    return false;

  outValue = static_cast<float>(parsed);
  return true;
}

void trimInPlace(char *line) {
  if (line == nullptr) {
    return;
  }

  size_t start = 0;
  const size_t len = strlen(line);
  while (start < len && isspace(static_cast<unsigned char>(line[start]))) {
    ++start;
  }

  size_t end = len;
  while (end > start && isspace(static_cast<unsigned char>(line[end - 1]))) {
    --end;
  }

  if (start > 0) {
    memmove(line, line + start, end - start);
  }
  line[end - start] = '\0';
}

bool isLikelyJsonObject(const char *line) {
  if (line == nullptr) {
    return false;
  }

  const size_t len = strlen(line);
  if (len < 2) {
    return false;
  }

  if (line[0] != '{' || line[len - 1] != '}') {
    return false;
  }

  bool inString = false;
  bool escaped = false;
  int braceDepth = 0;

  for (size_t i = 0; i < len; ++i) {
    const char c = line[i];

    if (inString) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        inString = false;
      }
      continue;
    }

    if (c == '"') {
      inString = true;
      continue;
    }

    if (c == '{') {
      ++braceDepth;
    } else if (c == '}') {
      --braceDepth;
      if (braceDepth < 0) {
        return false;
      }
    }
  }

  return (!inString && !escaped && braceDepth == 0);
}

bool targetsReached() {
  return absf(currentPanAbs - targetPanAbs) <= TARGET_REACHED_EPSILON_DEG &&
         absf(currentTiltAbs - targetTiltAbs) <= TARGET_REACHED_EPSILON_DEG;
}

const __FlashStringHelper *selfTestStateName(SelfTestState state) {
  switch (state) {
  case SelfTestState::Center:
    return F("center");
  case SelfTestState::PanMin:
    return F("pan_min");
  case SelfTestState::PanMax:
    return F("pan_max");
  case SelfTestState::TiltMin:
    return F("tilt_min");
  case SelfTestState::TiltMax:
    return F("tilt_max");
  case SelfTestState::ReturnCenter:
    return F("return_center");
  case SelfTestState::Done:
    return F("done");
  }
  return F("unknown");
}

void setTarget(float panAbs, float tiltAbs) {
  targetPanAbs = clampf(panAbs, PAN_MIN, PAN_MAX);
  targetTiltAbs = clampf(tiltAbs, TILT_MIN, TILT_MAX);
}

void moveSelfTestTo(SelfTestState nextState, float panAbs, float tiltAbs) {
  selfTestState = nextState;
  selfTestStateEnterMs = millis();
  setTarget(panAbs, tiltAbs);

  if (nextState == SelfTestState::PanMin) {
    panStrengthCmd = -SELFTEST_PAN_SPIN_STRENGTH;
    lastPanCmdMs = millis();
  } else if (nextState == SelfTestState::PanMax) {
    panStrengthCmd = SELFTEST_PAN_SPIN_STRENGTH;
    lastPanCmdMs = millis();
  } else {
    panStrengthCmd = 0.0f;
  }

  printLogPrefix("TX");
  Serial.print(F("{\"status\":\"ok\",\"self_test\":\""));
  Serial.print(selfTestStateName(nextState));
  Serial.println(F("\"}"));
}

void handleSelfTest() {
  if (selfTestState == SelfTestState::Done) {
    return;
  }

  const unsigned long elapsed = millis() - selfTestStateEnterMs;
  const bool timedOut = (elapsed >= SELFTEST_STATE_TIMEOUT_MS);

  if (!timedOut) {
    if (selfTestState == SelfTestState::PanMin ||
        selfTestState == SelfTestState::PanMax) {
      if (elapsed < SELFTEST_PAN_SPIN_MS) {
        return;
      }
    } else {
      if (!targetsReached()) {
        return;
      }
      if (elapsed < SELFTEST_HOLD_MS) {
        return;
      }
    }
  } else {
    lastError = F("SELF_TEST_TIMEOUT");
    printLogPrefix("TX");
    Serial.print(F(
        "{\"status\":\"error\",\"error\":\"SELF_TEST_TIMEOUT\",\"state\":\""));
    Serial.print(selfTestStateName(selfTestState));
    Serial.println(F("\"}"));
  }

  switch (selfTestState) {
  case SelfTestState::Center:
    moveSelfTestTo(SelfTestState::PanMin, SELFTEST_PAN_MIN, TILT_CENTER);
    break;
  case SelfTestState::PanMin:
    moveSelfTestTo(SelfTestState::PanMax, SELFTEST_PAN_MAX, TILT_CENTER);
    break;
  case SelfTestState::PanMax:
    moveSelfTestTo(SelfTestState::TiltMin, PAN_CENTER, SELFTEST_TILT_MIN);
    break;
  case SelfTestState::TiltMin:
    moveSelfTestTo(SelfTestState::TiltMax, PAN_CENTER, SELFTEST_TILT_MAX);
    break;
  case SelfTestState::TiltMax:
    moveSelfTestTo(SelfTestState::ReturnCenter, PAN_CENTER, TILT_CENTER);
    break;
  case SelfTestState::ReturnCenter:
    moveSelfTestTo(SelfTestState::Done, PAN_CENTER, TILT_CENTER);
    break;
  case SelfTestState::Done:
    break;
  }
}

void updateServos() {
  if (millis() - lastServoUpdateMs < SERVO_UPDATE_MS)
    return;
  lastServoUpdateMs = millis();

  currentTiltAbs = stepToward(currentTiltAbs, targetTiltAbs, MOVE_STEP_DEG);

  float desiredPanStrength = panStrengthCmd;
  if (selfTestState == SelfTestState::Done &&
      millis() - lastPanCmdMs >= PAN_CMD_TIMEOUT_MS) {
    desiredPanStrength = 0.0f;
  }

  desiredPanStrength = clampf(desiredPanStrength, -1.0f, 1.0f);
  if (absf(desiredPanStrength) < PAN_DEADBAND) {
    desiredPanStrength = 0.0f;
  }

  currentPanAbs = desiredPanStrength * 180.0f;
  targetPanAbs = currentPanAbs;

  const int panServoDeg = static_cast<int>(
      clampf(PAN_SERVO_STOP_DEG + desiredPanStrength * PAN_SPEED_MAX_OFFSET,
             0.0f, 180.0f) +
      0.5f);
  const int tiltServoDeg =
      static_cast<int>(clampf(currentTiltAbs, 0.0f, 180.0f) + 0.5f);

  if (panServoDeg != panServoWritten) {
    panServo.write(panServoDeg);
    panServoWritten = panServoDeg;
  }
  if (tiltServoDeg != tiltServoWritten) {
    tiltServo.write(tiltServoDeg);
    tiltServoWritten = tiltServoDeg;
  }
}

void processGimbalCommand(const char *line) {
  float panDelta = 0.0f;
  float tiltDelta = 0.0f;

  if (!extractNumber(line, "\"pan\"", panDelta) ||
      !extractNumber(line, "\"tilt\"", tiltDelta)) {
    sendError(F("INVALID_GIMBAL_FIELDS"));
    return;
  }

  if (panDelta < -180.0f || panDelta > 180.0f) {
    sendError(F("PAN_DELTA_OUT_OF_RANGE"));
    return;
  }

  if (selfTestState != SelfTestState::Done) {
    selfTestState = SelfTestState::Done;
    panStrengthCmd = 0.0f;
    targetPanAbs = 0.0f;
    targetTiltAbs = currentTiltAbs;
    printLogPrefix("TX");
    Serial.println(
        F("{\"status\":\"ok\",\"self_test\":\"aborted_by_command\"}"));
  }

  lastError = nullptr;

  panStrengthCmd = clampf(panDelta / 180.0f, -1.0f, 1.0f);
  lastPanCmdMs = millis();
  panTotalOffset = panDelta;
  targetPanAbs = panDelta;
  targetTiltAbs = clampf(targetTiltAbs + tiltDelta, TILT_MIN, TILT_MAX);

  printLogPrefix("TX");
  Serial.print(F("{\"status\":\"ok\",\"ack\":\"gimbal\",\"target_pan_abs\":"));
  Serial.print(targetPanAbs, 1);
  Serial.print(F(",\"target_tilt_abs\":"));
  Serial.print(targetTiltAbs, 1);
  Serial.println('}');
}

void processLine(char *line) {
  trimInPlace(line);
  if (line[0] == '\0') {
    return;
  }

  printLogPrefix("RX");
  Serial.println(line);

  if (!isLikelyJsonObject(line)) {
    sendError(F("INVALID_JSON"));
    return;
  }

  char cmd[16] = {0};
  if (!extractCmd(line, cmd, sizeof(cmd))) {
    sendError(F("MISSING_CMD"));
    return;
  }

  if (strcmp(cmd, "gimbal") == 0) {
    processGimbalCommand(line);
    return;
  }

  if (strcmp(cmd, "status") == 0) {
    lastError = nullptr;
    sendStatus();
    return;
  }

  sendError(F("UNKNOWN_CMD"));
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r')
      continue;

    if (c == '\n') {
      if (serialLen > 0) {
        serialBuf[serialLen] = '\0';
        processLine(serialBuf);
        serialLen = 0;
      }
      continue;
    }

    if (serialLen < SERIAL_BUF_SIZE - 1) {
      serialBuf[serialLen++] = c;
    } else {
      serialLen = 0;
      sendError(F("INPUT_TOO_LONG"));
    }
  }
}

} // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);

  panServo.attach(PIN_PAN);
  tiltServo.attach(PIN_TILT);

  setTarget(PAN_CENTER, TILT_CENTER);
  currentPanAbs = PAN_CENTER;
  currentTiltAbs = TILT_CENTER;
  panTotalOffset = PAN_CENTER;
  panStrengthCmd = 0.0f;
  lastPanCmdMs = millis();

  updateServos();

  printLogPrefix("TX");
  Serial.println(F("{\"status\":\"ok\",\"boot\":\"gimbal_demo_start\"}"));

  moveSelfTestTo(SelfTestState::Center, PAN_CENTER, TILT_CENTER);
}

void loop() {
  pollSerial();
  updateServos();
  handleSelfTest();

  if (millis() - lastStatusUpdateMs >= STATUS_UPDATE_MS) {
    lastStatusUpdateMs = millis();
    sendStatus();
  }
}
