#include <Wire.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>

#define OLED_SDA 21
#define OLED_SCL 22
#define MPU_SDA 23
#define MPU_SCL 26

#define HX711_DT 16
#define HX711_SCK 17

#define GPS_RX_PIN 34  // ESP32 RX, connect to GPS TX
#define GPS_TX_PIN 13  // ESP32 TX, connect to GPS RX. Optional if only reading GPS.

#define KEY1_PIN 4
#define KEY2_PIN 32
#define KEY3_PIN 33
#define KEY4_PIN 25

#define BUZZER_PIN 14
#define LED_PIN 27

#define BUZZER_ACTIVE_LOW 1
#define USE_BUZZER 1
#define LED_ON HIGH
#define LED_OFF LOW

float calibrationFactor = 420.0;
float zeroDeadbandGram = 8.0;
float weightLimitGram = 5000.0;
float postureAngleLimit = 18.0;
unsigned long postureHoldMs = 5000;

TinyGPSPlus gps;
TwoWire mpuWire = TwoWire(1);
HardwareSerial gpsSerial(1);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

enum PageMode {
  PAGE_MONITOR,
  PAGE_GPS,
  PAGE_SETTING
};

enum SettingMode {
  SET_WEIGHT,
  SET_ANGLE
};

struct ButtonState {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastDebounceMs;
};

ButtonState key1 = {KEY1_PIN, HIGH, HIGH, 0};
ButtonState key2 = {KEY2_PIN, HIGH, HIGH, 0};
ButtonState key3 = {KEY3_PIN, HIGH, HIGH, 0};
ButtonState key4 = {KEY4_PIN, HIGH, HIGH, 0};

const unsigned long debounceDelayMs = 25;
PageMode pageMode = PAGE_MONITOR;
SettingMode settingMode = SET_WEIGHT;
bool forceRefresh = true;

float pitchDeg = 0.0;
float rollDeg = 0.0;
float forwardAngle = 0.0;
float backwardAngle = 0.0;
float leftAngle = 0.0;
float rightAngle = 0.0;
float weightGram = 0.0;
long rawValue = 0;
long zeroOffset = 0;
uint8_t mpuAddress = 0x68;
uint8_t mpuWhoAmI = 0x00;
float pitchZeroDeg = 0.0;
float rollZeroDeg = 0.0;

// MPU6050 is now installed facing upward. Keep the bag still when powering on;
// that posture is used as 0 degrees.
const float frontBackSign = 1.0;
const float leftRightSign = 1.0;

unsigned long singleShoulderStartMs = 0;
unsigned long hunchbackStartMs = 0;
unsigned long backwardStartMs = 0;
bool singleShoulderAlarm = false;
bool hunchbackAlarm = false;
bool backwardAlarm = false;
bool overweightAlarm = false;
bool hx711Available = true;

unsigned long lastUpdateMs = 0;
unsigned long lastAlarmBlinkMs = 0;
unsigned long gpsCharsReceived = 0;
unsigned long lastGpsCharMs = 0;
bool alarmOutputState = false;

float positivePart(float value) {
  return value > 0 ? value : 0;
}

float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void buzzerOn() {
#if USE_BUZZER
#if BUZZER_ACTIVE_LOW
  digitalWrite(BUZZER_PIN, LOW);
#else
  digitalWrite(BUZZER_PIN, HIGH);
#endif
  pinMode(BUZZER_PIN, OUTPUT);
#else
  pinMode(BUZZER_PIN, INPUT);
#endif
}

void buzzerOff() {
#if USE_BUZZER
#if BUZZER_ACTIVE_LOW
  digitalWrite(BUZZER_PIN, HIGH);
#else
  digitalWrite(BUZZER_PIN, LOW);
#endif
  pinMode(BUZZER_PIN, OUTPUT);
#else
  pinMode(BUZZER_PIN, INPUT);
#endif
}

void ledOff() {
  digitalWrite(LED_PIN, LED_OFF);
}

void startupQuietDelay(unsigned long durationMs) {
  unsigned long startMs = millis();
  while (millis() - startMs < durationMs) {
    buzzerOff();
    ledOff();
    delay(10);
  }
}

void drawMessage(const char *line1, const char *line2) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_wqy12_t_gb2312);
  oled.drawUTF8(0, 22, line1);
  oled.drawUTF8(0, 44, line2);
  oled.sendBuffer();
}

bool i2cDeviceExists(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

uint8_t scanFirstI2cDevice(TwoWire &bus) {
  for (uint8_t address = 1; address < 127; address++) {
    if (i2cDeviceExists(bus, address)) {
      return address;
    }
    delay(2);
  }
  return 0;
}

bool readI2cRegister(TwoWire &bus, uint8_t address, uint8_t reg, uint8_t &value) {
  bus.beginTransmission(address);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) {
    return false;
  }

  if (bus.requestFrom(address, (uint8_t)1) != 1) {
    return false;
  }

  value = bus.read();
  return true;
}

bool writeI2cRegister(TwoWire &bus, uint8_t address, uint8_t reg, uint8_t value) {
  bus.beginTransmission(address);
  bus.write(reg);
  bus.write(value);
  return bus.endTransmission() == 0;
}

bool wakeMpuByRegister(uint8_t address) {
  return writeI2cRegister(mpuWire, address, 0x6B, 0x00);
}

void drawMpuErrorDetail() {
  uint8_t foundOnMpuBus = scanFirstI2cDevice(mpuWire);
  bool found68OnOledBus = i2cDeviceExists(Wire, 0x68);
  bool found69OnOledBus = i2cDeviceExists(Wire, 0x69);
  uint8_t whoAmI = 0;
  bool gotWhoAmI = false;
  char line[24];

  if (foundOnMpuBus != 0) {
    wakeMpuByRegister(foundOnMpuBus);
    delay(20);
    gotWhoAmI = readI2cRegister(mpuWire, foundOnMpuBus, 0x75, whoAmI);
  }

  oled.clearBuffer();
  oled.setFont(u8g2_font_wqy12_t_gb2312);
  oled.drawUTF8(0, 12, "MPU6050错误");
  oled.drawHLine(0, 16, 128);

  if (foundOnMpuBus == 0 && (found68OnOledBus || found69OnOledBus)) {
    oled.drawUTF8(0, 32, "MPU在OLED线上");
    oled.drawUTF8(0, 50, "代码用SDA23 SCL26");
  } else if (foundOnMpuBus == 0) {
    oled.drawUTF8(0, 32, "23/26未扫到设备");
    oled.drawUTF8(0, 50, "查VCC GND DA CL");
  } else {
    if (gotWhoAmI) {
      snprintf(line, sizeof(line), "地址0x%02X ID:0x%02X", foundOnMpuBus, whoAmI);
    } else {
      snprintf(line, sizeof(line), "地址0x%02X ID读取失败", foundOnMpuBus);
    }
    oled.drawUTF8(0, 32, line);
    oled.drawUTF8(0, 50, "拍下这行给我");
  }

  oled.sendBuffer();
}

void drawValueLine(int y, const char *name, float value, const char *unit) {
  char numberText[16];
  dtostrf(value, 6, 1, numberText);

  oled.drawUTF8(0, y, name);
  oled.drawUTF8(38, y, numberText);
  oled.drawUTF8(92, y, unit);
}

bool readMotionRaw(int16_t &axRaw, int16_t &ayRaw, int16_t &azRaw) {
  mpuWire.beginTransmission(mpuAddress);
  mpuWire.write(0x3B);
  if (mpuWire.endTransmission(false) != 0) {
    return false;
  }

  if (mpuWire.requestFrom(mpuAddress, (uint8_t)6) != 6) {
    return false;
  }

  axRaw = (int16_t)((mpuWire.read() << 8) | mpuWire.read());
  ayRaw = (int16_t)((mpuWire.read() << 8) | mpuWire.read());
  azRaw = (int16_t)((mpuWire.read() << 8) | mpuWire.read());
  return true;
}

bool initMotionSensorAt(uint8_t address) {
  uint8_t id = 0;

  if (!i2cDeviceExists(mpuWire, address)) {
    return false;
  }

  wakeMpuByRegister(address);
  delay(80);

  if (!readI2cRegister(mpuWire, address, 0x75, id)) {
    return false;
  }

  // 0x68 is a standard MPU6050 ID. 0x70 is common on MPU6500-compatible
  // boards that are often sold as MPU6050 modules.
  if (id != 0x68 && id != 0x70) {
    return false;
  }

  mpuAddress = address;
  mpuWhoAmI = id;
  writeI2cRegister(mpuWire, mpuAddress, 0x6B, 0x00);  // Wake up.
  writeI2cRegister(mpuWire, mpuAddress, 0x1A, 0x03);  // Low-pass filter.
  writeI2cRegister(mpuWire, mpuAddress, 0x1B, 0x08);  // Gyro +/-500 dps.
  writeI2cRegister(mpuWire, mpuAddress, 0x1C, 0x08);  // Accel +/-4 g.
  delay(80);
  return true;
}

bool initMotionSensor() {
  return initMotionSensorAt(0x68) || initMotionSensorAt(0x69);
}

void readRawTilt(float &rawPitchDeg, float &rawRollDeg) {
  int16_t axRaw = 0;
  int16_t ayRaw = 0;
  int16_t azRaw = 0;

  if (!readMotionRaw(axRaw, ayRaw, azRaw)) {
    rawPitchDeg = 0.0;
    rawRollDeg = 0.0;
    return;
  }

  float ax = (float)axRaw;
  float ay = (float)ayRaw;
  float az = (float)azRaw;

  rawPitchDeg = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  rawRollDeg = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
}

void calibratePostureZero() {
  drawMessage("姿态校准", "请保持默认姿态");
  startupQuietDelay(800);

  float pitchSum = 0.0;
  float rollSum = 0.0;
  for (int i = 0; i < 40; i++) {
    float rawPitch = 0.0;
    float rawRoll = 0.0;
    readRawTilt(rawPitch, rawRoll);
    pitchSum += rawPitch;
    rollSum += rawRoll;
    startupQuietDelay(20);
  }

  pitchZeroDeg = pitchSum / 40.0;
  rollZeroDeg = rollSum / 40.0;
}

bool hx711Ready() {
  return digitalRead(HX711_DT) == LOW;
}

long hx711ReadRaw() {
  unsigned long value = 0;

  noInterrupts();
  for (int i = 0; i < 24; i++) {
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    value = value << 1;
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);
    if (digitalRead(HX711_DT)) {
      value++;
    }
  }

  digitalWrite(HX711_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(HX711_SCK, LOW);
  interrupts();

  if (value & 0x800000) {
    value |= 0xFF000000;
  }
  return (long)value;
}

long hx711ReadAverage(int times) {
  long sum = 0;
  for (int i = 0; i < times; i++) {
    while (!hx711Ready()) {
      buzzerOff();
      delay(1);
    }
    sum += hx711ReadRaw();
  }
  return sum / times;
}

void hx711Tare() {
  drawMessage("称重模块", "正在清零...");
  startupQuietDelay(800);
  zeroOffset = hx711ReadAverage(10);
}

bool buttonReleased(ButtonState &button) {
  bool reading = digitalRead(button.pin);
  bool released = false;

  if (reading != button.lastReading) {
    button.lastDebounceMs = millis();
  }

  if (millis() - button.lastDebounceMs > debounceDelayMs) {
    if (reading != button.stableState) {
      button.stableState = reading;
      if (button.stableState == HIGH) {
        released = true;
      }
    }
  }

  button.lastReading = reading;
  return released;
}

void handleKeys() {
  if (buttonReleased(key1)) {
    if (pageMode == PAGE_MONITOR) {
      pageMode = PAGE_GPS;
    } else if (pageMode == PAGE_GPS) {
      pageMode = PAGE_SETTING;
    } else {
      pageMode = PAGE_MONITOR;
    }
    forceRefresh = true;
  }

  if (buttonReleased(key4)) {
    pageMode = PAGE_SETTING;
    settingMode = (settingMode == SET_WEIGHT) ? SET_ANGLE : SET_WEIGHT;
    forceRefresh = true;
  }

  if (buttonReleased(key2)) {
    pageMode = PAGE_SETTING;
    if (settingMode == SET_WEIGHT) {
      weightLimitGram += 2000.0;
      if (weightLimitGram > 20000.0) weightLimitGram = 20000.0;
    } else {
      postureAngleLimit += 2.0;
      if (postureAngleLimit > 40.0) postureAngleLimit = 40.0;
    }
    forceRefresh = true;
  }

  if (buttonReleased(key3)) {
    pageMode = PAGE_SETTING;
    if (settingMode == SET_WEIGHT) {
      weightLimitGram -= 2000.0;
      if (weightLimitGram < 1000.0) weightLimitGram = 1000.0;
    } else {
      postureAngleLimit -= 2.0;
      if (postureAngleLimit < 6.0) postureAngleLimit = 6.0;
    }
    forceRefresh = true;
  }
}

void updateGps() {
  while (gpsSerial.available() > 0) {
    char gpsChar = gpsSerial.read();
    gps.encode(gpsChar);
    gpsCharsReceived++;
    lastGpsCharMs = millis();
  }
}

void updatePostureState() {
  unsigned long nowMs = millis();
  bool singleNow = (leftAngle >= postureAngleLimit) || (rightAngle >= postureAngleLimit);
  bool forwardNow = forwardAngle >= postureAngleLimit;
  bool backwardNow = backwardAngle >= postureAngleLimit;

  if (singleNow) {
    if (singleShoulderStartMs == 0) {
      singleShoulderStartMs = nowMs;
    }
    singleShoulderAlarm = (nowMs - singleShoulderStartMs >= postureHoldMs);
  } else {
    singleShoulderStartMs = 0;
    singleShoulderAlarm = false;
  }

  if (forwardNow) {
    if (hunchbackStartMs == 0) {
      hunchbackStartMs = nowMs;
    }
    hunchbackAlarm = (nowMs - hunchbackStartMs >= postureHoldMs);
  } else {
    hunchbackStartMs = 0;
    hunchbackAlarm = false;
  }

  if (backwardNow) {
    if (backwardStartMs == 0) {
      backwardStartMs = nowMs;
    }
    backwardAlarm = (nowMs - backwardStartMs >= postureHoldMs);
  } else {
    backwardStartMs = 0;
    backwardAlarm = false;
  }
}

void updateAlarmOutput() {
  bool alarmNow = overweightAlarm || singleShoulderAlarm || hunchbackAlarm || backwardAlarm;

  if (millis() < 3000) {
    alarmOutputState = false;
    buzzerOff();
    ledOff();
    return;
  }

  if (!alarmNow) {
    alarmOutputState = false;
    buzzerOff();
    ledOff();
    return;
  }

  buzzerOn();

  if (millis() - lastAlarmBlinkMs >= 500) {
    lastAlarmBlinkMs = millis();
    alarmOutputState = !alarmOutputState;
    digitalWrite(LED_PIN, alarmOutputState ? LED_ON : LED_OFF);
  }
}

const char *getAlarmText() {
  if (overweightAlarm) return "超重";
  if (hunchbackAlarm) return "驼背";
  if (backwardAlarm) return "后仰";
  if (singleShoulderAlarm) {
    if (rightAngle > leftAngle) return "左单肩";
    return "右单肩";
  }
  return "正常";
}

void drawMonitorPage() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_wqy12_t_gb2312);

  drawValueLine(11, "重量", weightGram, "克");
  drawValueLine(23, "前倾", forwardAngle, "度");
  drawValueLine(35, "后仰", backwardAngle, "度");
  drawValueLine(47, "左倾", leftAngle, "度");
  drawValueLine(59, "右倾", rightAngle, "度");

  oled.drawUTF8(78, 11, getAlarmText());
  oled.sendBuffer();
}

void drawGpsPage() {
  char latText[20];
  char lngText[20];
  char satText[12];
  char dataText[24];
  char fixText[24];
  bool hasGpsData = gpsCharsReceived > 0 && millis() - lastGpsCharMs < 3000;

  oled.clearBuffer();
  oled.setFont(u8g2_font_wqy12_t_gb2312);
  oled.drawUTF8(0, 12, "GPS定位");
  oled.drawHLine(0, 16, 128);

  if (gps.location.isValid()) {
    dtostrf(gps.location.lat(), 11, 6, latText);
    dtostrf(gps.location.lng(), 11, 6, lngText);
    oled.drawUTF8(0, 30, "纬度");
    oled.drawUTF8(34, 30, latText);
    oled.drawUTF8(0, 45, "经度");
    oled.drawUTF8(34, 45, lngText);
  } else {
    oled.drawUTF8(0, 30, hasGpsData ? "有数据 未定位" : "未收到GPS数据");
    oled.drawUTF8(0, 44, hasGpsData ? "到室外等3-10分" : "查TX到GPIO34");
  }

  if (gps.satellites.isValid()) {
    ltoa(gps.satellites.value(), satText, 10);
  } else {
    strcpy(satText, "--");
  }
  snprintf(dataText, sizeof(dataText), "数据:%lu", gpsCharsReceived);
  snprintf(fixText, sizeof(fixText), "卫星:%s", satText);
  oled.drawUTF8(0, 63, fixText);
  oled.drawUTF8(58, 63, dataText);
  oled.sendBuffer();
}

void drawSettingPage() {
  char weightText[16];
  char angleText[16];
  dtostrf(weightLimitGram / 1000.0, 4, 1, weightText);
  dtostrf(postureAngleLimit, 4, 0, angleText);

  oled.clearBuffer();
  oled.setFont(u8g2_font_wqy12_t_gb2312);
  oled.drawUTF8(0, 12, "阈值调节");
  oled.drawHLine(0, 16, 128);

  oled.drawUTF8(0, 31, settingMode == SET_WEIGHT ? ">重量" : " 重量");
  oled.drawUTF8(46, 31, weightText);
  oled.drawUTF8(88, 31, "千克");

  oled.drawUTF8(0, 48, settingMode == SET_ANGLE ? ">角度" : " 角度");
  oled.drawUTF8(46, 48, angleText);
  oled.drawUTF8(88, 48, "度");

  oled.drawUTF8(0, 63, settingMode == SET_WEIGHT ? "当前:重量 +2kg" : "当前:角度 +2度");
  oled.sendBuffer();
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
#if BUZZER_ACTIVE_LOW
  digitalWrite(BUZZER_PIN, HIGH);
#else
  digitalWrite(BUZZER_PIN, LOW);
#endif
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  Serial.begin(115200);
  startupQuietDelay(100);

  pinMode(HX711_DT, INPUT);
  pinMode(HX711_SCK, OUTPUT);
  pinMode(KEY1_PIN, INPUT_PULLUP);
  pinMode(KEY2_PIN, INPUT_PULLUP);
  pinMode(KEY3_PIN, INPUT_PULLUP);
  pinMode(KEY4_PIN, INPUT_PULLUP);
  digitalWrite(HX711_SCK, LOW);
  buzzerOff();
  ledOff();

  Wire.begin(OLED_SDA, OLED_SCL);
  mpuWire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(100000);
  mpuWire.setClock(100000);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  oled.begin();
  oled.enableUTF8Print();
  buzzerOff();

  drawMessage("称重姿态测试", "正在启动...");
  startupQuietDelay(300);

  wakeMpuByRegister(0x68);
  wakeMpuByRegister(0x69);
  startupQuietDelay(50);

  if (!initMotionSensor()) {
    drawMpuErrorDetail();
    buzzerOff();
    Serial.println("MPU init failed.");
    while (true) {
      buzzerOff();
      ledOff();
      delay(1000);
    }
  }

  buzzerOff();
  calibratePostureZero();
  buzzerOff();

  unsigned long startMs = millis();
  while (!hx711Ready()) {
    buzzerOff();
    if (millis() - startMs > 3000) {
      hx711Available = false;
      drawMessage("HX711未连接", "继续姿态监测");
      Serial.println("HX711 not ready, continue posture monitor.");
      startupQuietDelay(800);
      break;
    }
    delay(10);
  }

  if (hx711Available) {
    hx711Tare();
  }
  buzzerOff();
  drawMessage("准备完成", "开始监测");
  startupQuietDelay(700);
}

void loop() {
  handleKeys();
  updateGps();

  float rawPitch = 0.0;
  float rawRoll = 0.0;
  readRawTilt(rawPitch, rawRoll);

  pitchDeg = normalizeAngle((rawPitch - pitchZeroDeg) * frontBackSign);
  rollDeg = normalizeAngle((rawRoll - rollZeroDeg) * leftRightSign);

  forwardAngle = positivePart(-pitchDeg);
  backwardAngle = positivePart(pitchDeg);
  leftAngle = positivePart(-rollDeg);
  rightAngle = positivePart(rollDeg);
  updatePostureState();

  if (hx711Available && hx711Ready()) {
    rawValue = hx711ReadRaw();
    float newWeightGram = (rawValue - zeroOffset) / calibrationFactor;
    if (fabs(newWeightGram) < zeroDeadbandGram) {
      newWeightGram = 0.0;
    }
    weightGram = weightGram * 0.7 + newWeightGram * 0.3;
    if (fabs(weightGram) < zeroDeadbandGram) {
      weightGram = 0.0;
    }
  }

  overweightAlarm = weightGram >= weightLimitGram;
  updateAlarmOutput();

  if (forceRefresh || millis() - lastUpdateMs >= 150) {
    forceRefresh = false;
    lastUpdateMs = millis();

    Serial.print("页面:");
    if (pageMode == PAGE_MONITOR) Serial.print("监测");
    if (pageMode == PAGE_GPS) Serial.print("定位");
    if (pageMode == PAGE_SETTING) Serial.print("设置");
    Serial.print(" 重量(g):");
    Serial.print(weightGram, 1);
    Serial.print(" 前倾:");
    Serial.print(forwardAngle, 1);
    Serial.print(" 后仰:");
    Serial.print(backwardAngle, 1);
    Serial.print(" 左倾:");
    Serial.print(leftAngle, 1);
    Serial.print(" 右倾:");
    Serial.print(rightAngle, 1);
    Serial.print(" 纬度:");
    if (gps.location.isValid()) Serial.print(gps.location.lat(), 6); else Serial.print("--");
    Serial.print(" 经度:");
    if (gps.location.isValid()) Serial.print(gps.location.lng(), 6); else Serial.print("--");
    Serial.print(" 卫星:");
    if (gps.satellites.isValid()) Serial.print(gps.satellites.value()); else Serial.print("--");
    Serial.print(" 状态:");
    Serial.println(getAlarmText());

    if (pageMode == PAGE_MONITOR) {
      drawMonitorPage();
    } else if (pageMode == PAGE_GPS) {
      drawGpsPage();
    } else {
      drawSettingPage();
    }
  }
}
