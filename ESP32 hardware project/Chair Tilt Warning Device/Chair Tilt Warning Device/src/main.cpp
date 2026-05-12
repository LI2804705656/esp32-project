#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <U8g2lib.h>

constexpr int OLED_SDA_PIN = 21;
constexpr int OLED_SCL_PIN = 22;
constexpr int MPU_SDA_PIN = 18;
constexpr int MPU_SCL_PIN = 19;
constexpr int POT_PIN = 34;
constexpr int BUZZER_PIN = 25;
constexpr int RED_LED_PIN = 26;
constexpr int GREEN_LED_PIN = 27;
constexpr int BUZZER_PWM_CHANNEL = 0;
constexpr int SCREEN_WIDTH = 128;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint32_t DISPLAY_REFRESH_MS = 500;
constexpr uint32_t ALARM_DELAY_MS = 2000;
constexpr uint32_t ALARM_BEEP_ON_MS = 200;
constexpr uint32_t ALARM_BEEP_OFF_MS = 200;
constexpr uint32_t BUZZER_FREQ_HZ = 2500;
constexpr float MIN_TILT_THRESHOLD_DEG = 5.0f;
constexpr float MAX_TILT_THRESHOLD_DEG = 30.0f;

TwoWire mpuWire = TwoWire(1);
Adafruit_MPU6050 mpu;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(
    U8G2_R0,
    U8X8_PIN_NONE,
    OLED_SCL_PIN,
    OLED_SDA_PIN);

float readTiltThresholdDeg();
void updateAlarmOutputs(bool alarmActive);

void drawTitle()
{
  const char *title = "翘椅子提醒装置";
  oled.setFont(u8g2_font_wqy12_t_gb2312);
  oled.drawUTF8((SCREEN_WIDTH - oled.getUTF8Width(title)) / 2, 13, title);
  oled.drawHLine(0, 16, SCREEN_WIDTH);
}

void showMessage(const char *line1, const char *line2)
{
  oled.clearBuffer();
  drawTitle();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 32, line1);
  oled.drawStr(0, 46, line2);
  oled.sendBuffer();
}

void calculateTiltAngles(const sensors_event_t &accel, float &pitchDeg, float &rollDeg)
{
  const float ax = accel.acceleration.x;
  const float ay = accel.acceleration.y;
  const float az = accel.acceleration.z;

  pitchDeg = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  rollDeg = atan2f(ay, az) * 180.0f / PI;
}

const char *getPostureText(float pitchDeg, float rollDeg, float thresholdDeg)
{
  const float absPitch = fabsf(pitchDeg);
  const float absRoll = fabsf(rollDeg);

  if (absPitch < thresholdDeg && absRoll < thresholdDeg) {
    return "正常";
  }

  if (absPitch >= absRoll) {
    return pitchDeg > 0.0f ? "后仰" : "前倾";
  }

  return rollDeg > 0.0f ? "右倾" : "左倾";
}

float readTiltThresholdDeg()
{
  const int rawValue = analogRead(POT_PIN);
  return MIN_TILT_THRESHOLD_DEG +
         (MAX_TILT_THRESHOLD_DEG - MIN_TILT_THRESHOLD_DEG) * rawValue / 4095.0f;
}

void updateAlarmOutputs(bool alarmActive)
{
  static bool beepOn = false;
  static uint32_t lastBeepChangeMs = 0;

  if (!alarmActive) {
    beepOn = false;
    lastBeepChangeMs = millis();
    ledcWriteTone(BUZZER_PWM_CHANNEL, 0);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    return;
  }

  const uint32_t nowMs = millis();
  const uint32_t intervalMs = beepOn ? ALARM_BEEP_ON_MS : ALARM_BEEP_OFF_MS;
  if (nowMs - lastBeepChangeMs >= intervalMs) {
    beepOn = !beepOn;
    lastBeepChangeMs = nowMs;
  }

  ledcWriteTone(BUZZER_PWM_CHANNEL, beepOn ? BUZZER_FREQ_HZ : 0);
  digitalWrite(RED_LED_PIN, beepOn ? HIGH : LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  ledcSetup(BUZZER_PWM_CHANNEL, BUZZER_FREQ_HZ, 8);
  ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANNEL);
  ledcWriteTone(BUZZER_PWM_CHANNEL, 0);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);

  pinMode(POT_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(POT_PIN, ADC_11db);

  oled.setI2CAddress(0x3C << 1);
  oled.setBusClock(I2C_CLOCK_HZ);
  oled.begin();
  oled.enableUTF8Print();

  mpuWire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  mpuWire.setClock(I2C_CLOCK_HZ);
  showMessage("OLED OK", "Starting MPU6050");

  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &mpuWire)) {
    Serial.println("MPU6050 init failed. Check VCC/GND/SDA/SCL on GPIO18/GPIO19.");
    showMessage("MPU6050 failed", "Check GPIO18/19");
    while (true) {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 initialized successfully.");
  showMessage("MPU6050 OK", "Reading data...");
  delay(800);
}

void loop()
{
  static uint32_t tiltStartMs = 0;
  static bool alarmActive = false;
  static uint32_t lastDisplayRefreshMs = 0;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float pitchDeg = 0.0f;
  float rollDeg = 0.0f;
  calculateTiltAngles(accel, pitchDeg, rollDeg);
  const float thresholdDeg = readTiltThresholdDeg();
  const char *posture = getPostureText(pitchDeg, rollDeg, thresholdDeg);
  const bool overThreshold = strcmp(posture, "后仰") == 0;

  if (overThreshold) {
    if (tiltStartMs == 0) {
      tiltStartMs = millis();
    }
    alarmActive = millis() - tiltStartMs >= ALARM_DELAY_MS;
  } else {
    tiltStartMs = 0;
    alarmActive = false;
  }

  updateAlarmOutputs(alarmActive);

  const uint32_t nowMs = millis();
  if (nowMs - lastDisplayRefreshMs < DISPLAY_REFRESH_MS) {
    delay(5);
    return;
  }
  lastDisplayRefreshMs = nowMs;

  char line[28];
  oled.clearBuffer();
  drawTitle();

  oled.setFont(u8g2_font_wqy12_t_gb2312);
  oled.drawUTF8(0, 31, "姿态:");
  oled.drawUTF8(42, 31, posture);
  oled.drawUTF8(88, 31, alarmActive ? "报警" : "监测");

  oled.setFont(u8g2_font_6x10_tf);
  snprintf(line, sizeof(line), "P:%6.1f  R:%6.1f", pitchDeg, rollDeg);
  oled.drawStr(0, 45, line);

  if (overThreshold && !alarmActive) {
    const uint32_t remainMs = ALARM_DELAY_MS - (millis() - tiltStartMs);
    snprintf(line, sizeof(line), "TH:%2.0f Wait:%1.1fs", thresholdDeg, remainMs / 1000.0f);
  } else {
    snprintf(line, sizeof(line), "TH:%2.0f Delay:2.0s", thresholdDeg);
  }
  oled.drawStr(0, 57, line);

  oled.sendBuffer();

  Serial.print("Posture: ");
  Serial.print(posture);
  Serial.print(" | Pitch: ");
  Serial.print(pitchDeg);
  Serial.print(" deg | Roll: ");
  Serial.print(rollDeg);
  Serial.print(" deg | Threshold: ");
  Serial.print(thresholdDeg);
  Serial.print(" deg | OverThreshold: ");
  Serial.print(overThreshold ? "YES" : "NO");
  Serial.print(" | Alarm: ");
  Serial.print(alarmActive ? "ON" : "OFF");
  Serial.print(" deg | Acc X/Y/Z: ");
  Serial.print(accel.acceleration.x);
  Serial.print(", ");
  Serial.print(accel.acceleration.y);
  Serial.print(", ");
  Serial.print(accel.acceleration.z);
  Serial.print(" m/s^2 | Gyro X/Y/Z: ");
  Serial.print(gyro.gyro.x);
  Serial.print(", ");
  Serial.print(gyro.gyro.y);
  Serial.print(", ");
  Serial.print(gyro.gyro.z);
  Serial.print(" rad/s | Temp: ");
  Serial.print(temp.temperature);
  Serial.println(" C");

  delay(5);
}
