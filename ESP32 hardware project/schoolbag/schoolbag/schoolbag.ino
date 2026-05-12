#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ESP32 default I2C pins. Connect both MPU6050 and OLED to this same I2C bus.
#define I2C_SDA 21
#define I2C_SCL 22

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

float pitchDeg = 0.0;
float rollDeg = 0.0;
unsigned long lastUpdateMs = 0;

void showMessage(const char *line1, const char *line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.println(line2);
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed. Check VCC/GND/SDA/SCL or I2C address.");
    while (true) {
      delay(1000);
    }
  }

  showMessage("Schoolbag Test", "Starting...");

  if (!mpu.begin(0x68, &Wire)) {
    showMessage("MPU6050 ERROR", "Check wiring/addr");
    Serial.println("MPU6050 init failed. Check VCC/GND/SDA/SCL.");
    while (true) {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  showMessage("MPU6050 + OLED", "Ready");
  delay(800);
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Calculate rough tilt angles from gravity acceleration.
  rollDeg = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
  pitchDeg = atan2(-accel.acceleration.x,
                   sqrt(accel.acceleration.y * accel.acceleration.y +
                        accel.acceleration.z * accel.acceleration.z)) *
             180.0 / PI;

  if (millis() - lastUpdateMs >= 200) {
    lastUpdateMs = millis();

    Serial.print("Acc X:");
    Serial.print(accel.acceleration.x, 2);
    Serial.print(" Y:");
    Serial.print(accel.acceleration.y, 2);
    Serial.print(" Z:");
    Serial.print(accel.acceleration.z, 2);
    Serial.print(" | Gyro X:");
    Serial.print(gyro.gyro.x, 2);
    Serial.print(" Y:");
    Serial.print(gyro.gyro.y, 2);
    Serial.print(" Z:");
    Serial.print(gyro.gyro.z, 2);
    Serial.print(" | Pitch:");
    Serial.print(pitchDeg, 1);
    Serial.print(" Roll:");
    Serial.println(rollDeg, 1);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("MPU6050 Test");

    display.setCursor(0, 12);
    display.print("AX:");
    display.print(accel.acceleration.x, 1);
    display.print(" AY:");
    display.print(accel.acceleration.y, 1);

    display.setCursor(0, 22);
    display.print("AZ:");
    display.print(accel.acceleration.z, 1);
    display.print(" m/s2");

    display.setCursor(0, 34);
    display.print("GX:");
    display.print(gyro.gyro.x, 1);
    display.print(" GY:");
    display.print(gyro.gyro.y, 1);

    display.setCursor(0, 44);
    display.print("GZ:");
    display.print(gyro.gyro.z, 1);
    display.print(" rad/s");

    display.setCursor(0, 56);
    display.print("P:");
    display.print(pitchDeg, 0);
    display.print(" R:");
    display.print(rollDeg, 0);
    display.print(" T:");
    display.print(temp.temperature, 0);

    display.display();
  }
}
