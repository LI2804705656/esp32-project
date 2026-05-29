#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define OLED_SDA 21
#define OLED_SCL 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void showMessage(const char *line1, const char *line2, const char *line3) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(line1);
  display.println(line2);
  display.println(line3);
  display.display();
}

void drawTestScreen() {
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("OLED Test OK");
  display.println("ESP32 Arduino IDE");
  display.println("I2C: SDA21 SCL22");

  display.drawRect(0, 34, 128, 30, SSD1306_WHITE);
  display.fillRect(4, 38, 30, 22, SSD1306_WHITE);
  display.drawCircle(56, 49, 11, SSD1306_WHITE);
  display.drawLine(78, 60, 122, 38, SSD1306_WHITE);

  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(OLED_SDA, OLED_SCL);

  Serial.println();
  Serial.println("Starting OLED test...");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed at address 0x3C. Trying 0x3D...");

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      Serial.println("OLED init failed. Check wiring, address, and library.");
      while (true) {
        delay(1000);
      }
    }
  }

  Serial.println("OLED init success.");
  showMessage("OLED Test OK", "Hello ESP32!", "Screen is working");
  delay(2000);
}

void loop() {
  drawTestScreen();
  delay(1500);

  display.invertDisplay(true);
  delay(500);
  display.invertDisplay(false);
  delay(500);

  for (int x = 0; x < SCREEN_WIDTH; x += 8) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Moving pixel test");
    display.fillCircle(x, 40, 3, SSD1306_WHITE);
    display.display();
    delay(40);
  }
}
