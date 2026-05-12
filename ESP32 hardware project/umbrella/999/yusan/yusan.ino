#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include <time.h>
#include <sys/time.h>

const char* WIFI_SSID = "Xiaomi17Pro";
const char* WIFI_PASSWORD = "88888888";
const char* LOCATION_NAME = "Haidian";
const float LATITUDE = 39.9593;
const float LONGITUDE = 116.2985;
const long GMT_OFFSET_SEC = 8 * 3600;

#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 22
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);

const int RTC_RST_PIN = 27;
const int RTC_DAT_PIN = 26;
const int RTC_CLK_PIN = 25;

const int HUMAN_SENSOR_PIN = 32;
const int HUMAN_LED_PIN = 33;
bool humanTriggerArmed = true;
bool ledBlinking = false;
bool ledBlinkState = true;
unsigned long ledBlinkEndMs = 0;
unsigned long ledNextToggleMs = 0;
unsigned long lastHumanPrintMs = 0;
const unsigned long LED_BLINK_DURATION_MS = 4000;
const unsigned long LED_BLINK_INTERVAL_MS = 250;
const unsigned long HUMAN_PRINT_INTERVAL_MS = 1000;

const int LED_STRIP_PIN = 14;
const int LED_STRIP_COUNT = 30;
const int LED_STRIP_BRIGHTNESS = 60;
Adafruit_NeoPixel strip(LED_STRIP_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);
unsigned long lastStripMs = 0;
const unsigned long STRIP_INTERVAL_MS = 60;

HardwareSerial mp3Serial(2);
const int MP3_RX_PIN = 16;
const int MP3_TX_PIN = 17;
uint8_t volumeLevel = 26;

String weatherText = "Loading";
float currentTemp = 0.0;
int currentHumidity = 0;
bool weatherOk = false;
unsigned long lastWeatherMs = 0;
const unsigned long WEATHER_INTERVAL_MS = 10UL * 60UL * 1000UL;

struct RtcDateTime { int y, mo, d, h, mi, s, wd; };
uint8_t decToBcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }
uint8_t bcdToDec(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }

void dsWriteByte(uint8_t v) {
  pinMode(RTC_DAT_PIN, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(RTC_DAT_PIN, v & 1);
    digitalWrite(RTC_CLK_PIN, HIGH); delayMicroseconds(2);
    digitalWrite(RTC_CLK_PIN, LOW); delayMicroseconds(2);
    v >>= 1;
  }
}

uint8_t dsReadByte() {
  uint8_t v = 0;
  pinMode(RTC_DAT_PIN, INPUT);
  for (int i = 0; i < 8; i++) {
    if (digitalRead(RTC_DAT_PIN)) v |= (1 << i);
    digitalWrite(RTC_CLK_PIN, HIGH); delayMicroseconds(2);
    digitalWrite(RTC_CLK_PIN, LOW); delayMicroseconds(2);
  }
  return v;
}

void dsWrite(uint8_t a, uint8_t v) {
  digitalWrite(RTC_RST_PIN, HIGH); delayMicroseconds(4);
  dsWriteByte(a); dsWriteByte(v);
  digitalWrite(RTC_RST_PIN, LOW);
}

uint8_t dsRead(uint8_t a) {
  digitalWrite(RTC_RST_PIN, HIGH); delayMicroseconds(4);
  dsWriteByte(a | 1);
  uint8_t v = dsReadByte();
  digitalWrite(RTC_RST_PIN, LOW);
  return v;
}

void initRtcPins() {
  pinMode(RTC_RST_PIN, OUTPUT);
  pinMode(RTC_CLK_PIN, OUTPUT);
  pinMode(RTC_DAT_PIN, OUTPUT);
  digitalWrite(RTC_RST_PIN, LOW);
  digitalWrite(RTC_CLK_PIN, LOW);
}

bool readRtc(RtcDateTime& t) {
  uint8_t sr = dsRead(0x80);
  t.s = bcdToDec(sr & 0x7F);
  t.mi = bcdToDec(dsRead(0x82));
  t.h = bcdToDec(dsRead(0x84) & 0x3F);
  t.d = bcdToDec(dsRead(0x86));
  t.mo = bcdToDec(dsRead(0x88));
  t.wd = bcdToDec(dsRead(0x8A));
  t.y = 2000 + bcdToDec(dsRead(0x8C));
  return !(sr & 0x80) && t.y >= 2024 && t.mo >= 1 && t.mo <= 12 && t.d >= 1 && t.d <= 31 && t.h <= 23 && t.mi <= 59 && t.s <= 59;
}

void writeRtc(const tm& ti) {
  dsWrite(0x8E, 0x00);
  dsWrite(0x80, decToBcd(ti.tm_sec) & 0x7F);
  dsWrite(0x82, decToBcd(ti.tm_min));
  dsWrite(0x84, decToBcd(ti.tm_hour));
  dsWrite(0x86, decToBcd(ti.tm_mday));
  dsWrite(0x88, decToBcd(ti.tm_mon + 1));
  dsWrite(0x8A, decToBcd(ti.tm_wday == 0 ? 7 : ti.tm_wday));
  dsWrite(0x8C, decToBcd((ti.tm_year + 1900) - 2000));
  dsWrite(0x8E, 0x80);
}

void loadTimeFromRtc() {
  RtcDateTime rt;
  if (!readRtc(rt)) return;
  tm ti = {};
  ti.tm_year = rt.y - 1900;
  ti.tm_mon = rt.mo - 1;
  ti.tm_mday = rt.d;
  ti.tm_hour = rt.h;
  ti.tm_min = rt.mi;
  ti.tm_sec = rt.s;
  time_t ts = mktime(&ti);
  timeval now = { ts, 0 };
  settimeofday(&now, nullptr);
}

bool waitNtp() {
  tm ti;
  for (int i = 0; i < 20; i++) if (getLocalTime(&ti, 500)) return true;
  return false;
}

void syncRtc() {
  tm ti;
  if (getLocalTime(&ti)) writeRtc(ti);
}

void sendMp3(uint8_t cmd, uint16_t param = 0) {
  uint8_t p[10] = {0x7E, 0xFF, 0x06, cmd, 0x00, (uint8_t)(param >> 8), (uint8_t)param, 0x00, 0x00, 0xEF};
  uint16_t sum = 0 - (p[1] + p[2] + p[3] + p[4] + p[5] + p[6]);
  p[7] = sum >> 8;
  p[8] = sum & 0xFF;
  mp3Serial.write(p, 10);
  mp3Serial.flush();
}

void playTrack(uint16_t n) { sendMp3(0x03, n); }
void setVolume(uint8_t v) { if (v > 30) v = 30; volumeLevel = v; sendMp3(0x06, v); }

String codeText(int c) {
  if (c == 0) return "Clear";
  if (c == 1 || c == 2) return "Partly";
  if (c == 3) return "Cloudy";
  if (c == 45 || c == 48) return "Fog";
  if (c >= 51 && c <= 67) return "Rain";
  if (c >= 71 && c <= 77) return "Snow";
  if (c >= 80 && c <= 82) return "Rain";
  if (c >= 95 && c <= 99) return "Thunder";
  return "Unknown";
}

bool getFloat(const String& s, const String& k, float& out) {
  int i = s.indexOf(k); if (i < 0) return false;
  i = s.indexOf(':', i); if (i < 0) return false;
  int j = ++i;
  while (j < s.length()) { char c = s[j]; if (!isDigit(c) && c != '-' && c != '.') break; j++; }
  out = s.substring(i, j).toFloat();
  return true;
}

bool getInt(const String& s, const String& k, int& out) { float f; if (!getFloat(s, k, f)) return false; out = (int)f; return true; }

String currentBlock(const String& p) {
  int i = p.indexOf("\"current\":{"); if (i < 0) return p;
  int a = p.indexOf('{', i), b = p.indexOf('}', a);
  if (a < 0 || b < 0) return p;
  return p.substring(a, b + 1);
}

void allStrip(uint32_t color) {
  for (int i = 0; i < LED_STRIP_COUNT; i++) strip.setPixelColor(i, color);
  strip.show();
}

void updateStrip() {
  if (millis() - lastStripMs < STRIP_INTERVAL_MS) return;
  lastStripMs = millis();
  unsigned long now = millis();
  if (weatherText == "Clear") {
    uint8_t ph = (now / 20) % 120, lv = ph < 60 ? ph : 119 - ph;
    allStrip(strip.Color(120 + lv * 2, 55 + lv, 0));
  } else if (weatherText == "Cloudy" || weatherText == "Partly") {
    int pos = (now / 180) % LED_STRIP_COUNT;
    for (int i = 0; i < LED_STRIP_COUNT; i++) strip.setPixelColor(i, (i == pos || i == (pos + LED_STRIP_COUNT - 1) % LED_STRIP_COUNT) ? strip.Color(120, 120, 130) : strip.Color(18, 28, 45));
    strip.show();
  } else if (weatherText.indexOf("Rain") >= 0 || weatherText == "Thunder") {
    int pos = (now / 120) % LED_STRIP_COUNT;
    for (int i = 0; i < LED_STRIP_COUNT; i++) strip.setPixelColor(i, i == pos ? strip.Color(0, 80, 180) : strip.Color(0, 8, 35));
    strip.show();
  } else {
    allStrip(strip.Color(20, 20, 20));
  }
}

void showMsg(const char* a, const char* b) {
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tf);
  display.drawStr(0, 22, a);
  display.drawStr(0, 42, b);
  display.sendBuffer();
}

void updateDisplay() {
  tm ti;
  char t[16] = "--:--:--", d[16] = "----/--/--", line[40];
  if (getLocalTime(&ti)) { strftime(t, sizeof(t), "%H:%M:%S", &ti); strftime(d, sizeof(d), "%Y/%m/%d", &ti); }
  display.clearBuffer();
  display.setFont(u8g2_font_logisoso20_tf);
  display.drawStr(0, 22, t);
  display.setFont(u8g2_font_6x12_tf);
  snprintf(line, sizeof(line), "Date:%s", d); display.drawStr(0, 35, line);
  snprintf(line, sizeof(line), "%s Weather:%s", LOCATION_NAME, weatherText.c_str()); display.drawStr(0, 47, line);
  if (weatherOk) snprintf(line, sizeof(line), "T:%.1fC H:%d%%", currentTemp, currentHumidity);
  else snprintf(line, sizeof(line), "T:-- H:--");
  display.drawStr(0, 60, line);
  display.sendBuffer();
}

void updateHuman() {
  bool human = digitalRead(HUMAN_SENSOR_PIN) == HIGH;
  if (millis() - lastHumanPrintMs >= HUMAN_PRINT_INTERVAL_MS) { lastHumanPrintMs = millis(); Serial.println(human ? "detected" : "none"); }
  if (human && humanTriggerArmed && weatherOk && weatherText == "Cloudy") {
    humanTriggerArmed = false;
    ledBlinking = true;
    ledBlinkState = false;
    ledBlinkEndMs = millis() + LED_BLINK_DURATION_MS;
    ledNextToggleMs = 0;
    playTrack(1);
  } else if (human && humanTriggerArmed) {
    humanTriggerArmed = false;
    digitalWrite(HUMAN_LED_PIN, HIGH);
  }
  if (!human) humanTriggerArmed = true;
  if (ledBlinking) {
    unsigned long now = millis();
    if ((long)(now - ledBlinkEndMs) >= 0) { ledBlinking = false; ledBlinkState = true; digitalWrite(HUMAN_LED_PIN, HIGH); }
    else if (ledNextToggleMs == 0 || (long)(now - ledNextToggleMs) >= 0) { ledBlinkState = !ledBlinkState; digitalWrite(HUMAN_LED_PIN, ledBlinkState ? HIGH : LOW); ledNextToggleMs = now + LED_BLINK_INTERVAL_MS; }
  } else digitalWrite(HUMAN_LED_PIN, HIGH);
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  showMsg("WiFi...", WIFI_SSID);
  for (int i = 0; i < 30; i++) { if (WiFi.status() == WL_CONNECTED) return true; delay(500); }
  showMsg("WiFi failed", "RTC time");
  return false;
}

void updateWeather() {
  if (WiFi.status() != WL_CONNECTED) { if (!weatherOk) weatherText = "NoWiFi"; return; }
  String url = "http://api.open-meteo.com/v1/forecast?latitude=";
  url += String(LATITUDE, 4); url += "&longitude="; url += String(LONGITUDE, 4); url += "&current=temperature_2m,relative_humidity_2m,weather_code";
  HTTPClient http; http.begin(url); http.setTimeout(8000);
  int code = http.GET();
  if (code == 200) {
    String block = currentBlock(http.getString());
    int wc = -1;
    bool ok = getFloat(block, "\"temperature_2m\"", currentTemp);
    ok = getInt(block, "\"relative_humidity_2m\"", currentHumidity) && ok;
    ok = getInt(block, "\"weather_code\"", wc) && ok;
    if (ok) { weatherText = codeText(wc); weatherOk = true; Serial.println("weather ok"); }
    else if (!weatherOk) weatherText = "ParseErr";
  } else if (!weatherOk) weatherText = "HttpErr";
  http.end(); lastWeatherMs = millis();
}

void initMp3() { sendMp3(0x0C); delay(1800); sendMp3(0x09, 0x02); delay(500); setVolume(volumeLevel); delay(300); }

void setup() {
  Serial.begin(115200); delay(500);
  setenv("TZ", "CST-8", 1); tzset();
  display.begin(); showMsg("OLED OK", "Starting...");
  strip.begin(); strip.setBrightness(LED_STRIP_BRIGHTNESS); strip.show();
  pinMode(HUMAN_SENSOR_PIN, INPUT); pinMode(HUMAN_LED_PIN, OUTPUT); digitalWrite(HUMAN_LED_PIN, HIGH);
  initRtcPins(); loadTimeFromRtc();
  mp3Serial.begin(9600, SERIAL_8N1, MP3_RX_PIN, MP3_TX_PIN); delay(500); initMp3();
  if (connectWiFi()) { configTime(GMT_OFFSET_SEC, 0, "pool.ntp.org", "ntp.aliyun.com"); if (waitNtp()) syncRtc(); updateWeather(); }
  updateDisplay();
}

void loop() {
  updateHuman();
  updateStrip();
  if (millis() - lastWeatherMs >= WEATHER_INTERVAL_MS) updateWeather();
  static unsigned long lastDisplayMs = 0;
  if (millis() - lastDisplayMs >= 1000) { updateDisplay(); lastDisplayMs = millis(); }
}