#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

// ESP32 + RFID-RC522 wiring.
constexpr uint8_t PIN_RFID_SS = 5;    // RC522 SDA / SS
constexpr uint8_t PIN_RFID_RST = 22;  // RC522 RST
constexpr uint8_t PIN_LED = 2;        // LED signal pin

constexpr uint8_t PIN_SPI_SCK = 18;
constexpr uint8_t PIN_SPI_MISO = 19;
constexpr uint8_t PIN_SPI_MOSI = 23;

constexpr unsigned long LED_ON_TIME_MS = 3000;

MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);

String uidToString(const MFRC522::Uid &uid);
void setLed(bool on);

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_LED, OUTPUT);
  setLed(false);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_RFID_SS);
  rfid.PCD_Init();

  Serial.println();
  Serial.println("=== ESP32 RFID-RC522 Card Test ===");
  Serial.println("Wiring: SDA=GPIO5, SCK=GPIO18, MOSI=GPIO23, MISO=GPIO19, RST=GPIO22, 3.3V, GND");
  Serial.println("Place a card near the RC522. LED turns on after UID is read successfully.");

  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print("RC522 VersionReg: 0x");
  if (version < 0x10) {
    Serial.print("0");
  }
  Serial.println(version, HEX);

  if (version == 0x00 || version == 0xFF) {
    Serial.println("RC522 not detected. Check wiring and make sure VCC is 3.3V, not 5V.");
  }
}

void loop() {
  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }

  Serial.print("Card UID: ");
  Serial.println(uidToString(rfid.uid));

  setLed(true);
  delay(LED_ON_TIME_MS);
  setLed(false);

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  while (rfid.PICC_IsNewCardPresent()) {
    delay(50);
  }
}

String uidToString(const MFRC522::Uid &uid) {
  String text;

  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) {
      text += "0";
    }
    text += String(uid.uidByte[i], HEX);
    if (i < uid.size - 1) {
      text += " ";
    }
  }

  text.toUpperCase();
  return text;
}

void setLed(bool on) {
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}
