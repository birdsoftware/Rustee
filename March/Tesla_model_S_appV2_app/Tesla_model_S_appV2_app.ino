#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <Wire.h>
#include "driver/twai.h"

// ---------------- OLED ----------------
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// ---------------- CAN pins ----------------
// ESP32 -> transceiver
// GPIO13 = TX to WCMCU CTX
// GPIO17 = RX from WCMCU CRX
#define CAN_TX_PIN GPIO_NUM_13
#define CAN_RX_PIN GPIO_NUM_17

// ---------------- Timing ----------------
unsigned long lastStatusUpdate = 0;
unsigned long frameCount = 0;
bool canStarted = false;

// Store last frame for OLED
uint32_t lastId = 0;
uint8_t lastLen = 0;
uint8_t lastData[8] = {0};
bool lastExt = false;
bool gotFrame = false;

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void drawOledStatus(const String& line1,
                    const String& line2 = "",
                    const String& line3 = "",
                    const String& line4 = "",
                    const String& line5 = "") {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, line1);
  if (line2.length()) display.drawString(0, 12, line2);
  if (line3.length()) display.drawString(0, 24, line3);
  if (line4.length()) display.drawString(0, 36, line4);
  if (line5.length()) display.drawString(0, 48, line5);
  display.display();
}

String formatBytesLine(const uint8_t* data, uint8_t len, uint8_t startIndex, uint8_t endIndex) {
  String s = "";
  for (uint8_t i = startIndex; i < endIndex && i < len; i++) {
    if (data[i] < 0x10) s += "0";
    s += String(data[i], HEX);
    if (i < endIndex - 1 && i < len - 1) s += " ";
  }
  s.toUpperCase();
  return s;
}

bool startCAN500kListenOnly() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("twai_driver_install failed: %d\n", err);
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("twai_start failed: %d\n", err);
    twai_driver_uninstall();
    return false;
  }

  return true;
}

void printFrame(const twai_message_t &msg) {
  Serial.print("ID: 0x");
  Serial.print(msg.identifier, HEX);
  Serial.print(msg.extd ? " EXT" : " STD");
  Serial.print(" DLC:");
  Serial.print(msg.data_length_code);
  Serial.print(" Data:");

  for (int i = 0; i < msg.data_length_code; i++) {
    Serial.print(" ");
    if (msg.data[i] < 0x10) Serial.print("0");
    Serial.print(msg.data[i], HEX);
  }
  Serial.println();
}

void updateOledWithFrame() {
  String idLine = "ID: 0x" + String(lastId, HEX);
  idLine.toUpperCase();

  String typeLine = String(lastExt ? "EXT" : "STD") + " DLC:" + String(lastLen);
  String data1 = formatBytesLine(lastData, lastLen, 0, 4);
  String data2 = formatBytesLine(lastData, lastLen, 4, 8);
  String countLine = "Frames: " + String(frameCount);

  drawOledStatus(
    "OBD CAN RX",
    idLine,
    typeLine,
    data1,
    data2
  );

  // Alternate info every so often if you want frame count shown instead
  // drawOledStatus("OBD CAN RX", idLine, typeLine, data1, countLine);
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  VextON();
  delay(100);
  display.init();

  drawOledStatus(
    "Starting...",
    "Heltec OLED OK",
    "Init CAN 500k",
    "Listen Only"
  );

  Serial.println();
  Serial.println("Heltec OBD-II CAN Sniffer Starting...");

  canStarted = startCAN500kListenOnly();

  if (canStarted) {
    Serial.println("CAN started at 500 kbps, listen-only mode");
    drawOledStatus(
      "CAN Started",
      "500 kbps",
      "Listen Only",
      "Waiting frames..."
    );
  } else {
    Serial.println("CAN start failed");
    drawOledStatus(
      "CAN FAILED",
      "Check wiring",
      "CTX->GPIO13",
      "CRX->GPIO17"
    );
  }
}

void loop() {
  if (!canStarted) {
    delay(1000);
    return;
  }

  twai_message_t message;
  esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(100));

  if (result == ESP_OK) {
    frameCount++;
    lastId = message.identifier;
    lastLen = message.data_length_code;
    lastExt = message.extd;
    gotFrame = true;

    for (int i = 0; i < 8; i++) {
      lastData[i] = (i < message.data_length_code) ? message.data[i] : 0;
    }

    printFrame(message);
    updateOledWithFrame();
  }

  // If no frames after startup, keep a waiting message alive
  if (!gotFrame && millis() - lastStatusUpdate > 1000) {
    lastStatusUpdate = millis();
    drawOledStatus(
      "CAN Started",
      "500 kbps",
      "Listen Only",
      "Waiting frames..."
    );
  }
}