//This program is normal CAN mode. Actively request PID 0x49 APP D
// 12V -> DC-DC converter -> 5V -> pin2 5V LoRa RSI side
// GND -> DC-DC converter -> GND -> pin1 GND LoRa RSI side
// | WCMCU-230 | Heltec ESP32 |
// | --------- | ------------ |
// | 3V3       | 3V3          | PRG Side pin 2
// | GND       | GND          | PRG Side pin 1
// | CTX       | GPIO19      | 
// | CRX       | GPIO20       | 
// | CANH      | OBD pin 6    |
// | CANL      | OBD pin 14   |

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <Wire.h>
#include "driver/twai.h"

// ---------------- OLED ----------------
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// ---------------- CAN pins ----------------
// ESP32 -> transceiver
// GPIO19 = TX to WCMCU CRX
// GPIO20 = RX from WCMCU CTX
#define CAN_TX_PIN GPIO_NUM_19
#define CAN_RX_PIN GPIO_NUM_20

// ---------------- Timing ----------------
unsigned long lastReqMs = 0;
unsigned long lastDisplayMs = 0;
//unsigned long lastFrameMs = 0;
unsigned long n = 0;

bool canStarted = false;

// ---------------- APP / output ----------------
float app = 0.0f;
bool seen49 = false;

//float appMin = 14.9f;          // fallback default
float appMax = 18.0f;          // will be auto-adjusted after calibration
float learnedAppMin = 1000.0f;
float effectiveAppMin = 14.9f;
bool calibratingAppMin = true;
//bool gotAppDuringCalibration = false;
int countApp = 0;

float minV = 0.35f;
float vin = 0.35f;
int pwm = 0;
int pwmMax = 200;

// function prototypes
const char* twaiStateName(twai_state_t s);
void printTwaiStatus(const char* tag);
void readTwaiAlerts();
void recoverTwaiIfNeeded();
bool sendPid(uint8_t pid);
void handlePid49(uint8_t A, uint8_t B, uint32_t respId);
void processCanFrames();
void updateDisplay();
float mapPedalToVoltage(float appRaw);
bool startCAN500kNormal();

// ---------------- Helper functions ----------------
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

float mapPedalToVoltage(float appRaw) {
  float range = appMax - effectiveAppMin;
  if (range < 0.5f) range = 0.5f;

  float x = (appRaw - effectiveAppMin) / range;

  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;

  return minV + x * (3.3f - minV);
}

bool startCAN500kNormal() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  g_config.alerts_enabled =
      TWAI_ALERT_TX_FAILED |
      TWAI_ALERT_TX_SUCCESS |
      TWAI_ALERT_BUS_ERROR |
      TWAI_ALERT_BUS_OFF |
      TWAI_ALERT_BUS_RECOVERED |
      TWAI_ALERT_RECOVERY_IN_PROGRESS;

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

  printTwaiStatus("after start");
  return true;
}

void readTwaiAlerts() {
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts != 0) {
    Serial.print("ALERTS:");
    if (alerts & TWAI_ALERT_TX_SUCCESS) Serial.print(" TX_SUCCESS");
    if (alerts & TWAI_ALERT_TX_FAILED) Serial.print(" TX_FAILED");
    if (alerts & TWAI_ALERT_BUS_ERROR) Serial.print(" BUS_ERROR");
    if (alerts & TWAI_ALERT_BUS_OFF) Serial.print(" BUS_OFF");
    if (alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) Serial.print(" RECOVERING");
    if (alerts & TWAI_ALERT_BUS_RECOVERED) Serial.print(" BUS_RECOVERED");
    Serial.println();
  }
}

bool sendPid(uint8_t pid) {
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    if (st.state != TWAI_STATE_RUNNING) {
      Serial.printf("TWAI not running before TX: %s\n", twaiStateName(st.state));
      return false;
    }
  }

  twai_message_t msg = {};
  msg.identifier = 0x7DF;
  msg.extd = 0;
  msg.rtr = 0;
  msg.ss = 0;
  msg.self = 0;
  msg.dlc_non_comp = 0;
  msg.data_length_code = 8;

  msg.data[0] = 0x02;
  msg.data[1] = 0x01;
  msg.data[2] = pid;
  msg.data[3] = 0x00;
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(50));
  if (err != ESP_OK) {
    Serial.printf("PID 0x%02X send failed: %d\n", pid, err);
    printTwaiStatus("tx fail");
    return false;
  }

  return true;
}

void handlePid49(uint8_t A, uint8_t B, uint32_t respId) {
  // lastRespId = respId;
  // lastA = A;
  // lastB = B;

  app = A * 100.0f / 255.0f;
  seen49 = true;
  //lastFrameMs = millis();

  if (calibratingAppMin) {
    countApp++;

    if (app < learnedAppMin) {
      learnedAppMin = app;
    }

    if (countApp >= 5) {
      calibratingAppMin = false;
      //gotAppDuringCalibration = true;

      effectiveAppMin = learnedAppMin + 0.5f;
      appMax = effectiveAppMin + 0.05f * (80.0f - effectiveAppMin);

      // Safety fallback so range stays sane
      if (appMax < effectiveAppMin + 1.0f) {
        appMax = effectiveAppMin + 1.0f;
      }
    }
  }

  if (seen49 && !calibratingAppMin) {
    vin = mapPedalToVoltage(app);
  } else {
    vin = minV;
  }

  pwm = (int)(vin * 255.0f / 3.3f + 0.5f);
  if (pwm < 0) pwm = 0;
  if (pwm > pwmMax) pwm = pwmMax;
}

void processCanFrames() {
  twai_message_t message;

  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd) continue;
    if (message.data_length_code != 8) continue;
    if (message.identifier < 0x7E8 || message.identifier > 0x7EF) continue;

    uint8_t* buf = message.data;

    if (buf[1] != 0x41) continue;  // Mode 01 response

    uint8_t pid = buf[2];
    uint8_t A = buf[3];
    uint8_t B = buf[4];

    if (pid == 0x49) {
      handlePid49(A, B, message.identifier);

      Serial.print("APP resp ID: 0x");
      Serial.print(message.identifier, HEX);
      Serial.print("  APP: ");
      Serial.print(app, 2);
      Serial.print("%  Vin: ");
      Serial.print(vin, 3);
      Serial.print("  PWM: ");
      Serial.println(pwm);
    }
  }
}

const char* twaiStateName(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED: return "STOPPED";
    case TWAI_STATE_RUNNING: return "RUNNING";
    case TWAI_STATE_BUS_OFF: return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default: return "UNKNOWN";
  }
}

void printTwaiStatus(const char* tag) {
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    Serial.printf(
      "[%s] state=%s tx_err=%u rx_err=%u tx_failed=%u bus_err=%u\n",
      tag,
      twaiStateName(st.state),
      st.tx_error_counter,
      st.rx_error_counter,
      st.tx_failed_count,
      st.bus_error_count
    );
  }
}

void recoverTwaiIfNeeded() {
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return;

  if (st.state == TWAI_STATE_BUS_OFF) {
    Serial.println("TWAI BUS-OFF -> initiate recovery");
    twai_initiate_recovery();
  } else if (st.state == TWAI_STATE_STOPPED) {
    Serial.println("TWAI STOPPED -> restart");
    twai_start();
  }
}

void updateDisplay() {
  String line1 = "APP / Pedal";
  String line2 = "APP: " + String(app, 1) + "%";
  String line3 = "Vin: " + String(vin, 3) + "V";
  String line4 = "PWM: " + String(pwm) + " #" + String(n);

  String line5;
  if (calibratingAppMin) {
    line5 = "Cal min... " + String(countApp);
  } else {
    line5 = "Min:" + String(effectiveAppMin, 1) + " Max:" + String(appMax, 1);
  }

  drawOledStatus(line1, line2, line3, line4, line5);
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
    "OBD APP Reader"
  );

  Serial.println();
  Serial.println("Heltec OBD-II CAN Reader Starting...");

  canStarted = startCAN500kNormal();

  if (canStarted) {
    Serial.println("CAN started at 500 kbps, NORMAL mode");
    drawOledStatus(
      "CAN Started",
      "500 kbps",
      "Normal Mode",
      "Request PID 49"
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

  unsigned long now = millis();

  recoverTwaiIfNeeded();


  // Send APP request every 500 ms
  if (now - lastReqMs >= 500) {
    lastReqMs = now;
    sendPid(0x49);   // Accelerator Pedal Position D
  }

  // Read all pending frames
  processCanFrames();

  readTwaiAlerts();

  // Update OLED about 5 times/sec
  if (now - lastDisplayMs >= 200) {
    lastDisplayMs = now;
    n++;
    if (n > 99) n = 0;

    if (!seen49) {
      twai_status_info_t st;
      String stateLine = "State: ?";
      if (twai_get_status_info(&st) == ESP_OK) {
        stateLine = "State: " + String(twaiStateName(st.state));
      }

      drawOledStatus(
        "APP / Pedal",
        "Waiting PID 49...",
        stateLine,
        "Check key ON"
      );
    } else {
      updateDisplay();
    }
  }
}