#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastReqMs = 0;
uint8_t pidIndex = 0;

const uint8_t pids[] = { 0x0C, 0x49, 0x4A, 0x11 };

float rpmVal = 0;
float app49 = 0;   // raw %
float app4a = 0;   // raw %
float th11 = 0;    // raw %

bool appCalibrated = false;
float appIdle = 0; // baseline raw %

unsigned long lastPrintMs = 0;
unsigned long startMs = 0;

void sendPid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
  CAN0.sendMsgBuf(0x7DF, 0, 8, req);
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 Init FAIL (try MCP_16MHZ or 250kbps)");
    while (1) {}
  }
  CAN0.setMode(MCP_NORMAL);

  startMs = millis();
  Serial.println("RPM + Pedal(APP49 normalized) + ThrottlePlate(11)");
  Serial.println("Keep foot OFF pedal for ~3 seconds to auto-calibrate baseline.");
}

void loop() {
  unsigned long now = millis();

  // request one PID every 50ms
  if (now - lastReqMs >= 50) {
    lastReqMs = now;
    sendPid(pids[pidIndex]);
    pidIndex = (pidIndex + 1) % (sizeof(pids) / sizeof(pids[0]));
  }

  if (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t rxId;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) return;
    if (len != 8) return;
    if (rxId < 0x7E8 || rxId > 0x7EF) return;
    if (buf[1] != 0x41) return;

    uint8_t pid = buf[2];

    if (pid == 0x0C) {
      uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
      rpmVal = raw / 4.0f;
    } else if (pid == 0x49) {
      app49 = buf[3] * 100.0f / 255.0f;
    } else if (pid == 0x4A) {
      app4a = buf[3] * 100.0f / 255.0f;
    } else if (pid == 0x11) {
      th11 = buf[3] * 100.0f / 255.0f;
    }
  }

  // Auto-calibrate baseline during first 3 seconds if RPM ~ idle and pedal steady
  if (!appCalibrated && (now - startMs) > 1500 && (now - startMs) < 4000 && rpmVal > 450 && rpmVal < 900) {
    // simple low-pass average baseline
    if (appIdle == 0) appIdle = app49;
    appIdle = appIdle * 0.85f + app49 * 0.15f;
    if ((now - startMs) > 3500) appCalibrated = true;
  }

  float appNorm = 0;
  if (appCalibrated) {
    float denom = (100.0f - appIdle);
    if (denom < 1.0f) denom = 1.0f;
    appNorm = (app49 - appIdle) * 100.0f / denom;
    appNorm = clampf(appNorm, 0, 100);
  }

  // print at 5 Hz
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;
    Serial.print("RPM:");
    Serial.print(rpmVal, 0);

    Serial.print("  APP49raw:");
    Serial.print(app49, 1);

    Serial.print("%  APP49norm:");
    if (appCalibrated) Serial.print(appNorm, 1);
    else Serial.print("CAL");
    Serial.print("%  TH11:");
    Serial.print(th11, 1);
    Serial.println("%");
  }
}