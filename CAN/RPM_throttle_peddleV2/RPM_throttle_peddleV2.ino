#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastReqMs = 0;
uint8_t pidIndex = 0;

const uint8_t pids[] = { 0x0C, 0x49, 0x4A, 0x11 };

float rpmVal = 0;
float app1 = 0;   // PID 49
float app2 = 0;   // PID 4A
float th11 = 0;   // PID 11

unsigned long lastPrintMs = 0;

static bool calibrated = false;
static float appIdle = 0;

void sendPid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
  CAN0.sendMsgBuf(0x7DF, 0, 8, req);
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

  Serial.println("Reading: RPM(0C), APP(49), APP2(4A), ThrottlePlate(11)");
}

void loop() {
  unsigned long now = millis();

  // request one PID every 50ms -> ~20 requests/sec total
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
      app1 = buf[3] * 100.0f / 255.0f;
    } else if (pid == 0x4A) {
      app2 = buf[3] * 100.0f / 255.0f;
    } else if (pid == 0x11) {
      th11 = buf[3] * 100.0f / 255.0f;
    }
  }

  if (!calibrated && rpmVal < 800) {   // idle-ish and foot off
    appIdle = app1;                    // app1 is APP49%
    calibrated = true;
  }

float appNorm = calibrated ? (app1 - appIdle) : 0;
if (appNorm < 0) appNorm = 0;

  // print at 5 Hz
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;
    Serial.print("RPM:");
    Serial.print(rpmVal, 0);
    Serial.print("  APP49:");
    Serial.print(app1, 1);
    Serial.print("%  APP4A:");
    Serial.print(app2, 1);
    Serial.print("%  TH11:");
    Serial.print(th11, 1);
    Serial.println("%");
  }
}