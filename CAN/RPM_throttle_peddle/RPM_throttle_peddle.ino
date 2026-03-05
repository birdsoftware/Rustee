#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

const uint8_t pids[] = { 0x0C, 0x11, 0x49, 0x4A, 0x4B };
uint8_t pidIndex = 0;
unsigned long lastReqMs = 0;

void sendPid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00 };
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
  Serial.println("OBD Reader: RPM(0C), Throttle(11), APP(49/4A/4B)");
}

void loop() {
  unsigned long now = millis();

  // request ~10 times/sec total, spread across PIDs
  if (now - lastReqMs >= 100) {
    lastReqMs = now;
    uint8_t pid = pids[pidIndex];
    pidIndex = (pidIndex + 1) % (sizeof(pids) / sizeof(pids[0]));
    sendPid(pid);
  }

  if (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t rxId;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) return;
    if (len != 8) return;

    // Expect: [0]=03, [1]=41, [2]=PID, [3..]=data
    if (rxId < 0x7E8 || rxId > 0x7EF) return;
    if (buf[1] != 0x41) return;

    uint8_t pid = buf[2];

    if (pid == 0x0C) {
      // RPM = ((A*256)+B)/4
      uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
      float rpm = raw / 4.0f;
      Serial.print("RPM: ");
      Serial.println(rpm, 0);
    }
    else if (pid == 0x11 || pid == 0x49 || pid == 0x4A || pid == 0x4B) {
      uint8_t raw = buf[3];
      float pct = raw * 100.0f / 255.0f;

      Serial.print("PID 0x");
      if (pid < 0x10) Serial.print("0");
      Serial.print(pid, HEX);
      Serial.print(": ");
      Serial.print(pct, 1);
      Serial.print("% raw=0x");
      if (raw < 0x10) Serial.print("0");
      Serial.println(raw, HEX);
    }
  }
}