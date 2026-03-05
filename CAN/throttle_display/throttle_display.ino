#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastReqMs = 0;
int lastRaw = -1;

void sendThrottleRequest() {
  byte req[8] = { 0x02, 0x01, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00 };
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
  Serial.println("OBD Throttle (PID 01 11) ready.");
}

void loop() {
  unsigned long now = millis();

  // 5 Hz request
  if (now - lastReqMs >= 200) {
    lastReqMs = now;
    sendThrottleRequest();
  }

  if (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t rxId;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) return;
    if (len != 8) return;

    if (rxId >= 0x7E8 && rxId <= 0x7EF && buf[1] == 0x41 && buf[2] == 0x11) {
      uint8_t raw = buf[3];
      if (raw != lastRaw) {
        lastRaw = raw;
        float pct = raw * 100.0f / 255.0f;

        Serial.print("Throttle(PID11): ");
        Serial.print(pct, 1);
        Serial.print("% raw=0x");
        if (raw < 0x10) Serial.print("0");
        Serial.println(raw, HEX);
      }
    }
  }
}