#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

void setup() {
  Serial.begin(115200);
  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN INIT FAIL");
    while (1);
  }

  CAN0.setMode(MCP_LISTENONLY);
  Serial.println("Listening for J1939 APP...");
}

void loop() {
  if (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) return;
    if (len != 8) return;

    uint32_t pgn = (rxId >> 8) & 0xFFFF;

    if (pgn == 0xF003) {
      float app = buf[1] * 0.4f;

      Serial.print("PGN F003 APP: ");
      Serial.print(app, 1);
      Serial.println(" %");
    }
  }
}