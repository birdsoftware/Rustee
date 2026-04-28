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
  Serial.println("Listening 250kbps 8MHz...");
}

void loop() {
  if (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) return;
    if (len > 8) return;

    Serial.print("ID: 0x");
    Serial.print(rxId, HEX);
    Serial.print(" LEN: ");
    Serial.print(len);
    Serial.print(" DATA: ");

    for (byte i = 0; i < len; i++) {
      if (buf[i] < 0x10) Serial.print("0");
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }

    Serial.println();
  }
}