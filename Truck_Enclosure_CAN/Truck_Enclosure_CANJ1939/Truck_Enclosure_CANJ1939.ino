#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

void setup() {
  Serial.begin(115200);

  while (!Serial);

  pinMode(CAN_INT_PIN, INPUT);

  // Init MCP2515
  if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN INIT FAIL");
    while (1);
  }

  // Listen only mode
  CAN0.setMode(MCP_LISTENONLY);

  Serial.println("CAN init OK");
  Serial.println("Listening for J1939 frames...");
}

void loop() {

  // MCP2515 INT pin goes LOW when message received
  if (digitalRead(CAN_INT_PIN) == LOW) {

    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    // Read CAN frame
    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) {
      return;
    }

    // J1939 decode
    uint8_t priority = (rxId >> 26) & 0x7;
    uint8_t pf       = (rxId >> 16) & 0xFF;
    uint8_t ps       = (rxId >> 8)  & 0xFF;
    uint8_t sa       = rxId & 0xFF;

    uint32_t pgn;

    if (pf < 240)
      pgn = (pf << 8);
    else
      pgn = (pf << 8) | ps;

    // Print frame info
    Serial.print("ID: 0x");
    Serial.print(rxId, HEX);

    Serial.print(" PRI: ");
    Serial.print(priority);

    Serial.print(" PGN: 0x");
    Serial.print(pgn, HEX);

    Serial.print(" SA: 0x");
    Serial.print(sa, HEX);

    Serial.print(" LEN: ");
    Serial.print(len);

    Serial.print(" DATA: ");

    for (int i = 0; i < len; i++) {
      if (buf[i] < 0x10) Serial.print("0");
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }

    Serial.println();

    // Example proprietary frame decoder
    if (rxId == 0x0A100100) {

      Serial.print("Proprietary Frame Data -> ");

      for (int i = 0; i < len; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }

      Serial.println();
    }
  }
}