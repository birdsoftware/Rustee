#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

uint8_t lastGear = 0xFF;

const char* decodePRND(uint8_t v) {
  switch (v) {
    case 0x80: return "P";
    case 0x04: return "R";
    case 0x88: return "N";
    case 0x10: return "D";
    default:   return "-";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN FAIL");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);

  Serial.println("PRND READY (ID 0x170 byte0)");
}

void loop() {
  while (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t id;
    uint8_t len;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&id, &len, buf) != CAN_OK) break;

    if (id == 0x170 && len >= 1) {
      uint8_t g = buf[0];

      if (g != lastGear) {
        lastGear = g;

        Serial.print("Gear: ");
        Serial.print(decodePRND(g));

        Serial.print("  Raw:0x");
        Serial.println(g, HEX);
      }
    }
  }
}