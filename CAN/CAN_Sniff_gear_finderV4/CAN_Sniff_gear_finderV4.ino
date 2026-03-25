#include <SPI.h>
#include <mcp_can.h>

const byte CAN_CS_PIN  = 10;
const byte CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

String lastPrintedGear = "";

bool isIgnoreFrame(const byte *buf) {
  return (buf[1] == 0x00 && buf[4] == 0x60);
}

String decodeGear(const byte *buf) {
  switch (buf[4]) {
    case 0x00: return "P";
    case 0x20: return "R";
    case 0x40: return "N";
    case 0x60:
      // Only accept as Drive when byte2 indicates valid gear frame
      if (buf[1] == 0x10) return "D";
      return "?";
    case 0x80: return "M";
    case 0xA0: return "1";
    case 0xC0: return "2";
    default:   return "?";
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init fail");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("Watching gear on 0x151");
}

void loop() {
  while (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;
    rxId &= 0x1FFFFFFF;

    if (rxId == 0x151 && len == 8) {
      if (isIgnoreFrame(buf)) continue;

      String gear = decodeGear(buf);
      if (gear == "?") continue;

      if (gear != lastPrintedGear) {
        Serial.print("Gear: ");
        Serial.println(gear);
        lastPrintedGear = gear;
      }
    }
  }
}