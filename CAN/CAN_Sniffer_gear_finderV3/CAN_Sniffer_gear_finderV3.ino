#include <SPI.h>
#include <mcp_can.h>

const byte CAN_CS_PIN  = 10;
const byte CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

String lastPrintedGear = "";
String pendingGear = "";
unsigned long gearChangedTime = 0;
const unsigned long settleTimeMs = 150;

String decodeGear(byte *buf) {
  switch (buf[4]) {
    case 0x00: return "P";
    case 0x20: return "R";
    case 0x40: return "N";
    case 0x60: return "D";
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
      String gear = decodeGear(buf);
      if (gear == "?") continue;

      if (gear != pendingGear) {
        pendingGear = gear;
        gearChangedTime = millis();
      }
    }
  }

  if (pendingGear != "" &&
      pendingGear != lastPrintedGear &&
      millis() - gearChangedTime >= settleTimeMs) {
    Serial.print("Gear: ");
    Serial.println(pendingGear);
    lastPrintedGear = pendingGear;
  }
}