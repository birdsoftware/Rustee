#include <SPI.h>
#include <mcp_can.h>

const byte CAN_CS_PIN  = 10;
const byte CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

byte lastData[8];
bool haveLast = false;

void print151(byte *buf) {
  Serial.print("0x151: ");
  for (byte i = 0; i < 8; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.print("   byte4=");
  if (buf[4] < 0x10) Serial.print("0");
  Serial.println(buf[4], HEX);
}

bool changed(byte *buf) {
  if (!haveLast) return true;
  for (byte i = 0; i < 8; i++) {
    if (buf[i] != lastData[i]) return true;
  }
  return false;
}

void save(byte *buf) {
  for (byte i = 0; i < 8; i++) lastData[i] = buf[i];
  haveLast = true;
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
  Serial.println("Watching 0x151 only");
}

void loop() {
  while (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;
    rxId &= 0x1FFFFFFF;

    if (rxId == 0x151 && len == 8) {
      if (changed(buf)) {
        print151(buf);
        save(buf);
      }
    }
  }
}