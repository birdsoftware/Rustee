#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastStatMs = 0;
unsigned long frameCount = 0;
unsigned long totalFrames = 0;
unsigned long errorCount = 0;

void printHexByte(byte v) {
  if (v < 0x10) Serial.print("0");
  Serial.print(v, HEX);
}

void printData(byte len, byte *buf) {
  for (byte i = 0; i < len; i++) {
    printHexByte(buf[i]);
    if (i < len - 1) Serial.print(" ");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Init FAIL @ 125k / 8MHz");
    while (1) {}
  }

  CAN0.setMode(MCP_LISTENONLY);

  Serial.println("MS-CAN raw test");
  Serial.println("CANH->3 CANL->11 GND->5");
}

void loop() {
  while (CAN0.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rawId = 0;
    byte len = 0;
    byte buf[8] = {0};

    if (CAN0.readMsgBuf(&rawId, &len, buf) == CAN_OK) {
      if (len > 8) continue;

      bool isExt = (rawId & 0x80000000UL) != 0;
      bool isRtr = (rawId & 0x40000000UL) != 0;
      unsigned long canId = isExt ? (rawId & 0x1FFFFFFFUL) : (rawId & 0x7FFUL);

      frameCount++;
      totalFrames++;

      Serial.print("ID 0x");
      Serial.print(canId, HEX);
      Serial.print(isExt ? " (EXT) " : " (STD) ");
      Serial.print(isRtr ? "RTR " : "DATA ");
      Serial.print("DLC:");
      Serial.print(len);

      if (!isRtr) {
        Serial.print(" Data: ");
        printData(len, buf);
      }
      Serial.println();
    }
  }

  if (CAN0.checkError() == CAN_CTRLERROR) {
    errorCount++;
  }

  if (millis() - lastStatMs >= 1000) {
    lastStatMs = millis();
    Serial.print("frames/sec=");
    Serial.print(frameCount);
    Serial.print(" total=");
    Serial.print(totalFrames);
    Serial.print(" errors=");
    Serial.println(errorCount);
    frameCount = 0;
  }
}