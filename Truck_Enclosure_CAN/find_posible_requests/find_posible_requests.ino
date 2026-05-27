#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastHeader = 0;

void printByte2(byte b) {
  if (b < 0x10) Serial.print("0");
  Serial.print(b, HEX);
}

float readFloatBE(byte *b) {
  union {
    uint32_t u;
    float f;
  } val;

  val.u =
    ((uint32_t)b[0] << 24) |
    ((uint32_t)b[1] << 16) |
    ((uint32_t)b[2] << 8)  |
    ((uint32_t)b[3]);

  return val.f;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(CAN_INT_PIN, INPUT_PULLUP);

  while (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN INIT FAIL");
    delay(1000);
  }

  CAN0.setMode(MCP_LISTENONLY);

  Serial.println("CAN init OK");
  Serial.println("Logging ALL request/response CAN frames...");
  Serial.println();
  Serial.println("RXID      DATA                          FLOAT_DECODE");
  Serial.println("----------------------------------------------------");
}

void loop() {
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) return;

    rxId &= 0x1FFFFFFF;

    Serial.print("0x");
    Serial.print(rxId, HEX);
    Serial.print("   ");

    for (byte i = 0; i < len; i++) {
      printByte2(buf[i]);
      Serial.print(" ");
    }

    for (byte i = len; i < 8; i++) {
      Serial.print("   ");
    }

    // Decode likely OpenECU response:
    // FF 00 [transaction] [float byte0] [float byte1] [float byte2] [float byte3] xx
    if ((rxId == 0x6EE || rxId == 0x6F8) &&
        len == 8 &&
        buf[0] == 0xFF &&
        buf[1] == 0x00) {

      float value = readFloatBE(&buf[3]);

      Serial.print("   RESP tx=0x");
      printByte2(buf[2]);

      Serial.print(" floatHex=");
      printByte2(buf[3]);
      printByte2(buf[4]);
      printByte2(buf[5]);
      printByte2(buf[6]);

      Serial.print(" value=");
      Serial.print(value, 5);
    }

    // Print possible request frames too
    if (rxId != 0x6EE && rxId != 0x6F8) {
      Serial.print("   POSSIBLE REQUEST/BROADCAST");
    }

    Serial.println();
  }

  if (millis() - lastHeader > 10000) {
    lastHeader = millis();
    Serial.println();
    Serial.println("RXID      DATA                          FLOAT_DECODE");
    Serial.println("----------------------------------------------------");
  }
}