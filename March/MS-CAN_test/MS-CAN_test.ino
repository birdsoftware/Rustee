#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

struct CanEntry {
  unsigned long id;
  byte len;
  byte data[8];
  bool valid;
};

const byte MAX_IDS = 24;   // Nano-safe
CanEntry table[MAX_IDS];

unsigned long lastStatMs = 0;
unsigned long frameCount = 0;
unsigned long totalFrames = 0;
byte uniqueIdsSeen = 0;

int findSlot(unsigned long id) {
  for (byte i = 0; i < MAX_IDS; i++) {
    if (table[i].valid && table[i].id == id) return i;
  }
  for (byte i = 0; i < MAX_IDS; i++) {
    if (!table[i].valid) return i;
  }
  return -1;
}

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

void printDiff(byte oldLen, byte *oldBuf, byte newLen, byte *newBuf) {
  Serial.print(" diff:");
  byte maxLen = (oldLen > newLen) ? oldLen : newLen;
  for (byte i = 0; i < maxLen; i++) {
    byte a = (i < oldLen) ? oldBuf[i] : 0xFF;
    byte b = (i < newLen) ? newBuf[i] : 0xFF;
    if (a != b) {
      Serial.print(" b");
      Serial.print(i);
      Serial.print(":");
      printHexByte(a);
      Serial.print("->");
      printHexByte(b);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  pinMode(CAN_INT_PIN, INPUT);

  // Ford MS-CAN is commonly 125 kbps.
  // Most HW-184 boards are 8 MHz. If yours is 16 MHz, change MCP_8MHZ to MCP_16MHZ.
  if (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 init FAIL @ 125k / 8MHz");
    Serial.println("Try CAN_125KBPS + MCP_16MHZ, or CAN_250KBPS.");
    while (1) {}
  }

  // Passive sniffing
  CAN0.setMode(MCP_LISTENONLY);

  Serial.println("MS-CAN passive explorer ready");
  Serial.println("Pins: CANH->3 CANL->11 GND->5");
}

void loop() {
  if (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId = 0;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
      frameCount++;
      totalFrames++;

      int idx = findSlot(rxId);

      // If table full, ignore new IDs but still count bus traffic
      if (idx >= 0) {
        bool changed = false;
        bool isNew = !table[idx].valid;

        if (isNew) {
          changed = true;
          uniqueIdsSeen++;
        } else if (table[idx].len != len) {
          changed = true;
        } else {
          for (byte i = 0; i < len; i++) {
            if (table[idx].data[i] != buf[i]) {
              changed = true;
              break;
            }
          }
        }

        if (changed) {
          Serial.print("ID 0x");
          Serial.print(rxId, HEX);

          // crude standard/extended note
          if ((rxId & 0x80000000UL) == 0x80000000UL) {
            Serial.print(" (EXT)");
          } else {
            Serial.print(" (STD)");
          }

          Serial.print(" DLC:");
          Serial.print(len);
          Serial.print(" Data:");
          Serial.print(" ");
          printData(len, buf);

          if (isNew) {
            Serial.print(" NEW");
          } else {
            printDiff(table[idx].len, table[idx].data, len, buf);
          }

          Serial.println();

          table[idx].id = rxId;
          table[idx].len = len;
          for (byte i = 0; i < len; i++) table[idx].data[i] = buf[i];
          table[idx].valid = true;
        }
      }
    }
  }

  if (millis() - lastStatMs >= 1000) {
    lastStatMs = millis();
    Serial.print("frames/sec=");
    Serial.print(frameCount);
    Serial.print("  trackedIDs=");
    Serial.print(uniqueIdsSeen);
    Serial.print("/");
    Serial.print(MAX_IDS);
    Serial.print("  totalFrames=");
    Serial.println(totalFrames);
    frameCount = 0;
  }
}
