#include <SPI.h>
#include <mcp_can.h>

const byte CAN_CS_PIN  = 10;
const byte CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

const uint16_t ids[] = {
  0x151, 0x167, 0x178, 0x204, 0x230, 0x242, 0x2D0, 0x3D0, 0x3E1, 0x416, 0x43D, 0x43E
};
const byte ID_COUNT = sizeof(ids) / sizeof(ids[0]);

byte lastData[ID_COUNT][8];
byte lastLen[ID_COUNT];
bool seen[ID_COUNT];

unsigned long lastStat = 0;
unsigned long rxCount = 0;
unsigned long hitCount = 0;

int findId(uint16_t id) {
  for (byte i = 0; i < ID_COUNT; i++) {
    if (ids[i] == id) return i;
  }
  return -1;
}

void printHex2(byte v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

void setup() {
  Serial.begin(115200);
  pinMode(CAN_INT_PIN, INPUT);

  memset(seen, 0, sizeof(seen));
  memset(lastLen, 0, sizeof(lastLen));

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL");
    while (1) {}
  }

  CAN0.setMode(MCP_LISTENONLY);
  Serial.println("HS-CAN Ford PRNDL candidate watcher");
  Serial.println("Shift P -> R -> N -> D and hold each");
}

void loop() {
  while (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId = 0;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;
    rxId &= 0x7FF;   // standard IDs only
    rxCount++;

    int idx = findId((uint16_t)rxId);
    if (idx < 0) continue;

    bool changed = false;

    if (!seen[idx] || lastLen[idx] != len) {
      changed = true;
    } else {
      for (byte i = 0; i < len; i++) {
        if (buf[i] != lastData[idx][i]) {
          changed = true;
          break;
        }
      }
    }

    if (changed) {
      hitCount++;
      Serial.print("ID 0x");
      Serial.print(rxId, HEX);
      Serial.print(" LEN:");
      Serial.print(len);
      Serial.print(" DATA:");
      for (byte i = 0; i < len; i++) {
        Serial.print(" ");
        printHex2(buf[i]);
      }
      Serial.println();

      for (byte i = 0; i < len; i++) lastData[idx][i] = buf[i];
      lastLen[idx] = len;
      seen[idx] = true;
    }
  }

  if (millis() - lastStat >= 1000) {
    lastStat = millis();
    Serial.print("rx/sec=");
    Serial.print(rxCount);
    Serial.print(" interesting=");
    Serial.println(hitCount);
    rxCount = 0;
    hitCount = 0;
  }
}