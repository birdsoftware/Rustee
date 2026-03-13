#include <SPI.h>
#include <mcp_can.h>

const byte CAN_CS_PIN  = 10;
const byte CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

// Keep only a small cache of IDs to save RAM
const byte CACHE_SIZE = 24;

struct CanEntry {
  unsigned long id;
  byte len;
  byte data[8];
  bool used;
};

CanEntry cache[CACHE_SIZE];

void printFrame(unsigned long rxId, byte len, byte *buf) {
  Serial.print("ID: 0x");
  Serial.print(rxId, HEX);
  Serial.print(" DLC: ");
  Serial.print(len);
  Serial.print(" Data: ");

  for (byte i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

bool sameFrame(byte len1, byte *data1, byte len2, byte *data2) {
  if (len1 != len2) return false;
  for (byte i = 0; i < len1; i++) {
    if (data1[i] != data2[i]) return false;
  }
  return true;
}

int findEntry(unsigned long rxId) {
  for (byte i = 0; i < CACHE_SIZE; i++) {
    if (cache[i].used && cache[i].id == rxId) return i;
  }
  return -1;
}

int findFreeEntry() {
  for (byte i = 0; i < CACHE_SIZE; i++) {
    if (!cache[i].used) return i;
  }
  return -1;
}

void saveEntry(byte idx, unsigned long rxId, byte len, byte *buf) {
  cache[idx].used = true;
  cache[idx].id = rxId;
  cache[idx].len = len;
  for (byte i = 0; i < 8; i++) {
    cache[idx].data[i] = (i < len) ? buf[i] : 0;
  }
}

bool changedFromLast(unsigned long rxId, byte len, byte *buf) {
  int idx = findEntry(rxId);

  if (idx >= 0) {
    if (sameFrame(cache[idx].len, cache[idx].data, len, buf)) {
      return false;
    } else {
      saveEntry(idx, rxId, len, buf);
      return true;
    }
  }

  int freeIdx = findFreeEntry();
  if (freeIdx >= 0) {
    saveEntry(freeIdx, rxId, len, buf);
    return true;
  }

  // Cache full: overwrite slot 0, simple fallback
  saveEntry(0, rxId, len, buf);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(CAN_INT_PIN, INPUT);

  for (byte i = 0; i < CACHE_SIZE; i++) {
    cache[i].used = false;
  }

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Init OK");
  } else {
    Serial.println("MCP2515 Init FAIL");
    Serial.println("Check wiring / bitrate / crystal");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("Gear finder sniffer ready");
}

void loop() {
  while (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) {
      break;
    }

    rxId &= 0x1FFFFFFF; // normalize

    // Ignore extended IDs for now if you want:
    // if (rxId > 0x7FF) continue;

    if (changedFromLast(rxId, len, buf)) {
      printFrame(rxId, len, buf);
    }
  }
}