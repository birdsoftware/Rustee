#include <SPI.h>
#include <mcp_can.h>

const byte CAN_CS_PIN  = 10;
const byte CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

// ===== change these if needed =====
const uint16_t START_ID = 0x500;
const uint16_t END_ID   = 0x5FF;
// ================================

struct Track {
  uint16_t id;
  byte last[8];
  byte len;
  bool used;
};

const byte MAX_TRACK = 40;
Track tracks[MAX_TRACK];

unsigned long lastStat = 0;
unsigned long rxSec = 0;
unsigned long printedSec = 0;

void printHex2(byte v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

void clearTracks() {
  for (byte i = 0; i < MAX_TRACK; i++) {
    tracks[i].used = false;
    tracks[i].id = 0;
    tracks[i].len = 0;
    for (byte j = 0; j < 8; j++) tracks[i].last[j] = 0;
  }
}

int findTrack(uint16_t id) {
  for (byte i = 0; i < MAX_TRACK; i++) {
    if (tracks[i].used && tracks[i].id == id) return i;
  }
  return -1;
}

int allocTrack(uint16_t id) {
  for (byte i = 0; i < MAX_TRACK; i++) {
    if (!tracks[i].used) {
      tracks[i].used = true;
      tracks[i].id = id;
      tracks[i].len = 0;
      return i;
    }
  }
  return -1;
}

void setup() {
  Serial.begin(115200);
  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);
  clearTracks();

  Serial.println("Ford gear hunter - software filtered");
  Serial.print("Watching IDs 0x");
  Serial.print(START_ID, HEX);
  Serial.print(" to 0x");
  Serial.println(END_ID, HEX);
  Serial.println("Shift slowly: P -> R -> N -> D -> M");
  Serial.println("Hold each about 2 seconds");
  Serial.println("Only 1-2 byte changes are printed");
}

void loop() {
  while (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId = 0;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;

    // only standard frames
    if (rxId > 0x7FF) continue;
    if (len == 0 || len > 8) continue;

    rxSec++;

    uint16_t id = (uint16_t)rxId;

    // software filter ONLY
    if (id < START_ID || id > END_ID) continue;

    int idx = findTrack(id);
    if (idx < 0) idx = allocTrack(id);
    if (idx < 0) continue;

    if (tracks[idx].len == 0) {
      tracks[idx].len = len;
      for (byte i = 0; i < len; i++) tracks[idx].last[i] = buf[i];
      continue;
    }

    byte changed = 0;
    for (byte i = 0; i < len; i++) {
      if (tracks[idx].last[i] != buf[i]) changed++;
    }

    if (changed > 0 && changed <= 2) {
      Serial.print("ID 0x");
      Serial.print(id, HEX);
      Serial.print(" LEN:");
      Serial.print(len);
      Serial.print(" DATA:");
      for (byte i = 0; i < len; i++) {
        Serial.print(" ");
        printHex2(buf[i]);
      }
      Serial.print(" CHG:");
      for (byte i = 0; i < len; i++) {
        if (tracks[idx].last[i] != buf[i]) {
          Serial.print(" b");
          Serial.print(i);
          Serial.print(":");
          printHex2(tracks[idx].last[i]);
          Serial.print("->");
          printHex2(buf[i]);
        }
      }
      Serial.println();
      printedSec++;
    }

    tracks[idx].len = len;
    for (byte i = 0; i < len; i++) tracks[idx].last[i] = buf[i];
  }

  if (millis() - lastStat >= 1000) {
    lastStat = millis();
    Serial.print("rx/sec=");
    Serial.print(rxSec);
    Serial.print(" printed=");
    Serial.println(printedSec);
    rxSec = 0;
    printedSec = 0;
  }
}