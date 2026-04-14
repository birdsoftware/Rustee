#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

// ---------- CAN settings ----------
#define CAN_SPEED CAN_500KBPS      // Try CAN_250KBPS if needed
#define CAN_CLOCK MCP_8MHZ         // Change to MCP_16MHZ if your module uses 16MHz

// ---------- Logging behavior ----------
const unsigned long PRINT_HEARTBEAT_MS = 2000;   // prints a heartbeat
const unsigned long MIN_PRINT_GAP_MS   = 60;     // suppress very fast repeats for same ID
const bool SHOW_STD = true;
const bool SHOW_EXT = true;

// ---------- Track last frame per ID ----------
struct FrameState {
  bool used;
  uint32_t id;
  bool ext;
  uint8_t len;
  uint8_t data[8];
  unsigned long lastPrintMs;
};

const int MAX_TRACKED_IDS = 180;
FrameState states[MAX_TRACKED_IDS];

unsigned long lastHeartbeatMs = 0;
unsigned long frameCount = 0;
unsigned long changedCount = 0;
unsigned long droppedTrackCount = 0;

// ---------- Helpers ----------
void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print("0");
  Serial.print(b, HEX);
}

void printHexId(uint32_t id) {
  Serial.print("0x");
  Serial.print(id, HEX);
}

int findStateIndex(uint32_t id, bool ext) {
  for (int i = 0; i < MAX_TRACKED_IDS; i++) {
    if (states[i].used && states[i].id == id && states[i].ext == ext) {
      return i;
    }
  }
  return -1;
}

int allocStateIndex(uint32_t id, bool ext) {
  for (int i = 0; i < MAX_TRACKED_IDS; i++) {
    if (!states[i].used) {
      states[i].used = true;
      states[i].id = id;
      states[i].ext = ext;
      states[i].len = 0;
      states[i].lastPrintMs = 0;
      for (int j = 0; j < 8; j++) states[i].data[j] = 0;
      return i;
    }
  }
  return -1;
}

void printFrameLine(
  uint32_t id,
  bool ext,
  uint8_t len,
  const uint8_t* oldData,
  const uint8_t* newData,
  bool firstSeen
) {
  Serial.print(firstSeen ? "NEW " : "CHG ");
  printHexId(id);
  Serial.print(ext ? " EXT " : " STD ");
  Serial.print("DLC:");
  Serial.print(len);
  Serial.print(" Data:");

  for (uint8_t i = 0; i < len; i++) {
    Serial.print(" ");
    printHexByte(newData[i]);
  }

  if (!firstSeen) {
    Serial.print("  Diff:");
    bool any = false;
    for (uint8_t i = 0; i < len; i++) {
      if (oldData[i] != newData[i]) {
        any = true;
        Serial.print(" [");
        Serial.print(i);
        Serial.print(":");
        printHexByte(oldData[i]);
        Serial.print("->");
        printHexByte(newData[i]);
        Serial.print("]");
      }
    }
    if (!any) {
      Serial.print(" none");
    }
  }

  Serial.println();
}

void printBanner() {
  Serial.println();
  Serial.println(F("===== RAW CAN GEAR HUNTER ====="));
  Serial.println(F("Procedure:"));
  Serial.println(F("1) Key ON / engine idle"));
  Serial.println(F("2) Hold each gear for ~5 sec: P -> R -> N -> D"));
  Serial.println(F("3) Watch for IDs with one/few bytes changing predictably"));
  Serial.println(F("4) If quiet, try 250 kbps or another CAN pair"));
  Serial.println(F("================================"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_SPEED, CAN_CLOCK) != CAN_OK) {
    Serial.println(F("MCP2515 Init FAIL"));
    Serial.println(F("Try changing MCP_8MHZ <-> MCP_16MHZ or 500k <-> 250k"));
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);

  // Clear tracking table
  for (int i = 0; i < MAX_TRACKED_IDS; i++) {
    states[i].used = false;
  }

  printBanner();
  Serial.println(F("CAN sniffer started."));
}

void loop() {
  unsigned long now = millis();

  while (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId = 0;
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) {
      continue;
    }

    frameCount++;

    bool ext = (rxId & 0x80000000UL) == 0x80000000UL;
    uint32_t cleanId = ext ? (rxId & 0x1FFFFFFFUL) : rxId;

    if ((ext && !SHOW_EXT) || (!ext && !SHOW_STD)) {
      continue;
    }

    int idx = findStateIndex(cleanId, ext);
    bool firstSeen = false;

    if (idx < 0) {
      idx = allocStateIndex(cleanId, ext);
      if (idx < 0) {
        droppedTrackCount++;
        continue;
      }
      firstSeen = true;
    }

    FrameState &s = states[idx];

    // Normalize len
    if (len > 8) len = 8;

    bool changed = firstSeen || (s.len != len);
    if (!changed) {
      for (uint8_t i = 0; i < len; i++) {
        if (s.data[i] != buf[i]) {
          changed = true;
          break;
        }
      }
    }

    if (changed) {
      if (firstSeen || (now - s.lastPrintMs >= MIN_PRINT_GAP_MS)) {
        printFrameLine(cleanId, ext, len, s.data, buf, firstSeen);
        s.lastPrintMs = now;
        changedCount++;
      }

      s.len = len;
      for (uint8_t i = 0; i < len; i++) {
        s.data[i] = buf[i];
      }
    }
  }

  if (now - lastHeartbeatMs >= PRINT_HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    Serial.print(F("HB frames="));
    Serial.print(frameCount);
    Serial.print(F(" changed="));
    Serial.print(changedCount);
    Serial.print(F(" tracked="));
    int tracked = 0;
    for (int i = 0; i < MAX_TRACKED_IDS; i++) {
      if (states[i].used) tracked++;
    }
    Serial.print(tracked);
    Serial.print(F(" dropped="));
    Serial.println(droppedTrackCount);
  }
}