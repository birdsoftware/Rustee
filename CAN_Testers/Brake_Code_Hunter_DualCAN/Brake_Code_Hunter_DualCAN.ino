#include <SPI.h>
#include <mcp_can.h>

// =====================================================
// Brake Code Hunter - Dual CAN
// Arduino Mega + 2x MCP2515
//
// Purpose:
//   Help find the brake signal CAN message by watching
//   which IDs/bytes toggle repeatedly during brake press/release.
//
// Test procedure:
//   1) Key ON
//   2) Sit still
//   3) Release brake for 5 sec
//   4) Press brake for 2 sec
//   5) Release for 2 sec
//   6) Repeat 8-10 times
//
// Wiring based on your working dual-module setup.
// =====================================================

// =========================
// Pin assignments
// =========================
const uint8_t HS_CS_PIN   = 10;
const uint8_t HS_INT_PIN  = 2;
const uint8_t MS_CS_PIN   = 9;
const uint8_t MS_INT_PIN  = 3;

const uint8_t HS_LED_PIN  = 7;
const uint8_t MS_LED_PIN  = 8;

// =========================
// MCP2515 objects
// =========================
MCP_CAN CAN_HS(HS_CS_PIN);
MCP_CAN CAN_MS(MS_CS_PIN);

// =========================
// CAN settings
// =========================
#define HS_CAN_SPEED   CAN_500KBPS
#define MS_CAN_SPEED   CAN_500KBPS
#define MCP_CLOCK      MCP_8MHZ

// =========================
// Logging controls
// =========================
const unsigned long HEARTBEAT_MS         = 3000;
const unsigned long SUMMARY_MS           = 5000;
const unsigned long MIN_PRINT_GAP_MS     = 60;
const unsigned long LED_ON_MS            = 20;

const bool SHOW_STANDARD_IDS             = true;
const bool SHOW_EXTENDED_IDS             = true;

// Focus on MS body bus first
const bool ENABLE_HS                     = true;
const bool ENABLE_MS                     = true;

// If true, only print IDs that changed at least this many times
const bool USE_MIN_CHANGE_FILTER         = true;
const unsigned long MIN_CHANGE_PRINT     = 4;

// =========================
// Tracking
// =========================
struct FrameState {
  bool used;
  uint32_t id;
  bool ext;
  uint8_t len;
  uint8_t data[8];
  unsigned long lastPrintMs;

  // brake-hunt stats
  uint16_t seenCount;
  uint16_t changeCount;
  uint16_t byteChangeCount[8];
};

const int MAX_TRACKED_IDS = 80;
FrameState hsStates[MAX_TRACKED_IDS];
FrameState msStates[MAX_TRACKED_IDS];

// =========================
// Stats
// =========================
unsigned long lastHeartbeatMs = 0;
unsigned long lastSummaryMs = 0;

unsigned long hsFrameCount = 0;
unsigned long hsChangedCount = 0;
unsigned long hsDroppedCount = 0;

unsigned long msFrameCount = 0;
unsigned long msChangedCount = 0;
unsigned long msDroppedCount = 0;

unsigned long hsLedOffAt = 0;
unsigned long msLedOffAt = 0;

// =====================================================
// Helpers
// =====================================================
void clearStates(FrameState *states, int count) {
  for (int i = 0; i < count; i++) {
    states[i].used = false;
    states[i].id = 0;
    states[i].ext = false;
    states[i].len = 0;
    states[i].lastPrintMs = 0;
    states[i].seenCount = 0;
    states[i].changeCount = 0;
    for (int j = 0; j < 8; j++) {
      states[i].data[j] = 0;
      states[i].byteChangeCount[j] = 0;
    }
  }
}

void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print("0");
  Serial.print(b, HEX);
}

void printHexId(uint32_t id) {
  Serial.print("0x");
  Serial.print(id, HEX);
}

int findStateIndex(FrameState *states, uint32_t id, bool ext) {
  for (int i = 0; i < MAX_TRACKED_IDS; i++) {
    if (states[i].used && states[i].id == id && states[i].ext == ext) {
      return i;
    }
  }
  return -1;
}

int allocStateIndex(FrameState *states, uint32_t id, bool ext) {
  for (int i = 0; i < MAX_TRACKED_IDS; i++) {
    if (!states[i].used) {
      states[i].used = true;
      states[i].id = id;
      states[i].ext = ext;
      states[i].len = 0;
      states[i].lastPrintMs = 0;
      states[i].seenCount = 0;
      states[i].changeCount = 0;
      for (int j = 0; j < 8; j++) {
        states[i].data[j] = 0;
        states[i].byteChangeCount[j] = 0;
      }
      return i;
    }
  }
  return -1;
}

int countTracked(FrameState *states) {
  int c = 0;
  for (int i = 0; i < MAX_TRACKED_IDS; i++) {
    if (states[i].used) c++;
  }
  return c;
}

void flashLed(uint8_t pin, unsigned long &offAt, unsigned long now) {
  digitalWrite(pin, HIGH);
  offAt = now + LED_ON_MS;
}

void serviceLedOff(unsigned long now) {
  if (hsLedOffAt && now >= hsLedOffAt) {
    digitalWrite(HS_LED_PIN, LOW);
    hsLedOffAt = 0;
  }
  if (msLedOffAt && now >= msLedOffAt) {
    digitalWrite(MS_LED_PIN, LOW);
    msLedOffAt = 0;
  }
}

void printBanner() {
  Serial.println();
  Serial.println(F("===== BRAKE CODE HUNTER ====="));
  Serial.println(F("Goal: find brake-related CAN message"));
  Serial.println(F("Test: press/release brake repeatedly while stationary"));
  Serial.println(F("Watch summary for IDs with repeatable toggles"));
  Serial.println(F("Prefer MS-CAN candidates first"));
  Serial.println(F("================================"));
  Serial.println();
}

void printFrameDiff(
  const char *busName,
  uint32_t id,
  bool ext,
  uint8_t len,
  const uint8_t *oldData,
  const uint8_t *newData,
  unsigned long changeCount
) {
  Serial.print(busName);
  Serial.print(" CHG ");
  printHexId(id);
  Serial.print(ext ? " EXT " : " STD ");
  Serial.print("DLC:");
  Serial.print(len);
  Serial.print(" Data:");

  for (uint8_t i = 0; i < len; i++) {
    Serial.print(" ");
    printHexByte(newData[i]);
  }

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
  if (!any) Serial.print(" none");

  Serial.print("  changes=");
  Serial.println(changeCount);
}

void processOneBus(
  MCP_CAN &canDev,
  uint8_t intPin,
  uint8_t ledPin,
  unsigned long &ledOffAt,
  FrameState *states,
  const char *busName,
  unsigned long &frameCount,
  unsigned long &changedCount,
  unsigned long &droppedCount,
  unsigned long now
) {
  while (digitalRead(intPin) == LOW) {
    unsigned long rxId = 0;
    unsigned char len = 0;
    unsigned char buf[8];

    if (canDev.readMsgBuf(&rxId, &len, buf) != CAN_OK) {
      break;
    }

    frameCount++;
    flashLed(ledPin, ledOffAt, now);

    bool ext = (rxId & 0x80000000UL) == 0x80000000UL;
    uint32_t cleanId = ext ? (rxId & 0x1FFFFFFFUL) : rxId;

    if ((ext && !SHOW_EXTENDED_IDS) || (!ext && !SHOW_STANDARD_IDS)) {
      continue;
    }

    if (len > 8) len = 8;

    int idx = findStateIndex(states, cleanId, ext);
    bool firstSeen = false;

    if (idx < 0) {
      idx = allocStateIndex(states, cleanId, ext);
      if (idx < 0) {
        droppedCount++;
        continue;
      }
      firstSeen = true;
    }

    FrameState &s = states[idx];
    s.seenCount++;

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
      changedCount++;

      uint8_t oldData[8];
      for (uint8_t i = 0; i < 8; i++) oldData[i] = s.data[i];

      for (uint8_t i = 0; i < len; i++) {
        if (firstSeen || oldData[i] != buf[i]) {
          s.byteChangeCount[i]++;
        }
      }

      s.changeCount++;

      if (!firstSeen && (now - s.lastPrintMs >= MIN_PRINT_GAP_MS)) {
        if (!USE_MIN_CHANGE_FILTER || s.changeCount >= MIN_CHANGE_PRINT) {
          printFrameDiff(busName, cleanId, ext, len, oldData, buf, s.changeCount);
          s.lastPrintMs = now;
        }
      }

      s.len = len;
      for (uint8_t i = 0; i < len; i++) {
        s.data[i] = buf[i];
      }
    }
  }
}

void printTopCandidates(FrameState *states, const char *busName) {
  Serial.print(F("Top candidates on "));
  Serial.println(busName);

  for (int pass = 0; pass < 8; pass++) {
    int bestIdx = -1;
    unsigned long bestScore = 0;

    for (int i = 0; i < MAX_TRACKED_IDS; i++) {
      if (!states[i].used) continue;

      unsigned long score = states[i].changeCount;
      if (score == 0) continue;

      bool alreadyPicked = false;
      for (int j = 0; j < pass; j++) {
        // crude skip by zeroing later with sentinel not needed here because we print only few
      }

      if (score > bestScore) {
        bool duplicate = false;
        for (int k = 0; k < pass; k++) {
          // nothing persistent, handled below by marking
        }
        if (!duplicate) {
          bestScore = score;
          bestIdx = i;
        }
      }
    }

    if (bestIdx < 0) break;

    FrameState &s = states[bestIdx];

    Serial.print("  ");
    printHexId(s.id);
    Serial.print(s.ext ? " EXT " : " STD ");
    Serial.print(" seen=");
    Serial.print(s.seenCount);
    Serial.print(" changes=");
    Serial.print(s.changeCount);
    Serial.print(" byteHits:");

    for (uint8_t b = 0; b < 8; b++) {
      if (s.byteChangeCount[b] > 0) {
        Serial.print(" b");
        Serial.print(b);
        Serial.print("=");
        Serial.print(s.byteChangeCount[b]);
      }
    }

    Serial.print(" last:");
    for (uint8_t b = 0; b < s.len; b++) {
      Serial.print(" ");
      printHexByte(s.data[b]);
    }
    Serial.println();

    // mark as printed by zeroing temporary score
    s.changeCount = 0xFFFFFFFFUL - s.changeCount; // reversible marker
  }

  // restore marked scores
  for (int i = 0; i < MAX_TRACKED_IDS; i++) {
    if (!states[i].used) continue;
    if (states[i].changeCount > 0x80000000UL) {
      states[i].changeCount = 0xFFFFFFFFUL - states[i].changeCount;
    }
  }
}

void printSummary() {
  Serial.println();
  Serial.println(F("===== BRAKE HUNTER SUMMARY ====="));

  Serial.print(F("HS frames="));
  Serial.print(hsFrameCount);
  Serial.print(F(" changed="));
  Serial.print(hsChangedCount);
  Serial.print(F(" tracked="));
  Serial.print(countTracked(hsStates));
  Serial.print(F(" dropped="));
  Serial.println(hsDroppedCount);

  Serial.print(F("MS frames="));
  Serial.print(msFrameCount);
  Serial.print(F(" changed="));
  Serial.print(msChangedCount);
  Serial.print(F(" tracked="));
  Serial.print(countTracked(msStates));
  Serial.print(F(" dropped="));
  Serial.println(msDroppedCount);

  if (ENABLE_MS) printTopCandidates(msStates, "MS");
  if (ENABLE_HS) printTopCandidates(hsStates, "HS");

  Serial.println(F("Tip: likely brake IDs will toggle repeatedly with the same byte/bit pattern."));
  Serial.println(F("================================"));
  Serial.println();
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(HS_INT_PIN, INPUT);
  pinMode(MS_INT_PIN, INPUT);

  pinMode(HS_LED_PIN, OUTPUT);
  pinMode(MS_LED_PIN, OUTPUT);
  digitalWrite(HS_LED_PIN, LOW);
  digitalWrite(MS_LED_PIN, LOW);

  // shared SPI safety
  pinMode(HS_CS_PIN, OUTPUT);
  pinMode(MS_CS_PIN, OUTPUT);
  digitalWrite(HS_CS_PIN, HIGH);
  digitalWrite(MS_CS_PIN, HIGH);
  delay(10);

  clearStates(hsStates, MAX_TRACKED_IDS);
  clearStates(msStates, MAX_TRACKED_IDS);

  Serial.println(F("Starting dual MCP2515 Brake Code Hunter..."));

  if (CAN_HS.begin(MCP_ANY, HS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
    Serial.println(F("HS CAN init FAIL"));
    Serial.println(F("Try MCP_16MHZ or different HS speed"));
    while (1) {}
  }

  if (CAN_MS.begin(MCP_ANY, MS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
    Serial.println(F("MS CAN init FAIL"));
    Serial.println(F("Try MCP_16MHZ or different MS speed"));
    while (1) {}
  }

  CAN_HS.setMode(MCP_NORMAL);
  CAN_MS.setMode(MCP_NORMAL);

  printBanner();
  Serial.println(F("HS and MS brake hunt sniffers ready."));
}

// =========================
// Loop
// =========================
void loop() {
  unsigned long now = millis();

  if (ENABLE_HS) {
    processOneBus(
      CAN_HS,
      HS_INT_PIN,
      HS_LED_PIN,
      hsLedOffAt,
      hsStates,
      "HS",
      hsFrameCount,
      hsChangedCount,
      hsDroppedCount,
      now
    );
  }

  if (ENABLE_MS) {
    processOneBus(
      CAN_MS,
      MS_INT_PIN,
      MS_LED_PIN,
      msLedOffAt,
      msStates,
      "MS",
      msFrameCount,
      msChangedCount,
      msDroppedCount,
      now
    );
  }

  serviceLedOff(now);

  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;

    Serial.print(F("HB HS frames="));
    Serial.print(hsFrameCount);
    Serial.print(F(" changed="));
    Serial.print(hsChangedCount);
    Serial.print(F(" tracked="));
    Serial.print(countTracked(hsStates));

    Serial.print(F(" | MS frames="));
    Serial.print(msFrameCount);
    Serial.print(F(" changed="));
    Serial.print(msChangedCount);
    Serial.print(F(" tracked="));
    Serial.println(countTracked(msStates));
  }

  if (now - lastSummaryMs >= SUMMARY_MS) {
    lastSummaryMs = now;
    printSummary();
  }
}