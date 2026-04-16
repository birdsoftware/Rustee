#include <SPI.h>
#include <mcp_can.h>

// ==========================================
// Brake Code Filtered
// Arduino Mega + 2x MCP2515
//
// Focused brake hunter:
// only prints a small set of likely brake IDs
//
// Current watch list:
//   0x204  (best candidate)
//   0x4B0  (strong candidate)
//   0x77   (possible)
//   0x82   (possible)
//
// Recommended test:
//   1) Key ON, vehicle parked
//   2) 2 sec brake released
//   3) 2 sec brake pressed
//   4) Repeat 10 times
// ==========================================

const uint8_t HS_CS_PIN   = 10;
const uint8_t HS_INT_PIN  = 2;
const uint8_t MS_CS_PIN   = 9;
const uint8_t MS_INT_PIN  = 3;

const uint8_t HS_LED_PIN  = 7;
const uint8_t MS_LED_PIN  = 8;

MCP_CAN CAN_HS(HS_CS_PIN);
MCP_CAN CAN_MS(MS_CS_PIN);

#define HS_CAN_SPEED   CAN_500KBPS
#define MS_CAN_SPEED   CAN_500KBPS
#define MCP_CLOCK      MCP_8MHZ

const bool ENABLE_HS = true;
const bool ENABLE_MS = true;

const unsigned long HEARTBEAT_MS = 3000;
const unsigned long LED_ON_MS    = 20;

// ------------------------------------------
// Only track these IDs
// ------------------------------------------
const uint32_t WATCH_IDS[] = { 0x204, 0x4B0, 0x77, 0x82 };
const uint8_t NUM_WATCH_IDS = sizeof(WATCH_IDS) / sizeof(WATCH_IDS[0]);

struct WatchState {
  bool used;
  uint32_t id;
  uint8_t lastLen;
  uint8_t lastData[8];
  uint16_t changes;
};

WatchState hsWatch[NUM_WATCH_IDS];
WatchState msWatch[NUM_WATCH_IDS];

unsigned long lastHeartbeatMs = 0;
unsigned long hsFrameCount = 0;
unsigned long msFrameCount = 0;
unsigned long hsLedOffAt = 0;
unsigned long msLedOffAt = 0;

// ==========================================
// Helpers
// ==========================================
void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print("0");
  Serial.print(b, HEX);
}

void printHexId(uint32_t id) {
  Serial.print("0x");
  Serial.print(id, HEX);
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

bool isWatchedId(uint32_t id) {
  for (uint8_t i = 0; i < NUM_WATCH_IDS; i++) {
    if (WATCH_IDS[i] == id) return true;
  }
  return false;
}

int watchIndex(uint32_t id) {
  for (uint8_t i = 0; i < NUM_WATCH_IDS; i++) {
    if (WATCH_IDS[i] == id) return i;
  }
  return -1;
}

void initWatchArray(WatchState *arr) {
  for (uint8_t i = 0; i < NUM_WATCH_IDS; i++) {
    arr[i].used = false;
    arr[i].id = WATCH_IDS[i];
    arr[i].lastLen = 0;
    arr[i].changes = 0;
    for (uint8_t j = 0; j < 8; j++) arr[i].lastData[j] = 0;
  }
}

void printBanner() {
  Serial.println();
  Serial.println(F("===== BRAKE CODE FILTERED ====="));
  Serial.println(F("Watching only: 0x204, 0x4B0, 0x77, 0x82"));
  Serial.println(F("Best current brake candidate: 0x204 byte 4"));
  Serial.println(F("Test: press/release brake repeatedly while parked"));
  Serial.println(F("================================"));
  Serial.println();
}

void printDiffLine(const char *busName, uint32_t id, uint8_t len, const uint8_t *oldData, const uint8_t *newData, uint16_t changes) {
  Serial.print(busName);
  Serial.print(" ");
  printHexId(id);
  Serial.print(" chg=");
  Serial.print(changes);
  Serial.print(" data:");

  for (uint8_t i = 0; i < len; i++) {
    Serial.print(" ");
    printHexByte(newData[i]);
  }

  Serial.print(" diff:");
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

  Serial.println();
}

void printSummary(const char *busName, WatchState *arr) {
  Serial.print(busName);
  Serial.println(" summary:");
  for (uint8_t i = 0; i < NUM_WATCH_IDS; i++) {
    Serial.print("  ");
    printHexId(arr[i].id);
    Serial.print(" changes=");
    Serial.print(arr[i].changes);
    Serial.print(" last:");
    for (uint8_t j = 0; j < arr[i].lastLen; j++) {
      Serial.print(" ");
      printHexByte(arr[i].lastData[j]);
    }
    Serial.println();
  }
}

void processBus(
  MCP_CAN &canDev,
  uint8_t intPin,
  uint8_t ledPin,
  unsigned long &ledOffAt,
  WatchState *watchArr,
  const char *busName,
  unsigned long &frameCount
) {
  while (digitalRead(intPin) == LOW) {
    unsigned long rxId = 0;
    unsigned char len = 0;
    unsigned char buf[8];

    if (canDev.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;

    frameCount++;
    flashLed(ledPin, ledOffAt, millis());

    bool ext = (rxId & 0x80000000UL) == 0x80000000UL;
    if (ext) continue; // focus on standard IDs only

    uint32_t cleanId = rxId;
    if (!isWatchedId(cleanId)) continue;
    if (len > 8) len = 8;

    int idx = watchIndex(cleanId);
    if (idx < 0) continue;

    WatchState &w = watchArr[idx];

    if (!w.used) {
      w.used = true;
      w.lastLen = len;
      for (uint8_t i = 0; i < len; i++) w.lastData[i] = buf[i];

      Serial.print(busName);
      Serial.print(" INIT ");
      printHexId(cleanId);
      Serial.print(" data:");
      for (uint8_t i = 0; i < len; i++) {
        Serial.print(" ");
        printHexByte(buf[i]);
      }
      Serial.println();
      continue;
    }

    bool changed = (w.lastLen != len);
    if (!changed) {
      for (uint8_t i = 0; i < len; i++) {
        if (w.lastData[i] != buf[i]) {
          changed = true;
          break;
        }
      }
    }

    if (changed) {
      uint8_t oldData[8];
      for (uint8_t i = 0; i < 8; i++) oldData[i] = w.lastData[i];

      if (w.changes < 65535) w.changes++;
      printDiffLine(busName, cleanId, len, oldData, buf, w.changes);

      w.lastLen = len;
      for (uint8_t i = 0; i < len; i++) w.lastData[i] = buf[i];
    }
  }
}

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(HS_INT_PIN, INPUT);
  pinMode(MS_INT_PIN, INPUT);

  pinMode(HS_LED_PIN, OUTPUT);
  pinMode(MS_LED_PIN, OUTPUT);
  digitalWrite(HS_LED_PIN, LOW);
  digitalWrite(MS_LED_PIN, LOW);

  pinMode(HS_CS_PIN, OUTPUT);
  pinMode(MS_CS_PIN, OUTPUT);
  digitalWrite(HS_CS_PIN, HIGH);
  digitalWrite(MS_CS_PIN, HIGH);
  delay(10);

  initWatchArray(hsWatch);
  initWatchArray(msWatch);

  Serial.println(F("Starting Brake Code Filtered..."));

  if (ENABLE_HS) {
    if (CAN_HS.begin(MCP_ANY, HS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
      Serial.println(F("HS CAN init FAIL"));
      while (1) {}
    }
    CAN_HS.setMode(MCP_NORMAL);
  }

  if (ENABLE_MS) {
    if (CAN_MS.begin(MCP_ANY, MS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
      Serial.println(F("MS CAN init FAIL"));
      while (1) {}
    }
    CAN_MS.setMode(MCP_NORMAL);
  }

  printBanner();
}

// ==========================================
// Loop
// ==========================================
void loop() {
  unsigned long now = millis();

  if (ENABLE_HS) {
    processBus(CAN_HS, HS_INT_PIN, HS_LED_PIN, hsLedOffAt, hsWatch, "HS", hsFrameCount);
  }

  if (ENABLE_MS) {
    processBus(CAN_MS, MS_INT_PIN, MS_LED_PIN, msLedOffAt, msWatch, "MS", msFrameCount);
  }

  serviceLedOff(now);

  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;

    Serial.print("HB HS=");
    Serial.print(hsFrameCount);
    Serial.print(" MS=");
    Serial.println(msFrameCount);

    if (ENABLE_MS) printSummary("MS", msWatch);
    if (ENABLE_HS) printSummary("HS", hsWatch);
  }
}