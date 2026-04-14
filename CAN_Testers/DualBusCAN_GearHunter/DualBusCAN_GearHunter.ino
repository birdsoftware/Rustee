// Arduino Mega, HW-184 #1 & #2, DC-DC converter.
// Pin | Function                   |
// 16  | +12V battery               |
// 5   | Signal ground (use this)   |
// 6   | CAN High (HS-CAN) #1       |
// 14  | CAN Low (HS-CAN)  #1       |
// 3   | CAN High (MS-CAN / body)#2 |
// 11  | CAN Low (MS-CAN / body) #2 |

// OBD Pin 16 (+12V) ─────> DC-DC IN+
// OBD Pin 5 (GND) ───────> DC-DC IN-
// DC-DC OUT +5V ─────>  Mega 5V pin
// DC-DC GND ─────────>  Mega GND

// SPI (shared by BOTH modules)
// Mega D50 (MISO) ─────> BOTH modules SO
// Mega D51 (MOSI) ─────> BOTH modules SI
// Mega D52 (SCK)  ─────> BOTH modules SCK

// Module #1 (HS-CAN → Pins 6/14)
// CS  ─────> Mega D10
// INT ─────> Mega D2
// VCC ─────> 5V
// GND ─────> GND

// CANH ────> OBD Pin 6
// CANL ────> OBD Pin 14

// Module #2 (MS-CAN → Pins 3/11)
// CS  ─────> Mega D9
// INT ─────> Mega D3
// VCC ─────> 5V
// GND ─────> GND

// CANH ────> OBD Pin 3
// CANL ────> OBD Pin 11

//  OBD PORT
//  ----------------
//  16 (+12V) ── DC-DC ── 5V ── Mega + Modules
//  5 (GND)  ─────────── GND ── Mega + Modules
// 6 (CANH) ────────────── Module1 CANH
// 14 (CANL) ────────────── Module1 CANL
// 3 (CANH) ────────────── Module2 CANH
// 11 (CANL) ────────────── Module2 CANL

// Arduino Pin ─── LED ─── 1k ─── GND
// D7 → HS-CAN activity LED
// D8 → MS-CAN activity LED

// // LED test ---------------------
// const int LED_HS = 7;
// const int LED_MS = 8;
// void setup() {
//   pinMode(LED_HS, OUTPUT);
//   pinMode(LED_MS, OUTPUT);
// }

// void loop() {
//   digitalWrite(LED_HS, HIGH);
//   digitalWrite(LED_MS, LOW);
//   delay(500);

//   digitalWrite(LED_HS, LOW);
//   digitalWrite(LED_MS, HIGH);
//   delay(500);
// }
// // led test ---------------------

#include <SPI.h>
#include <mcp_can.h>

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
// Adjust these if needed
// =========================
#define HS_CAN_SPEED   CAN_500KBPS
#define MS_CAN_SPEED   CAN_500KBPS
#define MCP_CLOCK      MCP_8MHZ

// =========================
// Logging controls
// =========================
const unsigned long HEARTBEAT_MS         = 2000;
const unsigned long MIN_PRINT_GAP_MS     = 75;
const unsigned long LED_ON_MS            = 20;
const bool SHOW_STANDARD_IDS             = true;
const bool SHOW_EXTENDED_IDS             = true;
const bool PRINT_FIRST_SEEN_FRAMES       = true;
const bool PRINT_ONLY_CHANGED_FRAMES     = true;

// Max tracked IDs per bus
const int MAX_TRACKED_IDS = 150;//180

// =========================
// Frame tracking
// =========================
struct FrameState {
  bool used;
  uint32_t id;
  bool ext;
  uint8_t len;
  uint8_t data[8];
  unsigned long lastPrintMs;
};

FrameState hsStates[MAX_TRACKED_IDS];
FrameState msStates[MAX_TRACKED_IDS];

// =========================
// Stats
// =========================
unsigned long lastHeartbeatMs = 0;

unsigned long hsFrameCount = 0;
unsigned long hsChangedCount = 0;
unsigned long hsDroppedCount = 0;

unsigned long msFrameCount = 0;
unsigned long msChangedCount = 0;
unsigned long msDroppedCount = 0;

unsigned long hsLedOffAt = 0;
unsigned long msLedOffAt = 0;

// =========================
// Helpers
// =========================
void clearStates(FrameState *states, int count) {
  for (int i = 0; i < count; i++) {
    states[i].used = false;
    states[i].id = 0;
    states[i].ext = false;
    states[i].len = 0;
    states[i].lastPrintMs = 0;
    for (int j = 0; j < 8; j++) states[i].data[j] = 0;
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
      for (int j = 0; j < 8; j++) states[i].data[j] = 0;
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

void printBanner() {
  Serial.println();
  Serial.println(F("===== DUAL BUS RAW CAN GEAR HUNTER ====="));
  Serial.println(F("Buses:"));
  Serial.println(F("  HS = OBD pins 6/14"));
  Serial.println(F("  MS = OBD pins 3/11"));
  Serial.println(F("LEDs:"));
  Serial.println(F("  D7 = HS activity"));
  Serial.println(F("  D8 = MS activity"));
  Serial.println(F("Procedure:"));
  Serial.println(F("  1) Idle in P for ~5 sec"));
  Serial.println(F("  2) Hold R for ~5 sec"));
  Serial.println(F("  3) Hold N for ~5 sec"));
  Serial.println(F("  4) Hold D for ~5 sec"));
  Serial.println(F("  5) Repeat with brake on/off if needed"));
  Serial.println(F("Watch for IDs where 1-2 bytes change with PRNDL."));
  Serial.println(F("========================================"));
  Serial.println();
}

void printFrameLine(
  const char *busName,
  uint32_t id,
  bool ext,
  uint8_t len,
  const uint8_t *oldData,
  const uint8_t *newData,
  bool firstSeen
) {
  Serial.print(busName);
  Serial.print(" ");
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

    bool changed = firstSeen || (s.len != len);
    if (!changed) {
      for (uint8_t i = 0; i < len; i++) {
        if (s.data[i] != buf[i]) {
          changed = true;
          break;
        }
      }
    }

    if (firstSeen && PRINT_FIRST_SEEN_FRAMES) {
      if (now - s.lastPrintMs >= MIN_PRINT_GAP_MS) {
        printFrameLine(busName, cleanId, ext, len, s.data, buf, true);
        s.lastPrintMs = now;
        changedCount++;
      }
    } else if (changed && PRINT_ONLY_CHANGED_FRAMES) {
      if (now - s.lastPrintMs >= MIN_PRINT_GAP_MS) {
        printFrameLine(busName, cleanId, ext, len, s.data, buf, false);
        s.lastPrintMs = now;
        changedCount++;
      }
    }

    s.len = len;
    for (uint8_t i = 0; i < len; i++) {
      s.data[i] = buf[i];
    }
  }
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
  pinMode(MS_LED_PIN, OUTPUT);   //MODULE 2
  digitalWrite(HS_LED_PIN, LOW);
  digitalWrite(MS_LED_PIN, LOW);  //MODULE 2

  // Critical fix for shared SPI:
  // force both chip selects HIGH before touching either MCP2515
  pinMode(HS_CS_PIN, OUTPUT);
  pinMode(MS_CS_PIN, OUTPUT);
  digitalWrite(HS_CS_PIN, HIGH);
  digitalWrite(MS_CS_PIN, HIGH);
  delay(10);

  clearStates(hsStates, MAX_TRACKED_IDS);
  clearStates(msStates, MAX_TRACKED_IDS);

  Serial.println(F("Starting dual MCP2515 CAN sniffer..."));

  if (CAN_HS.begin(MCP_ANY, HS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
    Serial.println(F("HS CAN init FAIL"));
    Serial.println(F("Try MCP_16MHZ or different HS speed"));
    while (1) {}
  }

//MODULE 2
  if (CAN_MS.begin(MCP_ANY, MS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
    Serial.println(F("MS CAN init FAIL"));
    Serial.println(F("Try MCP_16MHZ or different MS speed"));
    while (1) {}
  }

  CAN_HS.setMode(MCP_NORMAL);
  CAN_MS.setMode(MCP_NORMAL);             //MODULE 2

  printBanner();
  Serial.println(F("HS and MS sniffers ready."));
}

// =========================
// Loop
// =========================
void loop() {
  unsigned long now = millis();

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

// //MODULE 2
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

  serviceLedOff(now);

  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;

    Serial.print(F("HB  HS frames="));
    Serial.print(hsFrameCount);
    Serial.print(F(" changed="));
    Serial.print(hsChangedCount);
    Serial.print(F(" tracked="));
    Serial.print(countTracked(hsStates));
    Serial.print(F(" dropped="));
    Serial.print(hsDroppedCount);

    //Serial.print(F("   |   MS disabled"));
    //Serial.println();

    Serial.print(F("   |   MS frames="));
    Serial.print(msFrameCount);
    Serial.print(F(" changed="));
    Serial.print(msChangedCount);
    Serial.print(F(" tracked="));
    Serial.print(countTracked(msStates));
    Serial.print(F(" dropped="));
    Serial.println(msDroppedCount);
  }
}
