#include <SPI.h>
#include <mcp_can.h>

// =====================================================
// DODGE 3500 APP + GEAR WATCHER
// Arduino Mega + dual MCP2515
// Keeps your original dual-bus layout
// Focuses on likely APP + gear candidates only
// =====================================================

// -------------------------
// Pins
// -------------------------
const uint8_t HS_CS_PIN  = 10;
const uint8_t HS_INT_PIN = 2;
const uint8_t MS_CS_PIN  = 9;
const uint8_t MS_INT_PIN = 3;

const uint8_t HS_LED_PIN = 7;
const uint8_t MS_LED_PIN = 8;

// -------------------------
// CAN config
// -------------------------
#define HS_CAN_SPEED CAN_500KBPS
#define MS_CAN_SPEED CAN_500KBPS
#define MCP_CLOCK    MCP_8MHZ

MCP_CAN CAN_HS(HS_CS_PIN);
MCP_CAN CAN_MS(MS_CS_PIN);

// -------------------------
// Timing
// -------------------------
const unsigned long HEARTBEAT_MS      = 1000;
const unsigned long PRINT_INTERVAL_MS = 120;
const unsigned long LED_ON_MS         = 20;

// -------------------------
// Candidate IDs
// -------------------------
const uint16_t ID_APP1  = 0x108; // strongest APP candidate
const uint16_t ID_APP2  = 0x118; // second APP candidate
const uint16_t ID_GEAR1 = 0x126; // likely gear/status
const uint16_t ID_GEAR2 = 0x23A; // likely gear/status

// -------------------------
// Frame storage
// -------------------------
struct FrameStore {
  bool seen = false;
  uint8_t len = 0;
  uint8_t data[8] = {0};
  unsigned long lastUpdateMs = 0;
};

FrameStore hs108, hs118, hs126, hs23A;
FrameStore ms108, ms118, ms126, ms23A;

unsigned long lastHeartbeatMs = 0;
unsigned long lastPrintMs = 0;
unsigned long hsFrames = 0;
unsigned long msFrames = 0;

unsigned long hsLedOffAt = 0;
unsigned long msLedOffAt = 0;

// =====================================================
// Helpers
// =====================================================
void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print("0");
  Serial.print(b, HEX);
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

void saveFrame(FrameStore &f, uint8_t len, const uint8_t *buf, unsigned long now) {
  f.seen = true;
  f.len = (len > 8) ? 8 : len;
  for (uint8_t i = 0; i < f.len; i++) f.data[i] = buf[i];
  for (uint8_t i = f.len; i < 8; i++) f.data[i] = 0;
  f.lastUpdateMs = now;
}

int getByteSafe(const FrameStore &f, uint8_t idx) {
  if (!f.seen) return -1;
  if (idx >= f.len) return -1;
  return f.data[idx];
}

void printByteDecOrDash(int v) {
  if (v < 0) {
    Serial.print("--");
  } else {
    Serial.print(v);
  }
}

void printByteHexOrDash(int v) {
  if (v < 0) {
    Serial.print("--");
  } else {
    if (v < 0x10) Serial.print("0");
    Serial.print(v, HEX);
  }
}

void printFrameHexShort(const FrameStore &f, const char *label) {
  Serial.print(label);
  Serial.print(":");
  if (!f.seen) {
    Serial.print(" --");
    return;
  }

  for (uint8_t i = 0; i < f.len; i++) {
    Serial.print(" ");
    printHexByte(f.data[i]);
  }
}

void printCandidateLine(const char *bus, const FrameStore &f108, const FrameStore &f118,
                        const FrameStore &f126, const FrameStore &f23A) {
  int app108_b3 = getByteSafe(f108, 3);
  int app108_b5 = getByteSafe(f108, 5);
  int app118_b5 = getByteSafe(f118, 5);
  int app118_b6 = getByteSafe(f118, 6);

  Serial.print(bus);
  Serial.print(" APP 108[b3]=");
  printByteDecOrDash(app108_b3);
  Serial.print(" 108[b5]=");
  printByteDecOrDash(app108_b5);
  Serial.print(" | 118[b5]=");
  printByteDecOrDash(app118_b5);
  Serial.print(" 118[b6]=");
  printByteDecOrDash(app118_b6);

  Serial.print(" || G126 b1=");
  printByteHexOrDash(getByteSafe(f126, 1));
  Serial.print(" b3=");
  printByteHexOrDash(getByteSafe(f126, 3));
  Serial.print(" b6=");
  printByteHexOrDash(getByteSafe(f126, 6));

  Serial.print(" || G23A b1=");
  printByteHexOrDash(getByteSafe(f23A, 1));
  Serial.print(" b2=");
  printByteHexOrDash(getByteSafe(f23A, 2));

  Serial.println();
}

void printRawDebugBlock(const char *bus, const FrameStore &f108, const FrameStore &f118,
                        const FrameStore &f126, const FrameStore &f23A) {
  Serial.print(bus);
  Serial.print(" RAW ");
  printFrameHexShort(f108, "108");
  Serial.print(" | ");
  printFrameHexShort(f118, "118");
  Serial.print(" | ");
  printFrameHexShort(f126, "126");
  Serial.print(" | ");
  printFrameHexShort(f23A, "23A");
  Serial.println();
}

// =====================================================
// CAN reader
// =====================================================
void processBus(MCP_CAN &canDev,
                uint8_t intPin,
                uint8_t ledPin,
                unsigned long &ledOffAt,
                const char *busName,
                FrameStore &f108,
                FrameStore &f118,
                FrameStore &f126,
                FrameStore &f23A,
                unsigned long &frameCounter,
                unsigned long now) {
  while (digitalRead(intPin) == LOW) {
    unsigned long rxId = 0;
    unsigned char len = 0;
    unsigned char buf[8];

    if (canDev.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;

    frameCounter++;
    flashLed(ledPin, ledOffAt, now);

    bool ext = (rxId & 0x80000000UL) != 0;
    if (ext) continue; // standard IDs only

    uint16_t id = (uint16_t)rxId;

    switch (id) {
      case ID_APP1:  saveFrame(f108, len, buf, now); break;
      case ID_APP2:  saveFrame(f118, len, buf, now); break;
      case ID_GEAR1: saveFrame(f126, len, buf, now); break;
      case ID_GEAR2: saveFrame(f23A, len, buf, now); break;
      default: break;
    }
  }
}

// =====================================================
// Setup
// =====================================================
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

  Serial.println();
  Serial.println(F("===== DODGE 3500 APP + GEAR WATCH ====="));
  Serial.println(F("APP candidates: HS 0x108[b3,b5], HS 0x118[b5,b6]"));
  Serial.println(F("Gear candidates: HS 0x126, HS 0x23A"));
  Serial.println(F("Run pedal test:"));
  Serial.println(F("  foot off 3 sec"));
  Serial.println(F("  tiny touch"));
  Serial.println(F("  medium touch"));
  Serial.println(F("  release"));
  Serial.println(F("  repeat twice"));
  Serial.println();

  if (CAN_HS.begin(MCP_ANY, HS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
    Serial.println(F("HS CAN init FAIL"));
    while (1) {}
  }

  if (CAN_MS.begin(MCP_ANY, MS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
    Serial.println(F("MS CAN init FAIL"));
    while (1) {}
  }

  CAN_HS.setMode(MCP_NORMAL);
  CAN_MS.setMode(MCP_NORMAL);

  Serial.println(F("Both sniffers ready."));
}

// =====================================================
// Loop
// =====================================================
void loop() {
  unsigned long now = millis();

  processBus(CAN_HS, HS_INT_PIN, HS_LED_PIN, hsLedOffAt, "HS",
             hs108, hs118, hs126, hs23A, hsFrames, now);

  processBus(CAN_MS, MS_INT_PIN, MS_LED_PIN, msLedOffAt, "MS",
             ms108, ms118, ms126, ms23A, msFrames, now);

  serviceLedOff(now);

  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;

    // Main compact lines
    printCandidateLine("HS", hs108, hs118, hs126, hs23A);

    // Only print MS line if anything relevant is seen
    if (ms108.seen || ms118.seen || ms126.seen || ms23A.seen) {
      printCandidateLine("MS", ms108, ms118, ms126, ms23A);
    }

    // Optional raw lines for sanity check
    printRawDebugBlock("HS", hs108, hs118, hs126, hs23A);
    if (ms108.seen || ms118.seen || ms126.seen || ms23A.seen) {
      printRawDebugBlock("MS", ms108, ms118, ms126, ms23A);
    }

    Serial.println();
  }

  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    Serial.print(F("HB HS="));
    Serial.print(hsFrames);
    Serial.print(F(" MS="));
    Serial.println(msFrames);
  }
}