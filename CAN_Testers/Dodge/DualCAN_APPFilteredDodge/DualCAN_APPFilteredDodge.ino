#include <SPI.h>
#include <mcp_can.h>

// ========================================
// Dodge 3500 APP + Gear Watch
// APP = HS 0x108, bytes 0:1 combined
// Gear candidates = HS 0x126 and HS 0x23A
// Arduino Mega + dual MCP2515
// ========================================

const uint8_t HS_CS_PIN  = 10;
const uint8_t HS_INT_PIN = 2;
const uint8_t MS_CS_PIN  = 9;
const uint8_t MS_INT_PIN = 3;

const uint8_t HS_LED_PIN = 7;
const uint8_t MS_LED_PIN = 8;

#define HS_CAN_SPEED CAN_500KBPS
#define MS_CAN_SPEED CAN_500KBPS
#define MCP_CLOCK    MCP_8MHZ

MCP_CAN CAN_HS(HS_CS_PIN);
MCP_CAN CAN_MS(MS_CS_PIN);

const uint16_t ID_APP   = 0x108;
const uint16_t ID_GEAR1 = 0x126;
const uint16_t ID_GEAR2 = 0x23A;

struct FrameStore {
  bool seen = false;
  uint8_t len = 0;
  uint8_t data[8] = {0};
};

FrameStore hs108, hs126, hs23A;
FrameStore ms108, ms126, ms23A;

unsigned long lastHeartbeatMs = 0;
unsigned long lastPrintMs = 0;
unsigned long hsFrames = 0;
unsigned long msFrames = 0;

unsigned long hsLedOffAt = 0;
unsigned long msLedOffAt = 0;

const unsigned long HEARTBEAT_MS = 1000;
const unsigned long PRINT_INTERVAL_MS = 120;
const unsigned long LED_ON_MS = 20;

// First-pass calibration from your capture
const int APP_RAW_MIN = 690;
const int APP_RAW_MAX = 1870;

// ---------------- helpers ----------------
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

void saveFrame(FrameStore &f, uint8_t len, const uint8_t *buf) {
  f.seen = true;
  f.len = (len > 8) ? 8 : len;
  for (uint8_t i = 0; i < f.len; i++) f.data[i] = buf[i];
  for (uint8_t i = f.len; i < 8; i++) f.data[i] = 0;
}

uint16_t makeU16(uint8_t hi, uint8_t lo) {
  return ((uint16_t)hi << 8) | lo;
}

int getByteSafe(const FrameStore &f, uint8_t idx) {
  if (!f.seen || idx >= f.len) return -1;
  return f.data[idx];
}

int calcAppPercent(int raw) {
  long pct = (long)(raw - APP_RAW_MIN) * 100L / (APP_RAW_MAX - APP_RAW_MIN);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)pct;
}

void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print("0");
  Serial.print(b, HEX);
}

void printHexOrDash(int v) {
  if (v < 0) Serial.print("--");
  else {
    if (v < 0x10) Serial.print("0");
    Serial.print(v, HEX);
  }
}

void processBus(MCP_CAN &canDev,
                uint8_t intPin,
                uint8_t ledPin,
                unsigned long &ledOffAt,
                FrameStore &f108,
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
    if (ext) continue;

    uint16_t id = (uint16_t)rxId;

    if (id == ID_APP) {
      saveFrame(f108, len, buf);
    } else if (id == ID_GEAR1) {
      saveFrame(f126, len, buf);
    } else if (id == ID_GEAR2) {
      saveFrame(f23A, len, buf);
    }
  }
}

void printLine(const char *bus, const FrameStore &f108, const FrameStore &f126, const FrameStore &f23A) {
  Serial.print(bus);

  if (f108.seen && f108.len >= 2) {
    int appRaw = makeU16(f108.data[0], f108.data[1]);
    int appPct = calcAppPercent(appRaw);

    Serial.print(" APPraw=");
    Serial.print(appRaw);
    Serial.print(" APP%=");
    Serial.print(appPct);
  } else {
    Serial.print(" APPraw=-- APP%=--");
  }

  Serial.print(" || 126 b1=");
  printHexOrDash(getByteSafe(f126, 1));
  Serial.print(" b3=");
  printHexOrDash(getByteSafe(f126, 3));
  Serial.print(" b6=");
  printHexOrDash(getByteSafe(f126, 6));

  Serial.print(" || 23A b1=");
  printHexOrDash(getByteSafe(f23A, 1));
  Serial.print(" b2=");
  printHexOrDash(getByteSafe(f23A, 2));

  Serial.println();
}

void printRaw108(const char *bus, const FrameStore &f108) {
  Serial.print(bus);
  Serial.print(" RAW 108:");
  if (!f108.seen) {
    Serial.println(" --");
    return;
  }
  for (uint8_t i = 0; i < f108.len; i++) {
    Serial.print(" ");
    printHexByte(f108.data[i]);
  }
  Serial.println();
}

// ---------------- setup ----------------
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
  Serial.println(F("===== DODGE 3500 APP + GEAR ====="));
  Serial.println(F("APP = HS 0x108 bytes 0:1"));
  Serial.println(F("Gear candidates = HS 0x126 and HS 0x23A"));
  Serial.println(F("Next: hold P, R, N, D about 5 sec each"));
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

  Serial.println(F("Ready."));
}

// ---------------- loop ----------------
void loop() {
  unsigned long now = millis();

  processBus(CAN_HS, HS_INT_PIN, HS_LED_PIN, hsLedOffAt, hs108, hs126, hs23A, hsFrames, now);
  processBus(CAN_MS, MS_INT_PIN, MS_LED_PIN, msLedOffAt, ms108, ms126, ms23A, msFrames, now);

  serviceLedOff(now);

  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;

    printLine("HS", hs108, hs126, hs23A);
    printRaw108("HS", hs108);
    Serial.println();

    if (ms108.seen || ms126.seen || ms23A.seen) {
      printLine("MS", ms108, ms126, ms23A);
      printRaw108("MS", ms108);
      Serial.println();
    }
  }

  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    Serial.print(F("HB HS="));
    Serial.print(hsFrames);
    Serial.print(F(" MS="));
    Serial.println(msFrames);
  }
}