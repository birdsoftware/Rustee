
#include <SPI.h>
#include <mcp_can.h>

// =========================
// Pins
// =========================
const uint8_t HS_CS_PIN  = 10;
const uint8_t HS_INT_PIN = 2;
const uint8_t MS_CS_PIN  = 9;
const uint8_t MS_INT_PIN = 3;

const uint8_t HS_LED_PIN = 7;
const uint8_t MS_LED_PIN = 8;

// =========================
// MCP2515 objects
// =========================
MCP_CAN CAN_HS(HS_CS_PIN);
MCP_CAN CAN_MS(MS_CS_PIN);

// =========================
// CAN settings
// =========================
#define HS_CAN_SPEED CAN_500KBPS
#define MS_CAN_SPEED CAN_500KBPS
#define MCP_CLOCK    MCP_8MHZ

// =========================
// Timing
// =========================
const unsigned long HEARTBEAT_MS    = 2000;
const unsigned long LED_ON_MS       = 20;
const unsigned long GEAR_TIMEOUT_MS = 1000;   // default to P on timeout

unsigned long lastHeartbeatMs = 0;
unsigned long hsLedOffAt = 0;
unsigned long msLedOffAt = 0;
unsigned long lastValidGearMs = 0;

unsigned long hsFrameCount = 0;
unsigned long msFrameCount = 0;

// =========================
// Gear filtering
// =========================
const int STABLE_THRESHOLD = 4;

char lastSampleGear = '?';
char stableGear     = 'P';
int stableCount     = 0;
uint8_t stableB1    = 0x08;

// =========================
// Helpers
// =========================
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

char decodeGearFromB1(uint8_t b1) {
  switch (b1 & 0xF0) {
    case 0x00: return 'P';
    case 0x20: return 'R';
    case 0x40: return 'N';
    case 0x60: return 'D';
    case 0x80: return 'M';
    default:   return '?';
  }
}

void printStableGear(char g, uint8_t b1) {
  Serial.print("GEAR: ");
  Serial.print(g);
  Serial.print("  CAN b1=");
  printHexByte(b1);
  Serial.println();
}

void setStableGear(char g, uint8_t b1) {
  if (g == stableGear && b1 == stableB1) return;

  stableGear = g;
  stableB1 = b1;
  printStableGear(stableGear, stableB1);
}

void processDecodedGear(uint8_t b1, unsigned long now) {
  char g = decodeGearFromB1(b1);
  if (g == '?') return;

  lastValidGearMs = now;

  if (g == lastSampleGear) {
    stableCount++;
  } else {
    lastSampleGear = g;
    stableCount = 1;
  }

  if (stableCount >= STABLE_THRESHOLD) {
    setStableGear(g, b1);
  }
}

void handleMS78(uint8_t len, uint8_t *buf, unsigned long now) {
  if (len < 2) return;
  processDecodedGear(buf[1], now);   // b1 holds gear group
}

void processOneBus(
  MCP_CAN &canDev,
  uint8_t intPin,
  uint8_t ledPin,
  unsigned long &ledOffAt,
  const char* busName,
  unsigned long &frameCount,
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

    if (ext) continue;
    if (len > 8) len = 8;

    // Decode only from MS bus, ID 0x78
    if (busName[0] == 'M' && cleanId == 0x78) {
      handleMS78(len, buf, now);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(HS_INT_PIN, INPUT);
  pinMode(MS_INT_PIN, INPUT);

  pinMode(HS_LED_PIN, OUTPUT);
  pinMode(MS_LED_PIN, OUTPUT);
  digitalWrite(HS_LED_PIN, LOW);
  digitalWrite(MS_LED_PIN, LOW);

  // Shared SPI safety
  pinMode(HS_CS_PIN, OUTPUT);
  pinMode(MS_CS_PIN, OUTPUT);
  digitalWrite(HS_CS_PIN, HIGH);
  digitalWrite(MS_CS_PIN, HIGH);
  delay(10);

  Serial.println("Starting final MS 0x78 gear decoder...");
  Serial.println("Using MS 0x78 byte 1");
  Serial.println("Mapping: 0x0?=P  0x2?=R  0x4?=N  0x6?=D  0x8?=M");
  Serial.println();

  if (CAN_HS.begin(MCP_ANY, HS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
    Serial.println("HS CAN init FAIL");
    while (1) {}
  }

  if (CAN_MS.begin(MCP_ANY, MS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
    Serial.println("MS CAN init FAIL");
    while (1) {}
  }

  CAN_HS.setMode(MCP_NORMAL);
  CAN_MS.setMode(MCP_NORMAL);

  stableGear = 'P';
  stableB1 = 0x08;
  lastSampleGear = '?';
  stableCount = 0;
  lastValidGearMs = millis();

  printStableGear(stableGear, stableB1);
}

void loop() {
  unsigned long now = millis();

  processOneBus(CAN_HS, HS_INT_PIN, HS_LED_PIN, hsLedOffAt, "HS", hsFrameCount, now);
  processOneBus(CAN_MS, MS_INT_PIN, MS_LED_PIN, msLedOffAt, "MS", msFrameCount, now);

  serviceLedOff(now);

  // Timeout -> default to Park
  if ((now - lastValidGearMs) > GEAR_TIMEOUT_MS) {
    if (stableGear != 'P') {
      stableGear = 'P';
      stableB1 = 0x08;
      lastSampleGear = '?';
      stableCount = 0;
      printStableGear(stableGear, stableB1);
    }
  }

  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;

    Serial.print("HB HS=");
    Serial.print(hsFrameCount);
    Serial.print(" MS=");
    Serial.print(msFrameCount);
    Serial.print("  StableGear=");
    Serial.print(stableGear);
    Serial.print("  b1=");
    printHexByte(stableB1);
    Serial.println();
  }
}