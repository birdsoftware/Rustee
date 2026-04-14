#include <SPI.h>
#include <mcp_can.h>
#include <math.h>

// =====================================================
// DriveTrace EV Efficiency Logger
// Arduino Mega + dual MCP2515
//
// HS-CAN:
//   - sends OBD-II PID requests
//   - reads verified values: RPM, MPH, Load, ECT, IAT,
//     Volts, Fuel, MAF, APP, Throttle, Fuel Rate
//
// MS-CAN:
//   - passive listen only
//   - decodes gear from MS 0x78 byte1 upper nibble
//
// Output:
//   - METRIC lines
//   - BEST steady-cruise line when a new best ratio is found
//   - heartbeat lines
//
// Read-only on MS.
// HS only sends standard OBD requests.
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
// Timing
// =========================
const unsigned long HEARTBEAT_MS      = 3000;
const unsigned long LED_ON_MS         = 20;
const unsigned long OBD_REQ_MS        = 40;
const unsigned long METRIC_PRINT_MS   = 250;
const unsigned long RAW_PRINT_MS      = 75;

// =========================
// Raw frame logging control
// =========================
const bool PRINT_RAW_HS_FRAMES = false;
const bool PRINT_RAW_MS_FRAMES = false;
const bool PRINT_GEAR_CHANGES  = true;

// =========================
// OBD request list
// =========================
struct ReqPid { uint8_t pid; };
const ReqPid reqs[] = {
  {0x0C}, // RPM
  {0x0D}, // Speed
  {0x04}, // Engine load
  {0x05}, // Coolant temp
  {0x0F}, // Intake air temp
  {0x42}, // Control module voltage
  {0x2F}, // Fuel level
  {0x10}, // MAF
  {0x49}, // APP D
  {0x11}, // Throttle plate
  {0x5E}, // Fuel rate
};
const uint8_t NUM_REQS = sizeof(reqs) / sizeof(reqs[0]);
uint8_t reqIndex = 0;

// =========================
// Stats
// =========================
unsigned long lastHeartbeatMs = 0;
unsigned long lastReqMs = 0;
unsigned long lastMetricPrintMs = 0;
unsigned long lastRawPrintMs = 0;

unsigned long hsFrameCount = 0;
unsigned long msFrameCount = 0;
unsigned long hsObdRespCount = 0;
unsigned long msGearMsgCount = 0;

unsigned long hsLedOffAt = 0;
unsigned long msLedOffAt = 0;

// =========================
// Verified signal store
// =========================
float rpm = 0.0f;
float mph = 0.0f;
float loadPct = 0.0f;
float ectF = 0.0f;
float iatF = 0.0f;
float volts = 0.0f;
float fuelPct = 0.0f;
float maf = 0.0f;
float app = 0.0f;
float th = 0.0f;
float fuelRateLph = 0.0f;

bool seen0C = false;
bool seen0D = false;
bool seen04 = false;
bool seen05 = false;
bool seen0F = false;
bool seen42 = false;
bool seen2F = false;
bool seen10 = false;
bool seen49 = false;
bool seen11 = false;
bool seen5E = false;

// =========================
// Gear on MS-CAN
// confirmed:
// MS 0x78 byte1 upper nibble
// 0x0? P  0x2? R  0x4? N  0x6? D  0x8? M
// =========================
char gear[4] = "?";
bool gearValid = false;
char lastPrintedGear[4] = "?";

// =========================
// Derived metrics
// =========================
float rpmPerMph = 0.0f;
float slipX = 0.0f;

// suppress duplicate metric lines
float lastPrintedMph = -999.0f;
float lastPrintedRpm = -999.0f;
float lastPrintedApp = -999.0f;
float lastPrintedLoad = -999.0f;
float lastPrintedTh = -999.0f;

// =========================
// Best cruise tracker
// =========================
float bestCruiseRpmPerMph = 9999.0f;
float bestCruiseMph = 0.0f;
float bestCruiseRpm = 0.0f;
float bestCruiseApp = 0.0f;
float bestCruiseTh = 0.0f;
float bestCruiseLoad = 0.0f;

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

void sendPid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
  CAN_HS.sendMsgBuf(0x7DF, 0, 8, req);
}

void updateDerivedMetrics() {
  rpmPerMph = 0.0f;
  slipX = 0.0f;

  if (seen0C && seen0D && mph > 1.0f && rpm > 100.0f) {
    rpmPerMph = rpm / mph;
    slipX = rpmPerMph;
  }
}

bool steadyCruise() {
  if (!seen0D || !seen49 || !seen11 || !seen04 || !seen0C) return false;

  bool roadSpeedOK = mph >= 40.0f;
  bool pedalOK     = app <= 22.5f;
  bool throttleOK  = th <= 18.5f;
  bool loadOK      = loadPct <= 72.0f;
  bool ratioOK     = (mph < 1.0f) ? false : ((rpm / mph) <= 30.0f);

  return roadSpeedOK && pedalOK && throttleOK && loadOK && ratioOK;
}

void decodeGearFromMS78(uint8_t b1) {
  uint8_t nib = b1 & 0xF0;

  switch (nib) {
    case 0x00: strcpy(gear, "P"); gearValid = true; break;
    case 0x20: strcpy(gear, "R"); gearValid = true; break;
    case 0x40: strcpy(gear, "N"); gearValid = true; break;
    case 0x60: strcpy(gear, "D"); gearValid = true; break;
    case 0x80: strcpy(gear, "M"); gearValid = true; break;
    default:   strcpy(gear, "?"); gearValid = false; break;
  }
}

void handleMs78(uint8_t *buf) {
  decodeGearFromMS78(buf[1]);

  if (PRINT_GEAR_CHANGES && strcmp(gear, lastPrintedGear) != 0) {
    Serial.print("GEAR ");
    Serial.print(gearValid ? gear : "?");
    Serial.print("  MS 0x78  b1=");
    printHexByte(buf[1]);
    Serial.print("  Data:");
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print(" ");
      printHexByte(buf[i]);
    }
    Serial.println();

    strcpy(lastPrintedGear, gear);
  }
}

void updateBestCruise() {
  if (!steadyCruise()) return;
  if (!(seen0C && seen0D) || mph < 1.0f) return;

  float ratio = rpm / mph;
  if (ratio < bestCruiseRpmPerMph) {
    bestCruiseRpmPerMph = ratio;
    bestCruiseMph = mph;
    bestCruiseRpm = rpm;
    bestCruiseApp = app;
    bestCruiseTh = th;
    bestCruiseLoad = loadPct;

    Serial.print("BEST");
    Serial.print(" MPH:");
    Serial.print(bestCruiseMph, 1);
    Serial.print(" RPM:");
    Serial.print(bestCruiseRpm, 0);
    Serial.print(" APP:");
    Serial.print(bestCruiseApp, 1);
    Serial.print("% TH:");
    Serial.print(bestCruiseTh, 1);
    Serial.print("% Load:");
    Serial.print(bestCruiseLoad, 1);
    Serial.print("% RPM/MPH:");
    Serial.println(bestCruiseRpmPerMph, 2);
  }
}

void printMetricLine() {
  unsigned long now = millis();
  if (now - lastMetricPrintMs < METRIC_PRINT_MS) return;

  updateDerivedMetrics();
  updateBestCruise();

  bool changedEnough = false;
  if (fabs(mph - lastPrintedMph) > 0.4f) changedEnough = true;
  if (fabs(rpm - lastPrintedRpm) > 20.0f) changedEnough = true;
  if (fabs(app - lastPrintedApp) > 0.7f) changedEnough = true;
  if (fabs(loadPct - lastPrintedLoad) > 1.0f) changedEnough = true;
  if (fabs(th - lastPrintedTh) > 0.7f) changedEnough = true;

  if (!changedEnough) return;

  Serial.print("METRIC");

  if (gearValid) {
    Serial.print(" G:");
    Serial.print(gear);
  }

  Serial.print(" Mode:");
  Serial.print(steadyCruise() ? "STEADY" : "TRANS");

  Serial.print(" RPM:");
  if (seen0C) Serial.print(rpm, 0);
  else Serial.print("-");

  Serial.print(" MPH:");
  if (seen0D) Serial.print(mph, 1);
  else Serial.print("-");

  Serial.print(" Load:");
  if (seen04) Serial.print(loadPct, 1);
  else Serial.print("-");
  Serial.print("%");

  Serial.print(" APP:");
  if (seen49) Serial.print(app, 1);
  else Serial.print("-");
  Serial.print("%");

  Serial.print(" TH:");
  if (seen11) Serial.print(th, 1);
  else Serial.print("-");
  Serial.print("%");

  if (seen10) {
    Serial.print(" MAF:");
    Serial.print(maf, 1);
    Serial.print("g/s");
  }

  Serial.print(" V:");
  if (seen42) Serial.print(volts, 2);
  else Serial.print("-");

  Serial.print(" Fuel:");
  if (seen2F) Serial.print(fuelPct, 1);
  else Serial.print("-");
  Serial.print("%");

  if (seen5E) {
    Serial.print(" FR:");
    Serial.print(fuelRateLph, 1);
    Serial.print("L/h");
  }

  if (seen0C && seen0D && mph > 1.0f) {
    Serial.print(" RPM/MPH:");
    Serial.print(rpmPerMph, 2);

    Serial.print(" SLIPX:");
    Serial.print(slipX, 2);
  }

  Serial.println();

  lastMetricPrintMs = now;
  lastPrintedMph = mph;
  lastPrintedRpm = rpm;
  lastPrintedApp = app;
  lastPrintedLoad = loadPct;
  lastPrintedTh = th;
}

// =====================================================
// HS OBD response handling
// =====================================================
void processHS() {
  while (digitalRead(HS_INT_PIN) == LOW) {
    uint32_t rxId = 0;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN_HS.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;

    hsFrameCount++;
    flashLed(HS_LED_PIN, hsLedOffAt, millis());

    if (PRINT_RAW_HS_FRAMES) {
      if (millis() - lastRawPrintMs >= RAW_PRINT_MS) {
        lastRawPrintMs = millis();
        Serial.print("HS 0x");
        Serial.print(rxId, HEX);
        Serial.print(" Data:");
        for (uint8_t i = 0; i < len; i++) {
          Serial.print(" ");
          printHexByte(buf[i]);
        }
        Serial.println();
      }
    }

    if (len != 8) continue;
    if (rxId < 0x7E8 || rxId > 0x7EF) continue;
    if (buf[1] != 0x41) continue;

    hsObdRespCount++;

    uint8_t pid = buf[2];
    uint8_t A = buf[3];
    uint8_t B = buf[4];

    switch (pid) {
      case 0x0C: {
        uint16_t raw = ((uint16_t)A << 8) | B;
        rpm = raw / 4.0f;
        seen0C = true;
      } break;

      case 0x0D: {
        float kph = A;
        mph = kph * 0.621371f;
        seen0D = true;
      } break;

      case 0x04: {
        loadPct = A * 100.0f / 255.0f;
        seen04 = true;
      } break;

      case 0x05: {
        float c = (float)A - 40.0f;
        ectF = c * 9.0f / 5.0f + 32.0f;
        seen05 = true;
      } break;

      case 0x0F: {
        float c = (float)A - 40.0f;
        iatF = c * 9.0f / 5.0f + 32.0f;
        seen0F = true;
      } break;

      case 0x42: {
        uint16_t raw = ((uint16_t)A << 8) | B;
        volts = raw / 1000.0f;
        seen42 = true;
      } break;

      case 0x2F: {
        fuelPct = A * 100.0f / 255.0f;
        seen2F = true;
      } break;

      case 0x10: {
        uint16_t raw = ((uint16_t)A << 8) | B;
        maf = raw / 100.0f;
        seen10 = true;
      } break;

      case 0x49: {
        app = A * 100.0f / 255.0f;
        seen49 = true;
      } break;

      case 0x11: {
        th = A * 100.0f / 255.0f;
        seen11 = true;
      } break;

      case 0x5E: {
        uint16_t raw = ((uint16_t)A << 8) | B;
        fuelRateLph = raw / 20.0f;
        seen5E = true;
      } break;
    }
  }
}

// =====================================================
// MS passive listening for gear
// =====================================================
void processMS() {
  while (digitalRead(MS_INT_PIN) == LOW) {
    uint32_t rxId = 0;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN_MS.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;

    msFrameCount++;
    flashLed(MS_LED_PIN, msLedOffAt, millis());

    if (PRINT_RAW_MS_FRAMES) {
      if (millis() - lastRawPrintMs >= RAW_PRINT_MS) {
        lastRawPrintMs = millis();
        Serial.print("MS 0x");
        Serial.print(rxId, HEX);
        Serial.print(" Data:");
        for (uint8_t i = 0; i < len; i++) {
          Serial.print(" ");
          printHexByte(buf[i]);
        }
        Serial.println();
      }
    }

    if (rxId == 0x78 && len >= 2) {
      msGearMsgCount++;
      handleMs78(buf);
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

  // Important for shared SPI
  pinMode(HS_CS_PIN, OUTPUT);
  pinMode(MS_CS_PIN, OUTPUT);
  digitalWrite(HS_CS_PIN, HIGH);
  digitalWrite(MS_CS_PIN, HIGH);
  delay(10);

  Serial.println(F("Starting DriveTrace EV Efficiency Logger..."));

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
  Serial.println(F("HS: OBD PIDs 0C 0D 04 05 0F 42 2F 10 49 11 5E"));
  Serial.println(F("MS: gear from 0x78 byte1 upper nibble"));
  Serial.println(F("Outputs: METRIC, BEST, HB"));
}

// =====================================================
// Loop
// =====================================================
void loop() {
  unsigned long now = millis();

  if (now - lastReqMs >= OBD_REQ_MS) {
    lastReqMs = now;
    sendPid(reqs[reqIndex].pid);
    reqIndex = (reqIndex + 1) % NUM_REQS;
  }

  processHS();
  processMS();

  printMetricLine();
  serviceLedOff(now);

  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;

    Serial.print("HB HS_frames=");
    Serial.print(hsFrameCount);
    Serial.print(" HS_obd=");
    Serial.print(hsObdRespCount);
    Serial.print(" MS_frames=");
    Serial.print(msFrameCount);
    Serial.print(" MS_gear=");
    Serial.print(msGearMsgCount);
    Serial.print(" G=");
    Serial.print(gearValid ? gear : "?");
    Serial.print(" RPM=");
    if (seen0C) Serial.print(rpm, 0); else Serial.print("-");
    Serial.print(" MPH=");
    if (seen0D) Serial.print(mph, 1); else Serial.print("-");
    Serial.print(" Best=");
    if (bestCruiseRpmPerMph < 9999.0f) Serial.print(bestCruiseRpmPerMph, 2);
    else Serial.print("-");
    Serial.println();
  }
}