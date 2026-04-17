#include <SPI.h>
#include <mcp_can.h>
#include <math.h>

// =====================================================
// TCC Reverse Engineering Tool v2
// Arduino Mega + dual MCP2515
//
// HS-CAN:
//   - OBD-II verified polling:
//     RPM, MPH, Load, Volts, APP, TH, MAF, FuelRate
//
// MS-CAN:
//   - passive gear decode from 0x78
//
// Also:
//   - brake event proxy using 0x204 byte4 delta-baseline
//   - TCC lock/unlock estimate from RPM/MPH clusters
//   - slip estimate
//   - estimated kW
//   - efficiency MPH/kW
//   - candidate CAN ID watcher near lock/unlock transitions
//
// Notes:
//   - This version uses a simpler and stronger TCC estimator:
//       LOCKED   if RPM/MPH < 36 at speed
//       UNLOCKED if RPM/MPH >= 36
//
//   - Based on your data, locked cruise was around ~32-33 RPM/MPH
//     and unlocked/slipping looked closer to ~40-41 RPM/MPH.
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
const unsigned long EVENT_WINDOW_MS   = 1200;

// =========================
// OBD request list
// =========================
struct ReqPid { uint8_t pid; };
const ReqPid reqs[] = {
  {0x0C}, // RPM
  {0x0D}, // Speed
  {0x04}, // Engine load
  {0x42}, // Control module voltage
  {0x49}, // APP D
  {0x11}, // Throttle plate
  {0x10}, // MAF
  {0x5E}, // Fuel rate
};

const uint8_t NUM_REQS = sizeof(reqs) / sizeof(reqs[0]);
uint8_t reqIndex = 0;

// =========================
// Stats / timing
// =========================
unsigned long lastHeartbeatMs = 0;
unsigned long lastReqMs = 0;
unsigned long lastMetricPrintMs = 0;

unsigned long hsFrameCount = 0;
unsigned long msFrameCount = 0;
unsigned long hsObdRespCount = 0;
unsigned long msGearMsgCount = 0;

unsigned long hsLedOffAt = 0;
unsigned long msLedOffAt = 0;

// =========================
// Verified OBD values
// =========================
float rpm = 0.0f;
float mph = 0.0f;
float loadPct = 0.0f;
float volts = 0.0f;
float app = 0.0f;
float th = 0.0f;
float maf = 0.0f;
float fuelRateLph = 0.0f;

bool seen0C = false;
bool seen0D = false;
bool seen04 = false;
bool seen42 = false;
bool seen49 = false;
bool seen11 = false;
bool seen10 = false;
bool seen5E = false;

// =========================
// Gear decode on MS 0x78
// =========================
char gear[4] = "?";
bool gearValid = false;
char lastPrintedGear[4] = "?";

// =========================
// Derived metrics
// =========================
float rpmPerMph = 0.0f;
float estKw = 0.0f;
float mphPerKw = 0.0f;
float slipEstimate = 0.0f;

// =========================
// TCC estimated state
// =========================
bool tccLocked = false;
bool lastTccLocked = false;

// Based on your actual logs
const float LOCKED_RATIO_REF   = 32.0f;
const float LOCK_THRESHOLD_RPM_PER_MPH = 36.0f;

// =========================
// Brake proxy from 0x204 b4
// =========================
struct BrakeProxy {
  bool used;
  uint8_t lastLen;
  uint8_t lastData[8];

  float baselineB4;
  uint8_t currentB4;

  uint8_t belowCount;
  uint8_t aboveCount;

  bool eventActive;
  uint16_t eventCount;
  uint8_t minDuringEvent;
};

BrakeProxy hsBrake;
BrakeProxy msBrake;

const uint8_t BRAKE_DROP_TRIGGER   = 4;
const uint8_t BRAKE_RELEASE_MARGIN = 1;
const uint8_t BRAKE_CONSEC_TRIGGER = 2;
const float   BRAKE_BASELINE_ALPHA = 0.015f;

// =========================
// Candidate watcher
// =========================
struct CandidateState {
  uint32_t id;
  bool used;
  uint8_t len;
  uint8_t lastData[8];
};

const uint32_t CAND_IDS[] = {
  0x204, 0x202, 0x4B0, 0x77, 0x82, 0x85, 0x91, 0x92, 0x415
};

const uint8_t NUM_CAND_IDS = sizeof(CAND_IDS) / sizeof(CAND_IDS[0]);
CandidateState hsCand[NUM_CAND_IDS];
CandidateState msCand[NUM_CAND_IDS];

unsigned long lastLockEventMs = 0;

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

void sendPid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
  CAN_HS.sendMsgBuf(0x7DF, 0, 8, req);
}

bool steadyCruise() {
  if (!(seen0C && seen0D && seen49 && seen04)) return false;
  if (mph < 20.0f) return false;
  if (app > 40.0f) return false;
  if (loadPct > 75.0f) return false;
  return true;
}

void updateDerivedMetrics() {
  rpmPerMph = 0.0f;
  estKw = 0.0f;
  mphPerKw = 0.0f;
  slipEstimate = 0.0f;

  if (seen0C && seen0D && mph > 1.0f) {
    rpmPerMph = rpm / mph;
  }

  // crude comparative power proxy
  if (seen0C && seen04) {
    estKw = (rpm * (loadPct / 100.0f)) / 300.0f;
    if (estKw < 0.1f) estKw = 0.0f;
  }

  if (seen0D && estKw > 0.1f) {
    mphPerKw = mph / estKw;
  }

  if (rpmPerMph > 0.0f) {
    slipEstimate = rpmPerMph - LOCKED_RATIO_REF;
    if (slipEstimate < 0.0f) slipEstimate = 0.0f;
  }
}

void updateTccEstimate() {
  lastTccLocked = tccLocked;

  if (!(seen0C && seen0D) || mph < 20.0f) {
    tccLocked = false;
    return;
  }

  if (rpmPerMph < LOCK_THRESHOLD_RPM_PER_MPH) {
    tccLocked = true;
  } else {
    tccLocked = false;
  }
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

  if (strcmp(gear, lastPrintedGear) != 0) {
    Serial.print("GEAR ");
    Serial.print(gearValid ? gear : "?");
    Serial.print(" MS 0x78 b1=");
    printHexByte(buf[1]);
    Serial.println();
    strcpy(lastPrintedGear, gear);
  }
}

// =========================
// Brake proxy helpers
// =========================
void initBrakeProxy(BrakeProxy &b) {
  b.used = false;
  b.lastLen = 0;
  for (uint8_t i = 0; i < 8; i++) b.lastData[i] = 0;
  b.baselineB4 = 0.0f;
  b.currentB4 = 0;
  b.belowCount = 0;
  b.aboveCount = 0;
  b.eventActive = false;
  b.eventCount = 0;
  b.minDuringEvent = 0xFF;
}

void printBrakeStart(const char *busName, const BrakeProxy &b, int delta) {
  Serial.print(busName);
  Serial.print(" BRAKE_EVENT START");
  Serial.print(" b4=");
  printHexByte(b.currentB4);
  Serial.print(" baseline=");
  Serial.print(b.baselineB4, 2);
  Serial.print(" drop=");
  Serial.print(delta);
  Serial.print(" event#=");
  Serial.println(b.eventCount);
}

void printBrakeEnd(const char *busName, const BrakeProxy &b, int delta) {
  Serial.print(busName);
  Serial.print(" BRAKE_EVENT END");
  Serial.print(" b4=");
  printHexByte(b.currentB4);
  Serial.print(" baseline=");
  Serial.print(b.baselineB4, 2);
  Serial.print(" drop=");
  Serial.print(delta);
  Serial.print(" min=");
  printHexByte(b.minDuringEvent);
  Serial.print(" event#=");
  Serial.println(b.eventCount);
}

void updateBrakeProxy(BrakeProxy &b, const char *busName) {
  int delta = (int)(b.baselineB4 + 0.5f) - (int)b.currentB4;

  if (!b.eventActive) {
    if (delta >= BRAKE_DROP_TRIGGER) {
      if (b.belowCount < 255) b.belowCount++;
    } else {
      b.belowCount = 0;
    }

    if (b.belowCount >= BRAKE_CONSEC_TRIGGER) {
      b.eventActive = true;
      b.eventCount++;
      b.minDuringEvent = b.currentB4;
      printBrakeStart(busName, b, delta);
      b.belowCount = 0;
    }
  } else {
    if (b.currentB4 < b.minDuringEvent) {
      b.minDuringEvent = b.currentB4;
    }

    if (delta <= BRAKE_RELEASE_MARGIN) {
      if (b.aboveCount < 255) b.aboveCount++;
    } else {
      b.aboveCount = 0;
    }

    if (b.aboveCount >= BRAKE_CONSEC_TRIGGER) {
      printBrakeEnd(busName, b, delta);
      b.eventActive = false;
      b.aboveCount = 0;
    }
  }
}

void handleBrake204(BrakeProxy &b, uint8_t *buf, uint8_t len, const char *busName) {
  if (len <= 4) return;

  uint8_t b4 = buf[4];

  if (!b.used) {
    b.used = true;
    b.lastLen = len;
    for (uint8_t i = 0; i < len; i++) b.lastData[i] = buf[i];
    b.currentB4 = b4;
    b.baselineB4 = (float)b4;
    return;
  }

  bool changed = false;
  for (uint8_t i = 0; i < len; i++) {
    if (b.lastData[i] != buf[i]) {
      changed = true;
      break;
    }
  }
  if (!changed) return;

  b.currentB4 = b4;

  if (!b.eventActive) {
    b.baselineB4 =
      (1.0f - BRAKE_BASELINE_ALPHA) * b.baselineB4 +
      BRAKE_BASELINE_ALPHA * (float)b4;
  }

  updateBrakeProxy(b, busName);

  b.lastLen = len;
  for (uint8_t i = 0; i < len; i++) b.lastData[i] = buf[i];
}

// =========================
// Candidate watcher
// =========================
void initCandArray(CandidateState *arr) {
  for (uint8_t i = 0; i < NUM_CAND_IDS; i++) {
    arr[i].id = CAND_IDS[i];
    arr[i].used = false;
    arr[i].len = 0;
    for (uint8_t j = 0; j < 8; j++) arr[i].lastData[j] = 0;
  }
}

int candidateIndex(uint32_t id) {
  for (uint8_t i = 0; i < NUM_CAND_IDS; i++) {
    if (CAND_IDS[i] == id) return i;
  }
  return -1;
}

void watchCandidate(CandidateState *arr, const char *busName, uint32_t id, uint8_t len, uint8_t *buf) {
  int idx = candidateIndex(id);
  if (idx < 0) return;

  CandidateState &c = arr[idx];

  if (!c.used) {
    c.used = true;
    c.len = len;
    for (uint8_t i = 0; i < len; i++) c.lastData[i] = buf[i];
    return;
  }

  bool changed = (c.len != len);
  if (!changed) {
    for (uint8_t i = 0; i < len; i++) {
      if (c.lastData[i] != buf[i]) {
        changed = true;
        break;
      }
    }
  }
  if (!changed) return;

  if (millis() - lastLockEventMs <= EVENT_WINDOW_MS) {
    Serial.print("CAND ");
    Serial.print(busName);
    Serial.print(" 0x");
    Serial.print(id, HEX);
    Serial.print(" data:");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(" ");
      printHexByte(buf[i]);
    }
    Serial.print(" diff:");
    for (uint8_t i = 0; i < len; i++) {
      if (c.lastData[i] != buf[i]) {
        Serial.print(" [");
        Serial.print(i);
        Serial.print(":");
        printHexByte(c.lastData[i]);
        Serial.print("->");
        printHexByte(buf[i]);
        Serial.print("]");
      }
    }
    Serial.println();
  }

  c.len = len;
  for (uint8_t i = 0; i < len; i++) c.lastData[i] = buf[i];
}

// =========================
// Print / state transitions
// =========================
void maybePrintTccTransition() {
  if (tccLocked != lastTccLocked) {
    lastLockEventMs = millis();
    Serial.print("TCC_EVENT ");
    Serial.println(tccLocked ? "LOCKED" : "UNLOCKED");
  }
}

void printMetricLine() {
  unsigned long now = millis();
  if (now - lastMetricPrintMs < METRIC_PRINT_MS) return;
  lastMetricPrintMs = now;

  updateDerivedMetrics();
  updateTccEstimate();
  maybePrintTccTransition();

  Serial.print("METRIC");

  if (gearValid) {
    Serial.print(" G:");
    Serial.print(gear);
  }

  Serial.print(" RPM:");
  if (seen0C) Serial.print(rpm, 0); else Serial.print("-");

  Serial.print(" MPH:");
  if (seen0D) Serial.print(mph, 1); else Serial.print("-");

  Serial.print(" Load:");
  if (seen04) Serial.print(loadPct, 1); else Serial.print("-");
  Serial.print("%");

  Serial.print(" APP:");
  if (seen49) Serial.print(app, 1); else Serial.print("-");
  Serial.print("%");

  Serial.print(" TH:");
  if (seen11) Serial.print(th, 1); else Serial.print("-");
  Serial.print("%");

  Serial.print(" V:");
  if (seen42) Serial.print(volts, 2); else Serial.print("-");

  if (seen10) {
    Serial.print(" MAF:");
    Serial.print(maf, 1);
    Serial.print("g/s");
  }

  if (seen5E) {
    Serial.print(" FR:");
    Serial.print(fuelRateLph, 1);
    Serial.print("L/h");
  }

  Serial.print(" kW:");
  if (estKw > 0.0f) Serial.print(estKw, 2); else Serial.print("-");

  Serial.print(" MPH/kW:");
  if (mphPerKw > 0.0f) Serial.print(mphPerKw, 2); else Serial.print("-");

  Serial.print(" RPM/MPH:");
  if (rpmPerMph > 0.0f) Serial.print(rpmPerMph, 2); else Serial.print("-");

  Serial.print(" SlipEst:");
  Serial.print(slipEstimate, 2);

  Serial.print(" LOCK?:");
  Serial.print(tccLocked ? "YES" : "NO");

  Serial.print(" Brake:");
  Serial.print((hsBrake.eventActive || msBrake.eventActive) ? "YES" : "NO");

  Serial.println();
}

// =========================
// HS OBD processing
// =========================
void processHS() {
  while (digitalRead(HS_INT_PIN) == LOW) {
    uint32_t rxId = 0;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN_HS.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;

    hsFrameCount++;
    flashLed(HS_LED_PIN, hsLedOffAt, millis());

    if (rxId == 0x204 && len >= 5) {
      handleBrake204(hsBrake, buf, len, "HS");
    }

    watchCandidate(hsCand, "HS", rxId, len, buf);

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

      case 0x42: {
        uint16_t raw = ((uint16_t)A << 8) | B;
        volts = raw / 1000.0f;
        seen42 = true;
      } break;

      case 0x49: {
        app = A * 100.0f / 255.0f;
        seen49 = true;
      } break;

      case 0x11: {
        th = A * 100.0f / 255.0f;
        seen11 = true;
      } break;

      case 0x10: {
        uint16_t raw = ((uint16_t)A << 8) | B;
        maf = raw / 100.0f;
        seen10 = true;
      } break;

      case 0x5E: {
        uint16_t raw = ((uint16_t)A << 8) | B;
        fuelRateLph = raw / 20.0f;
        seen5E = true;
      } break;
    }
  }
}

// =========================
// MS passive processing
// =========================
void processMS() {
  while (digitalRead(MS_INT_PIN) == LOW) {
    uint32_t rxId = 0;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN_MS.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;

    msFrameCount++;
    flashLed(MS_LED_PIN, msLedOffAt, millis());

    if (rxId == 0x78 && len >= 2) {
      msGearMsgCount++;
      handleMs78(buf);
    }

    if (rxId == 0x204 && len >= 5) {
      handleBrake204(msBrake, buf, len, "MS");
    }

    watchCandidate(msCand, "MS", rxId, len, buf);
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
  pinMode(MS_LED_PIN, OUTPUT);
  digitalWrite(HS_LED_PIN, LOW);
  digitalWrite(MS_LED_PIN, LOW);

  pinMode(HS_CS_PIN, OUTPUT);
  pinMode(MS_CS_PIN, OUTPUT);
  digitalWrite(HS_CS_PIN, HIGH);
  digitalWrite(MS_CS_PIN, HIGH);
  delay(10);

  initBrakeProxy(hsBrake);
  initBrakeProxy(msBrake);
  initCandArray(hsCand);
  initCandArray(msCand);

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

  Serial.println("Starting TCC Reverse Engineering Tool v2...");
  Serial.println("HS: OBD RPM MPH Load V APP TH MAF FR");
  Serial.println("MS: Gear from 0x78");
  Serial.println("Brake proxy: 0x204 b4 delta-baseline");
  Serial.println("Candidates: 0x204 0x202 0x4B0 0x77 0x82 0x85 0x91 0x92 0x415");
  Serial.println("TCC logic: LOCK if RPM/MPH < 36 above 20 MPH");
}

// =========================
// Loop
// =========================
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
    Serial.print(" Brake=");
    Serial.print((hsBrake.eventActive || msBrake.eventActive) ? "YES" : "NO");
    Serial.print(" LOCK?=");
    Serial.print(tccLocked ? "YES" : "NO");
    Serial.println();
  }
}