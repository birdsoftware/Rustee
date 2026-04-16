#include <SPI.h>
#include <mcp_can.h>

// ==========================================
// Brake204_DeltaHunter
// Arduino Mega + 2x MCP2515
//
// Watches only ID 0x204.
// Learns rolling baseline of byte 4.
// Reports an event when byte 4 drops enough below baseline.
//
// Goal:
// compare brake-tap runs vs no-brake runs
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
const uint32_t TARGET_ID         = 0x204;

// Delta detection tuning
const uint8_t DROP_TRIGGER       = 4;   // event when baseline - b4 >= 3
const uint8_t RELEASE_MARGIN     = 1;   // release when baseline - b4 <= 1
const uint8_t CONSEC_TRIGGER     = 2;   // require 3 consecutive frames
const float   BASELINE_ALPHA     = 0.015f; // slow rolling average

struct WatchState {
  bool used;
  uint8_t lastLen;
  uint8_t lastData[8];
  uint16_t changes;

  float baselineB4;
  uint8_t currentB4;

  uint8_t belowCount;
  uint8_t aboveCount;

  bool eventActive;
  uint16_t eventCount;

  uint8_t minDuringEvent;
};

WatchState hsWatch;
WatchState msWatch;

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

void initWatch(WatchState &w) {
  w.used = false;
  w.lastLen = 0;
  w.changes = 0;
  w.baselineB4 = 0.0f;
  w.currentB4 = 0;
  w.belowCount = 0;
  w.aboveCount = 0;
  w.eventActive = false;
  w.eventCount = 0;
  w.minDuringEvent = 0xFF;
  for (uint8_t i = 0; i < 8; i++) w.lastData[i] = 0;
}

void printEventStart(const char *busName, const WatchState &w, int delta) {
  Serial.print(busName);
  Serial.print(" EVENT_START");
  Serial.print(" b4=");
  printHexByte(w.currentB4);
  Serial.print(" baseline=");
  Serial.print(w.baselineB4, 2);
  Serial.print(" drop=");
  Serial.print(delta);
  Serial.print(" event#=");
  Serial.println(w.eventCount);
}

void printEventEnd(const char *busName, const WatchState &w, int delta) {
  Serial.print(busName);
  Serial.print(" EVENT_END");
  Serial.print(" b4=");
  printHexByte(w.currentB4);
  Serial.print(" baseline=");
  Serial.print(w.baselineB4, 2);
  Serial.print(" drop=");
  Serial.print(delta);
  Serial.print(" min=");
  printHexByte(w.minDuringEvent);
  Serial.print(" event#=");
  Serial.println(w.eventCount);
}

void printSummary(const char *busName, const WatchState &w) {
  Serial.print(busName);
  Serial.print(" summary");
  Serial.print(" changes=");
  Serial.print(w.changes);

  if (w.used) {
    Serial.print(" b4=");
    printHexByte(w.currentB4);
    Serial.print(" baseline=");
    Serial.print(w.baselineB4, 2);
    Serial.print(" drop=");
    Serial.print((int)w.baselineB4 - (int)w.currentB4);
    Serial.print(" active=");
    Serial.print(w.eventActive ? "YES" : "NO");
    Serial.print(" events=");
    Serial.print(w.eventCount);
  }

  Serial.println();
}

void updateDetector(WatchState &w, const char *busName) {
  int delta = (int)(w.baselineB4 + 0.5f) - (int)w.currentB4;

  if (!w.eventActive) {
    if (delta >= DROP_TRIGGER) {
      if (w.belowCount < 255) w.belowCount++;
    } else {
      w.belowCount = 0;
    }

    if (w.belowCount >= CONSEC_TRIGGER) {
      w.eventActive = true;
      w.eventCount++;
      w.minDuringEvent = w.currentB4;
      printEventStart(busName, w, delta);
      w.belowCount = 0;
    }
  } else {
    if (w.currentB4 < w.minDuringEvent) {
      w.minDuringEvent = w.currentB4;
    }

    if (delta <= RELEASE_MARGIN) {
      if (w.aboveCount < 255) w.aboveCount++;
    } else {
      w.aboveCount = 0;
    }

    if (w.aboveCount >= CONSEC_TRIGGER) {
      printEventEnd(busName, w, delta);
      w.eventActive = false;
      w.aboveCount = 0;
    }
  }
}

void processBus(
  MCP_CAN &canDev,
  uint8_t intPin,
  uint8_t ledPin,
  unsigned long &ledOffAt,
  WatchState &watch,
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
    if (ext) continue;
    if (rxId != TARGET_ID) continue;
    if (len <= 4) continue;
    if (len > 8) len = 8;

    uint8_t b4 = buf[4];

    if (!watch.used) {
      watch.used = true;
      watch.lastLen = len;
      for (uint8_t i = 0; i < len; i++) watch.lastData[i] = buf[i];

      watch.currentB4 = b4;
      watch.baselineB4 = (float)b4;

      Serial.print(busName);
      Serial.print(" INIT b4=");
      printHexByte(b4);
      Serial.print(" baseline=");
      Serial.println(watch.baselineB4, 2);
      continue;
    }

    bool changed = false;
    for (uint8_t i = 0; i < len; i++) {
      if (watch.lastData[i] != buf[i]) {
        changed = true;
        break;
      }
    }
    if (!changed) continue;

    watch.changes++;
    watch.currentB4 = b4;

    // update baseline only when not in an active event
    if (!watch.eventActive) {
      watch.baselineB4 =
        (1.0f - BASELINE_ALPHA) * watch.baselineB4 +
        BASELINE_ALPHA * (float)b4;
    }

    updateDetector(watch, busName);

    watch.lastLen = len;
    for (uint8_t i = 0; i < len; i++) watch.lastData[i] = buf[i];
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

  initWatch(hsWatch);
  initWatch(msWatch);

  if (ENABLE_HS) {
    if (CAN_HS.begin(MCP_ANY, HS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
      Serial.println("HS CAN init FAIL");
      while (1) {}
    }
    CAN_HS.setMode(MCP_NORMAL);
  }

  if (ENABLE_MS) {
    if (CAN_MS.begin(MCP_ANY, MS_CAN_SPEED, MCP_CLOCK) != CAN_OK) {
      Serial.println("MS CAN init FAIL");
      while (1) {}
    }
    CAN_MS.setMode(MCP_NORMAL);
  }

  Serial.println("Starting Brake204_DeltaHunter...");
  Serial.print("Watching 0x204 byte4");
  Serial.print(" DROP_TRIGGER=");
  Serial.print(DROP_TRIGGER);
  Serial.print(" RELEASE_MARGIN=");
  Serial.print(RELEASE_MARGIN);
  Serial.print(" CONSEC_TRIGGER=");
  Serial.println(CONSEC_TRIGGER);
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