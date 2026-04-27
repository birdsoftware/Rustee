// This runs on an a arduino nano.
// 1. 🚛 Reads Semi Truck Data (J1939 via CAN)
//    Sends requests for (RPM, speed, pedal, gears, etc.) from ECU
//    Extract pedal position (APP)
// 2. 🦶 Learns Pedal “Resting Position” at Startup
//    During first few readings: Finds the lowest pedal value (foot off pedal)
//    Stored as effectiveAppMin
//    Adds a small buffer (+0.5)
//    This is your zero throttle baseline
// 3. 🎯 Sets the Max Range Automatically
//    After calibration: appMax = effectiveAppMin + 0.05f * (80.0f - effectiveAppMin);
//    It only uses ~5% of full pedal range
//    This compresses pedal input into a smaller usable window
// 4. 🔄 Convert pedal → voltage
//    Pedal min = 0.35v, pedal max 5v
//    Smooth linear scaling
// 5. ⚡ Convert voltage → PWM 
//    pwm = vin * 255 / 4
// 6. 🎛️ Limit PWM with a knob (potentiometer)
//    Potentiometer Limits Output
//    Acts like a manual power limiter / safety cap

// Pot to adjust max PWM out b.bird 3.3.26
// Right outer leg → 5V
// Middle (wiper) → A1
// Other outer leg → GND
//|----dead zone----|---------------- usable range ----------------|
//0%                10%                                             100%
//PWM=0             start ramping                                   PWM=235

// input
// CAN pedal position used for input signal

// INPUT HW-184 wiring
// HW-184 CAN-H -> J1939 Pin C CAN High
// HW-184 CAN-L -> J1939 Pin D CAN LOW
// HW-184 Vcc -> 5V Nano
// HW-184 GND -> GND Nano
// HW-184 CS -> D10 Nano
// HW-184 SO -> D12 Nano
// HW-184 SI -> D11 Nano
// HW-184 SCK -> D13 Nano
// HW-184 INT -> D2 Nano

// J1939 wiring
// J1939 pin B 12V -> +IN DC-DC Converter
// J1939 pin A GND  -> -IN DC-DC Converter

#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

const int potPin      = A0;  // POT middle/wiper -> A0
const int pwmPin      = 5;   // D5 PWM output
const int pwmPinHalf  = 6;   // D6 half-output

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastReqMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastAppMs = 0;
unsigned long n = 0;

struct ReqPid { uint8_t pid; };

const ReqPid reqs[] = {
  {0x0C}, // RPM
  {0x0D}, // Speed
  {0x04}, // Engine load
  {0x05}, // Coolant temp
  {0x0F}, // Intake air temp
  {0x42}, // Control module voltage
  {0x2F}, // Fuel level
  {0x49}, // APP D pedal position
  {0x11}, // Throttle plate
};

uint8_t reqIndex = 0;

// Latest values
float rpm = 0;
float mph = 0;
float loadPct = 0;
float ectF = 0;
float iatF = 0;
float volts = 0;
float fuelPct = 0;
float app = 0;
float th = 0;

// Seen flags
bool seen0C = false;
bool seen0D = false;
bool seen04 = false;
bool seen05 = false;
bool seen0F = false;
bool seen42 = false;
bool seen2F = false;
bool seen49 = false;
bool seen11 = false;

// Pedal calibration
float appMax = 18.0f;

// Output values
float minV = 0.780f;
float maxV = 4.0f;
float vin = 0.780f;

int pwm = 0;
int pwmHalf = 0;

// Boot-time APP min learning
bool calibratingAppMin = true;
int countApp = 0;
float learnedAppMin = 1000.0f;
float effectiveAppMin = 0.0f;

// Safety/settings
const unsigned long APP_TIMEOUT_MS = 500;
const int POT_DEADZONE_ADC = 100;
const int HARD_PWM_CAP = 200;

// Smooth output
int lastPwm = 0;
const int PWM_MAX_STEP = 5;

// --------------------------------------
// Map CAN APP % to continuous minV..maxV
// --------------------------------------
float mapPedalToVoltage(float appRaw) {
  float range = appMax - effectiveAppMin;

  if (range < 0.5f) {
    range = 0.5f;
  }

  float x = (appRaw - effectiveAppMin) / range;

  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;

  return minV + x * (maxV - minV);
}

// --------------------------------------
// Send OBD-II PID request
// --------------------------------------
void sendPid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
  CAN0.sendMsgBuf(0x7DF, 0, 8, req);
}

void setup() {
  analogReference(DEFAULT);

  Serial.begin(9600); // LoRa UART

  pinMode(CAN_INT_PIN, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(pwmPinHalf, OUTPUT);

  analogWrite(pwmPin, 0);
  analogWrite(pwmPinHalf, 0);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 Init FAIL");
    while (1) {
      analogWrite(pwmPin, 0);
      analogWrite(pwmPinHalf, 0);
    }
  }

  CAN0.setMode(MCP_NORMAL);
}

void loop() {
  unsigned long now = millis();

  // Send one PID request every 40ms
  if (now - lastReqMs >= 40) {
    lastReqMs = now;

    sendPid(reqs[reqIndex].pid);

    reqIndex++;
    if (reqIndex >= sizeof(reqs) / sizeof(reqs[0])) {
      reqIndex = 0;
    }
  }

  // Read CAN frames
  while (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t rxId;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;

    if (len != 8) continue;
    if (rxId < 0x7E8 || rxId > 0x7EF) continue;
    if (buf[1] != 0x41) continue;

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

      case 0x49: {
        app = A * 100.0f / 255.0f;
        seen49 = true;
        lastAppMs = now;

        if (calibratingAppMin) {
          countApp++;

          if (app < learnedAppMin) {
            learnedAppMin = app;
          }

          if (countApp >= 5) {
            calibratingAppMin = false;

            effectiveAppMin = learnedAppMin + 0.5f;

            // Use only about 5% of the APP range
            appMax = effectiveAppMin + 0.05f * (80.0f - effectiveAppMin);
          }
        }
      } break;

      case 0x11: {
        th = A * 100.0f / 255.0f;
        seen11 = true;
      } break;
    }
  }

  // Output update every 200ms
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    // ----------------------------
    // Read POT limiter on A0
    // ----------------------------
    int potADC = analogRead(potPin);

    int pwmMax = 0;

    if (potADC < POT_DEADZONE_ADC) {
      pwmMax = 0;
    } else {
      pwmMax = map(potADC, POT_DEADZONE_ADC, 1023, 0, 255);
    }

    // ----------------------------
    // APP fail-safe
    // ----------------------------
    bool appTimedOut = (now - lastAppMs > APP_TIMEOUT_MS);

    if (!seen49 || calibratingAppMin || appTimedOut) {
      vin = minV;
    } else {
      vin = mapPedalToVoltage(app);
    }

    // ----------------------------
    // Voltage to PWM
    // Nano PWM is 0-255 for 0-5V average
    // ----------------------------
    pwm = (int)((vin / 5.0f) * 255.0f + 0.5f);

    // ----------------------------
    // Apply limits
    // ----------------------------
    if (pwm < 0) pwm = 0;
    if (pwm > pwmMax) pwm = pwmMax;
    if (pwm > HARD_PWM_CAP) pwm = HARD_PWM_CAP;

    // ----------------------------
    // Smooth ramp / slew limit
    // ----------------------------
    if (pwm > lastPwm + PWM_MAX_STEP) {
      pwm = lastPwm + PWM_MAX_STEP;
    }

    if (pwm < lastPwm - PWM_MAX_STEP) {
      pwm = lastPwm - PWM_MAX_STEP;
    }

    lastPwm = pwm;

    // ----------------------------
    // Output PWM
    // ----------------------------
    analogWrite(pwmPin, pwm);

    pwmHalf = pwm / 2;
    analogWrite(pwmPinHalf, pwmHalf);

    // ----------------------------
    //  Serial output
    // ----------------------------
    n++;
    if (n > 99) n = 0;

    Serial.println(
      String(pwm) +
      " I: " + String(vin, 3) +
      " POT: " + String(potADC) +
      " LIM: " + String(pwmMax) +
      " APP: " + String(app, 2) +
      (calibratingAppMin ? " CAL" : "") +
      (appTimedOut ? " APP_TIMEOUT" : "") +
      " #" + String(n)
    );
  }
}