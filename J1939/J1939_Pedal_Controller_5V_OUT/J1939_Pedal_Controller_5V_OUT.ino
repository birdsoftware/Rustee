// future use
// PGN F003  → Accelerator pedal position
// PGN F004  → RPM, torque/load related values
// PGN FEF1  → Vehicle speed / cruise / brake switch data
// PGN FEEF  → Engine oil pressure / coolant temp
// PGN FEEE  → Engine coolant temp / fuel temp / oil temp
// PGN FEC1  → Total engine hours / revolutions
// PGN FECA  → Diagnostic lamp status
// PGN FEEF  → Fuel economy / fuel rate style data depending source
#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

const int potPin      = A0;
const int pwmPin      = 5;
const int pwmPinHalf  = 6;

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastPrintMs = 0;
unsigned long lastAppMs   = 0;
unsigned long lastRpmMs   = 0;
unsigned long n = 0;

float app = 0.0f;
float rpm = 0.0f;
float vin = 0.780f;

float minV = 0.780f;
float maxV = 5.000f;//max voltagew

const float APP_FULL_SCALE_PERCENT = 5.0f;//this makes pedal sensitive

int pwm = 0;
int pwmHalf = 0;

const int POT_DEADZONE_ADC = 100;
const int HARD_PWM_CAP = 255;
const unsigned long APP_TIMEOUT_MS = 500;
const unsigned long RPM_TIMEOUT_MS = 500;

int lastPwm = 0;
const int PWM_MAX_STEP = 50;

// linear
// float mapAppToVoltage(float appPercent) {
//   appPercent = constrain(appPercent, 0.0f, APP_FULL_SCALE_PERCENT);
//   float x = appPercent / APP_FULL_SCALE_PERCENT;
//   return minV + x * (maxV - minV);
// }
float mapAppToVoltage(float appPercent) {
  appPercent = constrain(appPercent, 0.0f, APP_FULL_SCALE_PERCENT);

  float x = appPercent / APP_FULL_SCALE_PERCENT;

  // Curved output:
  // slow at first, then ramps up faster
  x = x * x;

  return minV + x * (maxV - minV);
}

void setup() {
  analogReference(DEFAULT);
  Serial.begin(115200);

  pinMode(CAN_INT_PIN, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(pwmPinHalf, OUTPUT);

  analogWrite(pwmPin, 0);
  analogWrite(pwmPinHalf, 0);

  if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 Init FAIL");
    while (1) {
      analogWrite(pwmPin, 0);
      analogWrite(pwmPinHalf, 0);
    }
  }

  CAN0.setMode(MCP_LISTENONLY);
  Serial.println("J1939 APP + RPM + POT + PWM voltage tester started");
}

void loop() {
  unsigned long now = millis();

  if (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) == CAN_OK && len == 8) {
      uint32_t pgn = (rxId >> 8) & 0xFFFF;

      if (pgn == 0xF003) {
        app = buf[1] * 0.4f;
        lastAppMs = now;
      }

      if (pgn == 0xF004) {
        uint16_t rawRpm = ((uint16_t)buf[4] << 8) | buf[3];
        rpm = rawRpm * 0.125f;
        lastRpmMs = now;
      }
    }
  }

  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    int potADC = analogRead(potPin);

    int pwmMax = 0;
    if (potADC < POT_DEADZONE_ADC) {
      pwmMax = 0;
    } else {
      pwmMax = map(potADC, POT_DEADZONE_ADC, 1023, 0, 255);
    }

    bool appTimedOut = (now - lastAppMs > APP_TIMEOUT_MS);
    bool rpmTimedOut = (now - lastRpmMs > RPM_TIMEOUT_MS);

    if (appTimedOut) {
      vin = minV;
    } else {
      vin = mapAppToVoltage(app);
    }

    pwm = (int)((vin / 5.0f) * 255.0f + 0.5f);

    if (pwm < 0) pwm = 0;
    if (pwm > pwmMax) pwm = pwmMax;
    if (pwm > HARD_PWM_CAP) pwm = HARD_PWM_CAP;

    if (pwm > lastPwm + PWM_MAX_STEP) pwm = lastPwm + PWM_MAX_STEP;
    if (pwm < lastPwm - PWM_MAX_STEP) pwm = lastPwm - PWM_MAX_STEP;

    lastPwm = pwm;

    analogWrite(pwmPin, pwm);

    pwmHalf = pwm / 2;
    analogWrite(pwmPinHalf, pwmHalf);

    float actualOutVoltage = pwm * 5.0f / 255.0f;

    n++;
    if (n > 99) n = 0;

    Serial.print("APP: ");
    Serial.print(app, 1);
    Serial.print("%");

    Serial.print(" | RPM: ");
    if (rpmTimedOut) Serial.print("TIMEOUT");
    else Serial.print(rpm, 0);

    Serial.print(" | TARGET VIN: ");
    Serial.print(vin, 3);
    Serial.print("V");

    Serial.print(" | D5 OUT: ");
    Serial.print(actualOutVoltage, 3);
    Serial.print("V");

    Serial.print(" | POT: ");
    Serial.print(potADC);

    Serial.print(" | LIM: ");
    Serial.print(pwmMax);

    Serial.print(" | PWM D5: ");
    Serial.print(pwm);

    Serial.print(" | PWM D6: ");
    Serial.print(pwmHalf);

    if (appTimedOut) Serial.print(" | APP_TIMEOUT");

    Serial.print(" | #");
    Serial.println(n);
  }
}