// Control Arduino Nano 5v
// tested 3/31/26
//
// INPUT HW-184 wiring to PCB + Nano
// HW-184 CAN-H -> ODB-II Pin 6 CAN High
// HW-184 CAN-L -> ODB-II Pin 14 CAN LOW
// HW-184 Vcc -> 5V Nano E12
// HW-184 GND -> GND Nano E14
// HW-184 SO -> D12 Nano F1
// HW-184 SI -> D11 Nano F2
// HW-184 CS -> D10 Nano F3
// HW-184 SCK -> D13 Nano E1
// HW-184 INT -> D2 Nano F11
//
// ODB-II wiring
// ODB-II pin 16 12V -> +IN DC-DC Converter
// ODB-II pin 4 GND Chassis -> -IN DC-DC Converter
// ODB-II pin 5 Signal GND -> -IN DC-DC Converter
//
// SERVO: 25kg DC4.8v-6.8v DSSERVO
// Black: Ground (–)
// Red: Positive (+) / VCC — Connects to the positive power supply (4.8V – 6.8V)
// White: Signal (S) — Connects to the PWM control signal

#include <SPI.h>
#include <mcp_can.h>
#include <Servo.h>

const int CAN_CS_PIN   = 10;
const int CAN_INT_PIN  = 2;
const int SERVO_PIN    = 9;

MCP_CAN CAN0(CAN_CS_PIN);
Servo myServo;

unsigned long lastReqMs   = 0;
unsigned long lastPrintMs = 0;

struct ReqPid { uint8_t pid; };
const ReqPid reqs[] = {
  {0x49}, // APP D (pedal)
};

uint8_t reqIndex = 0;

// Latest APP value
float app = 0.0f;
bool seen49 = false;

// Servo range
const int SERVO_MIN_ANGLE = 5;//0;
const int SERVO_MAX_ANGLE = 40;//45;

// --- Calibration / sensitivity ---
float appZero = 0.0f;                 // learned pedal rest value
bool appZeroLearned = false;
float appFullScaleDelta = 5.0f;       // pedal movement above rest needed for full 45°
int appSamples = 0;
float appSum = 0.0f;

// Optional noise deadband above rest
float appDeadband = 0.2f;             // ignore tiny changes near rest

void sendPid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
  CAN0.sendMsgBuf(0x7DF, 0, 8, req);
}

int appToServoAngle(float appPercent) {
  // Keep a real zero
  float delta = appPercent - appZero;

  // Deadband so tiny noise doesn't move servo
  if (delta < appDeadband) {
    delta = 0.0f;
  }

  // Scale so a small pedal movement hits full angle
  float scaled = (delta / appFullScaleDelta) * SERVO_MAX_ANGLE;

  int servoAngle = (int)(scaled + 0.5f);
  servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  return servoAngle;
}

void setup() {
  Serial.begin(9600);

  pinMode(CAN_INT_PIN, INPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_MIN_ANGLE);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 Init FAIL");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);

  Serial.println("Starting... leave pedal at rest for calibration");
}

void loop() {
  unsigned long now = millis();

  // Request APP every 40 ms
  if (now - lastReqMs >= 40) {
    lastReqMs = now;
    sendPid(reqs[reqIndex].pid);
    reqIndex = (reqIndex + 1) % (sizeof(reqs) / sizeof(reqs[0]));
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
    uint8_t A   = buf[3];

    if (pid == 0x49) {
      app = A * 100.0f / 255.0f;
      seen49 = true;

      // Learn resting APP from first 20 samples
      if (!appZeroLearned) {
        appSum += app;
        appSamples++;

        if (appSamples >= 20) {
          appZero = appSum / appSamples;
          appZeroLearned = true;

          Serial.print("Learned APP zero: ");
          Serial.println(appZero, 2);
        }
      }

      if (appZeroLearned) {
        int servoAngle = appToServoAngle(app);
        myServo.write(servoAngle);//SERVO_MIN_ANGLE SERVO_MAX_ANGLE
      } else {
        myServo.write(0);
      }
    }
  }

  // Debug print
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    if (seen49) {
      int servoAngle = appZeroLearned ? appToServoAngle(app) : 0;

      Serial.print("APP: ");
      Serial.print(app, 2);
      Serial.print("%  Zero: ");
      Serial.print(appZero, 2);
      Serial.print("%  Delta: ");
      Serial.print(app - appZero, 2);
      Serial.print("%  Servo: ");
      Serial.println(servoAngle);
    } else {
      Serial.println("Waiting for APP...");
    }
  }
}