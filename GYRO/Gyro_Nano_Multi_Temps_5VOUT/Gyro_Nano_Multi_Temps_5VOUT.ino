// Nano
//GYRO:
// GY-521 VCC -> Nano 5V
// GY-521 GND -> Nano GND
// GY-521 SDA -> Nano A4
// GY-521 SCL -> Nano A5

//5V Analog In and Out from PWM
// Signal in → A0
// PWM Output signal → D9
// Grounds must be common

//safety
// A0 wire gets disconnected, the pin will usually float and read random values
// A0 → 100k resistor → GND

// Output RC Filter (2 stage filter) PWM -> Analog out 5V
// D9   -> 1K -> Node -> 1K -> OUT
//                |             |
//              10uF          10uF
//                |             |
//               GND           GND

// Temperature output RC filter on D10 if desired
// D10  -> 1K -> Node -> 1K -> OUT
//                |             |
//              10uF          10uF
//                |             |
//               GND           GND

//3 NTC Temp Sensors
// Picks the coldest temperature reading from sensor A1-A3
// Output voltage 0-5v for 15-5 oC. As temperature drops from 15C to 5C, voltage increases from 0v to 5v.
// Output 5v if sensors disconnected. If all sensors are invalid, outputs steady 5V (PWM 255).
//
// A1 -> Node -> NTC 10K to GND, and 10K resistor to 5V
// A2 -> same
// A3 -> same

// 12V signal to disable the gyro logic (kill switch)
// 12V -> 3.8V High for Nano
// 12V -> 10K --|-- 4.7K -> GND
//              D2

#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;

// ---------------- Kill Switch ---------
const int overridePin = 2;

// ---------------- GYRO ----------------
float pitch = 0.0;
float roll  = 0.0;
float yaw   = 0.0;   // drifts over time
unsigned long lastTime = 0;

const int analogInPin = A0;
const int analogOutPin = 9;   // gyro/throttle output PWM

// ---------------- TEMPERATURE ----------------
#define RT0 10000.0  //
#define B 3977.0     //  K
#define VCC 5.0      //Supply  voltage
#define R 10000.0    //R=10K

const int analogOutTempPin = 10; // temperature output PWM
float T0;

// //Variables temperature
// float RT, VR, ln, TX, T0, VRT;
// float Vout;
// int pwmValueTemp;

// Temp mapping:
// 15C -> 0V
//  5C -> 5V
float T_low  = 5.0;
float T_high = 15.0;
float V_high = 5.0;
float V_low  = 0.0;

// ADC thresholds to detect open/short/disconnect
const int ADC_MIN_VALID = 2;     // anything <=2 treated as invalid
const int ADC_MAX_VALID = 1021;  // anything >=1021 treated as invalid

// ---------------- HELPERS ----------------
float gxOffset = 0, gyOffset = 0, gzOffset = 0;
bool anglesInitialized = false;

void calibrateGyro() {
  long sx = 0, sy = 0, sz = 0;
  const int samples = 1000;

  Serial.println("Calibrating gyro... keep still");
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sx += gx;
    sy += gy;
    sz += gz;
    delay(2);
  }

  gxOffset = sx / (float)samples;
  gyOffset = sy / (float)samples;
  gzOffset = sz / (float)samples;
}

float mapTemperatureToVoltage(float T) {
  float V = V_high + (T - T_low) * (V_low - V_high) / (T_high - T_low);

  // Clamp output
  if (V > V_high) V = V_high;
  if (V < V_low)  V = V_low;

  return V;
}

float readThermistorC(int pin) {
  int raw = analogRead(pin);

  if (raw <= ADC_MIN_VALID || raw >= ADC_MAX_VALID) {
    return 999.0; // invalid
  }

  float vrt = (VCC / 1023.0) * raw;
  float vr  = VCC - vrt;

  if (vr <= 0.001) return 999.0;

  float rt = vrt / (vr / R);
  if (rt <= 0.0) return 999.0;

  float lnR = log(rt / RT0);
  float tempK = 1.0 / ((lnR / B) + (1.0 / T0));
  float tempC = tempK - 273.15;

  return tempC;
}

void setup() {
  Serial.begin(9600);

  pinMode(analogOutPin, OUTPUT);
  pinMode(analogOutTempPin, OUTPUT);
  pinMode(overridePin, INPUT);

  T0 = 25.0 + 273.15;  //Temperature conversion from Celsius to kelvin

  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not found");
    while (1);
  }

  delay(1000); // let sensor settle
  calibrateGyro(); //dont move sensor 1 second
  lastTime = micros();

  Serial.println("System ready");
}
float VoutTemp;
  int pwmValueTemp;

void loop() {
  // ================= TEMPERATURE SECTION =================
  int bestPin = -1;
  float bestTemp = 9999.0;
  bool anyValid = false;

  for (int pin = A1; pin <= A3; pin++) {
    float tempC = readThermistorC(pin);

    if (tempC < 900.0) {
      if (tempC < bestTemp) {
        bestTemp = tempC;
        bestPin = pin;
        anyValid = true;
      }
    }
  }

  if (!anyValid) {
    VoutTemp = 5.0;
    pwmValueTemp = 255;
    analogWrite(analogOutTempPin, pwmValueTemp);
  } else {
    VoutTemp = mapTemperatureToVoltage(bestTemp);
    pwmValueTemp = (int)((VoutTemp / 5.0) * 255.0);
    pwmValueTemp = constrain(pwmValueTemp, 0, 255);
    analogWrite(analogOutTempPin, pwmValueTemp);
  }

  // ================= GYRO SECTION =================
  int16_t axRaw, ayRaw, azRaw;
  int16_t gxRaw, gyRaw, gzRaw;
  int16_t tempRaw;

  mpu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);
  tempRaw = mpu.getTemperature();

  // Time delta in seconds
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // Convert raw gyro to deg/sec (default MPU6050 sensitivity = 131 LSB/deg/s)
  float gx = (gxRaw - gxOffset) / 131.0;
float gy = (gyRaw - gyOffset) / 131.0;
float gz = (gzRaw - gzOffset) / 131.0;
  // float gx = gxRaw / 131.0;
  // float gy = gyRaw / 131.0;
  // float gz = gzRaw / 131.0;

  // Accelerometer angle estimates
  float ax = (float)axRaw;
  float ay = (float)ayRaw;
  float az = (float)azRaw;

  float accelPitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  float accelRoll  = atan2(-ax, az) * 180.0 / PI;

  if (!anglesInitialized) {
  pitch = accelPitch;
  roll = accelRoll;
  anglesInitialized = true;
}

  // Complementary filter
  // 0.98 gyro + 0.02 accel is a good starting point
  pitch = 0.98 * (pitch + gx * dt) + 0.02 * accelPitch;
  roll  = 0.98 * (roll  + gy * dt) + 0.02 * accelRoll;
  //pitch = 0.95 * (pitch + gx * dt) + 0.05 * accelPitch;
  //roll  = 0.95 * (roll  + gy * dt) + 0.05 * accelRoll;
  // Yaw from gyro only -> drifts
  yaw += gz * dt;

  float mpuTempC = (tempRaw / 340.0) + 36.53;

  // ================= ANALOG INPUT -> GYRO OUTPUT =================
  int sensorValue = analogRead(analogInPin);
  sensorValue = constrain(sensorValue, 0, 1023);

  float inputVolts = sensorValue * (5.0 / 1023.0);
  float outputVolts = inputVolts;

//kill switch
  bool overrideActive = digitalRead(overridePin);

  if (overrideActive) {
    // Override ON → bypass gyro completely
    outputVolts = inputVolts;
  }
  else {
    // Normal gyro behavior
    // Only override output when input is near full 5V
    if (inputVolts >= 4.8) {
      if (pitch >= 55.0 && pitch <= 70.0) {
        outputVolts = -0.2 * pitch + 16.0; // 55->5V, 60->4V, 65->3V, 70->2V
      } else if (pitch > 70.0) {
        outputVolts = 2.0;
      } else {
        outputVolts = 5.0;
      }
    }
  }

// ================= Safety =================
  //input disconnected?
  bool inputDisconnected = (inputVolts < 0.25);

  if (inputDisconnected) 
  {outputVolts = 0.0;}
// Safety: hard roll left\right cut power
  if(abs(roll) >= 70 ) 
  {outputVolts = 0.0;}

  outputVolts = constrain(outputVolts, 0.0, 5.0);

// ================= out =================
  int pwmValue = (int)(outputVolts / 5.0 * 255.0);
  pwmValue = constrain(pwmValue, 0, 255);
  analogWrite(analogOutPin, pwmValue);

  Serial.print("Pitch: ");
  Serial.print(pitch, 1);
  Serial.print(" deg, Roll: ");
  Serial.print(roll, 1);
  Serial.print(" deg, Temp: ");
  Serial.print(mpuTempC, 1);
  Serial.print(" C, In: ");
  Serial.print(inputVolts, 2);
  Serial.print(" V, Out: ");
  Serial.print(outputVolts, 2);
  Serial.print(" V, PWM: ");
  Serial.print(pwmValue);

  Serial.print(" | Coldest Temp: ");
  if (anyValid) {
    Serial.print(bestTemp, 2);
    Serial.print(" C on A");
    Serial.print(bestPin - A0);
  } else {
    Serial.print("INVALID");
  }
if(overrideActive){Serial.print("override Active");}
  Serial.print(" | Temp Out: ");
  Serial.print(VoutTemp, 2);
  Serial.print(" V, PWM T: ");
  Serial.println(pwmValueTemp);

  

  delay(20);
}