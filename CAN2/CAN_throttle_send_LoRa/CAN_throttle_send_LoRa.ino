// THis runs on an a arduino nano. Nano UART analog write to LoRa

// CAN ODB-II -> [HW187] -> [Arduino nano] -> [LoRa V3] ~.~.~> [LoRa V2]
// CAN ODB-II GND and 12V+ -> [DC-DC Voltage adjuster] -> 5V for nano and LoRa

// Pot to adjust max PWM out b.bird 3.3.26
// Right outer leg → 5V
// Middle (wiper) → A1
// Other outer leg → GND
//|----dead zone----|---------------- usable range ----------------|
//0%                10%                                             100%
//PWM=0             start ramping                                   PWM=235

// update lora
// UART to Lora Analog to pin RX_PIN 19 GPIO.

// input
// CAN pedal position used for input signal

// INPUT HW-184 wiring
// HW-184 CAN-H -> ODB-II Pin 6 CAN High
// HW-184 CAN-L -> ODB-II Pin 14 CAN LOW
// HW-184 Vcc -> 5V Nano
// HW-184 GND -> GND Nano
// HW-184 CS -> D10 Nano
// HW-184 SO -> D12 Nano
// HW-184 SI -> D11 Nano
// HW-184 SCK -> D13 Nano
// HW-184 INT -> D2 Nano

// Nano wiring
// Nano TX -> 1K -> Node -> 2K -> GND
// Node -> RX LoRa

// ODB-II wiring
// ODB-II pin 16 12V -> +IN DC-DC Converter
// ODB-II pin 4 GND Chassis -> -IN DC-DC Converter
// ODB-II pin 5 Signal GND -> -IN DC-DC Converter



#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;
const int potPin      = A1;

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastReqMs = 0;
unsigned long lastPrintMs = 0;
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
  {0x49}, // APP D (pedal)
  {0x11}, // Throttle plate
};
uint8_t reqIndex = 0;

// Latest values + “seen” flags
float rpm=0, mph=0, loadPct=0, ectF=0, iatF=0, volts=0, fuelPct=0, maf=0, app=0, th=0;
bool seen0C=false, seen0D=false, seen04=false, seen05=false, seen0F=false, seen42=false, seen2F=false, seen49=false, seen11=false;

// Dynamic pedal min/max
float appMin = 14.9f;//15.3
float appMax = 18.0f;//80.4f; //78.8 //adjust the max V
const float APP_EPSILON = 0.15f; // Prevent tiny noise from constantly changing min/max

// Output values
float vin = 0.7f;          // mapped continuous voltage from APP
float discreteVin = 0.7f;  // stepped output voltage
int pwm = 0;

void updatePedalMinMax(float appRaw) {
  if (appRaw < (appMin - APP_EPSILON)) {
    appMin = appRaw;
  }

  if (appRaw > (appMax + APP_EPSILON)) {
    appMax = appRaw;
  }
}

// --------------------------------------
// Map CAN APP % to continuous 0.7V..3.3V
// --------------------------------------
float mapPedalToVoltage(float appRaw) {
  float range = appMax - appMin;
  if (range < 0.5f) range = 0.5f;  // avoid divide by zero / tiny range

  float x = (appRaw - appMin) / range;
  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;

  return 0.7f + x * (3.3f - 0.7f);
}

void sendPid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
  CAN0.sendMsgBuf(0x7DF, 0, 8, req);
}

void setup() {
  analogReference(DEFAULT); //keep for pot?
  Serial.begin(9600);   // keep this at 9600 for LoRa UART

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 Init FAIL (try MCP_16MHZ or 250kbps)");
    while (1) {}
  }
  CAN0.setMode(MCP_NORMAL);
}

void loop() {
  unsigned long now = millis();

  // CAN polling, Send one request every 40ms (~25 req/s total)
  if (now - lastReqMs >= 40) {
    lastReqMs = now;
    sendPid(reqs[reqIndex].pid);
    reqIndex = (reqIndex + 1) % (sizeof(reqs) / sizeof(reqs[0]));
  }

  // Read frames
  while (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t rxId;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;
    if (len != 8) continue;
    if (rxId < 0x7E8 || rxId > 0x7EF) continue;
    if (buf[1] != 0x41) continue; // Mode 01 response

    uint8_t pid = buf[2];
    uint8_t A = buf[3];
    uint8_t B = buf[4];

    switch (pid) {
      case 0x0C: { // RPM
        uint16_t raw = ((uint16_t)A << 8) | B;
        rpm = raw / 4.0f;
        seen0C=true;
      } break;

      case 0x0D: { // Speed km/h
        float kph = A;
        mph = kph * 0.621371f;
        seen0D=true;
      } break;

      case 0x04: { // Engine load
        loadPct = A * 100.0f / 255.0f;
        seen04=true;
      } break;

      case 0x05: { // Coolant temp
        float c = (float)A - 40.0f;
        ectF = c * 9.0f/5.0f + 32.0f;
        seen05=true;
      } break;

      case 0x0F: { // Intake air temp
        float c = (float)A - 40.0f;
        iatF = c * 9.0f/5.0f + 32.0f;
        seen0F=true;
      } break;

      case 0x42: { // Control module voltage
        uint16_t raw = ((uint16_t)A << 8) | B;
        volts = raw / 1000.0f;
        seen42=true;
      } break;

      case 0x2F: { // Fuel level
        fuelPct = A * 100.0f / 255.0f;
        seen2F=true;
      } break;

      // case 0x10: { // MAF
      //   uint16_t raw = ((uint16_t)A << 8) | B;
      //   maf = raw / 100.0f; // g/s
      //   seen10=true;
      // } break;

      case 0x49: { // APP D - foot pedal position
        app = A * 100.0f / 255.0f;
        seen49=true;
        updatePedalMinMax(app);
      } break;

      case 0x11: { // Throttle plate
        th = A * 100.0f / 255.0f;
        seen11=true;
      } break;

      // case 0x5E: { // Fuel rate (optional)
      //   uint16_t raw = ((uint16_t)A << 8) | B;
      //   fuelRateLph = raw / 20.0f;
      //   seen5E=true;
      // } break;
    }
  }

  // Output / LoRa serial line every 500 ms
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    if (seen49) {
      vin = mapPedalToVoltage(app);

      //KEEP FOR ANALOG OUTPUT ON 5V NANO SYSTEM
      // pwm = (int)(vin * 255.0f / 5.0f + 0.5f);
      // if (pwm < 0) pwm = 0;
      // if (pwm > 180) pwm = 180;  // cap output

      //UPDATE SENDING TO LoRA 3.3V SYSTEM
      pwm = (int)(vin * 255.0f / 3.33f + 0.5f);
      if (pwm < 0) pwm = 0;
      if (pwm > 255) pwm = 255;  // cap output

      // Pot sets max clamp
      int potADC = analogRead(potPin);
      int pwmMax = map(potADC, 0, 1023, 0, 255); //was 180
      if (pwm > pwmMax) pwm = pwmMax;
    } else {
      // safe default before pedal seen
      vin = 0.7f;
      //KEEP FOR ANALOG OUTPUT ON 5V NANO SYSTEM
      //pwm = (int)(vin * 255.0f / 5.0f + 0.5f);
      pwm = (int)(vin * 255.0f / 3.33f + 0.5f);

      // Pot sets max clamp
      int potADC = analogRead(potPin);
      int pwmMax = map(potADC, 0, 1023, 0, 255); //was 180
      if (pwm > pwmMax) pwm = pwmMax;
    }

    n++;
    if (n > 99) n = 0;

    // This is what LoRa reads over UART at 9600
    Serial.println(
      String(pwm) +
      " I: " + String(vin, 3) +
      //" APP: " + String(app, 2) + 
      " #" + String(n)
    );
  }
}
