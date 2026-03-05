#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastReqMs = 0;
unsigned long lastPrintMs = 0;

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
  {0x49}, // APP D (pedal)
  {0x11}, // Throttle plate
  {0x5E}, // Fuel rate (optional)
};
uint8_t reqIndex = 0;

// Latest values + “seen” flags
float rpm=0, mph=0, loadPct=0, ectF=0, iatF=0, volts=0, fuelPct=0, maf=0, app=0, th=0, fuelRateLph=0;
bool seen0C=false, seen0D=false, seen04=false, seen05=false, seen0F=false, seen42=false, seen2F=false, seen10=false, seen49=false, seen11=false, seen5E=false;

void sendPid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
  CAN0.sendMsgBuf(0x7DF, 0, 8, req);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 Init FAIL (try MCP_16MHZ or 250kbps)");
    while (1) {}
  }
  CAN0.setMode(MCP_NORMAL);
  Serial.println("OBD Dashboard: RPM MPH Load ECT IAT Volts Fuel MAF APP TH FuelRate");
}

void loop() {
  unsigned long now = millis();

  // Send one request every 40ms (~25 req/s total)
  if (now - lastReqMs >= 40) {
    lastReqMs = now;
    sendPid(reqs[reqIndex].pid);
    reqIndex = (reqIndex + 1) % (sizeof(reqs) / sizeof(reqs[0]));
  }

  // Read frames
  if (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t rxId;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) return;
    if (len != 8) return;
    if (rxId < 0x7E8 || rxId > 0x7EF) return;
    if (buf[1] != 0x41) return; // Mode 01 response

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

      case 0x10: { // MAF
        uint16_t raw = ((uint16_t)A << 8) | B;
        maf = raw / 100.0f; // g/s
        seen10=true;
      } break;

      case 0x49: { // APP D
        app = A * 100.0f / 255.0f;
        seen49=true;
      } break;

      case 0x11: { // Throttle plate
        th = A * 100.0f / 255.0f;
        seen11=true;
      } break;

      case 0x5E: { // Fuel rate (optional)
        uint16_t raw = ((uint16_t)A << 8) | B;
        fuelRateLph = raw / 20.0f;
        seen5E=true;
      } break;
    }
  }

  // Print 5x/sec
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    Serial.print("RPM:");  if(seen0C) Serial.print(rpm,0); else Serial.print("-");
    Serial.print("  MPH:"); if(seen0D) Serial.print(mph,1); else Serial.print("-");
    Serial.print("  Load:"); if(seen04) Serial.print(loadPct,1); else Serial.print("-");
    Serial.print("%  ECT:"); if(seen05) Serial.print(ectF,0); else Serial.print("-");
    Serial.print("F  IAT:"); if(seen0F) Serial.print(iatF,0); else Serial.print("-");
    Serial.print("F  V:"); if(seen42) Serial.print(volts,2); else Serial.print("-");
    Serial.print("  Fuel:"); if(seen2F) Serial.print(fuelPct,1); else Serial.print("-");
    Serial.print("%  MAF:"); if(seen10) Serial.print(maf,1); else Serial.print("-");
    Serial.print("g/s  APP:"); if(seen49) Serial.print(app,1); else Serial.print("-");
    Serial.print("%  TH:"); if(seen11) Serial.print(th,1); else Serial.print("-");
    Serial.print("%  FR:"); if(seen5E) Serial.print(fuelRateLph,1); else Serial.print("-");
    Serial.println("L/h");
  }
}
