// Ford Mode 22 direct requests to the TCM at 0x7E1 for:

// 221E12 → Commanded Gear
// 2211B3 → Transmission Range Status
// optional fallback 221E1F → Transmission Gear Engaged

// I also decode the PRNDM status the way you described.
//The 2024 Super Duty uses the same core extended PID architecture as other modern Ford trucks for gear reporting. PID: 221E12 Long Name: 
//Commanded Gear Short Name: Gear OBD Header: 7E1 (Direct TCM request) or 7E0 (PCM request) Min/Max: 0 / 10 Equation: A (The raw byte returned typically corresponds 
//directly to the gear number 1–10) Alternative (if 1E12 fails): Try PID 221E1F for "Transmission Gear Engaged". 2. Transmission Range (PRNDM Status) This PID tracks 
//the physical position of the gear selector lever or dial. On the 10R140, this is often reported as a status code rather than raw voltage. 
//PID: 2211B3 Long Name: Transmission Range Status OBD Header: 7E1 Equation: A Expected Values (Hex to Status): 00 = In Between / Transition 01 = Park 02 = Reverse 
//03 = Neutral 04 = Drive 05 = Manual / Sport Critical Setup Tips Module Selection: While some data is mirrored in the PCM (7E0), the TCM (7E1) is the high-confidence 
//source for real-time gear changes.
#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastReqMs = 0;
unsigned long lastPrintMs = 0;

// -----------------------------
// Request scheduler
// -----------------------------
enum ReqType : uint8_t {
  REQ_MODE01,
  REQ_MODE22
};

struct RequestItem {
  ReqType type;
  uint8_t service;   // 0x01 or 0x22
  uint16_t pid;      // for Mode 01 use low byte only
  uint16_t header;   // 0x7DF broadcast or 0x7E1 direct TCM
};

const RequestItem reqs[] = {
  {REQ_MODE01, 0x01, 0x000C, 0x7DF}, // RPM
  {REQ_MODE01, 0x01, 0x000D, 0x7DF}, // Speed
  {REQ_MODE01, 0x01, 0x0004, 0x7DF}, // Engine load
  {REQ_MODE01, 0x01, 0x0005, 0x7DF}, // Coolant temp
  {REQ_MODE01, 0x01, 0x000F, 0x7DF}, // Intake air temp
  {REQ_MODE01, 0x01, 0x0042, 0x7DF}, // Control module voltage
  {REQ_MODE01, 0x01, 0x002F, 0x7DF}, // Fuel level
  {REQ_MODE01, 0x01, 0x0010, 0x7DF}, // MAF
  {REQ_MODE01, 0x01, 0x0049, 0x7DF}, // APP D
  {REQ_MODE01, 0x01, 0x0011, 0x7DF}, // Throttle
  {REQ_MODE01, 0x01, 0x005E, 0x7DF}, // Fuel rate

  {REQ_MODE22, 0x22, 0x1E12, 0x7E1}, // TCM Commanded Gear
  {REQ_MODE22, 0x22, 0x11B3, 0x7E1}, // TCM Transmission Range Status
  {REQ_MODE22, 0x22, 0x1E1F, 0x7E1}, // fallback: Transmission Gear Engaged
};

const uint8_t NUM_REQS = sizeof(reqs) / sizeof(reqs[0]);
uint8_t reqIndex = 0;

// -----------------------------
// Live values
// -----------------------------
float rpm = 0, mph = 0, loadPct = 0, ectF = 0, iatF = 0, volts = 0;
float fuelPct = 0, maf = 0, app = 0, th = 0, fuelRateLph = 0;

bool seen0C=false, seen0D=false, seen04=false, seen05=false, seen0F=false;
bool seen42=false, seen2F=false, seen10=false, seen49=false, seen11=false, seen5E=false;

// Ford TCM values
int gearCmd = -1;          // 221E12
int gearEngaged = -1;      // 221E1F fallback
uint8_t trStatus = 0xFF;   // 2211B3
bool seen1E12 = false;
bool seen1E1F = false;
bool seen11B3 = false;

// -----------------------------
// Helpers
// -----------------------------
void sendMode01Pid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
  CAN0.sendMsgBuf(0x7DF, 0, 8, req);
}

void sendMode22Pid(uint16_t header, uint16_t pid) {
  byte req[8] = {
    0x03,
    0x22,
    (uint8_t)(pid >> 8),
    (uint8_t)(pid & 0xFF),
    0, 0, 0, 0
  };
  CAN0.sendMsgBuf(header, 0, 8, req);
}

void sendRequest(const RequestItem &r) {
  if (r.type == REQ_MODE01) {
    sendMode01Pid((uint8_t)(r.pid & 0xFF));
  } else {
    sendMode22Pid(r.header, r.pid);
  }
}

const char* decodeRangeStatus(uint8_t v) {
  switch (v) {
    case 0x00: return "TRANS";
    case 0x01: return "P";
    case 0x02: return "R";
    case 0x03: return "N";
    case 0x04: return "D";
    case 0x05: return "M";
    default:   return "?";
  }
}

// -----------------------------
// Setup
// -----------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 Init FAIL (try MCP_16MHZ or 250kbps)");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);

  Serial.println("Ford OBD + TCM Dashboard");
  Serial.println("Mode01 on 7DF, Mode22 direct to TCM 7E1");
  Serial.println("Watching 221E12 GearCmd, 2211B3 Range, 221E1F GearEngaged");
}

// -----------------------------
// Loop
// -----------------------------
void loop() {
  unsigned long now = millis();

  // one request every 50 ms
  if (now - lastReqMs >= 50) {
    lastReqMs = now;
    sendRequest(reqs[reqIndex]);
    reqIndex = (reqIndex + 1) % NUM_REQS;
  }

  // Read all pending frames
  while (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t rxId;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;
    if (len < 3) continue;

    // -------------------------
    // Mode 01 responses
    // -------------------------
    if (rxId >= 0x7E8 && rxId <= 0x7EF) {
      if (len == 8 && buf[1] == 0x41) {
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
            ectF = c * 9.0f/5.0f + 32.0f;
            seen05 = true;
          } break;

          case 0x0F: {
            float c = (float)A - 40.0f;
            iatF = c * 9.0f/5.0f + 32.0f;
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

      // -------------------------
      // Mode 22 responses
      // positive response = 0x62
      // expected from TCM/PCM reply IDs like 7E9 etc.
      // -------------------------
      if (buf[1] == 0x62 && len >= 5) {
        uint16_t pid22 = ((uint16_t)buf[2] << 8) | buf[3];
        uint8_t A = buf[4];

        switch (pid22) {
          case 0x1E12:
            gearCmd = A;
            seen1E12 = true;
            break;

          case 0x11B3:
            trStatus = A;
            seen11B3 = true;
            break;

          case 0x1E1F:
            gearEngaged = A;
            seen1E1F = true;
            break;
        }
      }
    }
  }

  // Print 5x/sec
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    Serial.print("RPM:");
    if (seen0C) Serial.print(rpm, 0); else Serial.print("-");

    Serial.print("  MPH:");
    if (seen0D) Serial.print(mph, 1); else Serial.print("-");

    Serial.print("  Load:");
    if (seen04) Serial.print(loadPct, 1); else Serial.print("-");
    Serial.print("%");

    Serial.print("  ECT:");
    if (seen05) Serial.print(ectF, 0); else Serial.print("-");
    Serial.print("F");

    Serial.print("  IAT:");
    if (seen0F) Serial.print(iatF, 0); else Serial.print("-");
    Serial.print("F");

    Serial.print("  V:");
    if (seen42) Serial.print(volts, 2); else Serial.print("-");

    Serial.print("  Fuel:");
    if (seen2F) Serial.print(fuelPct, 1); else Serial.print("-");
    Serial.print("%");

    Serial.print("  MAF:");
    if (seen10) Serial.print(maf, 1); else Serial.print("-");
    Serial.print("g/s");

    Serial.print("  APP:");
    if (seen49) Serial.print(app, 1); else Serial.print("-");
    Serial.print("%");

    Serial.print("  TH:");
    if (seen11) Serial.print(th, 1); else Serial.print("-");
    Serial.print("%");

    Serial.print("  FR:");
    if (seen5E) Serial.print(fuelRateLph, 1); else Serial.print("-");
    Serial.print("L/h");

    Serial.print("  GearCmd:");
    if (seen1E12) Serial.print(gearCmd); else Serial.print("-");

    Serial.print("  Range:");
    if (seen11B3) Serial.print(decodeRangeStatus(trStatus)); else Serial.print("-");

    Serial.print("  GearEng:");
    if (seen1E1F) Serial.print(gearEngaged); else Serial.print("-");

    Serial.println();
  }
}