#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

unsigned long lastReqMs = 0;
unsigned long lastPrintMs = 0;

// --------------------------------------------------
// Request scheduler
// --------------------------------------------------
enum ReqType : uint8_t {
  REQ_MODE01,
  REQ_MODE22
};

struct RequestItem {
  ReqType type;
  uint8_t service;
  uint16_t pid;
  uint16_t header;
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

  {REQ_MODE22, 0x22, 0x1E12, 0x7E1}, // TCM selector / commanded state
  {REQ_MODE22, 0x22, 0x1E1F, 0x7E1}, // TCM engaged / actual state
};

const uint8_t NUM_REQS = sizeof(reqs) / sizeof(reqs[0]);
uint8_t reqIndex = 0;

// --------------------------------------------------
// Standard OBD values
// --------------------------------------------------
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

// --------------------------------------------------
// Ford TCM raw values
// --------------------------------------------------
uint8_t gearCmdRaw = 0xFF;   // 221E12
uint8_t gearEngRaw = 0xFF;   // 221E1F
bool seen1E12 = false;
bool seen1E1F = false;

// --------------------------------------------------
// Helpers
// --------------------------------------------------
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

const char* decodeGearCmd(uint8_t v) {
  switch (v) {
    case 0x46: return "P";
    case 0x3C: return "R";
    case 0x32: return "N";
    case 0x01: return "M1";
    case 0x02: return "M2";
    default:   return "?";
  }
}

const char* decodeGearEng(uint8_t v) {
  switch (v) {
    case 0x46: return "P";
    case 0x80: return "D";
    case 0x82: return "D*";   // drive / under-load state seen in your log
    case 0x01: return "1";
    case 0x02: return "2";
    case 0x03: return "3";
    case 0x04: return "4";
    case 0x05: return "5";
    case 0x06: return "6";
    case 0x07: return "7";
    case 0x08: return "8";
    case 0x09: return "9";
    case 0x0A: return "10";
    default:   return "?";
  }
}

void printHex2(uint8_t v) {
  if (v < 0x10) Serial.print("0");
  Serial.print(v, HEX);
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 Init FAIL (try MCP_16MHZ or different speed)");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);

  Serial.println("Ford F-550 OBD + TCM Dashboard");
  Serial.println("Mode01 via 7DF, Mode22 via TCM 7E1");
  Serial.println("TCM PIDs: 221E12 selector, 221E1F engaged");
}

// --------------------------------------------------
// Loop
// --------------------------------------------------
void loop() {
  unsigned long now = millis();

  // One request every 50 ms
  if (now - lastReqMs >= 50) {
    lastReqMs = now;
    sendRequest(reqs[reqIndex]);
    reqIndex = (reqIndex + 1) % NUM_REQS;
  }

  // Read all pending CAN frames
  while (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t rxId = 0;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;
    if (len < 3) continue;

    // Positive Mode 01 replies: 0x41
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

      // Positive Mode 22 replies: 0x62
      if (buf[1] == 0x62 && len >= 5) {
        uint16_t pid22 = ((uint16_t)buf[2] << 8) | buf[3];
        uint8_t A = buf[4];

        switch (pid22) {
          case 0x1E12:
            gearCmdRaw = A;
            seen1E12 = true;
            break;

          case 0x1E1F:
            gearEngRaw = A;
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

    Serial.print("  Sel:");
    if (seen1E12) Serial.print(decodeGearCmd(gearCmdRaw)); else Serial.print("-");
    Serial.print("(");
    if (seen1E12) printHex2(gearCmdRaw); else Serial.print("--");
    Serial.print(")");

    Serial.print("  Eng:");
    if (seen1E1F) Serial.print(decodeGearEng(gearEngRaw)); else Serial.print("-");
    Serial.print("(");
    if (seen1E1F) printHex2(gearEngRaw); else Serial.print("--");
    Serial.print(")");

    Serial.println();
  }
}