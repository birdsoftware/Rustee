#include <SPI.h>
#include <mcp_can.h>

MCP_CAN CAN0(10);

int APP_RAW_MIN = 690;
int APP_RAW_MAX = 1870;

int currentGear = -1;
int appPercent = 0;

// ---------------- helpers ----------------
uint16_t makeU16(uint8_t hi, uint8_t lo) {
  return ((uint16_t)hi << 8) | lo;
}

int calcApp(int raw) {
  long pct = (long)(raw - APP_RAW_MIN) * 100L / (APP_RAW_MAX - APP_RAW_MIN);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)pct;
}

const char* gearName(int g) {
  switch (g) {
    case 0: return "P";
    case 1: return "R";
    case 2: return "N";
    case 3: return "D";
    default: return "M";
  }
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN FAIL");
    while(1);
  }

  CAN0.setMode(MCP_NORMAL);

  Serial.println("READY - APP + GEAR");
}

// ---------------- loop ----------------
void loop() {
  if (!digitalRead(2)) {

    unsigned long id;
    byte len;
    byte buf[8];

    if (CAN0.readMsgBuf(&id, &len, buf) == CAN_OK) {

      uint16_t sid = (uint16_t)id;

      // ===== APP =====
      if (sid == 0x108 && len >= 2) {
        int raw = makeU16(buf[0], buf[1]);
        appPercent = calcApp(raw);
      }

      // ===== GEAR =====
      if (sid == 0x126 && len >= 7) {
        int gearIndex = buf[6] >> 4;  // THIS IS THE KEY
        currentGear = gearIndex;
      }

      // ===== PRINT =====
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 200) {
        lastPrint = millis();

        Serial.print("APP: ");
        Serial.print(appPercent);
        Serial.print("%  GEAR: ");

        if (currentGear >= 0)
          Serial.print(gearName(currentGear));
        else
          Serial.print("?");

        Serial.print(" (");
        Serial.print(currentGear);
        Serial.println(")");
      }
    }
  }
}