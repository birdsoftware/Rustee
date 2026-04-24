#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

uint16_t brakeRaw = 0;
uint16_t lastBrake = 0;

float brakePct = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN FAIL");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);

  Serial.println("Brake Detector (0x11C)");
}

void loop() {
  while (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t id;
    uint8_t len;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&id, &len, buf) != CAN_OK) break;

    if (id == 0x11C && len >= 4) {
      brakeRaw = ((uint16_t)buf[2] << 8) | buf[3];

      // ignore tiny noise
      if (abs((int)brakeRaw - (int)lastBrake) > 3) {
        lastBrake = brakeRaw;

        // scale 0–2050 → 0–100%
        brakePct = (brakeRaw / 2050.0f) * 100.0f;

        Serial.print("Brake: ");
        Serial.print(brakePct, 1);
        Serial.print("%   Raw:");
        Serial.println(brakeRaw);
      }
    }
  }
}