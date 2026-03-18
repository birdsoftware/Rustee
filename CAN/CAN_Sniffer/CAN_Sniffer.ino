// green white stripe	12?
// orange	ground chassis
// brown	CANL
// green	CANH
// yellow	ground signal
// HW-184 Nano
// Vcc -> 5V
// GND -> GND
// CS -> D10
// SO - D12
// SI -> D11
// SCK -> D13
// INT - D2
// 115200 baud

#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;  // CS to D10
const int CAN_INT_PIN = 2;   // INT to D2

MCP_CAN CAN0(CAN_CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CAN_INT_PIN, INPUT);

  // Most Ford powertrain CAN is 500 kbps.
  // HW-184 modules are commonly 8MHz crystals. If yours is 16MHz, change MCP_8MHZ to MCP_16MHZ.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Init OK");
  } else {
    Serial.println("MCP2515 Init FAIL");
    Serial.println("Check wiring, crystal (8MHz vs 16MHz), and speed (500k vs 250k).");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("CAN Sniffer Ready");
}

void loop() {
  // INT goes LOW when a frame arrives
  if (digitalRead(CAN_INT_PIN) == LOW) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
      Serial.print("ID: 0x");
      Serial.print(rxId, HEX);

      // Distinguish standard vs extended IDs
      if ((rxId & 0x80000000) == 0x80000000) {
        Serial.print(" (EXT) ");
      } else {
        Serial.print(" (STD) ");
      }

      Serial.print(" DLC: ");
      Serial.print(len);
      Serial.print(" Data: ");

      for (byte i = 0; i < len; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
}