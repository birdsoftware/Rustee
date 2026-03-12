// ODB_II to HW-184
// red	12V+
// black	ground chassis and signal ground
// green	CANL
// white	CANH

// HW-184 _> Nano
// Vcc ->   5V
// GND ->   GND
// CS ->    D10
// SO -     D12
// SI ->    D11
// SCK ->   D13
// INT ->   D2

// Why this is better
// This version:
// reads all queued frames
// shows only changed frames
// makes shifter-related bytes easier to spot
// avoids drowning in repeating messages

#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

// Store last seen payload for standard IDs 0x000-0x7FF
bool seen[2048];
byte lastLen[2048];
byte lastData[2048][8];

void printFrame(unsigned long rxId, byte len, byte *buf) {
  Serial.print("ID: 0x");
  if (rxId < 0x100) Serial.print("0");
  if (rxId < 0x10)  Serial.print("0");
  Serial.print(rxId, HEX);

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

bool changedFromLast(unsigned long rxId, byte len, byte *buf) {
  if (rxId > 0x7FF) return true; // ignore storage for extended frames

  if (!seen[rxId]) {
    seen[rxId] = true;
    lastLen[rxId] = len;
    for (byte i = 0; i < 8; i++) lastData[rxId][i] = buf[i];
    return true;
  }

  if (lastLen[rxId] != len) {
    lastLen[rxId] = len;
    for (byte i = 0; i < 8; i++) lastData[rxId][i] = buf[i];
    return true;
  }

  for (byte i = 0; i < len; i++) {
    if (lastData[rxId][i] != buf[i]) {
      lastLen[rxId] = len;
      for (byte j = 0; j < 8; j++) lastData[rxId][j] = buf[j];
      return true;
    }
  }

  return false;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Init OK");
  } else {
    Serial.println("MCP2515 Init FAIL");
    Serial.println("Try checking wiring / speed / crystal.");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("Gear finder sniffer ready");
}

void loop() {
  while (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) {
      break;
    }

    // Focus on standard 11-bit IDs first
    rxId &= 0x1FFFFFFF;

    if (changedFromLast(rxId, len, buf)) {
      printFrame(rxId, len, buf);
    }
  }
}