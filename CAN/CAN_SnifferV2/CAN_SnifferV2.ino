#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

// --- simple cache of last value per ID ---
struct Entry {
  uint32_t id;
  uint8_t  len;
  uint8_t  data[8];
  bool     used;
};

const uint8_t TABLE_SIZE = 64;
Entry table[TABLE_SIZE];

int findEntry(uint32_t id) {
  // linear search is fine at this size
  for (int i = 0; i < TABLE_SIZE; i++) {
    if (table[i].used && table[i].id == id) return i;
  }
  return -1;
}

int allocEntry(uint32_t id) {
  for (int i = 0; i < TABLE_SIZE; i++) {
    if (!table[i].used) {
      table[i].used = true;
      table[i].id = id;
      table[i].len = 0;
      for (int b = 0; b < 8; b++) table[i].data[b] = 0;
      return i;
    }
  }
  return -1;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(CAN_INT_PIN, INPUT);

  // Try 500 kbps first; change to CAN_250KBPS if needed.
  // Change MCP_8MHZ to MCP_16MHZ if your module crystal is 16MHz.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 Init FAIL (try 250kbps or MCP_16MHZ)");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("CAN Change Tracker Ready");
  Serial.println("Slowly press/release throttle and watch which IDs/bytes change.");
}

void loop() {
  if (digitalRead(CAN_INT_PIN) == LOW) {
    uint32_t rxId;
    uint8_t len = 0;
    uint8_t buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) return;

    int idx = findEntry(rxId);
    if (idx < 0) idx = allocEntry(rxId);
    if (idx < 0) return; // table full

    Entry &e = table[idx];

    // First time: store but don't print a "change"
    if (e.len == 0) {
      e.len = len;
      for (uint8_t i = 0; i < 8; i++) e.data[i] = (i < len) ? buf[i] : 0;
      return;
    }

    // Compare and print only if changed
    bool changed = false;
    for (uint8_t i = 0; i < len; i++) {
      if (e.data[i] != buf[i]) { changed = true; break; }
    }
    if (!changed) return;

    Serial.print("ID:0x");
    Serial.print(rxId, HEX);
    Serial.print(" DLC:");
    Serial.print(len);
    Serial.print("  ");

    // Print which bytes changed: [i old->new]
    Serial.print("chg:");
    for (uint8_t i = 0; i < len; i++) {
      if (e.data[i] != buf[i]) {
        Serial.print(" [");
        Serial.print(i);
        Serial.print(" ");
        if (e.data[i] < 0x10) Serial.print("0");
        Serial.print(e.data[i], HEX);
        Serial.print("->");
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print("]");
      }
    }

    // Store new
    e.len = len;
    for (uint8_t i = 0; i < 8; i++) e.data[i] = (i < len) ? buf[i] : 0;

    Serial.println();
  }
}