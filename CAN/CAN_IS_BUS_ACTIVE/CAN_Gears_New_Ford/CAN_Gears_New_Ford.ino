#include <SPI.h>
#include <mcp_can.h>

const byte CAN_CS_PIN  = 10;
const byte CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

// ===== change this range =====
const uint16_t START_ID = 0x300;//6
const uint16_t END_ID   = 0x3FF;//6
// then try 0x300-0x3FF, then 0x700-0x7FF
// =============================

const byte NUM_PHASES = 5;   // P R N D M
const byte MAX_IDS    = 8;   // keep Uno RAM safe

const char phaseName[NUM_PHASES] = {'P','R','N','D','M'};

struct Track {
  uint16_t id;
  bool used;
  byte vals[8][NUM_PHASES];
  byte seenMask[8];   // bit0=P bit1=R bit2=N bit3=D bit4=M
};

Track tracks[MAX_IDS];

byte curPhase = 0;
unsigned long phaseStart = 0;
const unsigned long PHASE_MS = 2500;
bool captureStarted = false;
unsigned int rxCount = 0;

void clearTracks() {
  for (byte i = 0; i < MAX_IDS; i++) {
    tracks[i].used = false;
    tracks[i].id = 0;
    for (byte b = 0; b < 8; b++) {
      tracks[i].seenMask[b] = 0;
      for (byte p = 0; p < NUM_PHASES; p++) {
        tracks[i].vals[b][p] = 0;
      }
    }
  }
}

int findTrack(uint16_t id) {
  for (byte i = 0; i < MAX_IDS; i++) {
    if (tracks[i].used && tracks[i].id == id) return i;
  }
  return -1;
}

int allocTrack(uint16_t id) {
  for (byte i = 0; i < MAX_IDS; i++) {
    if (!tracks[i].used) {
      tracks[i].used = true;
      tracks[i].id = id;
      return i;
    }
  }
  return -1;
}

void printHex2(byte v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

bool phaseSeen(const Track &t, byte b, byte p) {
  return (t.seenMask[b] & (1 << p)) != 0;
}

byte countDistinct(const Track &t, byte b) {
  byte distinct = 0;

  for (byte i = 0; i < NUM_PHASES; i++) {
    if (!phaseSeen(t, b, i)) continue;

    bool duplicate = false;
    for (byte j = 0; j < i; j++) {
      if (phaseSeen(t, b, j) && t.vals[b][j] == t.vals[b][i]) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) distinct++;
  }

  return distinct;
}

void printResults() {
  Serial.println();
  Serial.println("===== CANDIDATES =====");

  bool found = false;

  for (byte i = 0; i < MAX_IDS; i++) {
    if (!tracks[i].used) continue;

    for (byte b = 0; b < 8; b++) {
      byte distinct = countDistinct(tracks[i], b);

      if (distinct >= 3) {
        found = true;
        Serial.print("ID 0x");
        Serial.print(tracks[i].id, HEX);
        Serial.print(" byte ");
        Serial.print(b);
        Serial.print(" -> ");

        for (byte p = 0; p < NUM_PHASES; p++) {
          Serial.print(phaseName[p]);
          Serial.print(":");
          if (phaseSeen(tracks[i], b, p)) printHex2(tracks[i].vals[b][p]);
          else Serial.print("--");
          Serial.print(" ");
        }

        Serial.print("distinct=");
        Serial.println(distinct);
      }
    }
  }

  if (!found) {
    Serial.println("No candidates found in this range.");
  }

  Serial.println("======================");
  Serial.println();
}

void startCapture() {
  clearTracks();
  captureStarted = true;
  curPhase = 0;
  phaseStart = millis();
  rxCount = 0;

  Serial.println();
  Serial.println("CAPTURE START");
  Serial.print("Range 0x");
  Serial.print(START_ID, HEX);
  Serial.print(" - 0x");
  Serial.println(END_ID, HEX);
  Serial.println("Put gear in P now");
  Serial.println("Hold each gear until prompted");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);
  clearTracks();

  Serial.println("Ford PRNDL phase hunter - low RAM");
  Serial.print("Watching range 0x");
  Serial.print(START_ID, HEX);
  Serial.print(" to 0x");
  Serial.println(END_ID, HEX);
  Serial.println("Type s then Enter to start");
}

void loop() {
  if (!captureStarted && Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      startCapture();
    }
  }

  while (digitalRead(CAN_INT_PIN) == LOW) {
    unsigned long rxId = 0;
    byte len = 0;
    byte buf[8];

    if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) break;
    if (!captureStarted) continue;
    if (rxId > 0x7FF) continue;   // standard only
    if (len != 8) continue;

    uint16_t id = (uint16_t)rxId;
    if (id < START_ID || id > END_ID) continue;

    rxCount++;

    int idx = findTrack(id);
    if (idx < 0) idx = allocTrack(id);
    if (idx < 0) continue; // ignore extra IDs once table full

    for (byte b = 0; b < 8; b++) {
      tracks[idx].vals[b][curPhase] = buf[b];
      tracks[idx].seenMask[b] |= (1 << curPhase);
    }
  }

  if (captureStarted) {
    unsigned long now = millis();
    if (now - phaseStart >= PHASE_MS) {
      curPhase++;
      phaseStart = now;

      if (curPhase >= NUM_PHASES) {
        captureStarted = false;
        Serial.println();
        Serial.println("CAPTURE END");
        Serial.print("Frames in range: ");
        Serial.println(rxCount);
        printResults();
        Serial.println("Type s to run again");
      } else {
        Serial.print("Now shift to ");
        Serial.println(phaseName[curPhase]);
      }
    }
  }
}