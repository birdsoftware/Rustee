#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN  = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

const uint16_t REQ_ID = 0x7DF;
uint16_t ECU_ID = 0;

const uint16_t REQ_INTERVAL_MS   = 80;
const uint16_t PRINT_INTERVAL_MS = 200;

uint32_t sup_01_20 = 0, sup_21_40 = 0, sup_41_60 = 0;
bool gotSup00=false, gotSup20=false, gotSup40=false;

float rpm=0, mph=0, loadPct=0, ectF=0, iatF=0, volts=0, fuelPct=0, app=0, th=0, mapKpa=0, timingDeg=0;
float stft1=0, ltft1=0, baroKpa=0, oilF=0;

bool seen_rpm=false, seen_mph=false, seen_load=false, seen_ect=false, seen_iat=false, seen_volts=false;
bool seen_fuel=false, seen_app=false, seen_th=false, seen_map=false, seen_timing=false;
bool seen_stft=false, seen_ltft=false, seen_baro=false, seen_oil=false;

void sendPid(uint8_t pid) {
  byte req[8] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
  CAN0.sendMsgBuf(REQ_ID, 0, 8, req);
}

bool pidSupported(uint8_t pid) {
  if (pid == 0x00 || pid == 0x20 || pid == 0x40) return true;
  if (pid >= 0x01 && pid <= 0x20) { uint8_t bit = 32 - pid; return (sup_01_20 >> bit) & 1; }
  if (pid >= 0x21 && pid <= 0x40) { uint8_t bit = 64 - pid; return (sup_21_40 >> bit) & 1; }
  if (pid >= 0x41 && pid <= 0x60) { uint8_t bit = 96 - pid; return (sup_41_60 >> bit) & 1; }
  return false;
}

float trimPct(uint8_t A) { // for STFT/LTFT
  return ((float)A - 128.0f) * 100.0f / 128.0f;
}

void handleMode01Response(uint32_t rxId, const uint8_t *buf, uint8_t len) {
  if (rxId != ECU_ID) return;
  if (len != 8) return;
  if (buf[1] != 0x41) return;

  uint8_t pid = buf[2];
  uint8_t A = buf[3], B = buf[4], C = buf[5], D = buf[6];

  if (pid == 0x00) { sup_01_20 = ((uint32_t)A<<24)|((uint32_t)B<<16)|((uint32_t)C<<8)|D; gotSup00=true; return; }
  if (pid == 0x20) { sup_21_40 = ((uint32_t)A<<24)|((uint32_t)B<<16)|((uint32_t)C<<8)|D; gotSup20=true; return; }
  if (pid == 0x40) { sup_41_60 = ((uint32_t)A<<24)|((uint32_t)B<<16)|((uint32_t)C<<8)|D; gotSup40=true; return; }

  switch (pid) {
    case 0x0C: { uint16_t raw=((uint16_t)A<<8)|B; rpm=raw/4.0f; seen_rpm=true; } break;
    case 0x0D: { mph=(float)A*0.621371f; seen_mph=true; } break;
    case 0x04: { loadPct=A*100.0f/255.0f; seen_load=true; } break;
    case 0x05: { float c=(float)A-40.0f; ectF=c*9.0f/5.0f+32.0f; seen_ect=true; } break;
    case 0x0F: { float c=(float)A-40.0f; iatF=c*9.0f/5.0f+32.0f; seen_iat=true; } break;
    case 0x42: { uint16_t raw=((uint16_t)A<<8)|B; volts=raw/1000.0f; seen_volts=true; } break;
    case 0x2F: { fuelPct=A*100.0f/255.0f; seen_fuel=true; } break;
    case 0x49: { app=A*100.0f/255.0f; seen_app=true; } break;
    case 0x11: { th=A*100.0f/255.0f; seen_th=true; } break;
    case 0x0B: { mapKpa=A; seen_map=true; } break;
    case 0x0E: { timingDeg=((float)A/2.0f)-64.0f; seen_timing=true; } break;

    case 0x06: { stft1 = trimPct(A); seen_stft=true; } break;
    case 0x07: { ltft1 = trimPct(A); seen_ltft=true; } break;
    case 0x33: { baroKpa = A; seen_baro=true; } break;

    case 0x5C: { // oil temp (if supported)
      float c=(float)A-40.0f;
      oilF=c*9.0f/5.0f+32.0f;
      seen_oil=true;
    } break;
  }
}

// Poll list (we will filter by supported PIDs after support maps load)
const uint8_t basePids[] = {
  0x0C,0x0D,0x04,0x05,0x0F,0x42,0x2F,0x49,0x11,0x0B,0x0E,
  0x06,0x07,0x33,0x5C
};

uint8_t pollList[sizeof(basePids)];
uint8_t pollCount=0, pollIndex=0;

void buildPollList() {
  pollCount=0;
  for (uint8_t i=0;i<sizeof(basePids);i++) {
    uint8_t pid=basePids[i];
    if (pidSupported(pid)) pollList[pollCount++]=pid;
  }
}

unsigned long lastReqMs=0, lastPrintMs=0, lastDetectMs=0, lastSupportReqMs=0;
bool ecuFound=false, pollReady=false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  pinMode(CAN_INT_PIN, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 Init FAIL (try MCP_16MHZ or CAN_250KBPS)");
    while (1) {}
  }
  CAN0.setMode(MCP_NORMAL);
  Serial.println("Auto-detecting ECU (RPM PID 0x0C)...");
}

void loop() {
  unsigned long now=millis();

  if (!ecuFound && (now-lastDetectMs>=150)) {
    lastDetectMs=now;
    sendPid(0x0C);
  }

  // Drain RX queue
  while (digitalRead(CAN_INT_PIN)==LOW) {
    uint32_t rxId; uint8_t len=0; uint8_t buf[8];
    if (CAN0.readMsgBuf(&rxId,&len,buf)!=CAN_OK) break;

    if (!ecuFound) {
      if (rxId>=0x7E8 && rxId<=0x7EF && len==8 && buf[1]==0x41 && buf[2]==0x0C) {
        ECU_ID=(uint16_t)rxId;
        ecuFound=true;
        Serial.print("ECU detected: 0x"); Serial.println(ECU_ID,HEX);
      }
      continue;
    }

    handleMode01Response(rxId,buf,len);
  }

  // Retry support requests until all received
  if (ecuFound && !pollReady && (now-lastSupportReqMs>=300)) {
    lastSupportReqMs=now;
    if (!gotSup00) sendPid(0x00);
    if (!gotSup20) sendPid(0x20);
    if (!gotSup40) sendPid(0x40);
  }

  if (ecuFound && !pollReady && gotSup00 && gotSup20 && gotSup40) {
    pollReady=true;
    buildPollList();
    Serial.print("Polling "); Serial.print(pollCount); Serial.println(" supported PIDs...");
  }

  if (pollReady && (now-lastReqMs>=REQ_INTERVAL_MS)) {
    lastReqMs=now;
    uint8_t pid=pollList[pollIndex];
    pollIndex=(pollIndex+1)%pollCount;
    sendPid(pid);
  }

  if (pollReady && (now-lastPrintMs>=PRINT_INTERVAL_MS)) {
    lastPrintMs=now;

    Serial.print("RPM:"); Serial.print(seen_rpm?String((int)rpm):"-");
    Serial.print(" MPH:"); Serial.print(seen_mph?String(mph,1):"-");
    Serial.print(" Load:"); Serial.print(seen_load?String(loadPct,1):"-"); Serial.print("%");
    Serial.print(" MAP:"); Serial.print(seen_map?String(mapKpa,0):"-"); Serial.print("kPa");
    Serial.print(" Tim:"); Serial.print(seen_timing?String(timingDeg,1):"-"); Serial.print("deg");
    Serial.print(" ECT:"); Serial.print(seen_ect?String(ectF,0):"-"); Serial.print("F");
    Serial.print(" IAT:"); Serial.print(seen_iat?String(iatF,0):"-"); Serial.print("F");
    Serial.print(" V:");   Serial.print(seen_volts?String(volts,2):"-");
    Serial.print(" Fuel:"); Serial.print(seen_fuel?String(fuelPct,1):"-"); Serial.print("%");
    Serial.print(" APP:"); Serial.print(seen_app?String(app,1):"-"); Serial.print("%");
    Serial.print(" TH:");  Serial.print(seen_th?String(th,1):"-"); Serial.print("%");

    Serial.print(" STFT1:"); Serial.print(seen_stft?String(stft1,1):"-"); Serial.print("%");
    Serial.print(" LTFT1:"); Serial.print(seen_ltft?String(ltft1,1):"-"); Serial.print("%");
    Serial.print(" BARO:"); Serial.print(seen_baro?String(baroKpa,0):"-"); Serial.print("kPa");
    Serial.print(" OIL:");  Serial.print(seen_oil?String(oilF,0):"-"); Serial.print("F");

    Serial.println();
  }
}