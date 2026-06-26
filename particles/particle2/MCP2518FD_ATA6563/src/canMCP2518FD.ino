#include "Particle.h"
#include <SPI.h>
#include <stdio.h>

SYSTEM_MODE(AUTOMATIC);

// MCP2518FD chip-select. Keep this aligned with your wiring.
static const uint8_t CS_PIN = D5;

// Set false if exact hardware filters result in frames=0. Exact filters reduce
// FIFO overrun while sniffing OpenECU CCP traffic.
static const bool USE_EXACT_ID_FILTERS = true;

// This build can actively poll OpenECU-style CCP SHORT_UP reads.
// Safety: if it sees another CCP master sending CRO frames, it disables its own
// active requests and behaves as a passive typed decoder.
// 0 = Normal, 3 = Listen-only, 4 = Configuration on MCP2518FD.
static const uint8_t MODE_NORMAL      = 0;
static const uint8_t MODE_LISTEN_ONLY = 3;
static const uint8_t MODE_CONFIG      = 4;

static const uint16_t REG_C1CON      = 0x000;
static const uint16_t REG_C1NBTCFG   = 0x004;
static const uint16_t REG_C1DBTCFG   = 0x008;
static const uint16_t REG_C1INT      = 0x01C;
static const uint16_t REG_C1RXIF     = 0x020;
static const uint16_t REG_C1RXOVIF   = 0x028;
static const uint16_t REG_C1TREC     = 0x034;
static const uint16_t REG_C1BDIAG0   = 0x038;
static const uint16_t REG_C1BDIAG1   = 0x03C;

static const uint16_t REG_C1FIFOCON1 = 0x05C;
static const uint16_t REG_C1FIFOSTA1 = 0x060;
static const uint16_t REG_C1FIFOUA1  = 0x064;
static const uint16_t REG_C1FIFOCON2 = 0x068;
static const uint16_t REG_C1FIFOSTA2 = 0x06C;
static const uint16_t REG_C1FIFOUA2  = 0x070;

static const uint16_t REG_C1FLTCON0  = 0x1D0;
static const uint16_t REG_C1FLTOBJ0  = 0x1F0;
static const uint16_t REG_C1MASK0    = 0x1F4;

static const uint16_t REG_OSC        = 0xE00;
static const uint16_t REG_IOCON      = 0xE04;

static const uint32_t ID_TRAILER_CRO = 0x6F9;
static const uint32_t ID_TRAILER_DTO = 0x6F8;
static const uint32_t ID_CAB_CRO     = 0x6EF;
static const uint32_t ID_CAB_DTO     = 0x6EE;
static const uint32_t ID_DCCL_BCAST  = 0x502;

static const bool ACTIVE_REQUESTS = true;
static const bool DISABLE_ACTIVE_IF_EXTERNAL_CRO_SEEN = true;
static const unsigned long ACTIVE_START_DELAY_MS = 5000;
static const unsigned long REQUEST_PERIOD_MS = 75;
static const unsigned long REQUEST_TIMEOUT_MS = 1000;
static const bool ENABLE_DCCL_BCAST_FILTER = false;
static const unsigned long PUBLISH_PERIOD_MS = 60000;
static const char *PUBLISH_EVENT_NAME = "dht_reading";

// Keep the hot path quiet. Full CCP trace is useful for discovery, but it is
// slow enough to cause FIFO overflow on this bus.
static const bool TRACE_ALL_CCP = false;
static const uint16_t CCP_TRACE_LIMIT = 40;
static const uint16_t MATCH_TRACE_LIMIT = 20;
static const unsigned long PENDING_TIMEOUT_MS = 1500;

enum ValueType {
    TYPE_F32,
    TYPE_S16,
    TYPE_U8,
    TYPE_U32,
    TYPE_ENUM32
};

struct LiveValue {
    double value;
    unsigned long updatedAt;
};

LiveValue cabUnitID              = { NAN, 0 };
LiveValue trailerUnitID          = { NAN, 0 };
LiveValue dischargeChargeCurrLim = { NAN, 0 };
LiveValue pumpLowSpeedRPM        = { NAN, 0 };
LiveValue pumpHighSpeedRPM       = { NAN, 0 };
LiveValue motorToPumpGearRatio   = { NAN, 0 };
LiveValue flipFlowDirection      = { NAN, 0 };
LiveValue ciTankerPressure       = { NAN, 0 };
LiveValue ciExternalPressure     = { NAN, 0 };
LiveValue canRectifierTemp       = { NAN, 0 };
LiveValue canMcuMotorTemp        = { NAN, 0 };
LiveValue canMcuMotorSpeed       = { NAN, 0 };
LiveValue canMcuDcVoltage        = { NAN, 0 };
LiveValue canMcuDcCurrent        = { NAN, 0 };
LiveValue ciRectifierTempDegC    = { NAN, 0 };
LiveValue ciGeneratorTempDegC    = { NAN, 0 };

struct SymbolSpec {
    uint32_t addr;
    const char *name;
    uint32_t requestId;
    ValueType type;
    uint8_t size;
    double minValue;
    double maxValue;
    LiveValue *live;
};

SymbolSpec symbols[] = {
    // Calibration block. OpenECU shows 0x0087 -> 135 for this one.
    { 0x00040408, "CAB_UnitID",             ID_CAB_CRO,     TYPE_U32, 4, 0, 4294967295.0, &cabUnitID },
    { 0x00040404, "TRAILER_UnitID",         ID_TRAILER_CRO, TYPE_U32, 4, 0, 4294967295.0, &trailerUnitID },
    { 0x0004048C, "DischargeChargeCurrLim", ID_TRAILER_CRO, TYPE_S16, 2, 0, 1000, &dischargeChargeCurrLim },
    { 0x000404CC, "PumpLowSpeedRPM",        ID_TRAILER_CRO, TYPE_F32, 4, 0, 5000, &pumpLowSpeedRPM },
    { 0x000404D0, "PumpHighSpeedRPM",       ID_TRAILER_CRO, TYPE_F32, 4, 0, 5000, &pumpHighSpeedRPM },
    { 0x000404DC, "MotorToPumpGearRatio",   ID_TRAILER_CRO, TYPE_F32, 4, 0, 1000, &motorToPumpGearRatio },
    { 0x000404EC, "FlipFlowDirection",      ID_TRAILER_CRO, TYPE_ENUM32, 4, 0, 1, &flipFlowDirection },

    // Trailer ECU runtime RAM.
    { 0x40002A18, "CI_TankerPressure",      ID_TRAILER_CRO, TYPE_F32, 4, -100, 100, &ciTankerPressure },
    { 0x40002A1C, "CI_ExternalPressure",    ID_TRAILER_CRO, TYPE_F32, 4, -100, 100, &ciExternalPressure },
    { 0x40002A34, "CAN_RectifierTemp",      ID_TRAILER_CRO, TYPE_F32, 4, -40, 150, &canRectifierTemp },
    { 0x40002A58, "CAN_McuMotorTemp",       ID_TRAILER_CRO, TYPE_F32, 4, -40, 150, &canMcuMotorTemp },
    { 0x40002A5C, "CAN_McuMotorSpeed",      ID_TRAILER_CRO, TYPE_F32, 4, -10000, 10000, &canMcuMotorSpeed },
    { 0x40002A68, "CAN_McuDcVoltage",       ID_TRAILER_CRO, TYPE_F32, 4, 0, 1000, &canMcuDcVoltage },
    { 0x40002A6C, "CAN_McuDcCurrent",       ID_TRAILER_CRO, TYPE_F32, 4, -1000, 1000, &canMcuDcCurrent },

    // Cab ECU runtime RAM.
    { 0x40002568, "CI_RectifierTempDegC",   ID_CAB_CRO, TYPE_F32, 4, -40, 150, &ciRectifierTempDegC },
    { 0x4000256C, "CI_GeneratorTempDegC",   ID_CAB_CRO, TYPE_F32, 4, -40, 150, &ciGeneratorTempDegC },
};

static const uint8_t SYMBOL_COUNT = sizeof(symbols) / sizeof(symbols[0]);

struct PendingRequest {
    bool valid;
    uint32_t addr;
    uint8_t size;
    uint8_t addrExt;
    uint32_t requestId;
    unsigned long requestedAt;
};

PendingRequest pending6F9[256];
PendingRequest pending6EF[256];

unsigned long frameCount = 0;
unsigned long ccpRequestCount = 0;
unsigned long ccpResponseCount = 0;
unsigned long ccpMatchedCount = 0;
unsigned long ccpOrphanCount = 0;
unsigned long ccpStaleCount = 0;
unsigned long ccpErrorCount = 0;
unsigned long ccpSizeMismatchCount = 0;
unsigned long ccpDecodeRejectCount = 0;
unsigned long ccpRangeRejectCount = 0;
unsigned long fifoOverflowCount = 0;
unsigned long txSentCount = 0;
unsigned long txSkipCount = 0;
unsigned long activeTimeoutCount = 0;
unsigned long externalCroCount = 0;
unsigned long lastMatchAt = 0;
unsigned long lastResponseAt = 0;
unsigned long lastTimeoutAt = 0;
unsigned long responseLatencyTotalMs = 0;
unsigned long responseLatencyCount = 0;
unsigned long responseLatencyMaxMs = 0;
unsigned long maxPendingSeen = 0;
unsigned long fullCycleCount = 0;
unsigned long lastCycleAt = 0;
unsigned long lastCycleMs = 0;
unsigned long lastRequestAt = 0;
unsigned long lastStatus = 0;
unsigned long lastPublish = 0;
uint16_t ccpTracePrinted = 0;
uint16_t matchTracePrinted = 0;
uint8_t nextCtrTrailer = 0x10;
uint8_t nextCtrCab = 0x80;
uint8_t pollIndex = 0;
bool activePollingEnabled = ACTIVE_REQUESTS;
bool activeDisabledByExternalMaster = false;

uint16_t makeCmd(uint16_t instruction, uint16_t address) {
    return (instruction << 12) | (address & 0x0FFF);
}

void csLow()  { digitalWrite(CS_PIN, LOW); }
void csHigh() { digitalWrite(CS_PIN, HIGH); }

uint8_t xfer(uint8_t v) {
    return SPI.transfer(v);
}

void mcpReset() {
    csLow();
    xfer(0x00);
    xfer(0x00);
    csHigh();
    delay(20);
}

uint32_t readReg32(uint16_t address) {
    uint16_t cmd = makeCmd(0x3, address);

    csLow();
    xfer(cmd >> 8);
    xfer(cmd & 0xFF);

    uint32_t v = 0;
    v |= ((uint32_t)xfer(0x00)) << 0;
    v |= ((uint32_t)xfer(0x00)) << 8;
    v |= ((uint32_t)xfer(0x00)) << 16;
    v |= ((uint32_t)xfer(0x00)) << 24;

    csHigh();
    return v;
}

void writeReg32(uint16_t address, uint32_t value) {
    uint16_t cmd = makeCmd(0x2, address);

    csLow();
    xfer(cmd >> 8);
    xfer(cmd & 0xFF);

    xfer((value >> 0) & 0xFF);
    xfer((value >> 8) & 0xFF);
    xfer((value >> 16) & 0xFF);
    xfer((value >> 24) & 0xFF);

    csHigh();
}

void readBytes(uint16_t address, uint8_t *buf, size_t len) {
    uint16_t cmd = makeCmd(0x3, address);

    csLow();
    xfer(cmd >> 8);
    xfer(cmd & 0xFF);

    for (size_t i = 0; i < len; i++) {
        buf[i] = xfer(0x00);
    }

    csHigh();
}

void writeBytes(uint16_t address, const uint8_t *buf, size_t len) {
    uint16_t cmd = makeCmd(0x2, address);

    csLow();
    xfer(cmd >> 8);
    xfer(cmd & 0xFF);

    for (size_t i = 0; i < len; i++) {
        xfer(buf[i]);
    }

    csHigh();
}

void requestMode(uint8_t mode) {
    uint32_t c1con = readReg32(REG_C1CON);
    c1con &= ~(0x7UL << 24);
    c1con |= ((uint32_t)mode << 24);
    writeReg32(REG_C1CON, c1con);
    delay(50);
}

void printReg(const char *name, uint16_t addr) {
    Serial.print(name);
    Serial.print(" = 0x");
    Serial.println(readReg32(addr), HEX);
}

void popFifo1() {
    uint32_t fcon = readReg32(REG_C1FIFOCON1);
    fcon |= 0x00000100;
    writeReg32(REG_C1FIFOCON1, fcon);
}

uint16_t filterObjReg(uint8_t n) {
    return REG_C1FLTOBJ0 + ((uint16_t)n * 8);
}

uint16_t filterMaskReg(uint8_t n) {
    return REG_C1MASK0 + ((uint16_t)n * 8);
}

uint16_t filterConReg(uint8_t n) {
    return REG_C1FLTCON0 + ((uint16_t)(n / 4) * 4);
}

void enableFilterToFifo1(uint8_t n) {
    uint16_t reg = filterConReg(n);
    uint8_t shift = (n % 4) * 8;
    uint32_t v = readReg32(reg);

    v &= ~(0xFFUL << shift);
    v |= ((uint32_t)0x81 << shift);  // FLTEN=1, FBP=1.
    writeReg32(reg, v);
}

void configureStandardIdFilter(uint8_t n, uint16_t sid) {
    writeReg32(filterObjReg(n), sid & 0x7FF);
    writeReg32(filterMaskReg(n), 0x000007FF);
    enableFilterToFifo1(n);
}

void clearPendingRequests() {
    for (int i = 0; i < 256; i++) {
        pending6F9[i].valid = false;
        pending6F9[i].addr = 0;
        pending6F9[i].size = 0;
        pending6F9[i].addrExt = 0;
        pending6F9[i].requestId = 0;
        pending6F9[i].requestedAt = 0;

        pending6EF[i].valid = false;
        pending6EF[i].addr = 0;
        pending6EF[i].size = 0;
        pending6EF[i].addrExt = 0;
        pending6EF[i].requestId = 0;
        pending6EF[i].requestedAt = 0;
    }
}

float readFloatBE(const uint8_t *b) {
    union {
        uint32_t u;
        float f;
    } val;

    val.u =
        ((uint32_t)b[0] << 24) |
        ((uint32_t)b[1] << 16) |
        ((uint32_t)b[2] << 8)  |
        ((uint32_t)b[3]);

    return val.f;
}

int16_t readS16BE(const uint8_t *b) {
    return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
}

uint32_t readU32BE(const uint8_t *b) {
    return
        ((uint32_t)b[0] << 24) |
        ((uint32_t)b[1] << 16) |
        ((uint32_t)b[2] << 8)  |
        ((uint32_t)b[3]);
}

uint32_t readAddrFromRequest(const uint8_t *p) {
    return
        ((uint32_t)p[4] << 24) |
        ((uint32_t)p[5] << 16) |
        ((uint32_t)p[6] << 8)  |
        ((uint32_t)p[7]);
}

SymbolSpec* findSymbol(uint32_t addr) {
    for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
        if (symbols[i].addr == addr) {
            return &symbols[i];
        }
    }
    return NULL;
}

bool isWatchedAddress(uint32_t addr) {
    return findSymbol(addr) != NULL;
}

bool decodeValue(const SymbolSpec *sym, const uint8_t *data, double *out) {
    switch (sym->type) {
        case TYPE_F32:
            *out = readFloatBE(data);
            return !isnan(*out) && !isinf(*out);
        case TYPE_S16:
            *out = readS16BE(data);
            return true;
        case TYPE_U8:
            *out = data[0];
            return true;
        case TYPE_U32:
        case TYPE_ENUM32:
            *out = readU32BE(data);
            return true;
    }
    return false;
}

bool storeDecodedValue(const SymbolSpec *sym, double value) {
    if (value < sym->minValue || value > sym->maxValue) {
        return false;
    }
    sym->live->value = value;
    sym->live->updatedAt = millis();
    return true;
}

void printHexByte(uint8_t v) {
    if (v < 0x10) {
        Serial.print('0');
    }
    Serial.print(v, HEX);
}

void printHex32(uint32_t v) {
    for (int shift = 28; shift >= 0; shift -= 4) {
        Serial.print((uint8_t)((v >> shift) & 0x0F), HEX);
    }
}

bool isCcpId(uint32_t id) {
    return id == ID_TRAILER_CRO || id == ID_TRAILER_DTO ||
           id == ID_CAB_CRO || id == ID_CAB_DTO;
}

PendingRequest* pendingTableForRequestId(uint32_t id) {
    return (id == ID_TRAILER_CRO) ? pending6F9 : pending6EF;
}

PendingRequest* pendingTableForResponseId(uint32_t id) {
    return (id == ID_TRAILER_DTO) ? pending6F9 : pending6EF;
}

uint8_t nextCounterForRequestId(uint32_t id) {
    if (id == ID_TRAILER_CRO) {
        return nextCtrTrailer++;
    }
    return nextCtrCab++;
}

void rememberPending(uint32_t requestId, uint8_t ctr, uint32_t addr, uint8_t size, uint8_t addrExt) {
    PendingRequest *table = pendingTableForRequestId(requestId);
    table[ctr].valid = true;
    table[ctr].addr = addr;
    table[ctr].size = size;
    table[ctr].addrExt = addrExt;
    table[ctr].requestId = requestId;
    table[ctr].requestedAt = millis();

    uint16_t pendingNow = 0;
    PendingRequest *tables[] = { pending6F9, pending6EF };
    for (uint8_t t = 0; t < 2; t++) {
        for (uint16_t i = 0; i < 256; i++) {
            if (tables[t][i].valid) {
                pendingNow++;
            }
        }
    }
    if (pendingNow > maxPendingSeen) {
        maxPendingSeen = pendingNow;
    }
}

uint16_t countPendingRequests() {
    uint16_t pendingNow = 0;
    PendingRequest *tables[] = { pending6F9, pending6EF };

    for (uint8_t t = 0; t < 2; t++) {
        for (uint16_t i = 0; i < 256; i++) {
            if (tables[t][i].valid) {
                pendingNow++;
            }
        }
    }

    return pendingNow;
}

unsigned long oldestPendingAgeMs() {
    unsigned long now = millis();
    unsigned long oldest = 0;
    PendingRequest *tables[] = { pending6F9, pending6EF };

    for (uint8_t t = 0; t < 2; t++) {
        for (uint16_t i = 0; i < 256; i++) {
            if (tables[t][i].valid) {
                unsigned long age = now - tables[t][i].requestedAt;
                if (age > oldest) {
                    oldest = age;
                }
            }
        }
    }

    return oldest;
}

void recordResponseLatency(const PendingRequest &req) {
    unsigned long latency = millis() - req.requestedAt;
    responseLatencyTotalMs += latency;
    responseLatencyCount++;
    if (latency > responseLatencyMaxMs) {
        responseLatencyMaxMs = latency;
    }
    lastMatchAt = millis();
}

bool sendClassicFrame(uint32_t id, const uint8_t *payload, uint8_t len) {
    if (len > 8) {
        len = 8;
    }

    // FIFO2 configured as TX FIFO. Bit0 in FIFOSTA2 means "not full".
    if ((readReg32(REG_C1FIFOSTA2) & 0x00000001) == 0) {
        txSkipCount++;
        return false;
    }

    uint32_t ua = readReg32(REG_C1FIFOUA2);
    uint16_t ramAddr = (uint16_t)(0x400 + ua);

    uint8_t raw[16];
    for (uint8_t i = 0; i < sizeof(raw); i++) {
        raw[i] = 0;
    }

    raw[0] = id & 0xFF;
    raw[1] = (id >> 8) & 0xFF;
    raw[2] = (id >> 16) & 0xFF;
    raw[3] = (id >> 24) & 0xFF;
    raw[4] = len & 0x0F;  // classic CAN DLC, no IDE/RTR/BRS/FDF bits.

    for (uint8_t i = 0; i < len; i++) {
        raw[8 + i] = payload[i];
    }

    writeBytes(ramAddr, raw, sizeof(raw));

    // Set UINC and TXREQ. These are bits 8 and 9 in CiFIFOCON.
    uint32_t fcon = readReg32(REG_C1FIFOCON2);
    fcon |= 0x00000300;
    writeReg32(REG_C1FIFOCON2, fcon);

    txSentCount++;
    return true;
}

bool sendCcpCommand(uint32_t requestId, uint8_t cmd, uint8_t ctr, uint8_t b2, uint8_t b3, uint32_t addr) {
    uint8_t payload[8];
    payload[0] = cmd;
    payload[1] = ctr;
    payload[2] = b2;
    payload[3] = b3;
    payload[4] = (addr >> 24) & 0xFF;
    payload[5] = (addr >> 16) & 0xFF;
    payload[6] = (addr >> 8) & 0xFF;
    payload[7] = addr & 0xFF;

    return sendClassicFrame(requestId, payload, 8);
}

bool sendConnect(uint32_t requestId) {
    uint8_t ctr = nextCounterForRequestId(requestId);
    return sendCcpCommand(requestId, 0x01, ctr, 0x00, 0x00, 0x00000000);
}

bool sendShortUp(const SymbolSpec *sym) {
    uint8_t ctr = nextCounterForRequestId(sym->requestId);
    if (sendCcpCommand(sym->requestId, 0x0F, ctr, sym->size, 0x00, sym->addr)) {
        rememberPending(sym->requestId, ctr, sym->addr, sym->size, 0x00);
        ccpRequestCount++;
        return true;
    }
    return false;
}

void checkActiveTimeouts() {
    PendingRequest *tables[] = { pending6F9, pending6EF };
    unsigned long now = millis();

    for (uint8_t t = 0; t < 2; t++) {
        for (uint16_t i = 0; i < 256; i++) {
            if (tables[t][i].valid && now - tables[t][i].requestedAt > REQUEST_TIMEOUT_MS) {
                tables[t][i].valid = false;
                activeTimeoutCount++;
                lastTimeoutAt = now;
            }
        }
    }
}

void serviceActivePolling() {
    if (!activePollingEnabled || millis() < ACTIVE_START_DELAY_MS) {
        return;
    }

    if (millis() - lastRequestAt < REQUEST_PERIOD_MS) {
        return;
    }
    lastRequestAt = millis();

    if (txSentCount == 0) {
        // OpenECU sends CONNECT before SHORT_UP. Do the same for both ECUs.
        sendConnect(ID_TRAILER_CRO);
        delay(2);
        sendConnect(ID_CAB_CRO);
        return;
    }

    const SymbolSpec *sym = &symbols[pollIndex];
    pollIndex++;
    bool wrapped = false;
    if (pollIndex >= SYMBOL_COUNT) {
        pollIndex = 0;
        wrapped = true;
    }

    if (sendShortUp(sym) && wrapped) {
        unsigned long now = millis();
        if (lastCycleAt != 0) {
            lastCycleMs = now - lastCycleAt;
        }
        lastCycleAt = now;
        fullCycleCount++;
    }
}

void traceCcpFrame(uint32_t id, uint8_t len, const uint8_t *p) {
    if (!TRACE_ALL_CCP || !isCcpId(id) || ccpTracePrinted >= CCP_TRACE_LIMIT) {
        return;
    }

    ccpTracePrinted++;

    Serial.print("CCP ");
    Serial.print(id, HEX);
    Serial.print(" ");

    for (uint8_t i = 0; i < len; i++) {
        printHexByte(p[i]);
        if (i + 1 < len) {
            Serial.print(' ');
        }
    }

    if ((id == ID_TRAILER_CRO || id == ID_CAB_CRO) && len == 8 && p[0] == 0x0F) {
        Serial.print("  SHORT_UP ctr=");
        printHexByte(p[1]);
        Serial.print(" size=");
        Serial.print(p[2]);
        Serial.print(" ext=");
        printHexByte(p[3]);
        Serial.print(" addr=0x");
        printHex32(readAddrFromRequest(p));
    } else if ((id == ID_TRAILER_DTO || id == ID_CAB_DTO) && len == 8 && p[0] == 0xFF) {
        Serial.print("  DTO err=");
        printHexByte(p[1]);
        Serial.print(" ctr=");
        printHexByte(p[2]);
        Serial.print(" data=");
        for (uint8_t i = 3; i < len; i++) {
            printHexByte(p[i]);
        }
    }

    Serial.println();
}

void handleFrame(uint32_t id, uint8_t len, uint8_t *p) {
    traceCcpFrame(id, len, p);

    if (id == ID_DCCL_BCAST && len == 8) {
        // Older DBC/broadcast path from the previous exploration. Keep it
        // conservative; CCP should overwrite it with the typed S16 value.
        dischargeChargeCurrLim.value = p[2];
        dischargeChargeCurrLim.updatedAt = millis();
        return;
    }

    // CCP SHORT_UP request:
    // [0] cmd=0x0F, [1] counter, [2] size, [3] address extension,
    // [4..7] big-endian target address.
    if ((id == ID_TRAILER_CRO || id == ID_CAB_CRO) &&
        len == 8 &&
        p[0] == 0x0F) {

        externalCroCount++;
        if (DISABLE_ACTIVE_IF_EXTERNAL_CRO_SEEN && activePollingEnabled) {
            activePollingEnabled = false;
            activeDisabledByExternalMaster = true;
            clearPendingRequests();
            Serial.println("Active polling disabled: external OpenECU CRO traffic detected");
        }

        PendingRequest *table = pendingTableForRequestId(id);
        uint8_t ctr = p[1];

        table[ctr].valid = true;
        table[ctr].addr = readAddrFromRequest(p);
        table[ctr].size = p[2];
        table[ctr].addrExt = p[3];
        table[ctr].requestId = id;
        table[ctr].requestedAt = millis();

        ccpRequestCount++;

        if (isWatchedAddress(table[ctr].addr) && ccpTracePrinted < CCP_TRACE_LIMIT) {
            ccpTracePrinted++;
            Serial.print("WATCH_REQ ");
            Serial.print(id, HEX);
            Serial.print(" ctr=");
            printHexByte(ctr);
            Serial.print(" size=");
            Serial.print(table[ctr].size);
            Serial.print(" addr=0x");
            printHex32(table[ctr].addr);
            Serial.print(" ");
            Serial.println(findSymbol(table[ctr].addr)->name);
        }

        return;
    }

    // CCP DTO response for SHORT_UP:
    // [0] packet id 0xFF, [1] error code, [2] counter, [3..7] data.
    if ((id == ID_TRAILER_DTO || id == ID_CAB_DTO) &&
        len == 8 &&
        p[0] == 0xFF) {

        ccpResponseCount++;
        lastResponseAt = millis();

        uint8_t err = p[1];
        uint8_t ctr = p[2];
        PendingRequest *table = pendingTableForResponseId(id);

        if (err != 0x00) {
            ccpErrorCount++;
            table[ctr].valid = false;
            return;
        }

        if (!table[ctr].valid) {
            ccpOrphanCount++;
            return;
        }

        PendingRequest req = table[ctr];
        table[ctr].valid = false;

        if (millis() - req.requestedAt > PENDING_TIMEOUT_MS) {
            ccpStaleCount++;
            return;
        }

        SymbolSpec *sym = findSymbol(req.addr);
        if (sym == NULL) {
            return;
        }

        if (req.size != sym->size) {
            // OpenECU may occasionally request adjacent bytes, but for this
            // target list mismatched size usually means a bad match.
            ccpSizeMismatchCount++;
            return;
        }

        double value = NAN;
        if (!decodeValue(sym, &p[3], &value)) {
            ccpDecodeRejectCount++;
            return;
        }

        if (!storeDecodedValue(sym, value)) {
            ccpRangeRejectCount++;
            return;
        }

        recordResponseLatency(req);
        ccpMatchedCount++;

        if (matchTracePrinted < MATCH_TRACE_LIMIT) {
            matchTracePrinted++;
            Serial.print("MATCH ");
            Serial.print(id, HEX);
            Serial.print(" ctr=");
            printHexByte(ctr);
            Serial.print(" addr=0x");
            printHex32(req.addr);
            Serial.print(" ");
            Serial.print(sym->name);
            Serial.print("=");
            Serial.println(value, (sym->type == TYPE_F32) ? 2 : 0);
        }

        return;
    }
}

void printValue(const char *name, const LiveValue &v, int decimals) {
    Serial.print(name);
    Serial.print(": ");

    if (isnan(v.value)) {
        Serial.println("--");
        return;
    }

    Serial.print(v.value, decimals);
    Serial.print("  age_ms=");
    Serial.println(millis() - v.updatedAt);
}

void printLiveTable() {
    Serial.println();
    Serial.println("===== OpenECU CCP Live Values =====");

    printValue("CAB_UnitID", cabUnitID, 0);
    printValue("TRAILER_UnitID", trailerUnitID, 0);
    printValue("DischargeChargeCurrLim", dischargeChargeCurrLim, 0);
    printValue("PumpLowSpeedRPM", pumpLowSpeedRPM, 0);
    printValue("PumpHighSpeedRPM", pumpHighSpeedRPM, 0);
    printValue("MotorToPumpGearRatio", motorToPumpGearRatio, 0);
    printValue("FlipFlowDirection", flipFlowDirection, 0);
    printValue("CI_TankerPressure", ciTankerPressure, 2);
    printValue("CI_ExternalPressure", ciExternalPressure, 2);
    printValue("CAN_RectifierTemp", canRectifierTemp, 2);
    printValue("CAN_McuMotorTemp", canMcuMotorTemp, 2);
    printValue("CAN_McuMotorSpeed", canMcuMotorSpeed, 0);
    printValue("CAN_McuDcVoltage", canMcuDcVoltage, 2);
    printValue("CAN_McuDcCurrent", canMcuDcCurrent, 2);
    printValue("CI_RectifierTempDegC", ciRectifierTempDegC, 2);
    printValue("CI_GeneratorTempDegC", ciGeneratorTempDegC, 2);

    Serial.println("===================================");
}

void publishToCloud() {
    if (!Particle.connected()) {
        return;
    }

    double successPct = NAN;
    if (ccpRequestCount > 0) {
        successPct = (100.0 * ccpMatchedCount) / ccpRequestCount;
    }

    char data[768];
    int written = snprintf(
        data,
        sizeof(data),
        "{\"cabUnitID\":%.0f,\"trailerUnitID\":%.0f,"
        "\"dccLim\":%.0f,\"pumpLowRPM\":%.2f,\"pumpRPM\":%.2f,"
        "\"gearRatio\":%.2f,\"flipFlow\":%.0f,"
        "\"tankerPressure\":%.2f,\"externalPressure\":%.2f,"
        "\"rectTemp\":%.2f,\"dcCurrent\":%.2f,\"dcVoltage\":%.2f,"
        "\"motorSpeed\":%.2f,\"motorTemp\":%.2f,"
        "\"ciRectTemp\":%.2f,\"ciGenTemp\":%.2f,"
        "\"frames\":%lu,\"ccpReq\":%lu,\"ccpResp\":%lu,"
        "\"matched\":%lu,\"successPct\":%.1f,"
        "\"activeTimeout\":%lu,\"txSkip\":%lu}",
        cabUnitID.value,
        trailerUnitID.value,
        dischargeChargeCurrLim.value,
        pumpLowSpeedRPM.value,
        pumpHighSpeedRPM.value,
        motorToPumpGearRatio.value,
        flipFlowDirection.value,
        ciTankerPressure.value,
        ciExternalPressure.value,
        canRectifierTemp.value,
        canMcuDcCurrent.value,
        canMcuDcVoltage.value,
        canMcuMotorSpeed.value,
        canMcuMotorTemp.value,
        ciRectifierTempDegC.value,
        ciGeneratorTempDegC.value,
        frameCount,
        ccpRequestCount,
        ccpResponseCount,
        ccpMatchedCount,
        successPct,
        activeTimeoutCount,
        txSkipCount
    );

    if (written <= 0 || written >= (int)sizeof(data)) {
        Serial.println("Particle publish payload was truncated; skipping publish.");
        return;
    }

    bool ok = Particle.publish(PUBLISH_EVENT_NAME, data, PRIVATE);
    Serial.print("Particle.publish ");
    Serial.print(PUBLISH_EVENT_NAME);
    Serial.print(ok ? " OK bytes=" : " FAILED bytes=");
    Serial.println(written);
}

void initCAN() {
    Serial.println();
    Serial.println("MCP2518FD OpenECU CCP active requester v8 publish");

    mcpReset();

    Serial.println("Config mode...");
    requestMode(MODE_CONFIG);

    // Keep the same nominal/data bit timing as the previous working sketch.
    writeReg32(REG_C1NBTCFG, 0x003E0F0F);
    writeReg32(REG_C1DBTCFG, 0x003E0F0F);

    // FIFO1 as RX FIFO, matching the previous known-good sketch.
    writeReg32(REG_C1FIFOCON1, 0x0000001F);

    // FIFO2 as TX FIFO: TxEnable=1, priority=1, attempts=3, fifo size=4 objects,
    // payload size=8 bytes. Based on MCP2518FD CiFIFOCON bit layout.
    writeReg32(REG_C1FIFOCON2, 0x03610080);

    writeReg32(REG_C1FLTCON0, 0x00000000);
    writeReg32(REG_C1FLTCON0 + 4, 0x00000000);

    if (USE_EXACT_ID_FILTERS) {
        configureStandardIdFilter(0, ID_TRAILER_CRO);
        configureStandardIdFilter(1, ID_TRAILER_DTO);
        configureStandardIdFilter(2, ID_CAB_CRO);
        configureStandardIdFilter(3, ID_CAB_DTO);
        if (ENABLE_DCCL_BCAST_FILTER) {
            configureStandardIdFilter(4, ID_DCCL_BCAST);
        }
    } else {
        // Fallback: accept all IDs into FIFO1.
        writeReg32(REG_C1FLTOBJ0, 0x00000000);
        writeReg32(REG_C1MASK0,   0x00000000);
        writeReg32(REG_C1FLTCON0, 0x00000081);
    }

    writeReg32(REG_C1INT,    0x00000000);
    writeReg32(REG_C1RXIF,   0x00000000);
    writeReg32(REG_C1RXOVIF, 0x00000000);

    clearPendingRequests();

    if (ACTIVE_REQUESTS) {
        Serial.println("Normal mode; active polling armed after startup guard...");
        requestMode(MODE_NORMAL);
    } else {
        Serial.println("Listen-only mode; active polling disabled by constant...");
        requestMode(MODE_LISTEN_ONLY);
    }

    Serial.println("After init:");
    printReg("OSC   ", REG_OSC);
    printReg("IOCON ", REG_IOCON);
    printReg("C1CON ", REG_C1CON);
    printReg("NBTCFG", REG_C1NBTCFG);
    printReg("FCON1 ", REG_C1FIFOCON1);
    printReg("FSTA1 ", REG_C1FIFOSTA1);
    printReg("FUA1  ", REG_C1FIFOUA1);
    printReg("FCON2 ", REG_C1FIFOCON2);
    printReg("FSTA2 ", REG_C1FIFOSTA2);
    printReg("FUA2  ", REG_C1FIFOUA2);
    printReg("FLTCON", REG_C1FLTCON0);
    printReg("TREC  ", REG_C1TREC);
    printReg("BDIAG0", REG_C1BDIAG0);
    printReg("BDIAG1", REG_C1BDIAG1);
}

void serviceOverflow() {
    if (readReg32(REG_C1RXOVIF) & 0x00000002) {
        fifoOverflowCount++;
        writeReg32(REG_C1RXOVIF, 0x00000002);
    }
}

void printAgeField(const char *label, unsigned long eventAt) {
    Serial.print(" ");
    Serial.print(label);
    Serial.print("=");
    if (eventAt == 0) {
        Serial.print("--");
    } else {
        Serial.print(millis() - eventAt);
    }
}

void loop() {
    serviceOverflow();

    while (readReg32(REG_C1RXIF) & 0x00000002) {
        uint32_t ua = readReg32(REG_C1FIFOUA1);
        uint16_t ramAddr = (uint16_t)(0x400 + ua);

        uint8_t raw[32];
        readBytes(ramAddr, raw, sizeof(raw));

        frameCount++;

        uint32_t id =
            ((uint32_t)raw[0]) |
            ((uint32_t)raw[1] << 8) |
            ((uint32_t)raw[2] << 16) |
            ((uint32_t)raw[3] << 24);

        uint8_t len = raw[4] & 0x0F;
        if (len > 8) {
            len = 8;
        }

        uint8_t *payload = &raw[8];
        handleFrame(id, len, payload);

        popFifo1();

        writeReg32(REG_C1RXIF,   0x00000002);
    }

    serviceOverflow();
    checkActiveTimeouts();
    serviceActivePolling();

    if (millis() - lastPublish >= PUBLISH_PERIOD_MS) {
        lastPublish = millis();
        publishToCloud();
    }

    if (millis() - lastStatus >= 5000) {
        lastStatus = millis();

        printLiveTable();

        uint16_t pendingNow = countPendingRequests();
        unsigned long oldestPending = oldestPendingAgeMs();
        unsigned long latencyAvg = responseLatencyCount == 0 ? 0 : responseLatencyTotalMs / responseLatencyCount;

        Serial.print("frames=");
        Serial.print(frameCount);
        Serial.print(" req=");
        Serial.print(ccpRequestCount);
        Serial.print(" resp=");
        Serial.print(ccpResponseCount);
        Serial.print(" matched=");
        Serial.print(ccpMatchedCount);
        Serial.print(" orphan=");
        Serial.print(ccpOrphanCount);
        Serial.print(" stale=");
        Serial.print(ccpStaleCount);
        Serial.print(" ccpErr=");
        Serial.print(ccpErrorCount);
        Serial.print(" sizeMis=");
        Serial.print(ccpSizeMismatchCount);
        Serial.print(" decodeRej=");
        Serial.print(ccpDecodeRejectCount);
        Serial.print(" rangeRej=");
        Serial.print(ccpRangeRejectCount);
        Serial.print(" successPct=");
        if (ccpRequestCount == 0) {
            Serial.print("--");
        } else {
            Serial.print((100.0 * ccpMatchedCount) / ccpRequestCount, 1);
        }
        Serial.print(" pending=");
        Serial.print(pendingNow);
        Serial.print(" maxPending=");
        Serial.print(maxPendingSeen);
        Serial.print(" oldestPendingMs=");
        Serial.print(oldestPending);
        Serial.print(" rtAvgMs=");
        Serial.print(latencyAvg);
        Serial.print(" rtMaxMs=");
        Serial.print(responseLatencyMaxMs);
        Serial.print(" cycles=");
        Serial.print(fullCycleCount);
        Serial.print(" cycleMs=");
        Serial.print(lastCycleMs);
        Serial.print(" fifoOv=");
        Serial.print(fifoOverflowCount);
        Serial.print(" tx=");
        Serial.print(txSentCount);
        Serial.print(" txSkip=");
        Serial.print(txSkipCount);
        Serial.print(" activeTimeout=");
        Serial.print(activeTimeoutCount);
        Serial.print(" externalCRO=");
        Serial.print(externalCroCount);
        Serial.print(" active=");
        Serial.print(activePollingEnabled ? 1 : 0);
        Serial.print(" disabledByMaster=");
        Serial.print(activeDisabledByExternalMaster ? 1 : 0);
        Serial.print(" trace=");
        Serial.print(ccpTracePrinted);
        Serial.print("/");
        Serial.print(CCP_TRACE_LIMIT);
        Serial.print(" matchTrace=");
        Serial.print(matchTracePrinted);
        Serial.print("/");
        Serial.print(MATCH_TRACE_LIMIT);
        printAgeField("lastRespAgeMs", lastResponseAt);
        printAgeField("lastMatchAgeMs", lastMatchAt);
        printAgeField("lastTimeoutAgeMs", lastTimeoutAt);

        Serial.print(" RXIF=0x");
        Serial.print(readReg32(REG_C1RXIF), HEX);
        Serial.print(" RXOVIF=0x");
        Serial.print(readReg32(REG_C1RXOVIF), HEX);
        Serial.print(" FSTA1=0x");
        Serial.print(readReg32(REG_C1FIFOSTA1), HEX);
        Serial.print(" TREC=0x");
        Serial.print(readReg32(REG_C1TREC), HEX);
        Serial.print(" BDIAG0=0x");
        Serial.print(readReg32(REG_C1BDIAG0), HEX);
        Serial.print(" BDIAG1=0x");
        Serial.println(readReg32(REG_C1BDIAG1), HEX);
    }
}

void setup() {
    Serial.begin(115200);
    waitFor(Serial.isConnected, 10000);

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    SPI.begin();
    SPI.setClockSpeed(2, MHZ);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);

    initCAN();
}
