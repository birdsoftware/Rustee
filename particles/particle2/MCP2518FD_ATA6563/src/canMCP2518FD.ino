#include "Particle.h"
#include <SPI.h>

SYSTEM_MODE(AUTOMATIC);

static const uint8_t CS_PIN = D5;

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

static const uint16_t REG_C1FLTCON0  = 0x1D0;
static const uint16_t REG_C1FLTOBJ0  = 0x1F0;
static const uint16_t REG_C1MASK0    = 0x1F4;

static const uint16_t REG_OSC        = 0xE00;
static const uint16_t REG_IOCON      = 0xE04;

unsigned long frameCount = 0;
unsigned long lastStatus = 0;

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
    fcon |= 0x00000100;   // UINC
    writeReg32(REG_C1FIFOCON1, fcon);
}

float readFloatBE(uint8_t *b) {
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

uint32_t readAddrFromPayload(uint8_t *p) {
    return
        ((uint32_t)p[4] << 24) |
        ((uint32_t)p[5] << 16) |
        ((uint32_t)p[6] << 8)  |
        ((uint32_t)p[7]);
}

void initCAN() {
    Serial.println();
    Serial.println("MCP2518FD OpenECU RX Decoder");

    mcpReset();

    Serial.println("Config mode...");
    requestMode(4);

    // Working 20 MHz / 250 kbps timing
    writeReg32(REG_C1NBTCFG, 0x003E0F0F);
    writeReg32(REG_C1DBTCFG, 0x003E0F0F);

    // FIFO1 as RX FIFO
    writeReg32(REG_C1FIFOCON1, 0x0000001F);

    // Accept-all filter 0 -> FIFO1
    writeReg32(REG_C1FLTOBJ0, 0x00000000);
    writeReg32(REG_C1MASK0,   0x00000000);
    writeReg32(REG_C1FLTCON0, 0x00000081);

    writeReg32(REG_C1INT,    0x00000000);
    writeReg32(REG_C1RXIF,   0x00000000);
    writeReg32(REG_C1RXOVIF, 0x00000000);

    Serial.println("Normal mode...");
    requestMode(0);

    Serial.println("After init:");
    printReg("OSC   ", REG_OSC);
    printReg("IOCON ", REG_IOCON);
    printReg("C1CON ", REG_C1CON);
    printReg("NBTCFG", REG_C1NBTCFG);
    printReg("FCON1 ", REG_C1FIFOCON1);
    printReg("FLTCON", REG_C1FLTCON0);
    printReg("TREC  ", REG_C1TREC);
    printReg("BDIAG0", REG_C1BDIAG0);
}

void printFrame(uint32_t id, uint8_t len, uint8_t *p) {
    Serial.print("ID=0x");
    Serial.print(id, HEX);

    Serial.print(" LEN=");
    Serial.print(len);

    Serial.print(" DATA=");
    for (uint8_t i = 0; i < len; i++) {
        if (p[i] < 0x10) Serial.print("0");
        Serial.print(p[i], HEX);
        Serial.print(" ");
    }

    if ((id == 0x6F9 || id == 0x6EF) && len == 8 && p[0] == 0x0F) {
        uint32_t addr = readAddrFromPayload(p);

        Serial.print(" REQUEST addr=0x");
        Serial.print(addr, HEX);
        Serial.print(" tx=0x");
        Serial.print(p[1], HEX);
    }

    if ((id == 0x6F8 || id == 0x6EE) && len == 8 && p[0] == 0xFF && p[1] == 0x00) {
        float value = readFloatBE(&p[3]);

        Serial.print(" RESPONSE tx=0x");
        Serial.print(p[2], HEX);
        Serial.print(" value=");
        Serial.print(value, 6);
    }

    Serial.println();
}

void loop() {
    uint32_t rxif = readReg32(REG_C1RXIF);

    if (rxif & 0x00000002) {
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
        if (len > 8) len = 8;

        uint8_t *payload = &raw[8];

        printFrame(id, len, payload);

        popFifo1();

        writeReg32(REG_C1RXIF,   0x00000002);
        writeReg32(REG_C1RXOVIF, 0x00000002);
    }

    if (millis() - lastStatus >= 5000) {
        lastStatus = millis();

        Serial.print("alive frames=");
        Serial.print(frameCount);
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