#include "Particle.h"
#include <SPI.h>

SYSTEM_MODE(AUTOMATIC);

static const uint8_t CS_PIN = D5;

// MCP2518FD registers
static const uint16_t REG_C1FLTCON0 = 0x1D0;
static const uint16_t REG_C1FLTOBJ0 = 0x1F0;
static const uint16_t REG_C1MASK0   = 0x1F4;

static const uint16_t REG_C1CON      = 0x000;
static const uint16_t REG_C1NBTCFG   = 0x004;
static const uint16_t REG_C1DBTCFG   = 0x008;
static const uint16_t REG_C1TDC      = 0x00C;
static const uint16_t REG_C1TBC      = 0x010;
static const uint16_t REG_C1TSCON    = 0x014;
static const uint16_t REG_C1VEC      = 0x018;
static const uint16_t REG_C1INT      = 0x01C;
static const uint16_t REG_C1RXIF     = 0x020;
static const uint16_t REG_C1TXIF     = 0x024;
static const uint16_t REG_C1RXOVIF   = 0x028;
static const uint16_t REG_C1TXATIF   = 0x02C;
static const uint16_t REG_C1TXREQ    = 0x030;
static const uint16_t REG_C1TREC     = 0x034;
static const uint16_t REG_C1BDIAG0   = 0x038;
static const uint16_t REG_C1BDIAG1   = 0x03C;
static const uint16_t REG_C1TEFCON   = 0x040;
static const uint16_t REG_C1TEFSTA   = 0x044;
static const uint16_t REG_C1TEFUA    = 0x048;
static const uint16_t REG_C1FIFOCON1 = 0x05C;
static const uint16_t REG_C1FIFOSTA1 = 0x060;
static const uint16_t REG_C1FIFOUA1  = 0x064;

static const uint16_t REG_OSC        = 0xE00;
static const uint16_t REG_IOCON      = 0xE04;

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

void printReg(const char *name, uint16_t addr) {
    Serial.print(name);
    Serial.print(" = 0x");
    Serial.println(readReg32(addr), HEX);
}

void requestMode(uint8_t mode) {
    uint32_t c1con = readReg32(REG_C1CON);

    c1con &= ~(0x7UL << 24);
    c1con |= ((uint32_t)mode << 24);

    writeReg32(REG_C1CON, c1con);
    delay(50);
}

void initCAN() {
    Serial.println();
    Serial.println("MCP2518FD CAN STATUS TEST");

    mcpReset();

    Serial.println("After reset:");
    printReg("OSC   ", REG_OSC);
    printReg("IOCON ", REG_IOCON);
    printReg("C1CON ", REG_C1CON);

    Serial.println("Config mode...");
    requestMode(4);

    // 20 MHz oscillator, rough 250 kbps nominal timing.
    writeReg32(REG_C1NBTCFG, 0x003E0F0F);
    writeReg32(REG_C1DBTCFG, 0x003E0F0F);

    // FIFO1 as RX FIFO
    writeReg32(REG_C1FIFOCON1, 0x0000001F);

    // Accept-all filter 0 -> FIFO1
    writeReg32(REG_C1FLTOBJ0, 0x00000000);
    writeReg32(REG_C1MASK0,   0x00000000);
    writeReg32(REG_C1FLTCON0, 0x00000081);

    // Clear flags.
    writeReg32(REG_C1INT, 0x00000000);
    writeReg32(REG_C1RXIF, 0x00000000);
    writeReg32(REG_C1RXOVIF, 0x00000000);

    Serial.println("Normal mode...");
    //requestMode(6);   // Normal mode
    requestMode(0);

    Serial.println("After init:");
    printReg("OSC   ", REG_OSC);
    printReg("IOCON ", REG_IOCON);
    printReg("C1CON ", REG_C1CON);
    printReg("NBTCFG", REG_C1NBTCFG);
    printReg("DBTCFG", REG_C1DBTCFG);
    printReg("FCON1 ", REG_C1FIFOCON1);
    printReg("FSTA1 ", REG_C1FIFOSTA1);
    printReg("FUA1  ", REG_C1FIFOUA1);
    printReg("C1INT ", REG_C1INT);
    printReg("C1RXIF", REG_C1RXIF);
    printReg("TREC  ", REG_C1TREC);
    printReg("BDIAG0", REG_C1BDIAG0);
    printReg("BDIAG1", REG_C1BDIAG1);
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

void loop() {
    if (millis() - lastStatus >= 2000) {
        lastStatus = millis();

        Serial.println();
        printReg("C1CON ", REG_C1CON);
        printReg("C1INT ", REG_C1INT);
        printReg("C1RXIF", REG_C1RXIF);
        printReg("RXOVIF", REG_C1RXOVIF);
        printReg("FSTA1 ", REG_C1FIFOSTA1);
        printReg("FUA1  ", REG_C1FIFOUA1);
        printReg("TREC  ", REG_C1TREC);
        printReg("BDIAG0", REG_C1BDIAG0);
        printReg("BDIAG1", REG_C1BDIAG1);
    }
}