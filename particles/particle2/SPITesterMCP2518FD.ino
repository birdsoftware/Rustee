/*
Particle Argon MCP2518FD Module
Both 3V3 and 5V are required:
- 3V3 powers MCP2518FD logic / SPI
- external 5V powers ATA6563 CAN transceiver

SPI Connections
Particle Argon MCP2518FD Module

D13 (SCK) -> SCK
D12 (MOSI) -> SDI / MOSI
D11 (MISO) -> SDO / MISO
D5 -> CS
D2 -> INT

CAN Connections
OpenECU MCP2518FD Module

CAN_H -> CAN_H
CAN_L -> CAN_L
GND -> GND (recommended)

Crystal
Module crystal frequency: 20 MHz

Firmware Settings
CAN Bit Rate: 250000 (250 kbps)

Particle Pin Definitions
static const uint8_t CS_PIN = D5;
static const uint8_t INT_PIN = D2;

Known-Good SPI Test
OSC Register:
0x60040000
IOCON Register:
0x03000303
C1CON Register:
0x60079804

If these values are readable and stable, SPI communication is functioning correctly.

TEST TO MAKE SURE These values are stable and repeatable:
OSC   = 0x60040000
IOCON = 0x03000303
C1CON = 0x60079804

That means:
Argon ⇄ MCP2518FD SPI communication is working
Power is good
CS/SCK/SDI/SDO wiring is good
*/
#include "Particle.h"
#include <SPI.h>

SYSTEM_MODE(AUTOMATIC);

static const uint8_t CS_PIN = D5;

static const uint16_t REG_C1CON  = 0x000;
static const uint16_t REG_C1INT  = 0x01C;
static const uint16_t REG_OSC    = 0xE00;
static const uint16_t REG_IOCON  = 0xE04;
static const uint16_t REG_CRC    = 0xE08;
static const uint16_t REG_ECCCON = 0xE0C;
static const uint16_t REG_ECCSTA = 0xE10;
static const uint16_t REG_DEVID  = 0xE14;

uint16_t makeCmd(uint16_t instruction, uint16_t address) {
    return (instruction << 12) | (address & 0x0FFF);
}

void selectChip() {
    digitalWrite(CS_PIN, LOW);
}

void deselectChip() {
    digitalWrite(CS_PIN, HIGH);
}

uint8_t xfer(uint8_t v) {
    return SPI.transfer(v);
}

void mcpReset() {
    selectChip();
    xfer(0x00);
    xfer(0x00);
    deselectChip();
    delay(20);
}

uint32_t readReg32(uint16_t address) {
    uint16_t cmd = makeCmd(0x3, address);

    selectChip();

    xfer(cmd >> 8);
    xfer(cmd & 0xFF);

    uint32_t v = 0;
    v |= ((uint32_t)xfer(0x00)) << 24;
    v |= ((uint32_t)xfer(0x00)) << 16;
    v |= ((uint32_t)xfer(0x00)) << 8;
    v |= ((uint32_t)xfer(0x00));

    deselectChip();

    return v;
}

void printReg(const char *name, uint16_t addr) {
    uint32_t v = readReg32(addr);

    Serial.print(name);
    Serial.print(" 0x");
    Serial.print(addr, HEX);
    Serial.print(" = 0x");
    Serial.println(v, HEX);
}

void setup() {
    Serial.begin(115200);
    waitFor(Serial.isConnected, 10000);

    Serial.println();
    Serial.println("MCP2518FD RAW REGISTER TEST");

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    SPI.begin();
    SPI.setClockSpeed(1, MHZ);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);

    Serial.println("Resetting MCP2518FD...");
    mcpReset();

    Serial.println("Initial register dump:");
}

void loop() {
    Serial.println();
    printReg("OSC   ", REG_OSC);
    printReg("IOCON ", REG_IOCON);
    printReg("DEVID ", REG_DEVID);
    printReg("C1CON ", REG_C1CON);
    printReg("C1INT ", REG_C1INT);
    printReg("CRC   ", REG_CRC);
    printReg("ECCCON", REG_ECCCON);
    printReg("ECCSTA", REG_ECCSTA);

    delay(2000);
}