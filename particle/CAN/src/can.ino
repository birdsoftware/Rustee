// =====================================================
// MCP2515 / HW-184 Connections
//
// HW-184        Particle Argon
// ----------------------------
// VCC           external 5V
// GND           GND
// CS            D5
// SO            MISO / D11
// SI            MOSI / D12
// SCK           SCK  / D13
// INT           D2
// CAN bus       250 kbps
// Crystal       8 MHz
// =====================================================
// TXS0108E UA -> Argon 3V3
// TXS0108E UB -> external 5V
// TXS0108E OE -> Argon 3V3
// All grounds common

#include "Particle.h"
#include <mcp_can.h>
#include <SPI.h>

SYSTEM_MODE(AUTOMATIC);

#define CAN_CS_PIN  D5
#define CAN_INT_PIN D2

MCP_CAN CAN0(CAN_CS_PIN);

// Particle.variable() requires double, not float
double dischargeChargeCurrLim = NAN;
double pumpHighSpeedRPM       = NAN;
double motorToPumpGearRatio   = NAN;
double canRectifierTemp       = NAN;
double canMcuDcCurrent        = NAN;
double canMcuDcVoltage        = NAN;
double canMcuMotorSpeed       = NAN;
double canMcuMotorTemp        = NAN;
double ciRectifierTempDegC    = NAN;
double ciGeneratorTempDegC    = NAN;

uint32_t pendingAddr[256];

unsigned long lastPrint = 0;
unsigned long lastPublish = 0;
unsigned long lastHeartbeat = 0;
unsigned long frameCount = 0;

float readFloatBE(byte *b) {
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

uint32_t readAddrFromRequest(byte *buf) {
    return
        ((uint32_t)buf[4] << 24) |
        ((uint32_t)buf[5] << 16) |
        ((uint32_t)buf[6] << 8)  |
        ((uint32_t)buf[7]);
}

void printValue(const char *name, double value, int decimals) {
    Serial.print(name);
    Serial.print(": ");

    if (isnan(value)) {
        Serial.println("--");
    } else if (isinf(value)) {
        Serial.println("ovf");
    } else {
        Serial.println(value, decimals);
    }
}

void printTable() {
    Serial.println();
    Serial.println("===== OpenECU Live Values =====");

    printValue("DischargeChargeCurrLim", dischargeChargeCurrLim, 0);
    printValue("PumpHighSpeedRPM", pumpHighSpeedRPM, 0);
    printValue("MotorToPumpGearRatio", motorToPumpGearRatio, 0);
    printValue("CAN_RectifierTemp", canRectifierTemp, 2);
    printValue("CAN_McuDcCurrent", canMcuDcCurrent, 2);
    printValue("CAN_McuDcVoltage", canMcuDcVoltage, 2);
    printValue("CAN_McuMotorSpeed", canMcuMotorSpeed, 0);
    printValue("CAN_McuMotorTemp", canMcuMotorTemp, 2);
    printValue("CI_RectifierTempDegC", ciRectifierTempDegC, 2);
    printValue("CI_GeneratorTempDegC", ciGeneratorTempDegC, 2);

    Serial.println("===============================");
}

void publishToCloud() {
    if (!Particle.connected()) return;

    char data[256];

    snprintf(data, sizeof(data),
        "{\"dccLim\":%.2f,\"pumpRPM\":%.2f,\"gearRatio\":%.2f,"
        "\"rectTemp\":%.2f,\"dcCurrent\":%.2f,\"dcVoltage\":%.2f,"
        "\"motorSpeed\":%.2f,\"motorTemp\":%.2f,"
        "\"ciRectTemp\":%.2f,\"ciGenTemp\":%.2f,\"frames\":%lu}",
        dischargeChargeCurrLim,
        pumpHighSpeedRPM,
        motorToPumpGearRatio,
        canRectifierTemp,
        canMcuDcCurrent,
        canMcuDcVoltage,
        canMcuMotorSpeed,
        canMcuMotorTemp,
        ciRectifierTempDegC,
        ciGeneratorTempDegC,
        frameCount
    );

    bool ok = Particle.publish("can_reading", data, PRIVATE);
    Serial.printlnf("Publish %s: %s", ok ? "OK" : "FAILED", data);
}

void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println();
    Serial.println("BOOT");
    Serial.println("Starting OpenECU CAN decoder...");

    pinMode(CAN_INT_PIN, INPUT_PULLUP);

    Particle.variable("frames", frameCount);
    Particle.variable("dcVoltage", canMcuDcVoltage);
    Particle.variable("motorTemp", canMcuMotorTemp);
    Particle.variable("rectTemp", canRectifierTemp);
    Particle.variable("pumpRPM", pumpHighSpeedRPM);

    SPI.begin();
    SPI.setClockSpeed(1, MHZ);

    while (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println("CAN INIT FAIL");
        delay(1000);
    }

    CAN0.setMode(MCP_LISTENONLY);
    //CAN0.setMode(MCP_NORMAL);

    for (int i = 0; i < 256; i++) {
        pendingAddr[i] = 0;
    }

    Serial.println("CAN init OK");
    Serial.println("OpenECU address-based decoder running...");
}

void loop() {
    unsigned long now = millis();

    if (now - lastHeartbeat >= 2000) {
        lastHeartbeat = now;
        Serial.printlnf("alive WiFi=%d Cloud=%d frames=%lu",
                        WiFi.ready(),
                        Particle.connected(),
                        frameCount);
    }

    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        unsigned long rxId;
        byte len = 0;
        byte buf[8];

        if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) return;

        frameCount++;
        rxId &= 0x1FFFFFFF;

        if (rxId == 0x502 && len == 8) {
            dischargeChargeCurrLim = buf[2];
        }

        if ((rxId == 0x6F9 || rxId == 0x6EF) &&
            len == 8 &&
            buf[0] == 0x0F) {

            byte tx = buf[1];
            uint32_t addr = readAddrFromRequest(buf);
            pendingAddr[tx] = addr;
        }

        if ((rxId == 0x6F8 || rxId == 0x6EE) &&
            len == 8 &&
            buf[0] == 0xFF &&
            buf[1] == 0x00) {

            byte tx = buf[2];
            uint32_t addr = pendingAddr[tx];
            float value = readFloatBE(&buf[3]);

            switch (addr) {
                case 0x0004048C: dischargeChargeCurrLim = value; break;
                case 0x000404D0: pumpHighSpeedRPM = value; break;
                case 0x000404DC: motorToPumpGearRatio = value; break;
                case 0x40002A34: canRectifierTemp = value; break;
                case 0x40002A6C: canMcuDcCurrent = value; break;
                case 0x40002A68: canMcuDcVoltage = value; break;
                case 0x40002A5C: canMcuMotorSpeed = value; break;
                case 0x40002A58: canMcuMotorTemp = value; break;
                case 0x40002568: ciRectifierTempDegC = value; break;
                case 0x4000256C: ciGeneratorTempDegC = value; break;
            }
        }
    }

    if (now - lastPrint >= 1000) {
        lastPrint = now;
        printTable();
    }

    if (now - lastPublish >= 60000) {
        lastPublish = now;
        publishToCloud();
    }
}