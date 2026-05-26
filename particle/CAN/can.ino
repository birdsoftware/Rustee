#include "Particle.h"
#include "mcp_can.h"

SYSTEM_MODE(AUTOMATIC);

#define CAN_CS D5

MCP_CAN CAN(CAN_CS);

void setup() {
    Serial.begin(115200);
    waitFor(Serial.isConnected, 5000);

    while (CAN_OK != CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ)) {
        Serial.println("CAN init fail...");
        delay(1000);
    }

    Serial.println("CAN init ok!");

    CAN.setMode(MCP_NORMAL);
}

void loop() {

    if (CAN.checkReceive() == CAN_MSGAVAIL) {

        long unsigned int rxId;
        unsigned char len = 0;
        unsigned char buf[8];

        CAN.readMsgBuf(&rxId, &len, buf);

        Serial.printf("ID: 0x%lX  Data: ", rxId);

        for (int i = 0; i < len; i++) {
            Serial.printf("%02X ", buf[i]);
        }

        Serial.println();
    }
}