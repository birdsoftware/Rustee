//simple take analog from Nano
// assumes we have arduino Tx going to a divider then to GPIO 19 on the LoRa and common grounds
// between lora and Nano.
// Tx -> 1K R -> Rx
// 1K R -> 2K R -> GND
// GND Arduino -> GND LoRa

#include <Arduino.h>

HardwareSerial Ext(1);

static const int RX_PIN = 19;   // <-- you wired Nano TX here
static const int TX_PIN = 34;   // not used, but must be a valid GPIO 19, 20, 21, 26, 33, 34, 35, 36

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\nBOOT: Ext UART on RX=33 (Nano TX) ...");

  Ext.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("Ext.begin OK. Echoing incoming bytes:");
}

void loop() {
  while (Ext.available()) {
    Serial.write((char)Ext.read());
  }
}