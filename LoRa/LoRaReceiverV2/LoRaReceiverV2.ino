#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"

#define RF_FREQUENCY 915000000 // Hz

#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define BUFFER_SIZE 30

#ifndef Vext
#define Vext 21
#endif

#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

static SSD1306Wire display(0x3C, 400000, OLED_SDA, OLED_SCL, GEOMETRY_128_64, OLED_RST);

char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t lastRssi = 0;
int16_t rxSize = 0;

bool lora_idle = true;

// ---- PWM output (ESP32 LEDC) ----
const int PWM_PIN = 25;      // pick a GPIO that isn't used by LoRa/OLED
const int PWM_FREQ = 5000;   // PWM frequency
const int PWM_RES  = 8;      // 8-bit resolution -> 0..255 duty

void drawOledStatus(const String& line1, const String& line2 = "", const String& line3 = "", const String& line4 = "") {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, line1);
  if (line2.length()) display.drawString(0, 12, line2);
  if (line3.length()) display.drawString(0, 24, line3);
  if (line4.length()) display.drawString(0, 36, line4);
  display.display();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi_in, int8_t snr);

void setup() {
  Serial.begin(115200);
  delay(200);

  // Start MCU/Radio stack
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // ---- OLED bring-up (known-good for V2) ----
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);     // LOW = ON for your board
  delay(200);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  delay(50);

  Wire.begin(OLED_SDA, OLED_SCL);
  delay(10);

  display.init();
  drawOledStatus("LoRa RX Ready", "Waiting for packets...");
  // ------------------------------------------

  // Radio init
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetRxConfig(MODEM_LORA,
                    LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE,
                    0,
                    LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT,
                    LORA_FIX_LENGTH_PAYLOAD_ON,
                    0,
                    true,
                    0,
                    0,
                    LORA_IQ_INVERSION_ON,
                    true);

  // ---- PWM init ----
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_PIN, 0);                
}

void loop() {
  if (lora_idle) {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0); // continuous RX
  }
  Radio.IrqProcess();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi_in, int8_t snr) {
  lastRssi = rssi_in;
  rxSize = size;

  // prevent overflow
  if (size >= BUFFER_SIZE) size = BUFFER_SIZE - 1;

  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';

  Radio.Sleep();

  Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",
                rxpacket, lastRssi, rxSize);

  // PWM output packet contains an integer like "137"
  int pwm = atoi(rxpacket);          // convert string to int
  pwm = constrain(pwm, 0, 200);      // your allowed range
  ledcWrite(PWM_PIN, pwm);            // output PWM duty (0..255)
  Serial.printf("PWM set to: %d\n", pwm);
  //end PWM              

  // Show on OLED
  drawOledStatus("RX Packet:",
                 String(rxpacket),
                 "PWM:  " + String(pwm),
                 "dBm: " + String(lastRssi));//"Len:  " + String(rxSize));

  lora_idle = true;
}