#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <Wire.h> 

#define RF_FREQUENCY            915000000
#define TX_OUTPUT_POWER         10
#define LORA_BANDWIDTH          0         // 125 kHz
#define LORA_SPREADING_FACTOR   7
#define LORA_CODINGRATE         1         // 4/5
#define LORA_PREAMBLE_LENGTH    8
#define LORA_IQ_INVERSION_ON    false

#define BUFFER_SIZE 128

// OLED
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

static RadioEvents_t RadioEvents;
bool lora_idle = true;

char txpacket[BUFFER_SIZE];

// Test ramp settings
const float VIN_MIN = 0.7f;
const float VIN_MAX = 3.3f;
const float VIN_STEP = 0.02f;

float vin = VIN_MIN;
float discreteVin = VIN_MIN;
int pwm = 0;
unsigned long n = 0;

void OnTxDone(void) {
  Serial.println("LoRa TX done");
  lora_idle = true;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("LoRa TX timeout");
  lora_idle = true;
}

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void drawOledStatus(const String& line1,
                    const String& line2 = "",
                    const String& line3 = "",
                    const String& line4 = "") {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, line1);
  if (line2.length()) display.drawString(0, 12, line2);
  if (line3.length()) display.drawString(0, 24, line3);
  if (line4.length()) display.drawString(0, 36, line4);
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\nLoRa Test Signal TX Starting...");

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  VextON();
  delay(100);
  display.init();
  drawOledStatus(
    "LoRa Test TX",
    "Format: PWM I O #",
    "0.7V -> 3.3V",
    "Looping"
  );

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(
    MODEM_LORA,
    TX_OUTPUT_POWER,
    0,
    LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR,
    LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH,
    false,
    true,
    0,
    0,
    LORA_IQ_INVERSION_ON,
    3000
  );

  Serial.println("Test signal transmitter ready.");
}
int pwm_max = 255;
void loop() {
  Radio.IrqProcess();

  if (lora_idle) {
    // Current values for this packet
    discreteVin = vin;
    pwm = (int)(vin * 255.0f / 3.33f + 0.5f);
    if (pwm < 0) pwm = 0;
    if (pwm > pwm_max) pwm = pwm_max;

    if (n > 99) n = 0;

    // Exact format requested
    String payload =
      String(pwm) + " I: " +
      String(vin, 3) + " O: " +
      String(discreteVin, 3) + " #" +
      String(n);

    payload.toCharArray(txpacket, BUFFER_SIZE);

    Serial.print("Sending over LoRa: ");
    Serial.println(txpacket);

    drawOledStatus(
      "LoRa Test TX",
      "PWM: " + String(pwm),
      "I: " + String(vin, 3) + " O: " + String(discreteVin, 3),
      "#" + String(n)
    );

    Radio.Send((uint8_t*)txpacket, strlen(txpacket));
    lora_idle = false;

    // Advance for next loop
    vin += VIN_STEP;
    if (vin > VIN_MAX) {
      vin = VIN_MIN;
    }

    n++;
    delay(200);
  }
}