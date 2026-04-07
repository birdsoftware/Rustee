// LoRa recieves UART APP and Selected Gear. OUTPUT Gear and APP voltages

// Update B.Bird 4.6.26 Switched LoRa V2 for V3
// Removing DAC going back to PWM

// Update: B.Bird 3.26.26
// Add M,2,1 Output = HIGH

// Update: B.Bird 3.25.26
// Added Parser for Gear (P,N,D,R,M,1,2,?)
// GPIO 12 HIGH only in Drive
// GPIO 13 HIGH only in R

// Update B.Bird 3.16.26 
// Removed PWM and using DAC output for pure analog voltage 0 - 3.3V

// GEAR_R_OUT = 5 // V2: pin 12 -> GPIO 1 R
// GEAR_D_OUT = 6 // V2: pin 13 -> GPIO 2 D
// APP_PWM_OUT = 7 // pin 25 -> app out
// GND -> GND lora
// 5v -> 5v lora

// APP (analog via PWM + filter)
// GPIO7 ── 1k resistor ──┬──→ APP signal
//                        |
//                      1µF
//                        |
//                       GND

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"

#define RF_FREQUENCY 915000000 // Hz

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define BUFFER_SIZE 64 //30

//V2 OLED
// #ifndef Vext
// #define Vext 21
// #endif

// #define OLED_SDA 4
// #define OLED_SCL 15
// #define OLED_RST 16

const int APP_PWM_OUT = 7;  // Accelerator Pedal Position out
const int GEAR_D_OUT = 6;   // HIGH only in Drive
const int GEAR_R_OUT = 5;   // HIGH only in Reverse
const int APP_PWM_FREQ = 20000;
const int APP_PWM_RES  = 8;

//static SSD1306Wire display(0x3C, 400000, OLED_SDA, OLED_SCL, GEOMETRY_128_64, OLED_RST);//V2
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
static RadioEvents_t RadioEvents;

char rxpacket[BUFFER_SIZE];

bool lora_idle = true;
// ---- Timeout (no packets recieved for 1 second) 
unsigned long lastPacketTime = 0;
const unsigned long SIGNAL_TIMEOUT_MS = 2000;
bool timedOut = true;

int16_t lastRssi = 0;
int16_t rxSize = 0;
String signalQuality;
String linkQuality;

// ---- PWM output (ESP32 LEDC) ----
// const int PWM_PIN = 25;      // pick a GPIO that isn't used by LoRa/OLED
// const int PWM_FREQ = 5000;   // PWM frequency
// const int PWM_RES  = 8;      // 8-bit resolution -> 0..255 duty

//V3 OLED Helper
void VextON(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void clearOutputs() {
  //dacWrite(APP_PWM_OUT, 0);
  ledcWrite(APP_PWM_OUT, 0);
  digitalWrite(GEAR_D_OUT, LOW);
  digitalWrite(GEAR_R_OUT, LOW);
}

void updateGearOutputs(char gear) {
  digitalWrite(GEAR_D_OUT, LOW);
  digitalWrite(GEAR_R_OUT, LOW);

  if (gear == 'D' || gear == 'M' || gear == '1' || gear == '2') {
    digitalWrite(GEAR_D_OUT, HIGH);
  } else if (gear == 'R') {
    digitalWrite(GEAR_R_OUT, HIGH);
  } else if (gear == 'P') {
    digitalWrite(GEAR_D_OUT, LOW);
    digitalWrite(GEAR_R_OUT, LOW);
  }

}

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
       
  pinMode(GEAR_D_OUT, OUTPUT);
  pinMode(GEAR_R_OUT, OUTPUT);

  // Start MCU/Radio stack
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  ledcAttach(APP_PWM_OUT, APP_PWM_FREQ, APP_PWM_RES);
  clearOutputs();

  // ---- OLED bring-up (known-good for V2) ----
  // pinMode(Vext, OUTPUT);
  // digitalWrite(Vext, LOW);     // LOW = ON for your board
  // delay(200);

  // pinMode(OLED_RST, OUTPUT);
  // digitalWrite(OLED_RST, LOW);
  // delay(20);
  // digitalWrite(OLED_RST, HIGH);
  // delay(50);

  // Wire.begin(OLED_SDA, OLED_SCL);
  // delay(10);
  //----

  // OLED V3 --
  VextON();
  delay(100);
  // ---

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

  //digitalWrite(GEAR_D_OUT, LOW);
  //digitalWrite(GEAR_R_OUT, LOW);      
}

void loop() {
  if (lora_idle) {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0); // continuous RX
  }

  Radio.IrqProcess();

  //timed out
  if (!timedOut && (millis() - lastPacketTime > SIGNAL_TIMEOUT_MS)) {
    clearOutputs();   // important fix
    //dacWrite(APP_PWM_OUT, 0);
    timedOut = true;
    Serial.println("TIMEOUT -> outputs cleared");
    drawOledStatus("LoRa timeout - PWM forced to 0");
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi_in, int8_t snr) {
  lastPacketTime = millis(); //reset timeout
  timedOut = false; //reset timeout

  lastRssi = rssi_in;
  rxSize = size;

  if (lastRssi > -70) signalQuality = "Strong";
  else if (lastRssi > -100) signalQuality = "Medium";
  else signalQuality = "Weak";

  if (snr > 8) linkQuality = "Excellent";
  else if (snr > 3) linkQuality = "Good";
  else if (snr > 0) linkQuality = "Fair";
  else linkQuality = "Weak";

  // prevent overflow
  if (size >= BUFFER_SIZE) size = BUFFER_SIZE - 1;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';

  Radio.Sleep();

  Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",
                rxpacket, lastRssi, rxSize);

  // PWM is the first number in the packet like "137"
  int pwm = atoi(rxpacket);          // convert string to int
  pwm = constrain(pwm, 0, 255);      // your allowed range
  ledcWrite(APP_PWM_OUT, pwm);         // output PWM duty (0..255)
  //dacWrite(25, pwm);//

  char gear = '?';
  char *g = strstr(rxpacket, "G: ");
  if (g != NULL) {
    gear = *(g + 3);   // character after "G: "
  } else {
    gear = 'P';  // force Park if parsing fails
  }

  updateGearOutputs(gear);

  Serial.printf("PWM set to: %d\n", pwm);
  Serial.printf("Gear: %c\n", gear);             

  // Show on OLED
  drawOledStatus("RX Packet:",
                 String(rxpacket),
                 "SNR:  " + linkQuality + " " + String(snr) + " dB",
                 "RSSI: " + signalQuality + " " + String(lastRssi) + " dBm");//"Len:  " + String(rxSize));

  lora_idle = true;
}