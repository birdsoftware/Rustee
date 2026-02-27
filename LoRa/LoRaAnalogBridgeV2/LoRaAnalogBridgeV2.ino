#include "LoRaWan_APP.h"
#include "Arduino.h"

HardwareSerial Ext(1);

#define RX_PIN 19      // Nano TX wired here
#define TX_PIN 34      // dummy TX pin

#define RF_FREQUENCY            915000000
#define TX_OUTPUT_POWER         10        // safe & strong
#define LORA_BANDWIDTH          0         // 125 kHz
#define LORA_SPREADING_FACTOR   7
#define LORA_CODINGRATE         1         // 4/5
#define LORA_PREAMBLE_LENGTH    8
#define LORA_IQ_INVERSION_ON    false

#define BUFFER_SIZE 128

static RadioEvents_t RadioEvents;
bool lora_idle = true;

char txpacket[BUFFER_SIZE];

void OnTxDone(void) {
  Serial.println("LoRa TX done");
  lora_idle = true;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("LoRa TX timeout");
  lora_idle = true;
}

// Read a full line from Nano
bool readLine(char* out, size_t maxLen, size_t* outLen) {
  static size_t idx = 0;

  while (Ext.available()) {
    char c = Ext.read();

    if (c == '\r') continue;

    if (c == '\n') {
      out[idx] = '\0';
      *outLen = idx;
      idx = 0;
      return (*outLen > 0);
    }

    if (idx < maxLen - 1) {
      out[idx++] = c;
    }
  }

  return false;
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\nLoRa UART Bridge Starting...");

  Ext.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

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

  Serial.println("Waiting for Nano input...");
}

void loop() {
  Radio.IrqProcess();

  if (lora_idle) {
    size_t len = 0;

    if (readLine(txpacket, BUFFER_SIZE, &len)) {
      Serial.print("Sending over LoRa: ");
      Serial.println(txpacket);

      Radio.Send((uint8_t*)txpacket, len);
      lora_idle = false;
    }
  }
}