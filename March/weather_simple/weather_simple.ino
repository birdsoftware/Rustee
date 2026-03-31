// Display wiring to Arduino Nano
// GND -> GND
// VCC -> 5V
// SCL -> D13  (SPI SCK)
// SDA -> D11  (SPI MOSI)
// RST -> D9
// DC  -> D8
// BLK -> 5V   (backlight)
// CS  -> not used / internally tied
// SPI TFT LCD, ST7789-compatible, no CS breakout, custom/nonstandard geometry.

//DHT11
//left pin (S) D2
//middle 5V
//right GND


#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <DHT.h>

#define TFT_DC   8
#define TFT_RST  9
#define TFT_CS   -1

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// DHT11
#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  delay(200);
    dht.begin();

  tft.init(235, 185);   // first try
  tft.setRotation(0);

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(2, 10);
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = t * 9.0 / 5.0 + 32;

  //tft.fillScreen(ST77XX_BLACK);   // clear screen
  tft.print("T: ");
  tft.print(t);
  tft.println(" C");
  tft.print("T: ");
  tft.print(f);
  tft.println(" F");
  tft.print("H: ");
  tft.print(h);
  tft.println(" %");
  delay(1000);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(2, 10);
}