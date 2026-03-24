#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// --------------------
// Pin wiring
// --------------------
#define TFT_DC   8
#define TFT_RST  9
#define TFT_CS   -1   // no external CS pin

// Create display object
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// --------------------
// Helpers
// --------------------
void drawHeader(const char* title) {
  tft.fillRect(0, 0, tft.width(), 24, ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(4, 4);
  tft.print(title);
}

void drawStatusBox(int x, int y, int w, int h, uint16_t color, const char* label, const char* value) {
  tft.drawRect(x, y, w, h, color);
  tft.setTextColor(color);
  tft.setTextSize(1);
  tft.setCursor(x + 4, y + 4);
  tft.print(label);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(x + 4, y + 18);
  tft.print(value);
}

void drawCenteredText(const char* text, int y, uint16_t color, int size) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.setTextSize(size);
  tft.getTextBounds((char*)text, 0, y, &x1, &y1, &w, &h);
  int x = (tft.width() - w) / 2;
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.print(text);
}

void startupScreen() {
  tft.fillScreen(ST77XX_BLACK);

  drawHeader("Display");

  drawCenteredText("ST7789 OK", 40, ST77XX_GREEN, 2);
  drawCenteredText("Arduino Nano", 70, ST77XX_WHITE, 1);

  tft.drawRect(10, 95, tft.width() - 20, 20, ST77XX_WHITE);
  tft.fillRect(12, 97, tft.width() - 24, 16, ST77XX_GREEN);

  delay(1500);
}

void screenInfo() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader("System");

  drawStatusBox(8, 35, 100, 40, ST77XX_CYAN, "Mode", "SPI");
  drawStatusBox(118, 35, 100, 40, ST77XX_YELLOW, "Pins", "OK");
  drawStatusBox(8, 85, 100, 40, ST77XX_GREEN, "Driver", "7789");
  drawStatusBox(118, 85, 100, 40, ST77XX_MAGENTA, "CS", "NONE");

  delay(2500);
}

void screenColors() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader("Colors");

  int y = 35;
  int h = 35;
  int w = tft.width() / 3;

  tft.fillRect(0, y, w, h, ST77XX_RED);
  tft.fillRect(w, y, w, h, ST77XX_GREEN);
  tft.fillRect(w * 2, y, w, h, ST77XX_BLUE);

  tft.fillRect(0, y + h + 10, w, h, ST77XX_YELLOW);
  tft.fillRect(w, y + h + 10, w, h, ST77XX_CYAN);
  tft.fillRect(w * 2, y + h + 10, w, h, ST77XX_MAGENTA);

  delay(2500);
}

void screenShapes() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader("Shapes");

  tft.drawRect(10, 35, 60, 40, ST77XX_WHITE);
  tft.fillRect(80, 35, 60, 40, ST77XX_RED);

  tft.drawCircle(180, 55, 20, ST77XX_GREEN);
  tft.fillCircle(35, 105, 18, ST77XX_BLUE);

  tft.drawLine(70, 90, 130, 125, ST77XX_YELLOW);
  tft.drawTriangle(150, 90, 200, 125, 220, 85, ST77XX_CYAN);

  delay(2500);
}

void screenText() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader("Text");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(10, 35);
  tft.println("Small text");

  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.setCursor(10, 55);
  tft.println("Medium");

  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(3);
  tft.setCursor(10, 85);
  tft.println("BIG");

  delay(2500);
}

void screenMeter(int value) {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader("Meter");

  tft.drawRect(2, 20, 170, 24, ST77XX_WHITE);
  int fillWidth = map(value, 0, 100, 0, 166);
  tft.fillRect(4, 22, fillWidth, 20, ST77XX_GREEN);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(85, 90);
  tft.print(value);
  tft.print("%");
}

void setup() {
  delay(200);

  // Your working init
  tft.init(235, 185);
  tft.setRotation(0);

  startupScreen();
}

void loop() {
  //screenInfo();
  //screenColors();
  //screenShapes();
  //screenText();

  for (int i = 0; i <= 80; i += 10) {
    screenMeter(i);
    delay(300);
  }

  for (int i = 80; i >= 0; i -= 10) {
    screenMeter(i);
    delay(300);
  }
}
