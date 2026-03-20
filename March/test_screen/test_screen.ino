// | Display Pin    | Function        | Arduino Nano                |
// | -------------- | --------------- | --------------------------- |
// | GND            | Ground          | GND                         |
// | VCC            | Power           | 5V (or 3.3V)                |
// | SCL            | SPI Clock (SCK) | D13                         |
// | SDA            | MOSI (Data)     | D11                         |
// | RST            | Reset           | D9 (any pin)                |
// | DC             | Data/Command    | D8 (any pin)                |
// | BLK            | Backlight       | 5V (or PWM pin for dimming) |
// | CS *(if used)* | Chip Select     | D10                         |
#include <U8g2lib.h>

// Try this first (most likely)
//U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/10, /* dc=*/8, /* reset=*/9);

// If that fails, try this:
 U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 10, 8, 9);

void setup() {
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0,15,"Hello Brian!");
  u8g2.sendBuffer();
}

void loop() {}