#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <AppInsights.h>
#include <RMaker.h>
#include <RMakerDevice.h>
#include <RMakerNode.h>
#include <RMakerParam.h>
#include <RMakerQR.h>
#include <RMakerType.h>
#include <RMakerUtils.h>

// SPECS
// 1 UNO R3
// 1 A0D4184A module (HW-317 “Two-MOS Drive”)
// 1 B 50 OHM POT
// test on (SPAL Automotive VA109-ABL321P/N-109A/SH) fan

// PWM MODULE
// 1. 10hZ - 500hZ ADJUSTABLE PROGRAMMED (20HZ RECOMMENDED FOR FAN) 
// 2. DUTY CYCLE 0 - 100 ADJUSTABLE VIA POT 

// Wiring
// 1) Fan power (constant 12V)
// Fan RED → +12V (use a fuse!)
// Fan BLACK → 12V GND
//
// 2) Common ground (required)
// Arduino GND ↔ 12V GND (same as fan BLACK)
//
// 3) Power the HW-317 board (do this from Arduino 5V for safety)
// Use the bottom-right power terminals:
// HW-317 DC+ → Arduino 5V
// HW-317 DC- → Arduino GND
// (This keeps the board “OUT+” rail at 5V so you’re not accidentally bringing 12V anywhere near the control wiring.)
//
// 4) Feed PWM into the board input
// Use the left input terminals:
// HW-317 PWM+ → Arduino D9
// HW-317 GND → Arduino GND
//
// 5) Connect the fan PWM control wire
// Use the top-right output terminals:
// HW-317 OUT- → Fan WHITE (PWM control input)
// What about OUT+?
// Leave OUT+ NOT CONNECTED for this SPAL control-input use case.
//
// 6) POT into the arduino to adjust duty cycle
// POT outerwire 1 -> Arduino 5V
// POT middle wire -> Arduino A0
// POT outerwire 2 -> Arduino GND


#include <Arduino.h>

const uint8_t PWM_PIN = 9;   // Timer1 OC1A
const uint8_t POT_PIN = A0;

// ===== SETTINGS =====
uint16_t PWM_HZ = 100;        // set 10..500 here (try 100 first)
const uint8_t DUTY_MIN = 10;   // %2
const uint8_t DUTY_MAX = 90;  // %98
// ====================

struct PrescOpt { uint16_t presc; uint8_t csBits; };

void setTimer1PwmHz(uint16_t hz) {
  if (hz < 10) hz = 10;
  if (hz > 500) hz = 500;

  const PrescOpt opts[] = {
    {1,    _BV(CS10)},
    {8,    _BV(CS11)},
    {64,   _BV(CS11) | _BV(CS10)},
    {256,  _BV(CS12)},
    {1024, _BV(CS12) | _BV(CS10)}
  };

  uint32_t top = 0;
  uint8_t chosen = 4;

  for (uint8_t i = 0; i < 5; i++) {
    top = (F_CPU / (uint32_t)opts[i].presc) / hz - 1;
    if (top <= 65535) { chosen = i; break; }
  }

  cli();
  // Fast PWM, TOP = ICR1 (Mode 14), non-inverting on OC1A (D9)
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13)  | _BV(WGM12);

  ICR1  = (uint16_t)top;  // sets frequency
  OCR1A = 0;              // duty

  TCCR1B = (TCCR1B & 0b11111000) | opts[chosen].csBits; // prescaler
  sei();
}

uint16_t readPotSmooth() {
  // simple smoothing for stable duty
  (void)analogRead(POT_PIN);
  delayMicroseconds(150);

  uint32_t sum = 0;
  const uint8_t N = 8;
  for (uint8_t i = 0; i < N; i++) {
    sum += analogRead(POT_PIN);
    delayMicroseconds(150);
  }
  return (uint16_t)(sum / N); // 0..1023
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  setTimer1PwmHz(PWM_HZ);
}

void loop() {
  uint16_t pot = readPotSmooth(); // 0..1023

  // Map pot to DUTY_MIN..DUTY_MAX percent
  uint8_t dutyPct = map(pot, 0, 1023, DUTY_MAX, DUTY_MIN);//flip max min to change dir of POT

  // Convert duty percent to OCR1A counts (0..ICR1)
  uint16_t top = ICR1;
  uint16_t dutyCounts = (uint32_t)dutyPct * top / 100;

  OCR1A = dutyCounts;

  delay(5);
}
