// Control Arduino Nano 5v
//
// INPUT HW-184 wiring
// HW-184 CAN-H -> ODB-II Pin 6 CAN High
// HW-184 CAN-L -> ODB-II Pin 14 CAN LOW
// HW-184 Vcc -> 5V Nano
// HW-184 GND -> GND Nano
// HW-184 CS -> D10 Nano
// HW-184 SO -> D12 Nano
// HW-184 SI -> D11 Nano
// HW-184 SCK -> D13 Nano
// HW-184 INT -> D2 Nano
//

//
// SERVO: 25kg DC4.8v-6.8v DSSERVO
// Black: Ground (–)
// Red: Positive (+) / VCC — Connects to the positive power supply (4.8V – 6.8V)
// White: Signal (S) — Connects to the PWM control signal

#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9);
  myServo.write(0);  // start position
}

void loop() {
  myServo.write(0);   // position 1
  delay(2000);

  myServo.write(45);  // position 2
  delay(2000);
}