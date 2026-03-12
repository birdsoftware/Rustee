const int leds[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
const int ledCount = 10;

void allOff() {
  for (int i = 0; i < ledCount; i++) {
    digitalWrite(leds[i], LOW);
  }
}

void allOn() {
  for (int i = 0; i < ledCount; i++) {
    digitalWrite(leds[i], HIGH);
  }
}

void swirlIn(int waitTime) {
  for (int i = 0; i < ledCount; i++) {
    digitalWrite(leds[i], HIGH);
    delay(waitTime);
  }
}

void swirlOut(int waitTime) {
  for (int i = ledCount - 1; i >= 0; i--) {
    digitalWrite(leds[i], LOW);
    delay(waitTime);
  }
}

void pulseBurst(int times, int onTime, int offTime) {
  for (int i = 0; i < times; i++) {
    allOn();
    delay(onTime);
    allOff();
    delay(offTime);
  }
}

void sparkleChaos(int times) {
  for (int i = 0; i < times; i++) {
    int led = random(0, ledCount);
    digitalWrite(leds[led], HIGH);
    delay(random(20, 90));
    digitalWrite(leds[led], LOW);
    delay(random(10, 50));
  }
}

void rippleSpell() {
  for (int i = 0; i < ledCount; i++) {
    allOff();
    digitalWrite(leds[i], HIGH);
    if (i > 0) digitalWrite(leds[i - 1], HIGH);
    if (i < ledCount - 1) digitalWrite(leds[i + 1], HIGH);
    delay(70);
  }

  for (int i = ledCount - 1; i >= 0; i--) {
    allOff();
    digitalWrite(leds[i], HIGH);
    if (i > 0) digitalWrite(leds[i - 1], HIGH);
    if (i < ledCount - 1) digitalWrite(leds[i + 1], HIGH);
    delay(70);
  }
}

void setup() {
  for (int i = 0; i < ledCount; i++) {
    pinMode(leds[i], OUTPUT);
  }

  allOff();
  randomSeed(analogRead(A0));
}

void loop() {
  // Phase 1: magic gathers
  swirlIn(80);
  delay(150);

  // Phase 2: energy pulses
  pulseBurst(3, 120, 80);
  delay(120);

  // Phase 3: spell travels
  rippleSpell();
  delay(100);

  // Phase 4: magical explosion
  pulseBurst(2, 200, 120);
  sparkleChaos(35);

  // calm down before repeating
  allOff();
  delay(600);
}