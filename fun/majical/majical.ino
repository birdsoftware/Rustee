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

void setOnly(int a) {
  allOff();
  digitalWrite(leds[a], HIGH);
}

void setTriple(int center) {
  allOff();
  digitalWrite(leds[center], HIGH);
  if (center > 0) digitalWrite(leds[center - 1], HIGH);
  if (center < ledCount - 1) digitalWrite(leds[center + 1], HIGH);
}

void setQuadWrap(int a, int b, int c, int d) {
  allOff();
  digitalWrite(leds[a % ledCount], HIGH);
  digitalWrite(leds[b % ledCount], HIGH);
  digitalWrite(leds[c % ledCount], HIGH);
  digitalWrite(leds[d % ledCount], HIGH);
}

void summonGlow() {
  for (int round = 0; round < 4; round++) {
    for (int i = 0; i < ledCount; i++) {
      digitalWrite(leds[i], HIGH);
      delay(35);
    }
    for (int i = ledCount - 1; i >= 0; i--) {
      digitalWrite(leds[i], LOW);
      delay(20);
    }
  }
}

void orbitSpell() {
  for (int i = 0; i < 24; i++) {
    setQuadWrap(i, i + 2, i + 5, i + 7);
    delay(70 - min(i, 10));   // speeds up a little
  }
}

void unstableEnergy() {
  for (int i = 0; i < 30; i++) {
    int center = random(0, ledCount);
    setTriple(center);
    delay(random(25, 80));
    allOff();
    delay(random(10, 40));
  }
}

void burstFlash() {
  for (int i = 0; i < 4; i++) {
    allOn();
    delay(90);
    allOff();
    delay(60);
  }
}

void magicalWave() {
  for (int i = 0; i < ledCount; i++) {
    setTriple(i);
    delay(60);
  }
  for (int i = ledCount - 1; i >= 0; i--) {
    setTriple(i);
    delay(60);
  }
}

void fallingStars() {
  for (int i = 0; i < 40; i++) {
    int led = random(0, ledCount);
    digitalWrite(leds[led], HIGH);
    delay(random(20, 70));
    digitalWrite(leds[led], LOW);
    delay(random(10, 50));
  }
}

void sacredCircle() {
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < ledCount; j += 2) {
      digitalWrite(leds[j], HIGH);
    }
    for (int j = 1; j < ledCount; j += 2) {
      digitalWrite(leds[j], LOW);
    }
    delay(120);

    for (int j = 0; j < ledCount; j += 2) {
      digitalWrite(leds[j], LOW);
    }
    for (int j = 1; j < ledCount; j += 2) {
      digitalWrite(leds[j], HIGH);
    }
    delay(120);
  }
  allOff();
}

void fadeToWhispers() {
  for (int i = 0; i < 12; i++) {
    int led = random(0, ledCount);
    digitalWrite(leds[led], HIGH);
    delay(40);
    digitalWrite(leds[led], LOW);
    delay(120);
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
  summonGlow();      // magic gathers
  delay(150);

  sacredCircle();    // ritual pattern
  delay(100);

  orbitSpell();      // rotating energy
  delay(100);

  unstableEnergy();  // spell becomes unstable
  delay(80);

  magicalWave();     // force pushes outward
  delay(100);

  burstFlash();      // cast!
  delay(120);

  fallingStars();    // spark fragments
  delay(150);

  fadeToWhispers();  // quiet ending
  delay(100);
}