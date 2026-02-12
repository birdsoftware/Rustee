// EN pin to 9

const int enPin = 9;

void setup() {
  pinMode(enPin, OUTPUT);
}

void loop() {
  digitalWrite(enPin, LOW);   // OFF (change to HIGH if your EN is inverted)
  delay(2000);
  digitalWrite(enPin, HIGH);  // ON
  delay(2000);
}
