void setup() { pinMode(9, OUTPUT); }
void loop() {
  analogWrite(9, 0);   delay(5000);
  analogWrite(9, 64);  delay(5000);
  analogWrite(9, 128); delay(5000);
  analogWrite(9, 191); delay(5000);
  analogWrite(9, 255); delay(5000);
}
