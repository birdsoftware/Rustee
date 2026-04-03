#define RT0 10000.0   // 10K thermistor nominal resistance
#define B   3977.0    // Beta value
#define VCC 5.0
#define R   10000.0   // Fixed 10K resistor

float T0 = 25.0 + 273.15;  // 25C in Kelvin

float readTempC(int pin) {
  int raw = analogRead(pin);

  // Detect obvious disconnect/open/short
  if (raw <= 2 || raw >= 1021) {
    return 999.0;  // invalid
  }

  float vrt = (raw / 1023.0) * VCC;
  float vr  = VCC - vrt;
  float rt  = vrt / (vr / R);
  float lnR = log(rt / RT0);
  float tempK = 1.0 / ((lnR / B) + (1.0 / T0));
  float tempC = tempK - 273.15;

  return tempC;
}

void printSensor(const char* name, int pin) {
  float t = readTempC(pin);

  Serial.print(name);
  Serial.print(": ");

  if (t == 999.0) {
    Serial.println("INVALID / DISCONNECTED");
  } else {
    Serial.print(t, 2);
    Serial.print(" C   ");
    Serial.print(t * 1.8 + 32.0, 2);
    Serial.println(" F");
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("3-Sensor Thermistor Tester");
  Serial.println("--------------------------");
}

void loop() {
  printSensor("A1", A1);
  printSensor("A2", A2);
  printSensor("A3", A3);
  Serial.println();

  delay(1000);
}