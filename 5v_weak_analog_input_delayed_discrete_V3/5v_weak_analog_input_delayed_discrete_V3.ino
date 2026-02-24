// Weak 5v analog input. Provide 3.3v output 0.5 Amp.
// If input is rising wait 500 msec before letting output rise
// If input drops then output drops immediately.
// Input of “0–5V in → 0–3.5V out” requirement maps to:
// PWM duty capped at 70% max (because 3.5/5 = 0.7)

// power
// using 12-24V To 5V 3A Step Down Voltage Regulator Module 5-Pack DC-DC Buck Converter

// output
// RC filter on D9, you’ll get a smooth analog voltage from ~0 to ~3.3V.
// D9 → 4.7k → SignalOut
// SignalOut → 10µF → GND

// input
// Correct way to handle a 5V input on a 3.3V Nano Use a voltage divider.
// Mini max is 3.3V so reduced 5V signal down using 5v x ( 20kΩ / 10kΩ + 20kΩ) ~3.3V
// A0 ── 0.1µF ── GND 
// Input signal (0-5v)



const int analogInPin = A0;
const int pwmOutPin   = 9;

const unsigned long riseDelayMs = 500;

// delayed-rise / instant-fall state (based on ADC reading)
int lastADC = 0;
int targetADC = 0;
unsigned long riseStartTime = 0;
bool rising = false;

// Nano ADC reference voltage
const float VREF_ADC = 5.0;

float adcToVin(int adc) {
  // This is Vin at A0 pin. If you have a divider from a 0–5V signal, adjust this.
  return (adc * VREF_ADC) / 1023.0;
}

float mapVinToOutput(float vin) {

  if (vin > 2.0) return (vin + 0.5);
  if (vin > 1.5) return 2.5;
  if (vin > 1.0) return 2.0;
  if (vin <= 0.8) return (1.0 - 0.3);

}

int readA0HiZAvg(int samples = 16) {
  analogRead(analogInPin);          // throw-away
  delayMicroseconds(2000);          // settle for high-Z sources

  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(analogInPin);
    delayMicroseconds(300);
  }
  return (int)(sum / samples);
}

void setup() {
  analogReference(DEFAULT);
  Serial.begin(9600);
  pinMode(pwmOutPin, OUTPUT);
  analogWrite(pwmOutPin, 0);
}

void loop() {
  int inputADC = readA0HiZAvg(16);//analogRead(analogInPin);
  unsigned long now = millis();
  // Compute Vin seen by ADC
  float vin = adcToVin(inputADC);

  float discreteVin = mapVinToOutput(vin);

  int pwm = (int)(discreteVin * 255.0 / 5.0 + 0.5);
  if (pwm < 0) pwm = 0;
  //if (pwm > 255) pwm = 255;
  if (pwm > 200) pwm = 200;

  analogWrite(pwmOutPin, pwm);

  Serial.print("A0_ADC:");
  Serial.print(inputADC);
  Serial.print(" | vin: ");
  Serial.print(vin);
  Serial.print(" | discreteVin: ");
  Serial.print(discreteVin);
  Serial.print(" | PWM: ");
  Serial.println(pwm);

  delay(5);
}
