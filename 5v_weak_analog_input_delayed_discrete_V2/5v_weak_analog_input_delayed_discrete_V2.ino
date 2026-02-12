// Weak 5v analog input. Provide 3.3v output 0.5 Amp.
// If input is rising wait 2 sec before letting output rise
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
// A0 ── 20kΩ ── GND 
// Input signal (0-5v) ── 10kΩ ── A0



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

// // Your desired scaling: Vout_target = 0.7 * Vin(0..5V)
// const float scaleTarget = 0.7;

// // Empirical output limits you measured at your “Vout” measurement point
// const float VOUT_MIN_MEAS = 0.00;
// const float VOUT_MAX_MEAS = 5.00;//4.6

// // Desired target max
// const float VOUT_TARGET_MAX = 3.50;

// // PWM high level reference at your measurement point.
// // Since your measured node can reach 4.66V, use 4.66V as the “100%” scale.
// const float VOUT_PWM_FULLSCALE = VOUT_MAX_MEAS;

float adcToVin(int adc) {
  // This is Vin at A0 pin. If you have a divider from a 0–5V signal, adjust this.
  return (adc * VREF_ADC) / 1023.0;
}

float mapVinToOutput(float vin) {

  // if (vin > 3.5) return 3.3;//3.3  //3.2truck
  // if (vin > 3.2) return 2.99;//2.8
  // if (vin > 3.0) return 2.66;//2.5
  // if (vin > 2.0) return 2.33;//2.2
  if (vin > 2.0) return vin;
  if (vin > 1.5) return 2.5;
  if (vin > 1.0) return 2.0;
  if (vin <= 0.8) return 1.0;
  //if (vin <= 2.0) return 0.80;

  // If vin <= 2V → pass through unchanged
  //return vin;
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

  // // delayed rise / immediate fall
  // if (inputADC > lastADC) {
  //   if (!rising) {
  //     rising = true;
  //     riseStartTime = now;
  //     targetADC = inputADC;
  //   }
  //   if (now - riseStartTime >= riseDelayMs) {
  //     lastADC = targetADC;
  //     rising = false;
  //   }
  // } else {
  //   lastADC = inputADC;
  //   rising = false;
  // }

  // // Compute Vin seen by ADC
  // float vin = adcToVin(lastADC);

  // Get true vin due to input A0 resistor vin/0.4728
  //vin = vin / 0.4728;

  // output D9 resistor adj.
  //vin = vin / 0.935;

  float discreteVin = mapVinToOutput(vin);

  // Your desired target output (ideal)
  // float vout_target = vin * scaleTarget;

  // // Clamp target to 0..3.5
  // if (vout_target < 0.0) vout_target = 0.0;
  // if (vout_target > VOUT_TARGET_MAX) vout_target = VOUT_TARGET_MAX;

  // // Map target into your measurable output range (1.4..4.66)
  // float vout_mapped = VOUT_MIN_MEAS +
  //                     (vout_target / VOUT_TARGET_MAX) * (VOUT_MAX_MEAS - VOUT_MIN_MEAS);

  // Convert mapped voltage to PWM (0..255), where 255 corresponds to ~4.66V at your measurement point
  //int pwm = (int)(255.0 * (discreteVin / VOUT_PWM_FULLSCALE) + 0.5);
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
