const int analogInPin = A0;
const int pwmOutPin   = 9;

const unsigned long riseDelayMs = 2000;

// Delayed-rise / instant-fall state
int lastOutput = 0;
int targetOutput = 0;
unsigned long riseStartTime = 0;
bool rising = false;

const float scaleFactor = 0.7; // 5V in -> 3.5V out

// ADC thresholds
const int ADC_1V   = 205;  // ~1.0V
const int ADC_0_5V = 102;  // ~0.5V
const int ADC_TOL  = 15;   // tolerance window

void setup() {
  Serial.begin(9600);
  pinMode(pwmOutPin, OUTPUT);
}

void loop() {
  int input = analogRead(analogInPin);  // 0–1023
  unsigned long now = millis();

  if (input > lastOutput) {
    // Voltage is rising
    if (!rising) {
      rising = true;
      riseStartTime = now;
      targetOutput = input;
    }

    // After delay, allow rise
    if (now - riseStartTime >= riseDelayMs) {
      lastOutput = targetOutput;
      rising = false;
    }

  } else {
    // Voltage is falling or stable → follow immediately
    lastOutput = input;
    rising = false;
  }

 // Convert ADC (0–1023) to PWM (0–255), then scale to 70%
  int pwm = (int)(lastOutput * 255.0 / 1023.0 * scaleFactor + 0.5);
  pwm = constrain(pwm, 0, 255);
  //int pwm = map(lastOutput, 0, 1023, 0, 255);//lastOutput (expected range 0–1023) and scales it proportionally to a new range of 0–255.
  analogWrite(pwmOutPin, pwm);

  Serial.print("ADC: ");
  Serial.print(lastOutput);
  Serial.print(" | PWM: ");
  Serial.println(pwm);

  delay(5); // small loop stability delay
}
