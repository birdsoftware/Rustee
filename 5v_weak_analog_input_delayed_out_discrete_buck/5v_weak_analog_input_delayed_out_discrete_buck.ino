// Weak 5v analog input. Provide 3.3v output 0.5 Amp.
// If input is rising wait 2 sec before letting output rise
// If input drops then output drops immediately.
// Input of “0–5V in → 0–3.5V out” requirement maps to:
// PWM duty capped at 70% max (because 3.5/5 = 0.7)

// using 12-24V To 5V 3A Step Down Voltage Regulator Module 5-Pack DC-DC Buck Converter

// pot 3 pins:
// Left outer pin → GND
// Right outer pin → +5V
// Middle pin (wiper) → A0 

//Example in/out put
//Vin	Vout
//5.0V	3.5V
//3.0V	2.1V
//2.0V	1.4V
//1.0V	1.0V (forced)
//0.5V	1.0V (forced)
//0V	0V

const int analogInPin = A0;
const int enPin = 9;                 // connect D9 -> EN (or use D8, any digital pin)

const unsigned long riseDelayMs = 2000;

// Choose a threshold that means “signal present”
// Example: 0.5V ≈ 102, 1.0V ≈ 205
const int onThresholdADC  = 205;      // ~1.0V
const int offThresholdADC = 180;      // hysteresis (slightly lower)

bool outputOn = false;
bool waiting = false;
unsigned long tStart = 0;

void setup() {
  Serial.begin(9600);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);          // OFF by default
}

void loop() {
  int adc = analogRead(analogInPin);
  unsigned long now = millis();

  if (!outputOn) {
    if (!waiting && adc >= onThresholdADC) {
      waiting = true;
      tStart = now;
    }

    if (waiting && (now - tStart >= riseDelayMs)) {
      if (analogRead(analogInPin) >= onThresholdADC) {
        digitalWrite(enPin, HIGH);   // buck ON -> 3.5V output
        outputOn = true;
      }
      waiting = false;
    }

    if (adc < onThresholdADC) {
      waiting = false;
    }
  } else {
    if (adc <= offThresholdADC) {
      digitalWrite(enPin, LOW);      // buck OFF immediately -> 0V output
      outputOn = false;
      waiting = false;
    }
  }

  Serial.print("ADC: ");
  Serial.print(adc);
  Serial.print(" | EN: ");
  Serial.println(outputOn ? "ON" : "OFF");

  delay(5);
}

