//Picks the coldest temperature reading from sensor A0-A6
// Output voltage 0-5v for 15-5 oC. As temperature drops from 15C to 5C, voltage increases from 0v to 5v.
// Output 5v if sensors disconnected. If all sensors are invalid, outputs steady 5V (PWM 255).
// Example: 15,14,13,...,5 = 0,0.5,1,...,5v
// wiring:
// A0 -> Node
// Node -> NTC 10K
// Node -> 10K ohm resistor
// NTC 10K other end -> GND
// 10k ohm R other end -> 5v

#define RT0 10000  //
#define B 3977     //  K
#define VCC 5    //Supply  voltage
#define R 10000  //R=10K
#define PWM_PIN 9 //PWM output pin

//Variables
float RT, VR, ln, TX, T0, VRT;
float Vout;
int pwmValue;

// Adjustable mapping parameters
float T_low  = 5.0;   // °C → 5V
float T_high = 15.0;  // °C → 0V
float V_high = 5.0;
float V_low  = 0.0;

// ADC thresholds to detect open/short/disconnect
const int ADC_MIN_VALID = 2;     // anything <=2 treated as invalid
const int ADC_MAX_VALID = 1021;  // anything >=1021 treated as invalid

float mapTemperatureToVoltage(float T) {
  float V = V_high + (T - T_low) * (V_low - V_high) / (T_high - T_low);

  // Clamp output
  if (V > V_high) V = V_high;
  if (V < V_low)  V = V_low;

  return V;
}

void setup() {
  Serial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);
  T0 = 25 + 273.15;  //Temperature  T0 from datasheet, conversion from Celsius to kelvin
}

void loop() {

  // Find lowest temperature sensor among A0-A5
  int bestPin = A0;
  float bestTemp = 9999.0;
  bool anyValid = false;

  for (int pin = A0; pin <= A5; pin++) {
    //Temperature (same math, just applied to each pin)
    VRT = analogRead(pin);  

    // Detect if disconnected / invalid extremes
    // if not disconnected perform the temp calc
    if (VRT > 2 && VRT < 1021) {
      VRT = (5.00 / 1023.00) * VRT;  
      VR = VCC - VRT;
      RT = VRT / (VR / R);  
      ln = log(RT / RT0);
      TX = (1 / ((ln / B) + (1 / T0)));  
      TX = TX - 273.15;
    }

    if (TX < bestTemp) {
      bestTemp = TX;
      bestPin = pin;
      anyValid = true;
    }
  }

// Use the lowest temperature reading
  TX = bestTemp;

// All sensors disconnected/invalid -> force steady 5V out
if(!anyValid){
  Vout = 5.0f;
  pwmValue = 255;
  analogWrite(PWM_PIN, pwmValue);

  Serial.println("\nALL SENSORS INVALID -> FORCING 5.0V (PWM 255)\n");
    delay(500);
    return;
}

  Vout = mapTemperatureToVoltage(TX);
  // Convert to PWM
  pwmValue = (int)((Vout / 5.0) * 255.0);
  analogWrite(PWM_PIN, pwmValue);

  //debug output
  Serial.print("\ Temperature:");
  Serial.print("\ ");
  Serial.print(TX);
  Serial.print("C\ ");
  Serial.print((TX * 1.8) + 32);  //Conversion to Fahrenheit
  Serial.print("F\ ");

  Serial.print(Vout);
  Serial.print(" V | PWM: ");
  Serial.println(pwmValue);

  delay(500);
}
