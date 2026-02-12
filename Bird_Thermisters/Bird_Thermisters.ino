#define RT0 10000  //
#define B 3977     //  K
//--------------------------------------
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
  //Temperature
  VRT = analogRead(A0);          //Acquisition analog value of VRT
  VRT = (5.00 / 1023.00) * VRT;  //Conversion to voltage
  VR = VCC - VRT;
  RT = VRT / (VR / R);  //Resistance of RT
  ln = log(RT / RT0);
  TX = (1 / ((ln / B) + (1 / T0)));  //Temperature from thermistor
  TX = TX - 273.15;  
  
  //Linear voltage mapping
  //Vout = 10.0 - 0.5 * TX;

  // Clamp output
  //if (Vout > 5.0) Vout = 5.0;
  //if (Vout < 0.0) Vout = 0.0;
  Vout = mapTemperatureToVoltage(TX);


  // Convert to PWM
  pwmValue = (int)((Vout / 5.0) * 255.0);
  analogWrite(PWM_PIN, pwmValue);

  //Conversion to Celsius
  Serial.print("Temperature:");
  Serial.print("\ ");
  Serial.print(TX);
  Serial.print("C\  \ ");
  Serial.print(TX + 273.15);  //Conversion to Kelvin
  Serial.print("K\  \ ");
  Serial.print((TX * 1.8) + 32);  //Conversion to Fahrenheit
  Serial.println("F");

  Serial.print(Vout);
  Serial.print(" V | PWM: ");
  Serial.println(pwmValue);
  
  delay(500);
}