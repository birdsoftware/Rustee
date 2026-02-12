// If either Thermister reads > 100 oF turn on Relay. If both < 80oF turn off relay.
// Added A0D4184A High power drive module for 12v out instead of 5v for the Relay.
// Relay is 12V active high

//-- WIRING --
// Arduino D8  -> Signal + PWM  A0D4184A
// Arduino GND -> GND A0D4184A

// Power in out of A0D4184A
// Supply 12V to DC+
// Supply GND to DC-
// Out+ to load realy +V
// Out- to load relay GND

// 1N4004 DIODE
// relay + side (the side going to OUT+ / +12V) A0D4184A OUT + -> Stripe anode side of 1N4004 DIODE
// relay − side (the side going to OUT−) A0D4184A OUT - -> Non-stripe end 1N4004 DIODE

// temp sensors
// Arduino AO -> Node 1 
// temp sensor (+/-) either one -> Node 1
// other end of temp sensor -> common GND
// Arduino A1 -> Node 2
// 10K ohm Restr -> Node 1
// other end of 10K ohm Restr -> 5V
// temp sensor (+/-) either one -> Node 2
// other end of temp sensor -> common GND
// 10K ohm Restr -> Node 2
// other end of 10K ohm Restr -> 5V


#define RT0 10000
#define B 3977
#define VCC 5
#define R 10000

//#define PWM_PIN 9
#define RELAY_PIN 8   // NEW

// Variables
float RT, VR, ln, TX, T0, VRT;
//float Vout;
//int pwmValue;
float onTempF  = 100.0;
float offTempF = 80.0;

// Read thermistor temperature in Celsius
float readThermistor(int pin) {
  VRT = analogRead(pin);
  VRT = (5.00 / 1023.00) * VRT;
  VR  = VCC - VRT;
  RT  = VRT / (VR / R);
  ln  = log(RT / RT0);
  TX  = (1 / ((ln / B) + (1 / T0)));
  return TX - 273.15;
}

void setup() {
  Serial.begin(9600);
  //pinMode(PWM_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  T0 = 25 + 273.15;  // Reference temperature in Kelvin
}

bool relayOn = false;

void loop() {

  // Read both thermistors
  float tempC1 = readThermistor(A0);
  float tempC2 = readThermistor(A1);

  float tempF1 = tempC1 * 1.8 + 32;
  float tempF2 = tempC2 * 1.8 + 32;

  // ----- RELAY LOGIC -----
  if (tempF1 > 100 || tempF2 > 100) {
    relayOn = true;
  }
  else if (tempF1 < 80 && tempF2 < 80) {
    relayOn = false;
  }

  digitalWrite(RELAY_PIN, relayOn ? HIGH : LOW);

  // ----- DEBUG OUTPUT -----
  Serial.print("T1: ");
  Serial.print(tempF1);
  Serial.print("F | T2: ");
  Serial.print(tempF2);
  Serial.print("F | Relay: ");
  Serial.println(relayOn ? "ON" : "OFF");
  //Serial.print(" | PWM: ");
  //Serial.println(pwmValue);

  delay(500);
}
