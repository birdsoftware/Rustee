//Using an Argon or Boron, the DHT11 is connected to D2. 
#include "Particle.h"
#include <DHT.h>

SYSTEM_MODE(AUTOMATIC);

#define DHTPIN   D2
#define DHTTYPE  DHT11

DHT dht(DHTPIN, DHTTYPE);

double tempC = NAN;
double tempF = NAN;
double humidity = NAN;

unsigned long lastReadMs = 0;
unsigned long lastPublishMs = 0;

const unsigned long READ_INTERVAL_MS = 3000;
const unsigned long PUBLISH_INTERVAL_MS = 60000;

void setup() {
    Serial.begin(9600);
    waitFor(Serial.isConnected, 5000);

    Particle.variable("tempC", tempC);
    Particle.variable("tempF", tempF);
    Particle.variable("humidity", humidity);

    dht.begin();
    delay(2000);

    Serial.println("DHT CLOUD TEST START");
}

void loop() {
    unsigned long now = millis();

    if (now - lastReadMs >= READ_INTERVAL_MS) {
        lastReadMs = now;

        float h = dht.readHumidity();
        float c = dht.readTemperature();

        if (!isnan(c) && !isnan(h) && h >= 0 && h <= 100) {
            tempC = c;
            tempF = c * 9.0 / 5.0 + 32.0;
            humidity = h;

            Serial.printlnf("GOOD: T=%.2fC F=%.2f H=%.2f%%", tempC, tempF, humidity);
        } else {
            Serial.println("DHT read failed, keeping last good values");
        }
    }

    if (Particle.connected() && !isnan(tempC) && !isnan(tempF) && !isnan(humidity) &&
        (now - lastPublishMs >= PUBLISH_INTERVAL_MS)) {
        lastPublishMs = now;

        char data[96];
        snprintf(data, sizeof(data),
                 "{\"tempC\":%.2f,\"tempF\":%.2f,\"humidity\":%.2f}",
                 tempC, tempF, humidity);
        char dataf[96];
        snprintf(dataf, sizeof(dataf),
                 "{\"tempF\":%.2f}",
                  tempF);

        bool ok = Particle.publish("dht_reading", dataf, PRIVATE);
        Serial.printlnf("Publish %s: %s", ok ? "OK" : "FAILED", dataf);
    }
}