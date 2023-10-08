#include <Arduino.h>
#include <DHT.h>
#include <ESP32Servo.h>

//Defining DHT11 sensor.
#define DHTPIN 17
#define DHTTYPE DHT11

//Defining all the parameters in the circuit.
#define LED_PIN_GREEN 19
#define LED_PIN_YELLOW 18
#define LED_PIN_RED 5
#define WATER_SENSOR 33
#define POWER_PIN 16
#define SMOKE 1500
#define HEAT 60
const int buzzer = 23;
int value = 0;

DHT dht(DHTPIN, DHTTYPE);

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

//Main part of the circuit that returns if there is a fire.
bool loop2()
{
    int sensorValue = analogRead(32);
    digitalWrite(POWER_PIN, HIGH);
    //Serial.println(analogRead(32));

    delay(500);

    value = analogRead(WATER_SENSOR);
    digitalWrite(POWER_PIN, LOW);
    //Serial.println(value);

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float f = dht.readTemperature(true);
    int analogValue = analogRead(DHTPIN);

    float hif = dht.computeHeatIndex(f, h);
    float hic = dht.computeHeatIndex(t, h, false);

    //Program can be developed to send data about rain at its location with information from water sensor.
    if (value >= 1500)
    {
        // Serial.println(F("Rainning"));
    }
    else
    {
        // Serial.println(F("Not rainning"));
    }

    //Program collects data from smoke (potentiometer) and heat sensor and turns on certian LED diode according to the probability of fire.
    //This exact code is written for DHT11 sensor but you can also use other.
    if (sensorValue >= SMOKE)
    {

        if (t >= HEAT)
        {

            ledcWriteTone(ledChannel, 261.626);

            digitalWrite(LED_PIN_GREEN, LOW);
            digitalWrite(LED_PIN_RED, HIGH);
            digitalWrite(LED_PIN_YELLOW, LOW);

            delay(1000);

            ledcWriteTone(ledChannel, 0);

            delay(1000);
            return 1;
        }
        else
        {
            digitalWrite(LED_PIN_YELLOW, HIGH);
            digitalWrite(LED_PIN_RED, LOW);
            digitalWrite(LED_PIN_GREEN, LOW);
            return 0;
        }
    }
    else
    {
        
        digitalWrite(LED_PIN_GREEN, HIGH);
        digitalWrite(LED_PIN_RED, LOW);
        digitalWrite(LED_PIN_YELLOW, LOW);
        return 0;

        
    }

    if (isnan(h) || isnan(t) || isnan(f))
    {
        //Serial.println(F("Failed to read from DHT sensor!"));
        return 0;
    }

    
    //If necessary, program can print data from DHT11 about humidity and temperature.
    // Serial.println(F("Humidity: "));
    // Serial.print(h);
    // Serial.println(F("% "));
    // Serial.println(F("Temperature: "));
    //Serial.print(t);
    //Serial.println(F("°C "));
    return 0;
}
