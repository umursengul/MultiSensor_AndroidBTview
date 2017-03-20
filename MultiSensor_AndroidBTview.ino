/********************************************************/
/*              Multisensor Remote Sensing              */
/********************************************************/

//////////////////
//  LIBRARIES   //
//////////////////
#include <dht11.h>


/////////////////////
//  SENSOR PINS   //
/////////////////////
//const int analogInPin = A5;
//const int analogOutPin = 9;
#define DHT11PIN 2
int rainSensor = 3;
int lightSensor = A5;

boolean isRaining = false;
boolean isDark = true;
int windSpeed = 0;
int pressure = 0;
dht11 DHT11;

void setup()
{
  pinMode(rainSensor, INPUT);
  pinMode(lightSensor, INPUT);
  Serial.begin(9600);
}

void loop()
{
  /////////////
  //  DHT11  //
  /////////////
  int chk = DHT11.read(DHT11PIN); // Sensor check - should be 0

  // Temperature in Celcius:
  //Serial.print("Sicaklik (Celcius): ");       // DEBUGGING
  Serial.print((float)DHT11.temperature, 2);
  Serial.print(",");                          // CSV

  // Temperature in Fahrenheit:
  //Serial.print("Sicaklik (Fahrenheit): ");    // DEBUGGING
  //Serial.print(DHT11.fahrenheit(), 2);
  //Serial.print(",");

  // Temperature in Kelvin:
  //Serial.print("Sicaklik (Kelvin): ");        // DEBUGGING
  //Serial.print(DHT11.kelvin(), 2);
  //Serial.print(",");

  // Send humidity:
  //Serial.print("Nem (%): ");                  // DEBUGGING
  Serial.print((float)DHT11.humidity, 2);
  Serial.print(",");                          // CSV

  // Dew Point:
  //Serial.print("Cig Olusma Noktasi: ");       // DEBUGGING
  //Serial.println(DHT11.dewPoint(), 2);        // Send dew point
  //Serial.print(",");

  // Wind speed
  Serial.print(windSpeed);
  Serial.print(",");

  // Pressure
  Serial.print(pressure);
  Serial.print(",");
  // Rain
  if(digitalRead(rainSensor == HIGH))
  {
    isRaining = !isRaining;
    Serial.print(isRaining);
    Serial.print(",");
  }
  else if(digitalRead(rainSensor == LOW))
  {
    Serial.print(isRaining);
    Serial.print(",");
  }

  // Light
  if(analogRead(lightSensor >= 500))
  {
    isDark = !isDark;
    Serial.print(isDark);
    Serial.println(",");
  }
  else if(analogRead(lightSensor < 500))
  {
    Serial.print(isDark);
    Serial.println(",");
  }
}
