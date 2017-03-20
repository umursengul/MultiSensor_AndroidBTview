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
const int analogInPin = A5;
const int analogOutPin = 9;
#define DHT11PIN 2

int sensorValue = 0;
int outputValue = 0;
int outputValue2 = 0;
int outputValue3 = 0;
int outputValue4 = 0;

dht11 DHT11;

void setup()
{
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
  Serial.println(DHT11.dewPoint(), 2);        // Send dew point
  Serial.print(",");

  /*
  sensorValue = analogRead(analogInPin);
  outputValue = map(sensorValue, 0, 1023, 0, 100);
  outputValue2 = map(sensorValue, 0, 1023, 200, 300);
  outputValue3 = map(sensorValue, 0, 1023, 400, 500);
  outputValue4 = map(sensorValue, 0, 1023, 600, 700);
  //analogWrite(analogOutPin, outputValue);
  //Serial.print(0);
  Serial.print(outputValue);
  Serial.print(",");
  Serial.print(outputValue2);
  Serial.print(",");
  Serial.print(outputValue3);
  Serial.print(",");
  Serial.print(outputValue4);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print("LIGHT");
  Serial.println(",");
  //Serial.println(';');
  delay(1000);
  */
}
