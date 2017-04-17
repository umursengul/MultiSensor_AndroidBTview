/********************************************************/
/*              Multisensor Remote Sensing              */
/*  By: Umur Ozhan SENGUL                               */
/*  Under GPLv3                                         */
/********************************************************/

//////////////////
//  LIBRARIES   //
//////////////////
#include <dht11.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <stdlib.h>

////////////////////////
//  PERIPHERAL PINS   //
////////////////////////
#define DHT11PIN 4
#define P_INT 2
int rainSensorPin = A2;
int lightSensorPin = A3;

////////////////////
//  DEFINED VARs  //
////////////////////
#define ANEMOMETER_DATA_COUNT 10
#define ALTITUDE 938.0            // Average altitude of Ankara in meters
float windSpeed = 0.0000;
float temp = 0.0000;
float humid = 0.0000;
int rainSensor;
int lightSensor;
unsigned long anemometerData[ANEMOMETER_DATA_COUNT] = { 0 };
void onInterrupt();
static char outstr[15];         // Temporary String 1
dht11 DHT11;
SFE_BMP180 pressure;
String tempStr;                 // Temporary String 2
String temperatureSensorValue;
String pressureSensorValue;
String humiditySensorValue;
String dewSensorValue;
String lightSensorValue;
String windSensorValue;
String rainSensorValue;
String sensorValues;

void setup()
{
  /////////////////
  //  PIN MODES  //
  /////////////////
  pinMode(rainSensor, INPUT);
  pinMode(lightSensor, INPUT);

  //////////////////
  //  BMP180 INIT //
  //////////////////
  pressure.begin();

  ////////////////////////
  //  WIND SENSOR INIT  //
  ////////////////////////
  // r_sensor ~= 1.75cm
  // circumference ~= 11cm
  attachInterrupt(digitalPinToInterrupt(P_INT), onInterrupt, RISING);

  ////////////////////
  //  SERIAL COMM   //
  ////////////////////
  // Initialize serial communications
  Serial.begin(9600);

}

void loop()
{
  /////////////
  //  DHT11  //
  /////////////
  int chk = DHT11.read(DHT11PIN); // Sensor check - should be 0

  // Temperature in Celcius:
  temp = DHT11.temperature;
  dtostrf(temp,4,2,outstr);
  tempStr = outstr;
  temperatureSensorValue = tempStr + ",";


  // Temperature in Fahrenheit:
  /*
  Serial.print(DHT11.fahrenheit(), 2);        // DEBUGGING
  Serial.print(",");                          // DEBUGGING
  */

  // Temperature in Kelvin:
  /*
  Serial.print(DHT11.kelvin(), 2);            // DEBUGGING
  Serial.print(",");                          // DEBUGGING
  */

  // Send humidity:
  //Serial.print("Nem (%): ");                  // DEBUGGING
  humid = DHT11.humidity;
  dtostrf(humid,4,2,outstr);
  tempStr = outstr;
  humiditySensorValue = tempStr + ",";

  // Send dew point:
  /*
  float z = DHT11.dewPoint();
  dtostrf(z,4,2,outstr);
  dewStr = outstr;
  dewSensorValue = dewStr + ",";
  */

  //////////////////
  //  WIND SENSOR //
  //////////////////
  if (anemometerData[0] != 0)
  {
    unsigned long now = millis();
    if (now - anemometerData[ANEMOMETER_DATA_COUNT - 1] < 1000)
    {
      unsigned long passedTimes = now - anemometerData[0];
      windSpeed = ((10000/passedTimes)*(0.11));
      dtostrf(windSpeed,5,2,outstr);
      tempStr = outstr;
      windSensorValue = tempStr + ",";
    }
    else
    {
      windSensorValue = "0,";
    }
  }
  else
  {
    windSensorValue = "0,";
  }

  //////////////////////
  //  PRESSURE SENSOR //
  //////////////////////
  char status;
  double T,P,p0,a;
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Temperature measurement:
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Pressure measurement:
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Pressure measurement:
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Relative sea-level pressure:
          p0 = pressure.sealevel(P,ALTITUDE);
          //Serial.print("relative (sea-level) pressure: ");  // DEBUGGING
          dtostrf(p0,6,2,outstr);                           // Pressure in mb
          tempStr = outstr;
          pressureSensorValue = tempStr + ",";
        }
      }
    }
  }

  ///////////////////
  //  RAIN SENSOR  //
  ///////////////////
  rainSensor = analogRead(A2);
  int rainSensorRange = map(rainSensor, 0, 1024, 0, 3);
  switch (rainSensorRange)
  {
    case 0:
      rainSensorValue = "0,";
      break;
    case 1:
      rainSensorValue = "1,";
      break;
    case 2:
      rainSensorValue = "2,";
      break;
  }

  ////////////////////
  //  LIGHT SENSOR  //
  ////////////////////
  int letThereBeLight = analogRead(lightSensorPin);
  dtostrf(letThereBeLight,6,2,outstr);
  tempStr = outstr;
  lightSensorValue = tempStr + ",";

  ////////////////////////
  //  BT SERIAL OUTPUT  //
  ////////////////////////
  sensorValues = temperatureSensorValue + humiditySensorValue + pressureSensorValue + windSensorValue + rainSensorValue + lightSensorValue;
  Serial.println(sensorValues);
  delay(1000);
}

////////////////////////////
//  WIND SENSOR FUNCTION  //
////////////////////////////
// by MucarTheEngineer
void onInterrupt()
{
  unsigned long newData = millis();
  for (int i = 1; i < ANEMOMETER_DATA_COUNT; i++)
    anemometerData[i - 1] = anemometerData[i];
  anemometerData[ANEMOMETER_DATA_COUNT - 1] = newData;
}
