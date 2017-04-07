/********************************************************/
/*              Multisensor Remote Sensing              */
/********************************************************/

//////////////////
//  LIBRARIES   //
//////////////////
#include <dht11.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <stdlib.h>

////////////////////
//  SENSOR PINS   //
////////////////////
#define DHT11PIN 4
#define REED A4
int rainSensor = 3;
int lightSensor = A5;
#define P_INT 2

////////////////////
//  DEFINED VARs  //
////////////////////
float windSpeed = 0;
#define ANEMOMETER_DATA_COUNT 10
unsigned long anemometerData[ANEMOMETER_DATA_COUNT] = { 0 };
void onInterrupt();
static char outstr[15];
String tempStr;
String temperatureSensorValue;
String pressureSensorValue;
String humiditySensorValue;
String lightSensorValue;
String windSensorValue;
String rainSensorValue;
String sensorValues;
dht11 DHT11;
SFE_BMP180 pressure;
#define ALTITUDE 938.0  // Average altitude of Ankara in meters


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
  //pressure.begin();

  ////////////////////////
  //  WIND SENSOR INIT  //
  ////////////////////////
  // r_sensor ~= 1.75cm
  // circumference ~= 11cm
  attachInterrupt(digitalPinToInterrupt(P_INT), onInterrupt, RISING);

  ////////////////////
  //  SERIAL COMM   //
  ////////////////////
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
  float x = DHT11.temperature;
  dtostrf(x,4,2,outstr);
  tempStr = outstr;
  temperatureSensorValue = tempStr + ",";

  // Temperature in Fahrenheit:
  /*
  Serial.print("Sicaklik (Fahrenheit): ");    // DEBUGGING
  Serial.print(DHT11.fahrenheit(), 2);        // DEBUGGING
  Serial.print(",");                          // DEBUGGING
  */

  // Temperature in Kelvin:
  /*
  Serial.print("Sicaklik (Kelvin): ");        // DEBUGGING
  Serial.print(DHT11.kelvin(), 2);            // DEBUGGING
  Serial.print(",");                          // DEBUGGING
  */

  // Send humidity:
  //Serial.print("Nem (%): ");                  // DEBUGGING
  float y = DHT11.humidity;
  dtostrf(y,4,2,outstr);
  tempStr = outstr;
  humiditySensorValue = tempStr + ",";

  // Send dew point:
  //Serial.print("Cig Olusma Noktasi: ");       // DEBUGGING
  //Serial.println(DHT11.dewPoint(), 2);        // Send dew point
  //Serial.print(",");

  //////////////////
  //  WIND SENSOR //
  //////////////////
  if (anemometerData[0] != 0)
  {
    unsigned long now = millis();
    if (now - anemometerData[ANEMOMETER_DATA_COUNT - 1] < 1000)
    {
      unsigned long passedTimes = now - anemometerData[0];
      windSpeed = ((passedTimes/1000)/3600)*((110/100)/1000);
      dtostrf(windSpeed,4,2,outstr);
      tempStr = outstr;
      pressureSensorValue = tempStr + ",";
      //Serial.println(x);
    }
    else
    {
      windSensorValue = "0,";
    }
  }
  //Serial.println(windSensorValue);      // DEBUGGING

  //////////////////////
  //  PRESSURE SENSOR //
  //////////////////////
  /*
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
      // DEBUGGING
      // Print out temperature measurement from BMP180:
      //Serial.print("temperature: ");
      //Serial.print(T,2);
      //Serial.print(" deg C, ");
      //Serial.print((9.0/5.0)*T+32.0,2);
      //Serial.println(" deg F");

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
          // DEBUGGING
          // Print out pressure measurement from BMP180:
          //Serial.print("absolute pressure: ");
          //Serial.print(P,2);
          //Serial.print(" mb, ");
          //Serial.print(P*0.0295333727,2);
          //Serial.println(" inHg");


          // Relative sea-level pressure:
          p0 = pressure.sealevel(P,ALTITUDE);
          //Serial.print("relative (sea-level) pressure: ");  // DEBUGGING
          dtostrf(p0,6,2,outstr);                           // Pressure in mb
          //float x = p0,2;
          String strOne = outstr;
          String strTwo = ",";
          String pressureSensorValue = strOne + strTwo;

          //Serial.print(" mb, ");                            // DEBUGGING
          //Serial.print(p0*0.0295333727,2);                  // DEBUGGING
          //Serial.println(" inHg");                          // DEBUGGING

          // DEBUGGING
          // Altitude from pressure:
          //a = pressure.altitude(P,p0);
          //Serial.print("computed altitude: ");
          //Serial.print(a,0);
          //Serial.print(" meters, ");

        }
      }
    }
  }*/
  float pressure_tester = 1.00;
  dtostrf(pressure_tester,4,2,outstr);
  tempStr = outstr;
  pressureSensorValue = tempStr + ",";
  ///////////////////
  //  RAIN SENSOR  //
  ///////////////////
  if(digitalRead(rainSensor == HIGH))
  {
    rainSensorValue = "400,";
    //Serial.print(rainSensorValue);
    //Serial.print(400);
    //Serial.print(",");
  }
  else if(digitalRead(rainSensor == LOW))
  {
    rainSensorValue = "500,";
    //Serial.print(500);
    //Serial.print(",");
  }

  ////////////////////
  //  LIGHT SENSOR  //
  ////////////////////
  if(analogRead(lightSensor) >= 850)
  {
    lightSensorValue = "700,";
    //Serial.print(700);
    //Serial.println(",");
  }
  else if(analogRead(lightSensor) < 850)
  {
    lightSensorValue = "600,";
    //Serial.print(600);
    //Serial.println(",");
  }
  sensorValues = temperatureSensorValue + humiditySensorValue + pressureSensorValue + windSensorValue + rainSensorValue + lightSensorValue + ";";
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
