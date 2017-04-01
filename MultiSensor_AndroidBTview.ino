/********************************************************/
/*              Multisensor Remote Sensing              */
/********************************************************/

//////////////////
//  LIBRARIES   //
//////////////////
#include <dht11.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include<stdlib.h>

////////////////////
//  SENSOR PINS   //
////////////////////
#define DHT11PIN 2
#define REED A4
int rainSensor = 3;
int lightSensor = A5;

////////////////////
//  DEFINED VARs  //
////////////////////
float windSpeed = 0;
static char outstr[15];
static char outstr2[15];
static char outstr3[15];
static char outstr4[15];
static char outstr5[15];
String temperatureSensorValue;
String pressureSensorValue;
String humiditySensorValue;
String lightSensorValue;
String windSensorValue;
String rainSensorValue;
String sensorValues;
/*
long timer;
float radius = 1.75;
int circumference;
int reedCounter = 0;
unsigned long start = 0;
unsigned long finished = 0;
unsigned long elapsed = 0;
*/
int time_span=0;
int previous_time=0;
int new_time=0;
float velocity;
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

  ////////////////////
  //  WIND SENSOR   //
  ////////////////////
  // v_max = 100kph ~= 2778cm/s
  // r_sensor ~= 1.75cm
  // circumference ~= 11cm
  //circumference = 11;
  pinMode(REED, INPUT);

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
  dtostrf(x,4,2,outstr2);
  //Serial.print((float)DHT11.temperature, 2);
  //Serial.print(",");                          // CSV
  String strThree = outstr2;
  //Serial.println(strThree);
  temperatureSensorValue = strThree + ",";
  //Serial.println(temperatureSensorValue);

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
  dtostrf(y,4,2,outstr3);
  String strFour = outstr3;
  //Serial.println(strFour);
  humiditySensorValue = strFour + ",";
  //Serial.println(humiditySensorValue);
  //Serial.print((float)DHT11.humidity, 2);
  //Serial.print(",");                          // CSV

  // Send dew point:
  //Serial.print("Cig Olusma Noktasi: ");       // DEBUGGING
  //Serial.println(DHT11.dewPoint(), 2);        // Send dew point
  //Serial.print(",");

  //////////////////
  //  WIND SENSOR //
  //////////////////
  windSpeed = anemometer(REED);
  dtostrf(windSpeed,6,2,outstr4);
  String strFive = outstr4;
  windSensorValue = strFive + ",";
  //Serial.println(windSensorValue);

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
  dtostrf(pressure_tester,4,2,outstr5);
  String strSix = outstr5;
  pressureSensorValue = strSix + ",";
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

float anemometer(int anemometer_input_pin)
{
  long int b = millis();
  int a = 0;
  int d = 0;
  int count = 0;
  float speedwind = 0;
  int i = 0;
  float totalspeed = 0;
  float velocity[100];
  while(1)
  {
    d = a;
    long int c = millis();

    if((c - b) >= 10000)
    {
      //Serial.println("*******");
      //Serial.println("Total count");
      //Serial.println(count);

      for (int j = 2; j <= count; j++)
      {
        totalspeed = totalspeed + velocity[j];
      }

      speedwind = (totalspeed/(count-1));
      //Serial.println(speedwind);
      delay(5000);
      //break;
    }

    else
    {
      a = digitalRead(REED);

      if(a == 1)
      {
        if(d == 0)
        {
          new_time = millis();
          count = count + 1;
          //Serial.println(count);
          time_span = new_time - previous_time;
          previous_time = new_time;
          //Serial.println(time_span);
          velocity[count] = (16049.96/time_span);
          //Serial.println(velocity[count]);
        }

        else if (d == 1)
        {
          count = count;
        }
      }
    }
  }
  //float x = ((float)speedwind,2);
  //speedwind = x;
  return speedwind;
  Serial.println(speedwind);
}
