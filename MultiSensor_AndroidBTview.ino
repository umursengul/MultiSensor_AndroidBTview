/********************************************************/
/*              Multisensor Remote Sensing              */
/********************************************************/

//////////////////
//  LIBRARIES   //
//////////////////
#include <dht11.h>
#include <SFE_BMP180.h>
#include <Wire.h>


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
long timer;
float radius = 1.75;
int circumference;
int reedCounter = 0;
unsigned long start = 0;
unsigned long finished = 0;
unsigned long elapsed = 0;
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
  circumference = 11;
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
  Serial.print((float)DHT11.temperature, 2);
  Serial.print(",");                          // CSV

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
  Serial.print((float)DHT11.humidity, 2);
  Serial.print(",");                          // CSV

  // Send dew point:
  //Serial.print("Cig Olusma Noktasi: ");       // DEBUGGING
  //Serial.println(DHT11.dewPoint(), 2);        // Send dew point
  //Serial.print(",");

  ///////////////////
  //  WIND SENSOR  //
  ///////////////////
  start = millis();
  for(int counter = 0; counter <= 1000; counter++)
  {
    if (REED == HIGH)
    {
      reedCounter = reedCounter + 1;
    }
  }
  finished = millis();
  
  if(reedCounter =! 0)
  {
    elapsed = finished - start;
    windSpeed = ((elapsed/reedCounter)/3600000)*circumference/1000;
    Serial.print(windSpeed);
    Serial.print(",");
    reedCounter = 0;
  }
  else if(reedCounter == 0)
  {
    windSpeed = 0;
    Serial.print(windSpeed);
    Serial.print(",");
    reedCounter = 0;
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
      // DEBUGGING
      // Print out temperature measurement from BMP180:
      /*
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      */
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
          /*
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");
          */

          // Relative sea-level pressure:
          p0 = pressure.sealevel(P,ALTITUDE);
          //Serial.print("relative (sea-level) pressure: ");  // DEBUGGING
          Serial.print(p0,2);                               // Pressure in mb
          Serial.print(",");
          //Serial.print(" mb, ");                            // DEBUGGING
          //Serial.print(p0*0.0295333727,2);                  // DEBUGGING
          //Serial.println(" inHg");                          // DEBUGGING

          // DEBUGGING
          // Altitude from pressure:
          /*
          a = pressure.altitude(P,p0);
          Serial.print("computed altitude: ");
          Serial.print(a,0);
          Serial.print(" meters, ");
          */
        }
      }
    }
  }
  
  ///////////////////
  //  RAIN SENSOR  //
  ///////////////////
  if(digitalRead(rainSensor == HIGH))
  {
    Serial.print(400);
    Serial.print(",");
  }
  else if(digitalRead(rainSensor == LOW))
  {
    Serial.print(500);
    Serial.print(",");
  }

  ////////////////////
  //  LIGHT SENSOR  //
  ////////////////////
  if(analogRead(lightSensor) >= 850)
  {
    Serial.print(700);
    Serial.println(",");
  }
  else if(analogRead(lightSensor) < 850)
  {
    Serial.print(600);
    Serial.println(",");
  }
  delay(1000);
}
