
const int analogInPin = A5;  
const int analogOutPin = 9; 
int sensorValue = 0;        
int outputValue = 0;
int outputValue2 = 0;        
int outputValue3 = 0;
int outputValue4 = 0;
void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
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
}

/*
#include <SoftwareSerial.h>

#define RxPin 8 //goes to TX pin on BT module
#define TxPin 7 //goes to RX pin on BT module
#define analogInPin A5
#define analogOutPin 9


//const int analogInPin = A5;
//const int analogOutPin = 9;

int sensorValue = 0;
int outputValue = 0;


SoftwareSerial blueToothSerial(RxPin, TxPin); // RX, TX

void setup()
{
  Serial.begin(57600);
  blueToothSerial.begin(9600);
  pinMode(RxPin, INPUT);
  pinMode(TxPin, OUTPUT);
}

void loop()
{
  while(blueToothSerial.available() > 0)
  {
    sensorValue = analogRead(analogInPin);
    outputValue = map(sensorValue, 0, 1023, 0, 255);
    analogWrite(analogOutPin, outputValue);
    //blueToothSerial.print(" " );
    Serial.println(sensorValue);
    blueToothSerial.println(sensorValue);
    delay(500);
  }
}
*/
