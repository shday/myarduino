/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */

#include <Sensirion.h>
#include "RTClib.h"
#include "Wire.h"

#define y1Pin 11
#define y2Pin 13
#define w1Pin 9
#define w2Pin 8
#define gPin  12

#define redLEDpin 2
#define greenLEDpin 3

#define READ_INTERVAL 10000
#define ECHO_TO_SERIAL 1

const uint8_t dataPin = 4;
const uint8_t clockPin = 5;

boolean y1State = false;
boolean y2State = false;
boolean w1State = false;
boolean w2State = false;
boolean gState = false;
float lastRecTemp = 0;

float temperature;
float humidity;
float dewpoint;
//char s_buffer[32];

Sensirion tempSensor = Sensirion(dataPin, clockPin); 

RTC_DS1307 RTC; 
 
int updateStates() 
{
  boolean y1 = false;
  boolean y2 = false;
  boolean w1 = false;
  boolean w2 = false;
  boolean g = false;
  
  for (int i = 0;i < 5; i++)
  {
    y1 = y1 || digitalRead(y1Pin);
    y2 = y2 || digitalRead(y2Pin);
    w1 = w1 || digitalRead(w1Pin);
    w2 = w2 || digitalRead(w2Pin);
    g = g || digitalRead(gPin);
    delay(3);
  }  
  boolean changed = !((y1==y1State) &&
                        (y2==y2State) &&
                        (w1==w1State) &&
                        (w2==w2State) &&
                        (g==gState));
                        
  y1State = y1;
  y2State = y2;
  w1State = w1;
  w2State = w2;
  gState = g;
  //digitalWrite(greenLEDpin, g);
  return changed; 
}  
  
int updateTemp()
{
  static unsigned long lastRead = 0;
  uint32_t m = millis();
  
    if ((m - lastRead) > READ_INTERVAL)
    { 
      digitalWrite(greenLEDpin, HIGH);
      tempSensor.measure(&temperature, &humidity, &dewpoint);
      digitalWrite(greenLEDpin, LOW);
      lastRead = m;
    }
    
    boolean changed =  abs(lastRecTemp - temperature) > 0.1;

    return changed;
}
  
  
  
 
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);
  pinMode(y1Pin,INPUT);
  pinMode(y2Pin,INPUT);
  pinMode(w1Pin,INPUT);
  pinMode(w2Pin,INPUT);
  pinMode(gPin,INPUT);
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  
    Wire.begin();  
    RTC.begin();
  
  
  digitalWrite(greenLEDpin, HIGH);
  tempSensor.measure(&temperature, &humidity, &dewpoint);
  digitalWrite(greenLEDpin, LOW);
}

// the loop routine runs over and over again forever:
void loop() {
  
  DateTime now;
  uint32_t m = millis();
  
  if (updateStates()== true || updateTemp() == true )
  {
  //int sensorValue = analogRead(A2);
  // print out the value you read:
   now = RTC.now();
  #if ECHO_TO_SERIAL
  Serial.print(m);
  Serial.print(", ");   
  Serial.print(now.year(), DEC);
  Serial.print("-");
  Serial.print(now.month(), DEC);
  Serial.print("-");
  Serial.print(now.day(), DEC);
  Serial.print("T");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  //Serial.print(dtostrf(now.second(), 2, 0, s_buffer));
  Serial.print(now.second(),DEC);
    Serial.print(", ");
  Serial.print(gState); 
  Serial.print(", ");
  Serial.print(y1State);
  Serial.print(", ");
    Serial.print(y2State);
  Serial.print(", ");
    Serial.print(w1State);
  Serial.print(", ");
    Serial.print(w2State);
     Serial.print(", ");
    Serial.print(temperature);
    Serial.print(", ");
    Serial.print(humidity);
  Serial.println("");
  #endif
  
  
  lastRecTemp = temperature;
  
  }
}
