/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */

#include <Sensirion.h>
#include "RTClib.h"
#include "Wire.h"
#include "SD.h"

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
const int chipSelect = 10; //for the SD card

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

char filename[] = "FURNACE00.CSV";

// the logging file
File logfile;

Sensirion tempSensor = Sensirion(dataPin, clockPin); 

RTC_DS1307 RTC; 

int updateTemp(boolean force = true)
{
  static unsigned long lastRead = 0;
  uint32_t m = millis();
  
    if (force == true || (m - lastRead) > READ_INTERVAL)
    { 
      digitalWrite(greenLEDpin, HIGH);
      tempSensor.measure(&temperature, &humidity, &dewpoint);
      digitalWrite(greenLEDpin, LOW);
      lastRead = m;
    }
    
    boolean changed =  abs(lastRecTemp - temperature) > 0.1;

    return changed;
}

 
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
  
  if (changed == true){ updateTemp(true); }
     
  //digitalWrite(greenLEDpin, g);
  return changed; 
}  
  
 
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  
  while(1);
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
  
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  //char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);

  logfile.println("millis,time,fan,Y1,Y2,W1,W1,temperature,humidity");    
#if ECHO_TO_SERIAL
  Serial.println("millis,time,fan,Y1,Y2,W1,W1,temperature,humidity");
#endif //ECHO_TO_SERIAL

  logfile.close();  
  
    Wire.begin();  
    RTC.begin();
  
  
  digitalWrite(greenLEDpin, HIGH);
  tempSensor.measure(&temperature, &humidity, &dewpoint);
  digitalWrite(greenLEDpin, LOW);
}

// the loop routine runs over and over again forever:
void loop() {
  
  DateTime now;
  
  if (updateStates()== true || updateTemp() == true )
  {
  digitalWrite(redLEDpin, HIGH);
  logfile = SD.open(filename, FILE_WRITE); 
  now = RTC.now();
  uint32_t m = millis();
  
  logfile.print(m,DEC);
  logfile.print(", ");   
  logfile.print(now.year(), DEC);
  logfile.print("-");
  logfile.print(now.month(), DEC);
  logfile.print("-");
  logfile.print(now.day(), DEC);
  logfile.print("T");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(),DEC);
  logfile.print(", ");
  logfile.print(gState); 
  logfile.print(", ");
  logfile.print(y1State);
  logfile.print(", ");
  logfile.print(y2State);
  logfile.print(", ");
  logfile.print(w1State);
  logfile.print(", ");
  logfile.print(w2State);
  logfile.print(", ");
  logfile.print(temperature);
  logfile.print(", ");
  logfile.print(humidity);
  logfile.println("");
 
  logfile.close();
  digitalWrite(redLEDpin, LOW); 
  
  #if ECHO_TO_SERIAL
  Serial.print(m,DEC);
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
