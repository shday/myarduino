/*

  The circuit:
to do

*/

// include the library code:
#include "Wire.h"
#include "LiquidCrystal.h"
#include <Sensirion.h>
#include "RTClib.h"
#include <stdio.h>
#include "Arduino.h"
#include <EEPROM.h>
#include <DB.h>
#include <Encoder.h>

#define HISTORY_INTERVAL  5000 // millis between history updates
#define READ_INTERVAL 60000 // millis between sensor reads
#define ECHO_TO_SERIAL   0 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()
#define RESET_EEPROM_DATA    0 // Wait for serial input in setup()
#define ALARM_DISABLE_TIME 3600000 //millis that a button push stops alarm

#define greenLEDpin 13
#define HUMIDITY  1
#define TEMPERATURE  0
#define MAX  0
#define MIN  1
#define TONE  100
#define DURATION  1
#define COLD_ROOM_DB 1
//#define COLD_ROOM_DB 400
#define ENC_A 14
#define ENC_B 15
#define NUM_MODES 3

const uint8_t dataPin  =  4;
const uint8_t clockPin =  3;
const uint8_t speakerPin =  6;
const uint8_t buttonPin =  2;
const uint8_t encButtonPin =  5;
//const int numRecords =  16;

float temperature;
float humidity;
float dewpoint;
char s[64];
int alarmHumidity = 85;
int alarmTemperature = 3;
int buttonState;             // the current reading from the input pin
int encButtonState;  
int mode = 0;
int buttonPushCounter = 0;
int lastButtonState = LOW;   // the previous reading from the input pin
int lastEncButtonState = LOW;
int lastMode = -1;
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers


char *headers[] = {
  "day MAX ",
  "day min ",
  "mth MAX ",
  "mth min ",
  " yr MAX ",
  " yr min ",
  "    MAX ",
  "    min "
};

byte degree[8] = {
  B01100,
  B10010,
  B10010,
  B01100,
  B00000,
  B00000,
  B00000,
};

// Connect via i2c, default address #0 (A0-A2 not jumpered)
LiquidCrystal lcd(7,8,9,10,11,12);

Sensirion tempSensor = Sensirion(dataPin, clockPin);

RTC_DS1307 RTC; // define the Real Time Clock object

DateTime now;

DB db;

Encoder myEnc(ENC_A, ENC_B);

struct climate {
  byte window;
  byte readingType;
  byte level;
  unsigned long time;
  float temperature;
  float humidity;
};

//climate dayClimate[numRecords];
climate dayClimate[4][2][2];
climate tempClimate;


void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // solid green LED indicates error
  digitalWrite(greenLEDpin, HIGH);
  
  while(1);
}

void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(tone);
  }
}


void updateRecord(byte window, byte readingType, byte level, unsigned long time, float temperature, float humidity)
{

  dayClimate[window][readingType][level].time = time;
  dayClimate[window][readingType][level].temperature = temperature;
  dayClimate[window][readingType][level].humidity = humidity;
  
}


void updateEEPROMRecord(byte window, byte readingType, byte level, unsigned long time, float temperature, float humidity)
{
  
  db.open(COLD_ROOM_DB); 
  for (int i = 1; i <= db.nRecs(); i++)
   {
    db.read(i, DB_REC tempClimate);
    if (tempClimate.window == window && tempClimate.readingType == readingType && tempClimate.level == level){
         tempClimate.time = time;
         tempClimate.temperature = temperature;
         tempClimate.humidity = humidity;
         db.write(i, DB_REC tempClimate);
         return;
    }
   } 
     error("No EEPROM record found during update\n");
}

struct climate readRecord(byte window, byte readingType, byte level)
{
  climate r;

  r.time =        dayClimate[window][readingType][level].time;
  r.temperature = dayClimate[window][readingType][level].temperature;
  r.humidity =    dayClimate[window][readingType][level].humidity;
  return r;
 
}



struct climate readEEPROMRecord(byte window, byte readingType, byte level)
{
  climate r;
  db.open(COLD_ROOM_DB); 
  for (int i = 1; i <= db.nRecs(); i++)
   {
    db.read(i, DB_REC r);
    if (r.window == window && r.readingType == readingType && r.level == level){
    return r;
    }
   } 
   error("No EEPROM record found during read\n");
  
}


void syncEEPROM()
{
climate r;
climate ram;

for (byte window = 1; window <= 3 ; window++)  
{
for (byte level = 0; level <= 1 ; level++)  
{
for (byte readingType = 0; readingType <= 1 ; readingType++)
  {
    r = readEEPROMRecord(window,readingType,level);
    ram = readRecord(window,readingType,level);  
          if (ram.time > r.time)
          {
             updateEEPROMRecord(window,readingType,level, ram.time, ram.temperature, ram.humidity); 
          }
     }
  }
}

}  

boolean needsReset(byte window, unsigned long oldTime, unsigned long newTime)
{

DateTime oldDate = DateTime(oldTime);
DateTime newDate = DateTime(newTime);
//sprintf(s, "window= %d old= %d/%02d/%02d %02d:%02d",window,oldDate.year(),oldDate.month(),oldDate.day(),oldDate.hour(),oldDate.minute()) ; 
//Serial.println(s);
//sprintf(s, "          new= %d/%02d/%02d %02d:%02d",newDate.year(),newDate.month(),newDate.day(),newDate.hour(),newDate.minute()) ; 
//Serial.println(s);


if (newDate.day() == oldDate.day() && newDate.month() == oldDate.month() && newDate.year() == oldDate.year()){
//Serial.println("Reset=false1!");
return false;}
else if (window >= 1 && newDate.month() == oldDate.month() && newDate.year() == oldDate.year()){
//Serial.println("Reset=false2!");
return false;}
else if (window >= 2 && newDate.year() == oldDate.year()){
//Serial.println("Reset=false3!");
return false;}
else if (window == 3){
//Serial.println("Reset=false4!");
return false;}
else {
//Serial.println("Reset=true!");
return true;}
  
}

void updatedb(unsigned long time, float temperature, float humidity)
{
climate r;
  
  for (byte window=0;window<=3;window++)
  {
    r = readRecord(window,TEMPERATURE,MAX);
          if (temperature > r.temperature || needsReset(window, r.time, time))
          {
             updateRecord(window,TEMPERATURE,MAX, time, temperature, humidity); 
          }
    r = readRecord(window,TEMPERATURE,MIN);
          if (temperature < r.temperature || needsReset(window, r.time, time))
          {
             updateRecord(window,TEMPERATURE,MIN, time, temperature, humidity); 
          }
    r = readRecord(window,HUMIDITY,MAX);
          if (humidity > r.humidity || needsReset(window, r.time, time))
          {
             updateRecord(window,HUMIDITY,MAX, time, temperature, humidity); 
          }
    r = readRecord(window,HUMIDITY,MIN);
          if (humidity < r.humidity || needsReset(window, r.time, time))
          {
             updateRecord(window,HUMIDITY,MIN, time, temperature, humidity); 
          }
  }
}



char *temptostr(float f){
  char *c;
 if (f >= 9.95) {
    c = dtostrf(f, 3, 0, s);
  } else {  
  c = dtostrf(f, 3, 1, s);
  }
  return c;
}

void lcdLine(int row, char text[], float t, float h) {
    lcd.setCursor(0, row);
    lcd.print(text);
  lcd.print(temptostr(t));
  lcd.write(0);
  lcd.print(dtostrf(h, 3, 0, s));
  lcd.print("%");
}

void lcdLine(int row, char text[]) {
    lcd.setCursor(0, row);
    lcd.print(text);
}

void lcdLine(int row, int n) {
    lcd.setCursor(6, row);
  lcd.print(n);
  lcd.print("   ");
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START


  Wire.begin();  
    if (!RTC.begin()) {
    error("RTC failed\n");
    }

#if RESET_EEPROM_DATA  

Serial.print("Creating Table...");
  db.create(COLD_ROOM_DB,sizeof(tempClimate));
  db.open(COLD_ROOM_DB);
  Serial.println("DONE");

tempClimate.time = 1235387554;
for (byte window = 0; window <= 3 ; window++)  
{
for (byte level = 0; level <= 1 ; level++)  
{
for (byte readingType = 0; readingType <= 1 ; readingType++)
  {
  tempClimate.window = window;  
  tempClimate.readingType = readingType;
  tempClimate.level = level;
  tempClimate.temperature = level*100.0 - 50.0;
  tempClimate.humidity = level*100.0;
  
  db.append(DB_REC tempClimate);
  
     }
  }
}

   lcd.print("EEPROM reset");
  while(1); 
  
#endif  



// int i = 0;
for (byte window = 0; window <=3 ; window++)
{
for (byte level = 0; level <= 1 ; level++)  
{
for (byte readingType = 0; readingType <= 1 ; readingType++)
  {
  tempClimate = readEEPROMRecord(window, readingType, level);    

updateRecord(window, readingType, level, tempClimate.time, tempClimate.temperature, tempClimate.humidity );   
     }
  }
}
  
#if ECHO_TO_SERIAL
  Serial.println("window, readingType,level ,time, temp, humidity");
  Serial.println("-----EEPROM");

for (byte window = 0; window <= 3 ; window++)  
{
for (byte level = 0; level <= 1 ; level++)  
{
for (byte readingType = 0; readingType <= 1 ; readingType++)
{
    
    tempClimate = readEEPROMRecord(window,readingType,level);
  
    Serial.print(tempClimate.window); Serial.print(", "); 
    Serial.print(tempClimate.readingType); Serial.print(", "); 
    Serial.print(tempClimate.level); Serial.print(", "); 
    Serial.print(tempClimate.time); Serial.print(", "); 
    Serial.print(tempClimate.temperature); Serial.print(", "); 
    Serial.println(tempClimate.humidity); 
}
}
}
  Serial.println("-----RAM");

for (byte window = 0; window <= 3 ; window++)  
{
for (byte level = 0; level <= 1 ; level++)  
{
for (byte readingType = 0; readingType <= 1 ; readingType++)
{
    tempClimate = readRecord(window,readingType,level);
  
    Serial.print(window); Serial.print(", "); 
    Serial.print(readingType); Serial.print(", "); 
    Serial.print(level); Serial.print(", "); 
    Serial.print(tempClimate.time); Serial.print(", "); 
    Serial.print(tempClimate.temperature); Serial.print(", "); 
    Serial.println(tempClimate.humidity); 
}
}
}
  Serial.println("-----");  
  Serial.println("millis,time,temp,humidity,dewpoint");
#endif //ECHO_TO_SERIAL
  
  pinMode(greenLEDpin, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(encButtonPin, INPUT);


  // set up the LCD's number of rows and columns: 
  lcd.createChar(0, degree);
  lcd.begin(16, 2);
  
  //do first read (blocking)
   now = RTC.now();
  tempSensor.measure(&temperature, &humidity, &dewpoint);
    updatedb(now.unixtime(), temperature, humidity);
    //  lcdLine(0, "Current ", temperature, humidity);
      
 // myEnc.write(132); 
  
  //lcd.setBacklight(LOW);
  // Print a message to the LCD.
  //lcd.print("hello, world!");
}


void loop()
{
  static unsigned long lastSync = 0;
  static unsigned long lastRead = 0;
  static unsigned long lastHistory = 0;
  static unsigned int lev = 0;
  static unsigned int window = 0;
  static unsigned int alarmDisabled = 0;
  static unsigned long alarmDisabledTime = 0;
   
  // log milliseconds since starting
  uint32_t m = millis();

    int rawKnobPosition = (int)myEnc.read();
    int knobPosition = rawKnobPosition/4;
  
  int reading = digitalRead(buttonPin);
  int reading2 = digitalRead(encButtonPin);
    if (reading != lastButtonState|| reading2 != lastEncButtonState ) { 
    // reset the debouncing timer
    lastDebounceTime = millis();
  } 
    if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    lastButtonState = buttonState;
    buttonState = reading;
    encButtonState = reading2;
    if (buttonState != lastButtonState && buttonState == HIGH) {
      buttonPushCounter++;
      lastMode = mode;
      mode = buttonPushCounter % NUM_MODES;
      lcd.clear();
    /*  Serial.print(buttonPushCounter);
      Serial.print(",");
      Serial.println(mode);
      Serial.print(","); 
      Serial.print(rawKnobPosition);
      Serial.print(",");   
      Serial.println(knobPosition); */
    }
  }
  lastButtonState = reading;
  lastEncButtonState = reading2;



  if (mode == 0 && (humidity > (float)alarmHumidity || temperature < (float)alarmTemperature) && m % 1000 > 500 && alarmDisabled == 0){
    tone(speakerPin,2000);
  } else {
    noTone(speakerPin);
  }    
 /* static uint8_t oldPosition  = 124;
  if (knobPosition != oldPosition) {
    oldPosition = knobPosition;
   //     tone(speakerPin,1000,100);
    Serial.print(rawKnobPosition);
     Serial.print(",");   
    Serial.println(knobPosition);
  } */
switch(mode) {

case 0:
  if (mode != lastMode) {
  lcdLine(0, "Current ", temperature, humidity);
  }
  if ((m - lastRead) > READ_INTERVAL)
    { 
      digitalWrite(greenLEDpin, HIGH);
      now = RTC.now();
      tempSensor.measure(&temperature, &humidity, &dewpoint);
      digitalWrite(greenLEDpin, LOW);
      updatedb(now.unixtime(), temperature, humidity);
      lastRead = m;
      lcdLine(0, "Current ", temperature, humidity);
 
      if (now.hour()==0 && (now.unixtime()-lastSync) > 60ul*60ul*23ul)
        { Serial.print("Syncing EEPROM\n");
          syncEEPROM();
          lastSync = now.unixtime();
        }
    }

  if (encButtonState == HIGH){
    alarmDisabledTime = m;
    alarmDisabled = 1;
  }
  
  if (m - alarmDisabledTime > ALARM_DISABLE_TIME){
    alarmDisabled = 0;
  } 
    
 //sprintf(s, "%d/%02d/%02d %02d:%02d",now.year(),now.month(),now.day(),now.hour(),now.minute()) ; 
//Serial.println(s);  
  
  if ((m - lastHistory) > HISTORY_INTERVAL || mode != lastMode) 
  {
  climate r;
  char *header;
  header = headers[window * 2 + lev];
  
  r = readRecord(window, TEMPERATURE, lev);
  float temp = r.temperature;
  r = readRecord(window, HUMIDITY, lev);
  float hum = r.humidity;

  lcdLine(1,header, temp, hum);
  lev = (lev + 1) % 2;
  if (lev == 0) {
    window = (window + 1) % 4;
    }
  lastHistory = m;
  }
  lastMode = mode;  
break;
case 1:
  if (lastMode != mode) {
    myEnc.write(alarmHumidity * 4);
    lcdLine(0,"Alarm Humidity");
  }
  if (alarmHumidity != knobPosition || lastMode != mode) {
  alarmHumidity = knobPosition;
  lcdLine(1, alarmHumidity);
  }
  lastMode = mode;
break;
case 2:
  if (lastMode != mode) {
    myEnc.write(alarmTemperature * 4);
    lcdLine(0,"Alarm Temp");
  }
  if (alarmTemperature != knobPosition || lastMode != mode) {
  alarmTemperature = knobPosition;
  lcdLine(1, alarmTemperature);
  }
  lastMode = mode;
break;
}
}
