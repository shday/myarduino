/*

  The circuit:
 * 5V to Arduino 5V pin
 * GND to Arduino GND pin
 * CLK to Analog #5
 * DAT to Analog #4
*/

// include the library code:
#include "SD.h"
#include "Wire.h"
//#include "LiquidCrystal.h"
#include <Sensirion.h>
#include "RTClib.h"
#include <stdio.h>

// A simple data logger for the Arduino analog pins
#define LOG_INTERVAL  10000 // mills between entries
#define ECHO_TO_SERIAL   0 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()


// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3





const uint8_t dataPin  =  4;
const uint8_t clockPin =  5;

float temperature;
float humidity;
float dewpoint;

char s[32];

char filename[] = "LOGGER00.CSV";

// Connect via i2c, default address #0 (A0-A2 not jumpered)
//LiquidCrystal lcd(0);

Sensirion tempSensor = Sensirion(dataPin, clockPin);

RTC_DS1307 RTC; // define the Real Time Clock object


// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  
  while(1);
}



void setup()
{
  Serial.begin(9600);
  Serial.println();
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

 // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
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
  
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }
  

  logfile.println("millis,time,temp,humidity,dewpoint");    
#if ECHO_TO_SERIAL
  Serial.println("millis,time,temp,humidity,dewpoint");
#endif //ECHO_TO_SERIAL

  logfile.close();

// attempt to write out the header to the file
 // if (logfile.writeError || !logfile.sync()) {
 //   error("write header");
 // }
  
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);

  
  
  // set up the LCD's number of rows and columns: 
  //lcd.begin(16, 2);
  //lcd.setBacklight(LOW);
  // Print a message to the LCD.
  //lcd.print("hello, world!");
}

void loop()
{
  
  DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  digitalWrite(greenLEDpin, HIGH);
  logfile = SD.open(filename, FILE_WRITE); 

  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
#endif

  // fetch the time
  now = RTC.now();
  // log time
  //logfile.print(now.get()); // seconds since 2000
  logfile.print(", ");
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
#if ECHO_TO_SERIAL
  //Serial.print(now.get()); // seconds since 2000
  Serial.print(", ");
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
#endif //ECHO_TO_SERIAL
  
  tempSensor.measure(&temperature, &humidity, &dewpoint);

//  lcd.setCursor(0, 1);
//  lcd.print(dtostrf(temperature, 4, 1, s));
//  lcd.print(" ");
//  lcd.print((int)(humidity + 0.5));
//  lcd.print("% ");
//  lcd.print(dtostrf(dewpoint, 4, 1, s));
  
  logfile.print(", ");
  logfile.print(temperature);
  logfile.print(", ");
  logfile.print(humidity);
  logfile.print(", ");
  logfile.println(dewpoint);
  
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(temperature);
  Serial.print(", ");
  Serial.print(humidity);
  Serial.print(", ");
  Serial.println(dewpoint);
#endif //ECHO_TO_SERIAL

  logfile.close();
  digitalWrite(greenLEDpin, LOW);

}

