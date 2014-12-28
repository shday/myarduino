const uint8_t motorPin = 3;
//int in1Pin = 10onst;
//int in2Pin = 9;
const uint8_t buttonPin = 7;
const uint8_t ledPin = 13;
//int potPin = 0;
int speed = 255;

int buttonState = HIGH;             // the current reading from the input pin
int blinkState = LOW;  
long dosesPerDay = 4;
int maxDosesPerDay = 8;
int lastButtonState = HIGH;   // the previous reading from the input pin
int blinkCount = 0;
int ledState = LOW;

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers
long doseInterval = 60000L*60L*24L/dosesPerDay;
long doseDuration = 30000; //160 sec per 100g with 6V and speed 200
long longPressTime = 2000;
long lastBlinkTime = 0;
long lastDoseTime = 0;
long blinkPause = 2000;
long blinkInterval = 400;
long lastButtonTime = 0;

 
void setup()
{
//pinMode(in1Pin, OUTPUT);
//pinMode(in2Pin, OUTPUT);
pinMode(motorPin, OUTPUT);
pinMode(buttonPin, INPUT_PULLUP);
pinMode(ledPin, OUTPUT);
Serial.begin(9600);
while (! Serial);
Serial.println("Speed 0 to 255");


}
 
void loop()
{
//if (Serial.available())
//    {
//    speed = Serial.parseInt();
//    }

 // static unsigned long lastSync = 0;
 // static unsigned long lastRead = 0;
  // log milliseconds since starting
  uint32_t m = millis();

  int reading = digitalRead(buttonPin);
    if (reading != lastButtonState) { 
    // reset the debouncing timer
    lastDebounceTime = millis();
  } 
    if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    //lastButtonState = buttonState;
    
    if (buttonState != reading){
    buttonState = reading;
    if (buttonState == HIGH 
        && m - lastButtonTime <= longPressTime) {
      dosesPerDay++;
      if (dosesPerDay > maxDosesPerDay) 
          { dosesPerDay = 1 ;}
      doseInterval = 60000L*60L*24L/dosesPerDay;
      blinkState = LOW;
      blinkCount = 0;
      digitalWrite(ledPin,LOW);
        }
    /*  Serial.print(buttonPushCounter);
      Serial.print(",");
      Serial.println(mode);
      Serial.print(","); 
      Serial.print(rawKnobPosition);
      Serial.print(",");   
      Serial.println(knobPosition); */
   lastButtonTime = lastDebounceTime; 
  }
  }
  lastButtonState = reading;
  
if (buttonState == LOW && millis() - lastDebounceTime > longPressTime){
  analogWrite(motorPin, speed);
  blinkState = LOW;
  blinkCount = 0;
  digitalWrite(ledPin,LOW);
  while (digitalRead(buttonPin)== LOW){};
  analogWrite(motorPin, 0);
  lastDoseTime = millis();
}
  
if (millis() - lastDoseTime > doseInterval) {
        lastDoseTime = millis();
        analogWrite(motorPin, speed);
        //Serial.println("got here"); 
      }
else if (millis() - lastDoseTime > doseDuration) {
        analogWrite(motorPin, 0); 
      }
   

    if (blinkState == HIGH){
        if (millis() - lastBlinkTime > blinkInterval) {
            lastBlinkTime = millis();
            
            //Serial.print(doseInterval);
            //Serial.print(","); 
            //Serial.println(lastDoseTime);
            if (ledState == LOW){
                ledState = HIGH;}
            else {
                ledState = LOW;
                blinkCount = blinkCount + 1;
                if (blinkCount == dosesPerDay){
                    blinkState = LOW;
                    blinkCount = 0;
                     }
                 }    
            digitalWrite(ledPin,ledState);
            }
        }
    else if (blinkState == LOW) {
        if (millis() - lastBlinkTime > blinkPause) {
            lastBlinkTime = millis();
            blinkState = HIGH;
            }
    }

    
} 
