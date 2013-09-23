/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */

#define y1Pin 11
#define y2Pin 13
#define w1Pin 9
#define w2Pin 8
#define gPin  12

#define redLEDpin 2
#define greenLEDpin 3

boolean y1State = false;
boolean y2State = false;
boolean w1State = false;
boolean w2State = false;
boolean gState = false;
 
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
  digitalWrite(greenLEDpin, g);
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
  
  updateStates();
}

// the loop routine runs over and over again forever:
void loop() {
  
  uint32_t m = millis();
  
  if (updateStates()== true) 
  {
  //int sensorValue = analogRead(A2);
  // print out the value you read:
  Serial.print(m);
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
  Serial.println("");
  }
}
