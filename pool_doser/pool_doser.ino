int enablePin = 11;
int in1Pin = 10;
int in2Pin = 9;
int switchPin = 7;
//int potPin = 0;
 
void setup()
{
pinMode(in1Pin, OUTPUT);
pinMode(in2Pin, OUTPUT);
pinMode(enablePin, OUTPUT);
pinMode(switchPin, INPUT_PULLUP);
}
 
void loop()
{
//int speed = analogRead(potPin) / 4;
boolean reverse = digitalRead(switchPin);
setMotor(255, reverse);
}
 
void setMotor(int enable, boolean reverse)
{
analogWrite(enablePin, enable);
digitalWrite(in1Pin, ! reverse);
digitalWrite(in2Pin, reverse);
}
