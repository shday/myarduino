#include <Servo.h>

/* Rotary encoder read example */
#define SERVO 5
#define LED 6
#define SWITCH 13

const unsigned long WAIT = 1000UL * 60 * 60 * 24 * 10;
unsigned long time;

Servo myservo; // create servo object to control a servo

void setup()
{
   myservo.attach(SERVO, 500, 2400);
   pinMode(SWITCH, INPUT);
   if (digitalRead(13)==HIGH)
   {myservo.write(90);}
   else
   {myservo.write(175);}
  //myservo.attach(SERVO);
}

void loop()
{
   //delay(1000);
   //myservo.write(150);
   // sets the servo position according to the scaled value
   time = millis();
   
   if (time%5000 <100){
     digitalWrite(LED, HIGH);
   }
   else
   { digitalWrite(LED, LOW);
   };
   
 if (time > WAIT){
   myservo.write(30);
 };
   // sets the servo position according to the scaled value
   //delay(1000);
   //myservo.write(180);
   // sets the servo position according to the scaled value

}
