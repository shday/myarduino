/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */

#define Y1pin 6
#define Y2pin 7
#define W1pin 8
#define W2pin 9
#define Gpin 10
 
 
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);
  pinMode(Y1pin,INPUT);
  pinMode(Y2pin,INPUT);
  pinMode(W1pin,INPUT);
  pinMode(W2pin,INPUT);
  pinMode(Gpin,INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  uint32_t m = millis();
  int sensorValue = analogRead(A2);
  int digitalValue = digitalRead(7);
  // print out the value you read:
  Serial.print(m);
  Serial.print(", ");
  Serial.print(digitalValue);
  Serial.print(", ");
  Serial.println(sensorValue);
  delay(1);        // delay in between reads for stability
}
