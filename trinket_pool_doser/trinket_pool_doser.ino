
 
#include <avr/sleep.h>
#include <EEPROM.h>
//#include <avr/power.h>
//#include <avr/wdt.h>

//#define DOSE_LEVEL_ADDR 0
#define DOSE_FREQUENCY_ADDR 0

#define INTER_BLINK_DELAY 2000
#define WDT_PRESCALER 9
//#define WDT_BASE_TICK 18.49
#define DOSE_BASE 4350 // 0.87 s/g * 5g = 4350ms
#define COOL_DOWN_TIME 10000
#define SHUTDOWN_VOLTAGE 6000


//const int wdt_tick = 18.49 * WDT_BASE_TICK * pow(2.0,WDT_PRESCALER) ;//millis per wdt count
const long day_millis = 86400000L; //24 hours
//const long day_millis = 600000L; //ten minutes

#if defined(__AVR_ATtiny85__)

#define WDT_TICK 9456 //millis between WDT ticks
#define ECHO_TO_SERIAL   0 // echo data to serial port
int led = 0;
int button = 3;
int motor = 1;
int battery = 2;//pin4 = A2
int current = 1;//pin2 = A1
#else

#define WDT_TICK 8600
#define ECHO_TO_SERIAL   1 // echo data to serial port
//assign UNO pins here
int led = 11;
int button = 2; //must stay as pin 2 for interrupt to work 
int motor = 3;
int battery = 4;
int current = 2;
#endif

int buttonState;
int lastButtonState = HIGH; 
long lastButtonTime = 0;
long lastDebounceTime = 0;  // the last time the output pin was toggled
int debounceDelay = 50;    // the debounce time; increase if the output flickers
int inputDelay = 1000;
boolean buttonChanged;

//int longDuration = 2000;
int mode;
//boolean pinChange = false;
volatile long watchdog_counter = 0;


ISR(WDT_vect) {
watchdog_counter += 1;
}

//ISR(PCINT0_vect) {
//pinChange = true;
//}

#if defined(__AVR_ATtiny85__)
EMPTY_INTERRUPT(PCINT0_vect)
#else
//pin interrupt ISR for UNO here
void wakeUpNow()        // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
}
#endif

#if defined(__AVR_ATtiny85__)
void sleep()
{
  PCMSK |= 1<<PCINT3; //Watch for Pin Change on Pin3
  GIMSK |= 1<<PCIE; //Enable Pin Change Interrupt

  ADCSRA &= ~(1<<ADEN);
  
  sleep_mode();
  
  ADCSRA |= (1<<ADEN);
  
 
  GIMSK &= ~(1<<PCIE); //Disable the interrupt so it doesn't keep flagging
  PCMSK &= ~(1<<PCINT3);  
}
#else
void sleep()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
 
    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin
 
    /* Now it is time to enable an interrupt. We do it here so an
     * accidentally pushed interrupt button doesn't interrupt
     * our running program. if you want to be able to run
     * interrupt code besides the sleep function, place it in
     * setup() for example.
     *
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.  
     *
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */
 
    attachInterrupt(0,wakeUpNow, CHANGE); // use interrupt 0 (pin 2) and run function
                                       // wakeUpNow when pin 2 gets LOW
 
    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
 
    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
                             // wakeUpNow code will not be executed
                             // during normal running time.
 
}
#endif



void setup() {  
  
  #if ECHO_TO_SERIAL
  Serial.begin(9600);
  Serial.println("Hello");
  #endif
  
  #if defined(__AVR_ATtiny85__)
  //
  #else
  analogReference(EXTERNAL);
  #endif

  
  pinMode(led, OUTPUT);  
  pinMode(button, INPUT_PULLUP); 
  pinMode(motor, OUTPUT);
  //pinMode(current, INPUT);
  //pinMode(battery, INPUT);

  analogRead(battery); //the first read should be ignored
  analogRead(current); //the first read should be ignored

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  setup_watchdog(WDT_PRESCALER);   
  
  buttonState = digitalRead(button);
  lastButtonState = buttonState;
  if (buttonState == LOW) {
    mode = 3;
    //EEPROM.write(DOSE_FREQUENCY_ADDR, 2);
    //EEPROM.write(DOSE_LEVEL_ADDR, 5);
    
  } else {
    mode = 0;
   
  }
  
  
}


void loop() {
  

  static int previous = -1;
  int reading = digitalRead(button);
    if (reading != previous) { 
      // reset the debouncing timer
      lastDebounceTime = millis();
    } 
    if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
      buttonState = reading;
    }
    
    if (buttonState != lastButtonState){
      buttonChanged = true;
      lastButtonTime = millis();
    } else {
      buttonChanged = false;
    }
  lastButtonState = buttonState; 
  previous = reading; 
  
  
  
  
  
  
  
  switch(mode) {
    
    case 0:
    
    static long last_watchdog_counter = 0;
    static int dose_count = 0;
    
    if (watchdog_counter == last_watchdog_counter) {
      //this means sleep was left by a button push or reset
      
      if (watchdog_counter > 0) delay(1000); 
      // there was a button push, do delay/debounce
      
      if (digitalRead(button)==HIGH) {
        //do the info routine
        //delay(1000);
        analogRead(battery);
        doBlink(readBatteryLevel(),250,500);        
        delay(INTER_BLINK_DELAY);
        doBlink(getDoseFrequency(),250,500);
        delay(INTER_BLINK_DELAY);
        doBlink(getDoseLevel(),250,500);
      } else {
        runMotor(0);
        //run the motor manually
      }
      
    } else if (watchdog_counter*WDT_TICK - dose_count*getDoseInterval() >= getDoseInterval()) {
      //time to dose
      long dose_duration = getDoseLevel() * DOSE_BASE;
      runMotor(dose_duration); 
      dose_count += 1;
    } else {
      doBlink(1,50,50);
    }
    
    #if ECHO_TO_SERIAL
    Serial.println("WDT:");
    Serial.print(watchdog_counter); Serial.print(", "); 
    //Serial.print(inputValue); Serial.print(", "); 
    //Serial.print(batteryVoltage);  Serial.print(", ");
    Serial.println(last_watchdog_counter);
    delay(10);
    #endif
      
    last_watchdog_counter = watchdog_counter;
    sleep();
    break;
    
    case 1:
    //programming code
    {
    static byte inputValue = 0;
    static byte inputIndex = 0;   
    if (buttonChanged) {
      if (buttonState == HIGH){
        inputValue++;
        digitalWrite(led,LOW);
      } else {
        digitalWrite(led,HIGH);
      }
    }
    if (buttonState == HIGH 
        && millis() - lastButtonTime > inputDelay) {
      EEPROM.write(inputIndex + DOSE_FREQUENCY_ADDR, inputValue);

      #if ECHO_TO_SERIAL
      Serial.print("InputIndex, Value:"); 
      Serial.print(inputIndex); Serial.print(", "); 
      //Serial.print(inputValue); Serial.print(", "); 
      //Serial.print(batteryVoltage);  Serial.print(", ");
      Serial.println(inputValue);
      #endif

      inputValue = 0;
      inputIndex++;
      if (inputIndex == 2) {
        mode = 0;
      } else {
        mode = 3;
      }
    }
   
    }
    //end programming code
    break;
    
    case 2:

    //motor stall code
    
    doBlink(2,50,200);
    sleep();
    
    break;
    
    case 3:
    // pulsing led mode
    {
      static int brightness = 0;
      static int fadeAmount = 5;
      static long time = 0;
      static int fadeDelay = 20;

      if (millis() - time > fadeDelay) {
        time = millis();
        brightness = brightness + fadeAmount;
        if (brightness == 0 || brightness == 255) {
          fadeAmount = -fadeAmount ;
        } 
      }   
      analogWrite(led, brightness); 
  
      if (buttonChanged && buttonState == LOW){
        mode = 1;
        brightness = 0;
        digitalWrite(led, HIGH);        
      } 
    
    }
    break;
    
  }
} 

void doBlink(int n_blinks, int blink_duration,int blink_delay) {
  for (int x = 0; x < n_blinks; x++) {
    digitalWrite(led,HIGH);
    delay(blink_duration);
    digitalWrite(led,LOW);
    delay(blink_delay);
  }
}

int readBatteryVoltage() {
  
  int rawReading = analogRead(battery);
  int rawVoltage = map(rawReading,0,1023,0,3300);
  //float rawVoltage = rawReading/1024.0 * 3.3;
  int batteryVoltage = 31L * rawVoltage/10;

  #if ECHO_TO_SERIAL
  Serial.print("raw,rawVolt,batVolt: "); 
  Serial.print(rawReading); Serial.print(", "); 
  Serial.print(rawVoltage); Serial.print(", "); 
  Serial.println(batteryVoltage);  
  #endif
  return batteryVoltage;
}


int readBatteryLevel() {
  int batteryLevel;
  
  //int rawReading = analogRead(battery);
  //int rawVoltage = map(rawReading,0,1023,0,3300);
  //float rawVoltage = rawReading/1024.0 * 3.3;
  int batteryVoltage = readBatteryVoltage();//32 * rawVoltage/10;

  if ( batteryVoltage > 7800 ) { batteryLevel = 4; }
  else if ( batteryVoltage > 7200 ) { batteryLevel = 3; }
  else if ( batteryVoltage > 6600 ) { batteryLevel = 2; }
  else { batteryLevel = 1; }

  #if ECHO_TO_SERIAL
  Serial.print("BatLevel: "); 
  Serial.println(batteryLevel);
  #endif
  return batteryLevel;
}

int getDoseFrequency() {
  return EEPROM.read(DOSE_FREQUENCY_ADDR);
}

int getDoseLevel() {
  return EEPROM.read(DOSE_FREQUENCY_ADDR + 1);
}

long getDoseInterval() {
  //millis between doses
  return day_millis/getDoseFrequency();
}

void runMotor(long duration) {
  int ary[3] = { 0,0,0 }; 
  analogRead(battery);
  int stallCurrent = map(readBatteryVoltage(),6000,8000,300,400); 
  int duty = 255; //(695000L - 63L*readBatteryVoltage())/1000L; 
  //if (duty>255)duty = 255;  
  long m = millis(); 
  analogWrite(motor,duty); 

  while (millis() - m < duration || digitalRead(button)==LOW) {
  
    int rawReading = analogRead(current); 
    int rawVoltage = map(rawReading,0,1023,0,3300);
    int motorCurrent = rawVoltage*10L/15;
    
    if (readBatteryVoltage() < SHUTDOWN_VOLTAGE){
      //don't damage the batteries
      mode = 2;
      break;
    }

    
    int index = (millis()%1500)/500; // 0, 1 or 2. Changes every 500ms
    ary[index] = motorCurrent;
    int sum = ary[0] + ary[1] + ary[2];
    
    #if ECHO_TO_SERIAL
    Serial.print("duty,stallCur,rawRead,rawVolt,motorCur,index,sum: ");
    Serial.print(duty); Serial.print(", "); 
    Serial.print(stallCurrent); Serial.print(", "); 
    Serial.print(rawReading); Serial.print(", "); 
    Serial.print(rawVoltage); Serial.print(", "); 
    Serial.print(motorCurrent); Serial.print(", "); 
    Serial.print(index); Serial.print(", "); 
    Serial.println(sum); 
    #endif
    
    if ( sum > stallCurrent * 3 ) {
      //motor stalled (or came close). Stop for a while.
      
      if (digitalRead(button)==LOW) break;
      //full stop if in manual mode
      
      analogWrite(motor,0);
      ary[0]=0; ary[1]=0; ary[2]=0;
      m = m + COOL_DOWN_TIME;
      doBlink(1,COOL_DOWN_TIME,0);
      analogWrite(motor,duty);
    }
    doBlink(1,50,100);
  }
  analogWrite(motor,0);
}


  
#if defined(__AVR_ATtiny85__)

//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec 4.75, 9=8sec 9.47
//From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {
if (timerPrescaler > 9 ) timerPrescaler = 9; //Limit incoming amount to legal settings
byte bb = timerPrescaler & 7;
if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary
//This order of commands is important and cannot be combined
MCUSR &= ~(1<<WDRF); //Clear the watch dog reset
WDTCR |= (1<<WDCE) | (1<<WDE); //Set WD_change enable, set WD enable
WDTCR = bb; //Set new watchdog timeout value
WDTCR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}
#else
void setup_watchdog(int timerPrescaler) {
  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();
  
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  
  // Enable interrupts again.
  interrupts();
 
}
#endif
