
 
#include <avr/sleep.h>

#define INTER_BLINK_DELAY 2000
#define WDT_PRESCALER 9
#define WDT_BASE_TICK 18.49


const int wdt_tick = WDT_BASE_TICK * pow(2,WDT_PRESCALER) ;//millis per wdt count
const long day_millis = 86400000L;

int led = 1;
int button = 2;
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


EMPTY_INTERRUPT(PCINT0_vect)


void sleep()
{
  
  GIMSK |= 1<<PCIE; //Enable Pin Change Interrupt
  PCMSK |= 1<<PCINT2; //Watch for Pin Change on Pin5 (PB0)
 
  ADCSRA &= ~(1<<ADEN);
  
  sleep_mode();
  
  ADCSRA |= (1<<ADEN);
  
 
 GIMSK &= ~(1<<PCIE); //Disable the interrupt so it doesn't keep flagging
 PCMSK &= ~(1<<PCINT2);  
  
}


void setup() {                
  
  pinMode(led, OUTPUT);  
  pinMode(button, INPUT_PULLUP); 
  
  if (digitalRead(button)== LOW) {
    mode = 1;
  } else {
    mode = 0;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    setup_watchdog(WDT_PRESCALER); 
    
  }
  
  
}


void loop() {
  switch(mode) {
    
    case 0:
    
    static long last_watchdog_counter = 0;
    
    if (watchdog_counter == last_watchdog_counter) {
      //this means sleep was left by a button push or reset
      digitalWrite(led,HIGH);
      delay(1000); //
      digitalWrite(led,LOW);
      
      if (digitalRead(button)==HIGH) {
        //do the info routine
        doBlink(readBatteryLevel(),250,500);
        delay(INTER_BLINK_DELAY);
        doBlink(getDoseFrequency(),250,500);
        delay(INTER_BLINK_DELAY);
        doBlink(getDoseDuration(),250,500);
      } else {
        runMotor(0);
        //run the motor manually
      }
      
    } else if (watchdog_counter >= getDoseIntervalTicks()) {
      //time to dose
      watchdog_counter = 0;
      runMotor(getDoseDuration());  
    }
      
    last_watchdog_counter = watchdog_counter;
    sleep();
    break;
    
    case 1:

    //programming code
    
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

int readBatteryLevel() {
  return 3;
}

int getDoseFrequency() {
  return 3;
}

int getDoseDuration() {
  return 3;
}

int getDoseIntervalTicks() {
  //watchdog_counter counts in a dose interval
  return day_millis/getDoseFrequency()/wdt_tick;
}

void runMotor(long duration) {
  long m = millis();
  
  while (millis() - m < duration || digitalRead(button)==LOW) {
    doBlink(1,100,200);
  }
}


  


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
