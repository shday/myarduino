
 
#include <avr/sleep.h>

 

int led = 1;
int button = 2;
int mode;
volatile long watchdog_counter = 0;


ISR(WDT_vect) {
watchdog_counter += 1;
}


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
    setup_watchdog(9); 
    
  }
  
  
}


void loop() {
  if (mode==0) {
    static long last_watchdog_counter = -1;
    if (watchdog_counter == last_watchdog_counter) {
      //this means sleep was left by a button push
      delay(2000);
      if (digitalRead(button)==LOW) {
        //run the motor manually
      } else {
        //do the info routine
      }
      
    } else if (true) {
      //nothing yet
      digitalWrite(led, !digitalRead(led));  
    }
      
    last_watchdog_counter = watchdog_counter;
    sleep();
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
