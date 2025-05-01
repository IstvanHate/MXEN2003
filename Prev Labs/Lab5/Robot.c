//Lab4 Lamp
#include "robot.h"

//Define variables
#define timeIncrement 1
// volatile uint16_t currentTime = 0;
volatile uint16_t ms = 0;
volatile uint16_t s = 0;
volatile uint16_t m = 0;
volatile bool startStop = true;

char time[50] = {};
//Reset Button Trigger
ISR(INT0_vect){
  /*char triggerText1[50] = {};
  sprintf(triggerText1, "\nButton 1 Pressed");
  serial0_print_string(triggerText1);*/
  /*char resetText[20] = {};
  sprintf(resetText, "\nStopwatch Reset");
  serial0_print_string(resetText);*/

  // reset time values
  ms = 0;
  s = 0;
  m = 0;
}

ISR(INT1_vect){
  /*char triggerText2[50] = {};
  sprintf(triggerText2, "\nButton 2 Pressed");
  serial0_print_string(triggerText2);*/
  startStop = !startStop;
}

ISR(TIMER1_COMPA_vect)
{
  if (startStop){

    // currentTime = currentTime + timeIncrement; // in seconds

    /*char time[40] = {};
    sprintf(time, "\n1 ms passed");
    serial0_print_string(time);*/
    ms += timeIncrement;
    if (ms >= 1000){
      ms = 0;
      s++;
    }
    if (s >= 60 ){
      s = 0;
      m++;
    }
  }

    sprintf(time, "\n%u m %u s %u ms", m, s, ms);
    serial0_print_string(time);

}

int main(void) {
  serial0_init();
  cli(); //turn off global interupts
  // Button 1 - RHS - Reset
  EICRA |= (1 << ISC01);  // Falling edge triggers interrupt
  EICRA &= ~(1 << ISC00); // Clear ISC00 bit
  EIMSK |= (1 << INT0); // Enable INT0 external interrupt

  //Button 2 - LHS - Start/Stop
  EICRA |= (1 << ISC11);
  EICRA &= ~(1 << ISC10);
  EIMSK |= (1 << INT1);

  //Timer Interrupt
  TCCR1A = 0;
  TCCR1B = (1<<WGM12)| (1<<CS11); //sets CTC Mode 4 and PRE = 8
  TCNT1 = 0;
  TIMSK1 = (1<<OCIE1A);
  OCR1A= 1999;

  //enable pull up
  PORTD |= (1 << PD0); //pull up on PD0,
  PORTD |= (1 << PD1); //PD1

  sei(); //enable global interupts
  _delay_ms(20);

  while(1){

  }

  return 1;
}