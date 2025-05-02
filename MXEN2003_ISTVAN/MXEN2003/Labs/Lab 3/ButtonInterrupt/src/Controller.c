//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
# define DEBOUNCE_PERIOD 100
//static function prototypes, functions only called in this file

int main(void)
{
  milliseconds_init();
  
  cli();
  DDRA = 0xFF; //put PORTA into output mode
  PORTA = 0; // set PORTA to low

  DDRD = 0; //put PORTD into input mode
  PORTD |= (1<<PD0);


  EICRA |= (1<<ISC01);
  EICRA &= ~(1<<ISC00);
  EIMSK |= (1<<INT0);

  sei();
  while(1)//main loop
  {
  }
  return(1);
}//end main 


static uint32_t previousTime = 0;

ISR(INT0_vect){
  uint32_t currentTime = milliseconds_now();

  if( (currentTime - previousTime) > DEBOUNCE_PERIOD ){
    PORTA ^= (1<<PA0);
    previousTime = currentTime;
  }
}