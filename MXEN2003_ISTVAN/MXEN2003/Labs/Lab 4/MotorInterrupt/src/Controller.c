//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
# define DEBOUNCE_PERIOD 200
//static function prototypes, functions only called in this file
bool FREEZE;

int main(void)
{
  milliseconds_init();
  
  FREEZE = false;
  char lcd_string[33] = {0};
  adc_init();
  lcd_init();


  _delay_ms(20);
  uint16_t rangeSensor1Val;
  uint16_t eqDist;

  cli();
  
  DDRD = 0; // put PORTD into input mode
  PORTD |= (1<<PD0); // enable internal pullup
  
  EICRA |= (1<<ISC01);
  EICRA &= ~(1<<ISC00);
  EIMSK |= (1<<INT0);
  
  sei();
  
  while(1)
  {
    lcd_goto(0);
    lcd_home();
      if(FREEZE == false){
        lcd_clrscr();
        rangeSensor1Val=adc_read(0);
        eqDist = (34-(rangeSensor1Val/20))/2+2;
        sprintf(lcd_string,"D=%4ucm",eqDist);

      }
      lcd_puts( lcd_string );
      _delay_ms(30);
  }
  return(1);
}//end main 


static uint32_t previousTime = 0;

ISR(INT0_vect){
  uint32_t currentTime = milliseconds_now();

  if( (currentTime - previousTime) > DEBOUNCE_PERIOD ){
    
    if(FREEZE == false){
      FREEZE = true;
    }
    else if(FREEZE == true){
      FREEZE = false;
    }
    previousTime = currentTime;



  }
}