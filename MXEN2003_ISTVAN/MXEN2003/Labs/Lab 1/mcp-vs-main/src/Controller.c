//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file

int main(void)
{
  DDRB = 0xFF;//put PORTA into output mode
  DDRC = 0xFF;
  PORTB = 0;
  PORTC = 0; 
  while(1)//main loop
  {
    for(uint8_t i = 0; i <= 3; i++){
      _delay_ms(100);
      PORTC = 0;
      PORTB = (1<<i);
    }
    
    for(uint8_t i = 0; i <= 3; i++){
      _delay_ms(100);
      PORTB = 0;
      PORTC = (1<<i);
    }

    /*_delay_ms(500);     //500 millisecond delay
    PORTA |= (1<<PA3);  // note here PA3 is just an alias for the number 3
                        // this line is equivalent to PORTA = PORTA | 0b00001000   which writes a HIGH to pin 3 of PORTA
    PORTA &= ~(1<<PA4);
    _delay_ms(500); 
    PORTA &= ~(1<<PA3); // this line is equivalent to PORTA = PORTA & (0b11110111)  which writes a HIGH to pin 3 of PORTA
    PORTA |= (1<<PA4); */
  }
  return(1);
}//end main 