//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file

int main(void)
{
  DDRA = 0xFF; //put PORTA into output mode
  PORTA = 0; // set PORTA to low

  while(1)//main loop
  {
    _delay_ms(1000);     //500 millisecond delay
    PORTA ^= (1<<PA0);  //Toggles PA0 on
  }
  return(1);
}//end main 