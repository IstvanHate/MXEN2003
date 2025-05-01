//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
uint16_t ADC0, ADC1 = 0; //init ADCval as 16bit value as 0
int LED = 0;

//static function prototypes, functions only called in this file

int main(void)
{
  adc_init(); //initialse ADC
  _delay_ms(20); //it's a good idea to wait a bit after your init section

  DDRC = 0xFF;//put PORTC into output mode
  DDRA = 0x00;
  PORTC = 0;
  PORTA |= (1<<PA0);

  while(1)//main loop
  {

    ADC0 = adc_read(0); //read voltage at ADC port 0
    LED = ADC0/128; //going from 1024 to 1-8
    PORTC |= (1<<LED);  // note here PA3 is just an alias for the number 3


    //turn the LEDs off
    for (uint8_t i = 0; i < 8; i++){
      PORTC &= ~(1<<i);
    }
  }
  return(1);
}//end main