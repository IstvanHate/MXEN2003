//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file

int main(void)
{
  DDRA = 0; //put PORTA into output mode
  DDRB = 0xFF; //put PORTB into output mode
  DDRC = 0xFF; //put PORTC into output mode
  
  PORTB = 0;
  PORTC = 0; 

  adc_init(); //initialse ADC
  _delay_ms(20); //it's a good idea to wait a bit after your init section

  int16_t inputSignal;

  while(1)//main loop
  {
      
    inputSignal = adc_read(3); //read the voltage at ADC0

    if(inputSignal >= 37){
      PORTB = 0;
      PORTC = 0;
    }
    if(inputSignal >= 39){
      PORTB = 0b00000001;
      PORTC = 0;
    }
    if(inputSignal >= 41){
      PORTB = 0b00000011;
      PORTC = 0;
    }
    if(inputSignal >= 43){
      PORTB = 0b00000111;
      PORTC = 0;
    }
    if(inputSignal >= 45){
      PORTB = 0b00001111;
      PORTC = 0;
    }
    if(inputSignal >= 47){
      PORTB = 0b00001111;
      PORTC = 0b00000001;
    }
    if(inputSignal >= 49){
      PORTB = 0b00001111;
      PORTC = 0b00000011;
    }
    if(inputSignal >= 51){
      PORTB = 0b00001111;
      PORTC = 0b00000111;
    }
    if(inputSignal >= 53){
      PORTB = 0b00001111;
      PORTC = 0b00001111;
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