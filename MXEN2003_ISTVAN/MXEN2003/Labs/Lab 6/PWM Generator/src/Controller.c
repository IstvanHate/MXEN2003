#include "Controller.h"








int main(void)
{
  cli();
  adc_init();
  _delay_ms(20);

  TCCR1B |= (1<<WGM13)|(1<<CS11);    // Mode 8, PRE=8
  TCCR1A |= (1<<COM1A1)|(1<<COM1B1); // Compare mode (2 compares)
  TCNT1 = 0;
  ICR1 = 20000;
  DDRB |= (1<<PB5)|(1<<PB6);
  OCR1A = 1500;
  OCR1B = 1500;
  sei();


  while(1)//main loop
  {
    OCR1A = adc_read(15) + 1000;
    OCR1B = adc_read(14) + 1000;

  }
  return(1);
}//end main 