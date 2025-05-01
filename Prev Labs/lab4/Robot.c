//Lab4 Lamp
#include "robot.h"

//Define variables
#define ADCMAX

uint16_t ADCVal;
uint16_t adcmax = 0;



int main(void) {
  adc_init();
  serial0_init();
  DDRA = 0xff; //Set to Output
  PORTA = 0; //Set all pins to low to start
  _delay_ms(44); //first IR sensor read

  while(true){
    ADCVal = adc_read(0); //read analogue pin 0
    if (ADCVal > 150) {
      PORTA |= (1<<PA0); //turn on motor
    } else {
      PORTA &= ~(1<<PA0); //turn off motor
    }
    _delay_ms(39);

    if (ADCVal > adcmax) {
      adcmax = ADCVal;
    }

    char countString[100] = {};
    sprintf(countString, "%u Distance\n", adcmax);
    serial0_print_string(countString);
  }
  return 1;
}