//Lab4 Lamp
#include "robot.h"

//Define variables


void set_up_timer(void)
{
  TCCR1A = (1<<WGM01) | (1<<COM1A1) | (1<<COM1B1); //Phase correct w/ OCR1A to compare
  TCCR1B = (1<<WGM13) | (1<<CS11); //Set PRESCALER = 8

  OCR1A = 20000; //set with hand calcs to get 50 Hz

  DDRB |= (1 << PB1);// Set OC1A (PB1 on ATmega328P) as output
}