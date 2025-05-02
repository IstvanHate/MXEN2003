#include "Controller.h"

volatile uint16_t ms = 0;
volatile uint16_t s = 0;
volatile uint16_t m = 0;
volatile bool pause = true;

static uint32_t previousTime = 0;
uint32_t currentTime = 0;

# define DEBOUNCE_PERIOD 200




void stopwatch_init(void)
{
    // PRE=1, MODE=4, 1ms period, TOP=15999
    cli();
    TCCR1A = 0; 
    TCCR1B |= (1<<CS11)|(1<<WGM12); // Enables CNC (Mode 4) and sets PRE=1
    TIMSK1 |= (1<<OCIE1A);          // Enables COMPARE timer interrupt 
    OCR1A = 1999; 
    sei();
}
void stopwatch_reset(void){
    ms = 0;
    s = 0;
    m = 0;
}




// Timer increase
ISR(TIMER1_COMPA_vect){
    if(pause == false){
		ms += 10;
	}
	if(ms>=1000){
		ms = 0;
		s++;
	}
	if(s>=60){
		s = 0;
		m++;
	}
	if(m>=60){
		stopwatch_reset();
	}

}
// Start-Stop Interrupt
ISR(INT0_vect){
  currentTime = milliseconds_now();
  if( (currentTime - previousTime) > DEBOUNCE_PERIOD ){
    if(pause == false){
      pause = true;
    }
    else{
      pause = false;
    }
  }
  previousTime = currentTime;
}
// RESET BUTTON
ISR(INT1_vect){
  currentTime = milliseconds_now();
  if( (currentTime - previousTime) > DEBOUNCE_PERIOD ){
	stopwatch_reset();
	previousTime = currentTime;
  }
}




int main(void)
{
  lcd_init();
  DDRD = 0;                   // set PORTD into input mode
  PORTD |= (1<<PD0)|(1<<PD1); // enable internal pull-ups

  TCCR1A = 0; 
  TCCR1B |= (1<<CS11)|(1<<WGM12); // Enables CNC (Mode 4) and sets PRE=1
  TCNT1 = 0;
  TIMSK1 |= (1<<OCIE1A);          // Enables COMPARE timer interrupt 
  OCR1A = 19999; 
  milliseconds_init();

  EICRA |= (1<<ISC01);
  EICRA &= ~(1<<ISC00);
  EICRA |= (1<<ISC11);
  EICRA &= ~(1<<ISC10);
  EIMSK |= (1<<INT0)|(1<<INT1);
  
  sei();
  
  char lcd_string[33] = {0};
  _delay_ms(20);

  while(1)//main loop
  {
    lcd_goto(0); 
	lcd_clrscr();
    sprintf(lcd_string,"%2um,%2us,%4ums",m,s,ms);
    lcd_puts(lcd_string);
    _delay_ms(20);
  }
  return(1);
}//end main 