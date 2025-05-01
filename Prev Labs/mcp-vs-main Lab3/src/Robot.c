//Code for Lab 3 task
#include "Robot.h"
#define DEBOUNCE_PERIOD 100

//Define variables / constants
int16_t ADC0val = 0; // init Photoresistor ADC val starting at 0 (zero brightness)
int LED, LightVolts;
char LEDstr[50] = {};
volatile int fallingEdgeCount = 0;
volatile bool lastButtonState = 0xFF;

//any functions
/*void serial0_init(void)
{
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    //turn on transmission and reception pins
    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
    UBRR0 = BAUD_PRESCALE; //load baud rate prescaler into UBRR
}

void serial0_print_string(char * string_ptr)
{
    while(*string_ptr)
    {
        while((UCSR0A & (1<<UDRE0)) == 0){} //wait until ready
        UDR0 = *string_ptr;
        //send char at ptr to seria data register (UDR)
        string_ptr++; //increment ptr pos in array by 1
    }
}
*/

//main program
int main(void)
{
    adc_init();
    serial0_init();
    _delay_ms(20); //delay by 20ms after init ADC
    milliseconds_init();

    //time to set up ports
    DDRC = 0xFF;//Set data direction register for port C as HIGH (output mode)
    DDRA = 0x00; // set Port A DDR as LOW (input mode)
    PORTC = 0; //set all pins in Port C to LOW
    PORTA |= (1<<PA0); //pin 0 in port A set to 1
    DDRD &= ~(1<<PD0);
    PORTD |= (1<<PD0);

    cli();
    EICRA |= (1<<ISC01);
    EICRA &= ~(1<<ISC00);  //falling trigger mode
    EIMSK |= (1<<INT0);
    sei();


    //now main program loop
    while(true){
      //ADC0val = adc_read(0); //ADC returns 10bit representation of photoresistor voltage, stored in 16bit int
      //LED = ADC0val/128; //going from 1024 to 1-8
      //PORTC = (1<<LED);  //set corresponding pin in port C to HIGH (turn on LED) and turn off all others

      //LightVolts = ADC0val * (5000/1024);

      //turn the LEDs off (experimenting just setting all LEDS at once atm)
      /*for (uint8_t i = 0; i < 8; i++){
          PORTC &= ~(1<<i);
          }*/
      char countString[100] = {};
      sprintf(countString, "%u per second\n", fallingEdgeCount);
      serial0_print_string(countString);

      fallingEdgeCount = 0;
      _delay_ms(1000);

    }

    return(1);
}

ISR(INT0_vect)
{
  //char serialString[100] = {};
  //char LEDchar = (char)(LED + '0'); // Cast to char
  //LEDstr[60] = LEDchar;
  //sprintf(serialString, "%u mV\n", LightVolts);
  //serial0_print_string(serialString);
  //_delay_ms(50);
  uint32_t currentTime = milliseconds_now(); //get time
  static uint32_t previousTime = 0;

  if ((currentTime - previousTime) > DEBOUNCE_PERIOD) {
    previousTime = currentTime;
    fallingEdgeCount++;
  }


}