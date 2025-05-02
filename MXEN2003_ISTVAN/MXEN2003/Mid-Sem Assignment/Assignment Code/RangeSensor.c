// C++ code
//
// Includes for testing in VS Code
//
//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <stdbool.h>
//#include <stdlib.h>
//#include <util/delay.h>
//#include <stdio.h>



// /************************************************************************
//  * BEFORE ADDING COMMENTS EXAMPLE *
// ************************************************************************/

// //Define USART constants for ATmega328
// #define USART_BAUDRATE 9600
// #define F_CPU 16000000
// #define BAUD_PRESCALE ((((F_CPU/16)+(USART_BAUDRATE/2))/(USART_BAUDRATE))-1)

// /************************************************************************
// Initialise USART 0
// See ATmega328P datasheet for register descriptions, pg 159
// Input: None
// Output: None
// ************************************************************************/
// void serial0_init(void)
// {
// 	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
// 	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);
// 	UBRR0 = BAUD_PRESCALE;
// }

// /************************************************************************
// Print string via USART 0
// See ATmega328P datasheet for register descriptions, pg 159
// Input:      string_pointer      char *      string to be printed to serial 0
// Output:     None
// ************************************************************************/
// void serial0_print_string(char * string_pointer) 
// {
// 	while(*string_pointer)
// 	{
// 		while((UCSR0A&(1<<UDRE0))==0){}
// 		UDR0 = *string_pointer;
// 		string_pointer++;
// 	}
// }

// /************************************************************************
// Test serial sprinting accross USART 0
//     Will initialise USART 0 if not initialised
// Input: None
// Output: None
// ************************************************************************/
// void test_print(void)
// {
//     if(UBRR0 != BAUD_PRESCALE)
//     {
//         serial0_init();
//         serial0_print_string("Serial 0 was not initialised. Now initialised and working");
//     }
//     else
//     {
//         serial0_print_string("Serial 0 is working");
//     }
// }

// */

/************************************************************************
 * AFTER ADDING COMMENTS EXAMPLE *
************************************************************************/

//Define USART constants for ATmega328, see ATmega328P datasheet, pg 145
#define USART_BAUDRATE 9600
#define F_CPU 16000000
#define BAUD_PRESCALE ((((F_CPU/16)+(USART_BAUDRATE/2))/(USART_BAUDRATE))-1)

/************************************************************************
Initialise USART 0
See ATmega328P datasheet for register descriptions, pg 159
Input: None
Output: None
************************************************************************/
void serial0_init(void)
{
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);     //Enable bits for transmit and recieve
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);   //Use 8- bit character sizes
	UBRR0 = BAUD_PRESCALE;              //Load baud rate prescaler into register
}

/************************************************************************
Print string via USART 0
See ATmega328P datasheet for register descriptions, pg 159
Input:      string_pointer      char array      string to be printed to serial 0
Output:     None
************************************************************************/
void serial0_print_string(char * string_pointer) 
{
	while(*string_pointer)              //While not null character (end of string)
	{
		while((UCSR0A&(1<<UDRE0))==0){} //Wait for register empty flag
		UDR0 = *string_pointer;         //Send what's at the string pointer to serial data register
		string_pointer++;               //Increment string pointer to go to next letter in string
	}
}

/************************************************************************
Test serial sprinting accross USART 0
    Will initialise USART 0 if not initialised
Input: None
Output: None
************************************************************************/
void test_print(void)
{
    if(UBRR0 != BAUD_PRESCALE)      //Check USART prescale set
    {
        //Run initialisation if not set and print test string
        serial0_init();
        serial0_print_string("Serial 0 was not initialised. Now initialised and working");
    }
    else
    {
        //Print test string
        serial0_print_string("Serial 0 is working");
    }
}

//Define global variable
volatile uint32_t microseconds = 0;

/************************************************************************
Initialise microsecond timer using timer 1
    Increments in 1000 microsecond intervals
    Tracks microseconds using counter register
Input: None
Output: None
************************************************************************/
void micros_init(void)
{
	cli();                          //Disable global interrupts
    TCCR1A = 0;                     //No pin outputs required
	TCCR1B = (1<<WGM12);            //Set CTC mode
	TCNT1 = 0;                      //Reset timer counter
	OCR1A = 15999;                  //Set comparison register A for 1000 microsecond intervals
	TIMSK1 |= (1<<OCIE1A);          //Set Output Compare Interrupt Enable 1 A
	TCCR1B |= (1<<CS10);            //Set prescaler to 1, starting timer
	sei();                          //Enable global interrupts
}

/************************************************************************
Reset microseconds timer
    Reset counter register and variable
Input: None
Output: None
************************************************************************/
void micros_reset(void)
{
    uint8_t oldSREG = SREG;
    TCNT1 = 0;
    microseconds = 0;
    SREG = oldSREG;
}

/************************************************************************
Returns the current microseconds count
Input: None
Output: uint32_t microseconds
************************************************************************/
uint32_t micros_now(void)
{
	uint32_t m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = microseconds;              //Assign microseconds
    m += (TCNT1>>4);               //Add counter value to microseconds, adjust for clock speed

	SREG = oldSREG;

	return m;
}

/************************************************************************
Timer 1 compare a interrupt service routine
    Increments global microseconds variable in 5 microsecond intervals
************************************************************************/
ISR(TIMER1_COMPA_vect)
{
    microseconds += 1000;              //increment microseconds
}


/***********************************************************************/

#define  t1 5
#define  t2 50
#define  t3 200 
#define  constant 344.8*100/1000000/2
#define  SIG_PIN     PB1

uint16_t ping_sensor(void){
        
        // Sets PORTB into input mode as a DELAY for the PING))) sensor
        // before the next measurement
        PORTB &= ~(1<<SIG_PIN);         // Writes a LOW value to PB1 
        DDRB &= ~(1<<SIG_PIN);          // Puts PB1 in PORTB into input mode (LOW)
        micros_reset();                 // Resets the microseconds timer to 0
        while((micros_now()) < t3){}    // Polling the code until the microsecond
                                        // timer is greater than t3 (200)
        

        // Sends out the Input Trigger Pulse from PB1 to the PING))) sensor
        // with typial pulse time of 5 microseconds
        DDRB |= (1<<SIG_PIN);           // Puts PB1 in PORTB into output mode (HIGH)
        PORTB |= (1<<SIG_PIN);          // Writes a HIGH value to PB1
        micros_reset();                 // Resets the microseconds timer to 0
        while((micros_now()) < t1){}    // Polling the code until the microsecond
                                        // timer is greater than t1 (5)
        
        // Sets PB1 to input without an internal pull-up
        PORTB &= ~(1<<SIG_PIN);         // Writes a LOW value to PB1
        DDRB &= ~(1<<SIG_PIN);          // Puts PB1 in PORTB into input mode (LOW)

        // Polls the code until a HIGH value is read at input pin PB1
        // OR the microseconds timer becomes greater than or equal to
        // t2 (50)
        while((!(PINB & (1<<SIG_PIN))|| (micros_now() < t2))){} 
       
        if(!(PINB & (1<<SIG_PIN)))      // If no signal is read at PB1
        {
            serial0_print_string("No object detected!");
            return 0xFFFF;
        }
        else
        {
            micros_reset();                 // Reset microseconds timer
            while(PINB & (1<<SIG_PIN)){}    // Polls the code until a LOW signal is
                                            // read at PB1
            return(micros_now()*constant);  // Returns microseconds reading multiplied
                                            // by a constant
            
        }
}

int main(void)
{
    serial0_init();
    micros_init();
    uint16_t distance = 0;
    char serial_string[16] = {0};
    DDRB |= (1<<PB0);
    PORTB = 0; 

    while(1)
    {
        distance = ping_sensor();
        if (distance != 0xFFFF)
        {
            sprintf(serial_string,"%4u cm\n",distance);
            serial0_print_string(serial_string);
            if(distance < 100)
            {
                PORTB |= (1<<PB0);
            }
            else
            {
                PORTB &= ~(1<<PB0);
            }
        }
  
    }
    return(1);
}//end main 