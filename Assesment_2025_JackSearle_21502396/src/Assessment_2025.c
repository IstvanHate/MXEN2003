#include "Assessment_2025.h"

/************************************************************************
Serial functions included for simulation on TinkerCad
************************************************************************/
//Define USART constants for ATmega328, see ATmega328P datasheet, pg 145
#define USART_BAUDRATE 9600
#define F_CPU 16000000          //Compiler warns for redefining F_CPU as I use -wall -pedantic, imma just let it be
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

//********************************************************************//
//*************************Comment Below Here*************************//
//********************************************************************//

#define STUDENT_ID 22221582

/***********************************************************************
Initialise ping timer
See ATmega328P datasheet for register descriptions, pg 108
Input:      None
Output:     None
************************************************************************/
void ping_timer_init(void)
{
	cli();                          //Disable interrupts
    TCCR1A = 0;                     //Set all bits to 0
	TCCR1B = (1<<WGM12)|(1<<WGM13); //set Timer/Counter Mode of Operation to CTC, TOP value is ICR1
	TCNT1 = 0;                      //Set counter value to 0
	ICR1 = 65535;                   //define timer TOP value
	TIMSK1 |= (1<<TOIE1);           //Overflow Interrupt Enabled
	TCCR1B |= (1<<CS10);            //prescaler to 1 (no prescaling)
	sei();                          //Enable interrupts
}

//PING))) Sensor Pins
#define PING_SENSOR_PIN PD2                     // I/O Pin the sensor is connected to (use one capable of global ints)
#define PING_SENSOR_PORT PORTD                  // Pot the connected pin is in
#define PING_SENSOR_DDR DDRD                    // Corresponding data direction registor for connected pin
#define PING_SENSOR_200_COUNT 200*16-1         // Delay before next measurement
#define PING_SENSOR_5_COUNT 5*16-1           //tOUT
#define PING_SENSOR_850_COUNT (750+100)*16-1   //tHOLDOFF
#define PING_SENSOR_NR_CODE 65535             //Reserved for no response
#define PING_SENSOR_EPE_CODE 65534             //Reserved for echo pulse error
#define CONVERTING_CONSTANT 348 /2                 //I assumed 21 degrees celcsius (c = 331.5 + (0.6 * Temp) m/s), then halved as the echo goes back and forth

//Global variables for PING))) Sensor
volatile uint16_t pingMicros;                   //accumulated microseconds between overflows (4ms at a time)
volatile uint32_t pingValue;                    //time (µs) between sending and recieving echo
volatile bool newReading;                       //new reading ready to be recorded

/***********************************************************************
Interupt vector run when the timer overflows
Handles timer overflow, records cumulated time in ms
Input:      None
Output:     None
************************************************************************/
ISR(TIMER1_OVF_vect)
{
    if(!newReading)
    {
      pingMicros += 4000;       //increments the total microseconds passed between overflow
      if(pingMicros > 18500)    //resets timer as tIN-MAX is 18.5 ms
      {
          newReading = true;
          pingMicros = PING_SENSOR_EPE_CODE;  //throws "echo pulse error" code
      }
    }
}

/***********************************************************************
Compare vector for to handle OCR1A interrupts
See PING))) Ultrasonic Distance Sensor (#28015) datasheet for Communication Protocol, pg2
Input:      None
Output:     None
************************************************************************/
ISR(TIMER1_COMPA_vect)
{
    if(OCR1A == PING_SENSOR_200_COUNT)            //triggers after 200µs (delay before next measurement)
    {
        PING_SENSOR_DDR |= (1<<PING_SENSOR_PIN);   //sets port D DDR PD2 to write
        PING_SENSOR_PORT |= (1<<PING_SENSOR_PIN);  //sets PD2 to HIGH
        OCR1A = PING_SENSOR_5_COUNT;            //set the compare register to run next satement after delay
        TCNT1 = 0;                                 //reset timer to zero
    }
    else if(OCR1A == PING_SENSOR_5_COUNT)       //triggers after 5µs(input trigger pulse)
    {
        PING_SENSOR_DDR &= ~(1<<PING_SENSOR_PIN);  //sets PD2 to read
        PING_SENSOR_PORT &= ~(1<<PING_SENSOR_PIN); //sets PD2 to LOW
        OCR1A = PING_SENSOR_850_COUNT;            //sets compare register to run next section after delay
        TCNT1 = 0;                                 //Resets timer
        EICRA |= (1<<ISC01)|(1<<ISC00);            //Sets Interupt 0 sense control to rising edge
      	EIFR = (1<<INTF0);                         //clears Exterinal interupt flag 0
        EIMSK |= (1<<INT0);                        //enables external pin interrupt
    }
    else if(OCR1A == PING_SENSOR_850_COUNT)       //triggers after 850µs (100µs after the 750µs hold off period)
    {
        pingMicros = PING_SENSOR_NR_CODE;        //throws "no response" error
        newReading = true;
    }
}

/***********************************************************************
INT0 interrupt for recieving sensor readings
First section starts recording the time since the sensor sent a burst, seconds records time since echo was recieved.
Input:      None
Output:     None
************************************************************************/
ISR(INT0_vect)
{
    if(PIND & (1<<PING_SENSOR_PIN))     //perform AND operation to check if specified pin is HIGH
    {
        TCNT1 = 0;                      //resets timer
        pingMicros = 0;                 //resets recorded ping microsends
        TIMSK1 &= ~(1<<OCIE1A);         //disables the Output Compare Match A interrupt
        EICRA &= ~(1<<ISC00);           //clears for next selection
        EICRA |= (1<<ISC01);            // ISC00 = 0 and ISC01 = 1 sets INT0 to trigger on rising edge
    }else                               // PIN val is low,echo has been recieved by the sensor
    {
        pingValue = (pingMicros + (TCNT1>>4)); //Addes microsends passed from prev overflows and TCNT scaled by 1/16
        newReading = true;              //new reading ready to record
        EIMSK &= ~(1<<INT0);            //disable INT0 pin interrupts for now
    }
}

/***********************************************************************
Main Program loop
Input:      None
Output:     None (continous operation until program terminated)
************************************************************************/
int main(void)  //Main function that runs at launch
{
    serial0_init();                 //initialise serial communcation (set registers and BAUD rate)
    ping_timer_init();              //initialise timer used for PING)))
    ping_sensor_start();            //Starts process of getting a sensor ping
    char serial_string[16] = {0};   //Character array to store info sent to serial
    uint16_t distance = 0;          //initialise distance at 0

    while (1)                       //Main program loop
    {
        if(newReading)                                      //Set after reading recieved or error
        {
            if (pingMicros == PING_SENSOR_NR_CODE)        //Start with error checking
            {
                serial0_print_string("No response\n");      //PING))) no rising edge detected, tHOLD-OFF timeout
            }
            else if (pingMicros == PING_SENSOR_EPE_CODE)
            {
                serial0_print_string("Echo Pulse Error\n"); //PING)) no falling edge deteced, tIN-MAX timeout
            }
            else
            {
                distance = CONVERTING_CONSTANT*(pingValue*1e-3); //convert from µs to ms then from ms to mm
              	if(distance < 3000)                             //readings over 30m are inaccurate.
                {
                	sprintf(serial_string,"%4u mm\n",distance); //format string
                	serial0_print_string(serial_string);        //send string to serial
                }
              	else
                {
                	serial0_print_string("Object too far\n");   //error for object out of range
                }
            }
            ping_sensor_start();                                //request next ping
            newReading = false;                                 //no longer have reading ready to interpret
        }
    }
}

/***********************************************************************
Starts the process of sending an input trigger pulse leading to a response in mm (or an error)
See PING))) Ultrasonic Distance Sensor (#28015) datasheet for more
Input:      None
Output:     None
************************************************************************/
void ping_sensor_start(void)
{
    PING_SENSOR_DDR &= ~(1<<PING_SENSOR_PIN);   //Sets pin to input
    PING_SENSOR_PORT &= ~(1<<PING_SENSOR_PIN);  //Sets pin to LOW
    OCR1A = PING_SENSOR_200_COUNT;             //sets compare reg for 200µs delay before next measurement
    TIMSK1 |= (1<<OCIE1A);                      //Enables Output Compare A Match Interrupt
  	TCNT1 = 0;                                  //Resets timer
}