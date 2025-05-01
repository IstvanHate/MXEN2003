//include this .c file's header file
#include "Robot.h"


//file scope variables
static char serial_string[200] = {};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;
static int16_t lm = 0; //left motor
static int16_t rm = 0; //right motor
static int16_t fc = 0; // forward vector
static int16_t rc = 0; //right vector

uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// data bytes sent
uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial sen


void setupMotors() {

	TCCR3A = (1<<COM3A1)|(1<<COM3B1)|(1<<WGM31);
  	TCCR3B = (1<<WGM33)|(1<<CS30);					//clock mode 8, no prescaling

  	ICR3 = 20000; 									// TOP value

  	DDRE |= (1<<PE3)|(1<PE4); 						// PWM pins
	DDRB |= (1<<DDB1)|(1<<DDB0)|(1<<DDB2)|(1<<DDB3);//Digital pins set output low impedence
}

void setupRangeSensors () {

}

void setupSerial () {

	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)

	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial1_init();		// microcontroller communication to/from another Arduino
	// or loopback communication to same Arduino
}


int main(void)
{
	cli(); 				//disable global interrupts while initialising
	adc_init(); 		// Initialize ADC
	milliseconds_init();
	setupMotors();
	setupSerial();
	_delay_ms(100);
	sei(); 				//enable interrupts

	while (1)
	{
		current_ms = milliseconds_now();

		// Read analog input value (assuming A0 for simplicity)
		dataByte4 = adc_read(0) * 0.242; // Scale ADC value for motor control and -8 for MAX 248
		dataByte3 = adc_read(1) * 0.242; // Additional input for right vector adjustment

		// Drive motors based on analog values
		//motorDrive();
		int rc = (int)dataByte3;
		int fc = (int)dataByte4;

		sprintf(serial_string, "%d\n", rc);
		serial0_print_string(serial_string);

		lm = fc + rc - 248;
		rm = fc - rc;

		OCR3A = (int32_t)abs(lm)*20000/124; //lm speed from magnitude of lm
		if (OCR3A > 20000) {
			OCR3A = 20000;
		}
		OCR3B = (int32_t)abs(rm)*20000/124; //rm speed from magnitude of rm
		if (OCR3B > 20000) {
			OCR3B = 20000;
		}

		if(lm>=0) //if lm is positive
		{
			//set direction forwards (Left motor)
			PORTB |= (1<<PB0);
			PORTB &= ~(1<<PB1);
		}
		else
		{
			//set direction reverse (Lmotor)
			PORTB &= ~(1<<PB0);
			PORTB |= (1<<PB1);
		}

		if(rm>=0) //if rm is positive
		{
			//set direction forwards (Rmotor)
			PORTB |= (1<<PB2);
			PORTB &= ~(1<<PB3);
		}
		else
		{
			//set direction reverse (Rmotor)
			PORTB &= ~(1<<PB2);
			PORTB |= (1<<PB3);
		}

		// Output serial debug info
		//serialOutput();

		// Small delay to stabilize readings
		_delay_ms(100);
	}

	return (1);
}

/*
ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0;		// data bytes received
	static uint8_t serial_fsm_state=0;									// used in the serial receive ISR
	uint8_t	serial_byte_in = UDR2; //move serial byte into variable

	switch(serial_fsm_state) //switch by the current state
	{
		case 0:
		//do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
		break;
		case 1: //waiting for first parameter
		recvByte1 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 2: //waiting for second parameter
		recvByte2 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 3: //waiting for second parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for second parameter
		recvByte4 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 5: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			dataByte3 = recvByte3;
			dataByte4 = recvByte4;

			new_message_received_flag=true;
		}
		// if the stop byte is not received, there is an error, so no commands are implemented
		serial_fsm_state = 0; //do nothing next time except check for start byte (below)
		break;
	}
	if(serial_byte_in == 0xFF) //if start byte is received, we go back to expecting the first data byte
	{
		serial_fsm_state=1; // reset on 255
	}
}
*/
/*
void motorDrive () {
	rc = dataByte3;
	fc = dataByte4;

	sprintf(serial_string, "%d\n", rc);
	serial0_print_string(serial_string);

	lm = fc + rc - 248;
	rm = fc - rc;

	OCR3A = (int32_t)abs(lm)*20000/124; //lm speed from magnitude of lm
	if (OCR3A > 20000) {
		OCR3A = 20000;
	}
	OCR3B = (int32_t)abs(rm)*20000/124; //rm speed from magnitude of rm
	if (OCR3B > 20000) {
		OCR3B = 20000;
	}

	if(lm>=0) //if lm is positive
	{
		//set direction forwards (Left motor)
		PORTB |= (1<<PB0);
		PORTB &= ~(1<<PB1);
	}
	else
	{
		//set direction reverse (Lmotor)
		PORTB &= ~(1<<PB0);
		PORTB |= (1<<PB1);
	}

	if(rm>=0) //if rm is positive
	{
		//set direction forwards (Rmotor)
		PORTB |= (1<<PB2);
		PORTB &= ~(1<<PB3);
	}
	else
	{
		//set direction reverse (Rmotor)
		PORTB &= ~(1<<PB2);
		PORTB |= (1<<PB3);
	}
}*/

void serialOutput() {
	sprintf(serial_string, "Left Motor: %d \n", lm);
	serial0_print_string(serial_string);
	sprintf(serial_string, "Right Motor: %d  \n", rm);
	serial0_print_string(serial_string);
};
