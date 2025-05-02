/****************************************************************************
	Project Students: 	 Jack Searle (21502396), Megan Attwill (idk)
	Description: Robot side code, runs on Arduino ATMEGA2560.abort
				 Contains both code to recieve controller single with XBEE
				 Contains autonomous portion of code too.
****************************************************************************/

//header file
#include "Robot.h"
#define START_BYTE 0xFF
#define STOP_BYTE 0xFE
#define PWM_TOP 20000
#define SERVO_MAX 180
#define SERVO_MIN 0
#define SERVO_MID 90
#define SERVO_OPEN 0
#define SERVO_CLOSE 180
#define SERVO1_PIN 0
#define SERVO2_PIN 1
#define SERVO1_PWM OCR1A
#define SERVO2_PWM OCR1B

//declare file scope variables
//*****************************************************************************************
//motor movement variables
static int16_t lm = 0, rm = 0, fc = 0, rc = 0; //forwards and left components, l and r motor duty
static int8_t servo1c = 0, servo2c = 0; //servo 1 (horizontal) and servo 2 (open close) components
//serial string
static char serial_string[200] = {};
//declerations for USART function
volatile bool new_message_received_flag = false; //get set as true when UDR2 recieves a byte
volatile uint8_t dataBytes[5];  // Store final message for use in main
volatile bool AUTONOMOUS = false; //autonomous mode turned on or off

//not sending anything to controller LCD atm
uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// data bytes sent
uint32_t current_ms=0, last_send_ms=0;						// used for timing the serial sending
//*********************************************************************************************


void setupMotors()
// set clock mode regs, top value and data direction regs
{
	TCCR3A = (1<<COM3A1)|(1<<COM3B1)|(1<<WGM31);
  	TCCR3B = (1<<WGM33)|(1<<CS30);					//clock mode 8, no prescaling

  	ICR3 = PWM_TOP; 									// TOP value

  	DDRE |= (1<<PE3)|(1<<PE4); 						// PWM pins
	DDRB |= (1<<DDB1)|(1<<DDB0)|(1<<DDB2)|(1<<DDB3);//Digital pins set output low impedence
}

void setupRangeSensors ()
{
 //I dread setting up automous
}

void setupSerial ()
{
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)

	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial1_init();		// microcontroller communication to/from another Arduino
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

	while (1) //main loop
	{
		current_ms = milliseconds_now();

		// Read analog input value (haven't implemented controller yet)
		fc = adc_read(0) * 0.242; // Scale ADC value for motor control and -8 for MAX 248
		rc = adc_read(1) * 0.242; // Additional input for right vector adjustment

		motorDrive(&fc, &rc);
		serialOutput();

		if (new_message_received_flag == true) {
			// Process data
			fc = dataBytes[0];
			rc = dataBytes[1];
			servo1c = dataBytes[2];
			servo2c = dataBytes[3];
			AUTONOMOUS = dataBytes[4];

			new_message_received_flag = false; // Clear the flag
		}
	}

	return (1);
}

ISR(USART2_RX_vect)  // ISR executed when a new byte is available in the serial buffer

	/*interrupt vector raised when UDR2 has a byte ready to be processed, serial comes from XBEE explorer regulated
	  Start byte, 5 data bytes (forward & right values, servo 1 & servo 2 pos change [8 bit values], autonomous mode [bool])*/

{
	uint8_t serial_byte_in = UDR2;  // Read received byte from USART data reg for USART2 serial port
	typedef enum { WAIT_START, PARAM1, PARAM2, PARAM3, PARAM4, PARAM5, WAIT_STOP } SerialState; //enumurate nums to what they mean
	static SerialState serial_fsm_state = WAIT_START; //static variable to keep track of the state machine
	static uint8_t recvBytes[5];  // Store received input from controller temporarily

	switch (serial_fsm_state)
	{
		case WAIT_START: //WAIT_START = 0, initial value of serial_fsm_state

			//if byte isn't start byte, it won't set serial_fsm_state to the first param
			if (serial_byte_in == START_BYTE)  // Start byte received (255 in dec)
				serial_fsm_state = PARAM1; //1, first actual byte of data
			break;

		case PARAM1: case PARAM2: case PARAM3: case PARAM4: case PARAM5:
			recvBytes[serial_fsm_state - 1] = serial_byte_in; //store byte coming into UDR2 through serial
			serial_fsm_state++;  // Move to the next parameter
			break;

		case WAIT_STOP:
			if (serial_byte_in == STOP_BYTE)  // Stop byte received (254 in dec)
			{
				//This little piece of code means if the last byte recieved wasn't the stop byte, it won't update the values for use in main
				memcpy((void*)dataBytes, (void*)recvBytes, sizeof(recvBytes));  // Copy received data
				new_message_received_flag = true;  // Set flag for main loop processing
			}
			serial_fsm_state = WAIT_START;  // Reset state for next message
			break;
	}
}

void motorDrive(int16_t *fc_ptr, int16_t *rc_ptr)

	/*Takes pointers to fc and rc to avoid use of globl variables
	  Used for both manual and autonomous motor control */

{
	//declare local variables taking value from pointer
    int16_t fc = *fc_ptr;
    int16_t rc = *rc_ptr;

	////left and right motor values from forwards and right vector
    lm = fc + rc - 253;
    rm = fc - rc;

	//set top compare value regs
    OCR3A = (int32_t)abs(lm) * PWM_TOP / 126;
    OCR3B = (int32_t)abs(rm) * PWM_TOP / 126;

    // Set motor directions with some cheeky compact inline if statements
    if (lm >= 0) { PORTB |= (1<<PB0); PORTB &= ~(1<<PB1); }
    else         { PORTB &= ~(1<<PB0); PORTB |= (1<<PB1); }

    if (rm >= 0) { PORTB |= (1<<PB2); PORTB &= ~(1<<PB3); }
    else         { PORTB &= ~(1<<PB2); PORTB |= (1<<PB3); }
}

void serialOutput() {
	sprintf(serial_string, "Left Motor: %d \n", lm);
	serial0_print_string(serial_string);
	sprintf(serial_string, "Right Motor: %d  \n", rm);
	serial0_print_string(serial_string);
};
