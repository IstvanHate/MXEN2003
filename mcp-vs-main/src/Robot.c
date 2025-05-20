/****************************************************************************
	Project Students: 	 Jack Searle (21502396), Megan Attwill (idk)
	Description: Robot side code, runs on Arduino ATMEGA2560.abort
				 Contains both code to recieve controller single with XBEE
				 Contains autonomous portion of code too.
****************************************************************************/

//header file
#include "Robot.h"
#include "Controller.h"

// serial comms definitions
#define START_BYTE 0xFF
#define STOP_BYTE 0xFE
// PWM definitions
#define PWM_TOP 20000		//180 degrees
#define SERVO_OPEN 920		//30 degrees
#define SERVO_CLOSE 1520	//90 degrees
#define SERVO_PIN PE3

// beacon freq codes + values
#define TIMER_TOP_VALUE 156250
#define TIMER_PERIOD 5


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
uint8_t sendDataByte1=0, sendDataByte2=0; //Photoresistor light level + frequency
uint32_t current_ms=0, last_send_ms=0;	  // used for timing the serial sending
//*********************************************************************************************


void setupMotors()
// set clock mode regs, top value and data direction regs
{
	TCCR3A = (1<<COM3A1)|(1<<COM3B1)|(1<<WGM31);
  	TCCR3B = (1<<WGM33)|(1<<CS31);					//clock mode 8, prescaler of 8 set --> may require changing if messing with the motors

  	ICR3 = PWM_TOP; 								// TOP value

  	DDRE |= (1<<PE4)|(1<<PE5); 						// PWM pins for DC motors to output
	DDRL |= (1<<DDL4)|(1<<DDL5)|(1<<DDL2)|(1<<DDL3);//Digital pins set output low impedence for motors				//Left motor pins set output low impedence
}

void setupServo()
//set pins for servo clock and PWM pin
{
	DDRE |= (1<<PE3);						//set servo PWM pin to output

	TCCR1A = 0; TCCR1A = 0;					//Set both registers all to 0
	TCCR1A |= (1 << COM1A1);				//Clear PWM signal on upcount, set on downcount
	TCCR1B |= (1<<WGM13)|(1<<WGM10);		//Timer mode 9, PWM, phase and frequency correct.
	TCCR1B = (1<<CS11);						//Set PRESCALER to 8
}

void setupBeacon()
{
	// both photoresistors wired into external interrupts
	// sets up clock for overflow period of 5 seconds with rising edge trigger for photoresistors
	cli();
	TCCR4A = 0;
	TCCR4B = (1<<WGM42) | (1<<WGM43) | (1<<CS42)
	TCNT4 = 0;
	ICR4 = TIMER_TOP_VALUE;
	TIMSK4 |= (1<<TOIE4);
	EICRA = (1<<ISC11) | (1<<ISC10); // rising edge trigger mode for EICRA
	sei();
}

void setupRangeSensors ()
{
 //I dread setting up automous
}

void setupSerial ()
{
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Com	ete interrupt (USART_RXC)

	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
}

int main(void)
{
	cli(); 				//disable global interrupts while initialising
	adc_init(); 		// Initialize ADC
	milliseconds_init();
	setupMotors();
	setupServo();
	setupSerial();
	_delay_ms(100);
	sei(); 				//enable interrupts


	while (1) //main loop
	{

		current_ms = milliseconds_now();
		batteryManagement();
		motorDrive(&fc, &rc);
		servoControl(&servo1c);
		beaconFreq();

		//beacon frequency detection not in xbee comms loop


		// For loop for testing purposes: input for servo & motor

		//for (servo1c = 1520;servo1c > 920; servo1c -= 100) {}
		servo1c = 1520;
		//fc = 100;
		//rc = 0;

		if (new_message_received_flag == true) {
			// Process data from servo
			fc = dataBytes[0];
			rc = dataBytes[1];
			servo1c = dataBytes[2];
			AUTONOMOUS = dataBytes[3];

			serialOutput(fc);
			serialOutput(rc);

			new_message_received_flag = false; // Clear the flag
		}

	}

	return (1);
}

void servoDrive(uint8_t servoInput)
//drive servo based on inputted data
{
	static uint8_t servo_angle = 90;	//initial angle as closed
	static int16_t t_pulse;

	if ((servoInput > 140)&(servo_angle < 90))
	{//if thumbstick is pushed forwards and isn't already closed
		servo_angle += 1;	//increment servo angle
	}
	else if ((servoInput < 100)&(servo_angle > 30))
	{//if thumbstick is pushed back and isn't already open
		servo_angle -= 1;	//decrement servo angle
	}

	t_pulse = 10*servo_angle + 620; //relationship between angle and t pulse from datasheet



}

ISR(USART2_RX_vect)  // ISR executed when a new byte is available in the serial buffer

	//interrupt vector raised when UDR2 has a byte ready to be processed, serial comes from XBEE explorer regulated
	//Start byte, 5 data bytes (forward & right values, servo 1 & servo 2 pos change [8 bit values], autonomous mode [bool])

{
	uint8_t serial_byte_in = UDR2;  // Read received byte from USART data reg for USART2 serial port
	typedef enum { WAIT_START, PARAM1, PARAM2, PARAM3, PARAM4, WAIT_STOP } SerialState; //enumurate nums to what they mean
	static SerialState serial_fsm_state = WAIT_START; //static variable to keep track of the state machine
	static uint8_t recvBytes[5];  // Store received input from controller temporarily

	switch (serial_fsm_state)
	{
		case WAIT_START: //WAIT_START = 0, initial value of serial_fsm_state

			//if byte isn't start byte, it won't set serial_fsm_state to the first param
			if (serial_byte_in == START_BYTE)  // Start byte received (255 in dec)
				serial_fsm_state = PARAM1; //1, first actual byte of data
			break;

		case PARAM1: case PARAM2: case PARAM3: case PARAM4:
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
    lm = fc + rc - 256; //Change this back to 253 when data is coming in from serial (max is 253 then)
    rm = fc - rc;

	//set top compare value regs
    OCR3C = (int32_t)abs(lm) * PWM_TOP / 128; //pwm duty cycle as portion of top value
    OCR3B = (int32_t)abs(rm) * PWM_TOP / 128; //change to 126 when coming in from serial

    // Set motor directions with some cheeky compact inline if statements
    if (lm >= 0) { PORTL |= (1<<PL2); PORTL &= ~(1<<PL3); }
    else         { PORTL &= ~(1<<PL2); PORTL |= (1<<PL3); }

    if (rm >= 0) { PORTL |= (1<<PL4); PORTL &= ~(1<<PL5); }
    else         { PORTL &= ~(1<<PL4); PORTL |= (1<<PL5); }
}

void serialOutput(int8_t val) {
	sprintf(serial_string, "Left Motor: %d \n", val);
	serial0_print_string(serial_string);
}

void batteryManagement(void) {

	DDRA = 0xFF;
	uint16_t batteryInput = adc_read(3);
	//debugging print to serial
	/*char serial_battery[50] = {};
	sprintf(serial_battery, "Battery Level: %u\n", batteryInput);
	serial0_print_string(serial_battery); */
	_delay_ms(100);

	if (batteryInput < 700){
		PORTA |= (1<<PA0);
	} else {
		PORTA &= ~(1<<PA0);
	}
}

void servoControl(int16_t *servo1c_ptr){
	int16_t servo1c = *servo1c_ptr;

	OCR3A = (int32_t)servo1c;
}

void beaconFreq(void){
	// adc reading of photoresistor input
	// values to
	uint16_t photoResLeft = adc_read(1);
	uint16_t photoResRight = adc_read(0);

	volatile uint16_t flashCountR;
	volatile uint16_t flashCountL;
	uint16_t frequencyAVG;
	uint16_t frequencyR;
	uint16_t frequencyL;
	char freq_string[50] = {0};

	//debugging print to serial
	/*char serial_photoRes[50] = {};
	sprintf(serial_photoRes, "Light Level Left: %u, Light Level Right: %u\n", photoResLeft, photoResRight);
	serial0_print_string(serial_photoRes);*/

	if(frequencyAVG > 0)
	{
		sprintf(freq_string, "Beacon Frequency: %u Hz\n", frequencyAVG);
		serial0_print_string(freq_string);
	}
	else
	{
		serial0_print_string("No Beacon Detected.\n");
	}

ISR(INT0_vect)
{
	flashCountL += 1;
}

ISR(INT1_vect)
{
	flashCountR +=1;
}

ISR(TIMER4_OVF_vect)
{
	if (flashCountL > 0 && flash_countR > 0)
	{
		frequencyL = flashCountL / TIME_PERIOD;
		frequencyR = flashCountR / TIME_PERIOD;
		frequencyAVG = (frequencyL + frequencyR) / 2;
	}
	flashCountL = 0;
	flashCountR = 0;
}

