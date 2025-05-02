/****************************************************************************
*	Author(s): 	 Istvan Savanyo (21492387), Quinn Brands (2xxxxxxx)
*	File:   	 Robot.c 
*	Description: Code for the ROBOT Arduino for the Recue Robot Project
*	References:  Algorithm is built upon the original gitHub code supplied
*                in the MXEN2003 labs.
****************************************************************************/

#include "Robot.h"

static char serial_string[200] = {0};										// String for serial communication.
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0, dataByte5=0;		// The recieve dataByte variables.
volatile bool new_message_received_flag=false;								// Boolean flag for recieving a message.

static int16_t lm = 0, rm = 0;				// Variables used for MOTOR and SERVO control
static int16_t fc = 0, rc = 0, cam = 126;


int main(void)
{
	/*	Initialisation	*/
	serial0_init();		// Initialise terminal communication with PC.
	serial2_init();		// Initialise serial communication with the controller's arduino.
	adc_init();			// Analogue-to-Digital converter initialisation.
	_delay_ms(20);		// 20ms delay.
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// The sent dataByte variables.
  	uint16_t rightSensorVal=0, leftSensorVal=0, frontSensorVal=0;					// All 3 sensor values.
	uint16_t rightSensorAvg=15, leftSensorAvg=15, frontSensorAvg=15;				// Average values sensors (default to 15 at start).
	uint8_t autonomousMode=0,turboMode=0;				// Variables for turning on, and pausing, autonomousMode mode
	uint32_t current_ms=0, last_send_ms=0;											// Variables used for timing the serial send.
	uint16_t batteryADCReadingAvg=485;
	int16_t comp=0,comp2=0;
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC).
	milliseconds_init();	 // Microsecond timer initialisation (Timer 5 on ATMega).

	/*	Initialise the PWM for MOTOR Control [Timer 3] */
	TCCR3B |= (1<<WGM33)|(1<<CS31);     // Mode 8, PRE=8.
	TCCR3A |= (1<<COM3A1)|(1<<COM3B1);  // Compare mode (2 compares).
	TCNT3 = 0;
	ICR3 = 4000;					  	// Setting TOP value.
	DDRE |= (1<<PE3)|(1<<PE4);			// Initialising PWM pins as OUTPUT.
	OCR3A = 2000;						// Setting first compare value.
	OCR3B = 2000;						// Setting second compare value.

	/*  Initialise the PWM for SERVO Control [Timer 1]  */
	TCCR1B |= (1<<WGM13)|(1<<CS11);    	// Mode 8, PRE=8.
  	TCCR1A |= (1<<COM1A1); 				// Compare mode (2 compares).
  	TCNT1 = 0;
  	ICR1 = 20000;						// Setting TOP value.
  	DDRB |= (1<<PB5);					// Initialising PWM pin as OUTPUT.
  	OCR1A = 1500;						// Setting compare value.
	/*	PORT/PIN Initialisation  */
	DDRA = 0xFF;		// For motor H-Bridge control.
	DDRB = 0xFF;		// For battery alert LED
	sei();              // Enable interrupts.


	/*  Start the Looping Algorithm  */
	while(1)
	{

		batteryADCReadingAvg = 0.9*batteryADCReadingAvg + 0.1*adc_read(4);
		if(batteryADCReadingAvg < 380){PORTB |= (1<<PB3);}		// Turn on LED if ADC is below 395 (below 7V volts).
		else{PORTB &= ~(1<<PB3);}								// Else turn it off.




		/*  Sending Section  */
		current_ms = milliseconds_now();		// Gets current time on the micros timer.

		rightSensorAvg = rightSensorAvg*9/10 + adc_read(7)/10;	// Get new sensor values based on average.
		leftSensorAvg = leftSensorAvg*9/10 + adc_read(6)/10;	// Creates smoother/nicer sensor values.
		frontSensorAvg = frontSensorAvg*9/10 + adc_read(5)/10;

		rightSensorVal = 1894/rightSensorAvg+2;		// Convert from ADC value to distance.
		leftSensorVal = 1894/leftSensorAvg+2;
		frontSensorVal = 6600/frontSensorAvg-5;

		if (rightSensorVal > 30 || rightSensorVal < 3) {rightSensorVal = 99;} 	// Sensor distance of 253 means OUT OF RANGE.
		if (leftSensorVal > 30 || leftSensorVal < 3) {leftSensorVal = 99;}	
		if (frontSensorVal > 80 || frontSensorVal < 3) {leftSensorVal = 99;}



		/*  MOTOR SHIT PISS  */
		if(autonomousMode == 0){
			lm = (fc-126) + (rc-126);
			rm = (fc-126) - (rc-126);
			if(cam >= 124 && cam <= 128){
				OCR1A = 0;
			}
			else{
				OCR1A = 1750-(int32_t)cam*3.953/2;
			}
		}
		else{
			comp=leftSensorVal-rightSensorVal;
			comp2=rightSensorVal-leftSensorVal;
			if (rightSensorVal<8){  //left turn
				sprintf(serial_string,"TOO CLOSE TO LEFT");
				serial0_print_string(serial_string);
				lm = -200;
				rm = -200;
			}
			else if (leftSensorVal<8){  // right turn
				sprintf(serial_string,"TOO CLOSE TO RIGHT");
				serial0_print_string(serial_string);
				lm = 200;
				rm = 200;				
			}
			
			else if( (frontSensorVal<20 && (comp>5)) ){			// left turn
				sprintf(serial_string,"LEFT");
				serial0_print_string(serial_string);
				lm = -200;
				rm = -200;
			}
			else if( (frontSensorVal<20 && (comp2>5))){		// right turn
				sprintf(serial_string,"RIGHT");
				serial0_print_string(serial_string);
				lm = 200;
				rm = 200;
			}
			else{			// Default - Forward
			sprintf(serial_string,"Fpoward");
			serial0_print_string(serial_string);
				lm = 127;
				rm = -127;
			}
		}	
		if(autonomousMode!=0){
			OCR3A = (int32_t)abs(lm)*2000/126; //lm speed from magnitude of lm
			OCR3B = (int32_t)abs(rm)*2000/126; //lm speed from magnitude of rm		
		}
		else if(turboMode==0){
			OCR3A = (int32_t)abs(lm)*3000/126; //lm speed from magnitude of lm
			OCR3B = (int32_t)abs(rm)*3000/126; //lm speed from magnitude of rm
		}
		else{
			OCR3A = (int32_t)abs(lm)*20000/126; //lm speed from magnitude of lm
			OCR3B = (int32_t)abs(rm)*20000/126; //lm speed from magnitude of rm
		}
		if(lm>=0){
			PORTA |= (1<<PA0);   // Set direction forwards (if lm positive).
			PORTA &= ~(1<<PA1);
		}
		else{
			PORTA &= ~(1<<PA0);  // Set direction reserse. 
			PORTA |= (1<<PA1);
		}

		if(rm>=0){
			PORTA |= (1<<PA2);   // Set direction forwards (if lm positive).
			PORTA &= ~(1<<PA3);
		}
		else{
			PORTA &= ~(1<<PA2);	 // Set direction reverse.
			PORTA |= (1<<PA3);
		}




		if(current_ms-last_send_ms >= 100) 		// Sending rate controlled here one message every 100ms (10Hz).
		{
			sendDataByte1 = frontSensorVal;
			sendDataByte2 = leftSensorVal;
			sendDataByte3 = rightSensorVal;
			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 			// Send start byte = 255
			serial2_write_byte(sendDataByte1); 	// Send first data byte:  Front Sensor Reading
			serial2_write_byte(sendDataByte2); 	// Send second parameter: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte3); 	// Send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte4); 	// Send second parameter: must be scaled to the range 0-253
			serial2_write_byte(0xFE); 			// Send stop byte = 254
			
			sprintf(serial_string,"R %2u L %2u F%2u\n",rightSensorVal,leftSensorVal,frontSensorVal);
			serial0_print_string(serial_string);
		}
     
		//if a new byte has been received
		if(new_message_received_flag) 
		{
			fc = dataByte1;
			rc = dataByte2;
			cam = dataByte3;
			autonomousMode = dataByte4;
			turboMode = dataByte5;
			new_message_received_flag=false;	// set the flag back to false
		}
	}
	return(1);
} //end main


ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0, recvByte5=0;		// data bytes received
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
		case 3: //waiting for third parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for fourth parameter
		recvByte4 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 5: //waiting for fifth parameter
		recvByte5 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 6: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			dataByte3 = recvByte3;
			dataByte4 = recvByte4;
			dataByte5 = recvByte5;
			
			new_message_received_flag=true;
		}
		// if the stop byte is not received, there is an error, so no commands are implemented
		serial_fsm_state = 0; //do nothing next time except check for start byte (below)
		break;
	}
	if(serial_byte_in == 0xFF) //if start byte is received, we go back to expecting the first data byte
	{
		serial_fsm_state=1;
	}
}