//include this .c file's header file
#include "Robot.h"

//static function prototypes, functions only called in this file


//file scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;

static int16_t lm = 0, rm = 0;
static int16_t fc = 0, rc = 0;


int main(void)
{
	// initialisation
	serial0_init();	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
	adc_init();
	_delay_ms(20);
	// or loopback communication to same Arduino
	
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// data bytes sent
  	uint16_t rightSensorVal = 0;
	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	milliseconds_init();


	TCCR3B |= (1<<WGM33)|(1<<CS31);    // Mode 8, PRE=8
	TCCR3A |= (1<<COM3A1)|(1<<COM3B1); // Compare mode (2 compares)
	TCNT3 = 0;
	ICR3 = 4000;
	DDRE |= (1<<PE3)|(1<<PE4);
	OCR3A = 2000;
	OCR3B = 2000;

	DDRA = 0xFF;

 	sei();











	while(1)
	{
		current_ms = milliseconds_now();


		//sending section
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
		{
			//rightSensorVal = 1894/adc_read(0)+2;
			//if (rightSensorVal > 30 || rightSensorVal < 0) {rightSensorVal = 0;}
			//sendDataByte1 = rightSensorVal;
					
			
			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 		//send start byte = 255
			serial2_write_byte(sendDataByte1); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte2); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte3); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte4); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(0xFE); 		//send stop byte = 254
		}
     
		//if a new byte has been received
		if(new_message_received_flag) 
		{
			fc = dataByte1;
			rc = dataByte2;

			lm = (fc-127) + (rc-125);
			rm = (fc-127) - (rc-125);

			OCR3A = (int32_t)abs(lm)*10000/126; //lm speed from magnitude of lm
			OCR3B = (int32_t)abs(rm)*10000/126; //lm speed from magnitude of rm


			if(lm>=0) //if lm is positive
			{
				//set direction forwards
				PORTA |= (1<<PA0);
				PORTA &= ~(1<<PA1);
			}
			else
			{
				//set direction reverse
				PORTA &= ~(1<<PA0);
				PORTA |= (1<<PA1);
			}

			if(rm>=0) //if rm is positive
			{
				//set direction forwards
				PORTA |= (1<<PA2);
				PORTA &= ~(1<<PA3);
			}
			else
			{
				//set direction reverse
				PORTA &= ~(1<<PA2);
				PORTA |= (1<<PA3);
			}

			new_message_received_flag=false;	// set the flag back to false
		}
	}
	return(1);
} //end main








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
		serial_fsm_state=1;
	}
}