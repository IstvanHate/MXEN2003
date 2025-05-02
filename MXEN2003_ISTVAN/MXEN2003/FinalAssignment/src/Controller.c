/****************************************************************************
*	Author(s): 	 Istvan Savanyo (21492387), Quinn Brands (2xxxxxxx)
*	File:   	 Controller.c 
*	Description: Code for the CONTROLLER Arduino for the Recue Robot Project
*	References:  Algorithm is built upon the original gitHub code supplied
*                in the MXEN2003 labs.
****************************************************************************/

#include "Controller.h"
# define DEBOUNCE_PERIOD 200

static char serial_string[200] = {0};										// String for serial communication.
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// The recieve dataByte variables.
volatile bool new_message_received_flag=false;								// Boolean flag for recieving a message.
volatile uint8_t autonomousMode=0,turboMode=0;  // Autonomous activator, 0 is OFF, 253 is ON

int main(void)
{
	/*	Initialisation	*/
	serial0_init(); 	// Initialise terminal communication with PC.
	serial2_init();		// Initialise serial communication with the robot's arduino.
	adc_init();			// Analogue-to-Digital converter initialisation.
	lcd_init();			// LCD Screen initialisation.
	_delay_ms(20);		// 20ms delay.
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0, sendDataByte5=0;	// The sent dataByte variables.
	uint8_t frontSensor=0,leftSensor=0,rightSensor=0;
	int16_t cameraJoyHor=0, motorJoyHor=0, motorJoyVer=0;							// Joystick input variables.
	uint32_t current_ms=0, last_send_ms=0;											// Variables used for timing the serial send.
	UCSR2B |= (1 << RXCIE2); 	// Enable the USART Receive Complete interrupt (USART_RXC).
	
	DDRD = 0;						// Put PORTD into input mode.
	PORTD |= (1<<PD2)|(1<<PD0);		// Enable internal pullup on PD2 (RXD1)

	EICRA |= (1<<ISC01)|(1<<ISC11)|(1<<ISC21);
	EIMSK |= (1<<INT0)|(1<<INT1)|(1<<INT2);

	milliseconds_init();		// Microsecond timer initialisation (Timer 5 on ATMega).
	
	sei();    					// Enable interrupts.


	/*	Start of Looping Algorithm	*/
	while(1)
	{
		/*	Sending Section	*/
		current_ms = milliseconds_now();		// Gets current time on the micros timer.
		if(current_ms-last_send_ms >= 100) 		// Sending rate controlled here one message every 100ms (10Hz)
		{
			/*	Reading/Modifying Joystick Input  */
			
			motorJoyHor = adc_read(14)*0.2475;			// Modified to be within a range of 253.
			motorJoyVer = 253-adc_read(15)*0.2475;		// Modified so pushing UP is MAX (253). Joystick is flipped on controller.
			cameraJoyHor = adc_read(1)*0.2475;			// Modified to be within a range of 253.
			
			if(motorJoyHor >= 120 && motorJoyHor <= 132){motorJoyHor=126;}	// If between 124 and 128, assume joystick is in middle position.
			if(motorJoyVer >= 120 && motorJoyVer <= 132){motorJoyVer=126;}
			if(cameraJoyHor >= 120 && cameraJoyHor <= 132){cameraJoyHor=126;}
			/* Debugging Serial Print */

			/*	Setting SEND Databytes */
			sendDataByte1 = motorJoyHor;		// Setting send databyes
			sendDataByte2 = motorJoyVer;
			sendDataByte3 = cameraJoyHor;
			sendDataByte4 = autonomousMode;
			sendDataByte5 = turboMode;

			/* Sending the Databyes	*/	
			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 			// Send start byte = 255
			serial2_write_byte(sendDataByte1); 	// Send first data byte:  Horizontal Motor Joystick
			serial2_write_byte(sendDataByte2); 	// Send second parameter: Vertical Motor Joystick
			serial2_write_byte(sendDataByte3); 	// Send third data byte:  Horizontal Camera Joystick
			serial2_write_byte(sendDataByte4); 	// Send fourth parameter: Autonomous BUTTON Trigger
			serial2_write_byte(sendDataByte5);  // Send fifth parameter:  TURBO BUTTON Trigger
			serial2_write_byte(0xFE); 			// Send stop byte = 254
		}

		//if a new byte has been received
		if(new_message_received_flag) 
		{
			frontSensor = dataByte1;
			leftSensor = dataByte2;
			rightSensor = dataByte3;

			lcd_goto(0);
			sprintf(serial_string,"F=%3u L=%2u R=%2u",frontSensor,leftSensor,rightSensor);
			lcd_puts(serial_string);
			lcd_goto(0x40);
			sprintf(serial_string,"T:%u  A:%u",turboMode,autonomousMode);
			lcd_puts(serial_string);

			
			if(frontSensor==250){
				serial0_print_string("Front: FAR     ");
			}
			else{
				sprintf(serial_string,"Front: %3ucm   ",frontSensor);
				serial0_print_string(serial_string);
			}
			if(leftSensor==99){
				serial0_print_string("Left: FAR     ");
			}
			else{
				sprintf(serial_string,"Left: %3ucm   ",leftSensor);
				serial0_print_string(serial_string);
			}
			if(rightSensor==99){
				serial0_print_string("Right: FAR     \n");
			}
			else{
				sprintf(serial_string,"Right: %3ucm   \n",rightSensor);
				serial0_print_string(serial_string);
			}

			if(dataByte1 != 0){
			}
			else{
			}
			new_message_received_flag=false;	// set the flag back to false
		}
	}
	return(1);
} //end main



static uint32_t previousTime = 0;
ISR(INT2_vect){		 // Toggles AUTONOMOUS MODE
	uint32_t currentTime = milliseconds_now();

	if( (currentTime - previousTime) > DEBOUNCE_PERIOD ){	// 100ms button debounce
		if( autonomousMode == 0 ){		// If NOT autonomous, turn it on.
			autonomousMode = 1;
		}
		else{							// Else, turn it off
			autonomousMode = 0;
		}
    	previousTime = currentTime;
  	}
}
ISR(INT1_vect){		 // Toggles TURBO MODE
	uint32_t currentTime = milliseconds_now();

	if( (currentTime - previousTime) > DEBOUNCE_PERIOD ){	// 100ms button debounce
		if( turboMode == 0 ){		// If NOT TURBO, turn it on.
			turboMode = 1;
		}
		else{							// Else, turn it off
			turboMode = 0;
		}
    	previousTime = currentTime;
  	}
}
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
		case 3: //waiting for third parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for fourth parameter
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