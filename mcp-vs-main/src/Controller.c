/****************************************************************************
	Project Students: 	 Jack Searle (21502396), Megan Attwill (idk)
	Description: Controller side code, Runs on Arduino ATMEGA 2560.
				 Takes thumbstick inputs to control 2 x servos and 2 x motors
				 Transmitts through XBEE to robot using USART protocol
****************************************************************************/

//include this .c file's header file
#include "Controller.h"

//file scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0; //data bytes recieved
volatile bool new_message_received_flag=false;


int main(void)
{
	//local variables
	uint32_t current_ms=0, last_send_ms=0; // used for timing the serial sending
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0; //data bytes to be sent
	Inputs CI; //Controller Inputs struct

  	char lcd_string[33] ={0};



	//initialise functions
	lcd_init();
	adc_init();
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
	// or loopback communication to same Arduino

	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)

	milliseconds_init();
	sei();

	while(1)
	{
		current_ms = milliseconds_now();
		//lcd_puts("hello");
		//sending section
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
		{
			// this is just incrementing variables to send for testing purposes
			// you will put the code here that puts the message you want to send into sendDataByte1 and sendDataByte2

			last_send_ms = current_ms;
			CI = readSticks(14, 15); //read the thumbsticks (x,y) and scale to 0-253

			serial2_write_bytes(CI.comp_x, CI.comp_y, CI.comp_servo, CI.autonomous);
			sprintf(serial_string, "sent: 1:%4d, 2:%4d\n", CI.comp_x, CI.comp_y);
			serial0_print_string(serial_string);  // print the received bytes to the USB

		}

		//if a new byte has been received
		if(new_message_received_flag)
		{
			// now that a full message has been received, we can process the whole message
			// the code in this section will implement the result of your message
			sprintf(serial_string, "received: 1:%4d, 2:%4d , 3:%4d , 4:%4d \n", dataByte1, dataByte2, dataByte3, dataByte4);
			serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received
			lcd_home();
			sprintf(lcd_string, "range: %d ", dataByte1);
			lcd_puts(lcd_string);
			new_message_received_flag=false;	// set the flag back to false
		}
	}
	return(1);
} //end main

Inputs readSticks(int pin1, int pin2) //function to read the thumbsticks
{
	//local variables
	uint16_t x_reading;	//for raw 10 bit ADC value
	uint16_t y_reading; //for raw 10 bit ADC value
	//compensated 8bit values to send
	uint8_t x_comp;
	uint8_t y_comp;
	//struct to return
	Inputs CI; //Controller Inputs

	//read the thumbstick values
	x_reading = adc_read(pin1)>>2; // read the x axis of the thumbstick and divide by 4
	y_reading = adc_read(pin2)>>2; // read the y axis of the thumbstick and divide by 4

	//compoensate for dead zone and special values
	if(x_reading < 5) x_comp = 0;			//if less than 5 set to 0
	else if (x_reading > 250) x_comp = 253;	//if greater than 250 set to 253
	else x_comp = x_reading;				//otherwise unchanged

	if(y_reading < 5) y_comp = 0;
	else if (y_reading > 250) y_comp = 253;
	else y_comp = y_reading;

	sprintf(serial_string, "x: %d, y: %d \n", x_comp, y_comp); //print to serial for debugging
	serial0_print_string(serial_string); // print the received bytes to the USB serial to make sure the right messages are received

	//set the struct values
	CI.comp_x = x_comp; //set the x value
	CI.comp_y = y_comp; //set the y value
	CI.comp_servo = 0; //set the servo value to 0 for now
	CI.autonomous = false; //set the autonomous mode to false for now

	//return the compensated values
	return (CI);
} //end readSticksMotor


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
