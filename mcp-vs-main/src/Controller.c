/****************************************************************************
	Project Students: 	 Jack Searle (21502396), Megan Attwill (214something)
	Description: Controller side code, Runs on Arduino ATMEGA 2560.abort
				 Takes thumbstick inputs to control 2 x servos and 2 x motors
				 Transmitts through XBEE to robot using USART protocol
****************************************************************************/

//include this .c file's header file
#include "Controller.h"
#define START_BYTE 0xFF
#define STOP_BYTE 0xFE

//file scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;
volatile bool Auto = false;
volatile uint8_t dataBytes[5];
//debounce variables
uint32_t last_debounce = 0;
uint32_t debounce_delay = 50;


int main(void)
{
	// initialisation
	char lcd_string[33] = {0};
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// data bytes sent
	uint32_t current_ms=0, last_send_ms=0;											// used for timing the serial send

	cli(); 				//disable global interrupts while initialising
	lcd_init();
	lcd_home();
	adc_init();
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
	milliseconds_init();

	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	DDRD &= ~(1<<PD2);		//Set PD2 to input
	PORTD |= (1<<PD2);		//enable internal pull-up
	EICRA |= (1<<ISC31);	//setting ISR to falling edge
	EICRA &= ~(1<<ISC30);	//^
	EIMSK |= (1<<INT3);		//enabling INT3
	sei();					//enabling interupts globally

	_delay_ms(20);


	sei();

	while(1)
	{
		current_ms = milliseconds_now();

		//sending section
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
		{
			// this is just incrementing variables to send for testing purposes
			// you will put the code here that puts the message you want to send into sendDataByte1 and sendDataByte2

			//setting bytes
			sendDataByte1 = adc_read(0)/4.1; // 1 HOR
      		sendDataByte2 = adc_read(1)/4.1; // 1 VER
      		sendDataByte3 = adc_read(14)/4.1; // 2 HOR
      		sendDataByte4 = Auto;

			//sending bytes
			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 		//send start byte = 255
			serial2_write_byte(sendDataByte1); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte2); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte3); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte4); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(0xFE); 		//send stop byte = 254

			/*lcd_goto(0);
			sprintf(lcd_string, " 1:%4d, 2:%4d\n", sendDataByte1, sendDataByte2);
			lcd_puts(lcd_string);
			lcd_goto(0x40);
			sprintf(lcd_string, " 3:%4d , 4:%4d\n", sendDataByte3, sendDataByte4);
			lcd_puts(lcd_string); */


		}

		//if a new byte has been received
		if(new_message_received_flag)
		{
			// now that a full message has been received, we can process the whole message
			// the code in this section will implement the result of your message
			sprintf(serial_string, "received: 1:%4d, 2:%4d , 3:%4d , 4:%4d , 5:%4d \n", dataBytes[0], dataBytes[1], dataBytes[2], dataBytes[3], dataBytes[4]);
			serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received
			lcd_goto(0);
			sprintf(lcd_string, "Lit:%3u Freq:%3u", dataBytes[1], dataBytes[0]);
			lcd_puts(lcd_string);
			lcd_goto(0x40);
			sprintf(lcd_string, "L:%2u R:%2u F:%2u", dataBytes[2], dataBytes[3], dataBytes[4]);
			lcd_puts(lcd_string);
			new_message_received_flag=false;	// set the flag back to false
		}
	}
	return(1);
} //end main

ISR(INT3_vect){
    // Software debounce for button on INT3 (D19)
    uint32_t now = milliseconds_now();
    if (now - last_debounce > debounce_delay) {
        Auto = !Auto;
        last_debounce = now;
    }
}

ISR(USART2_RX_vect)  // ISR executed when a new byte is available in the serial buffer

	//interrupt vector raised when UDR2 has a byte ready to be processed, serial comes from XBEE explorer regulated
	//Start byte, 5 data bytes (forward & right values, servo 1 & servo 2 pos change [8 bit values], autonomous mode [bool])

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