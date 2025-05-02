//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file


//file scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;


int main(void)
{
	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
	adc_init();
	lcd_init();
	_delay_ms(20);

	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// data bytes sent
	int16_t cameraJoyHor = 0, motorJoyHor = 0, motorJoyVer;
	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	milliseconds_init();
	sei();
	
	while(1)
	{
		current_ms = milliseconds_now();
		//sending section
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
		{

			motorJoyHor = adc_read(14)*0.2475;
			motorJoyVer = 253-adc_read(15)*0.2475;

			sprintf(serial_string, "Hor: %4u\nVert:%4u",motorJoyHor,motorJoyVer);
         	serial0_print_string(serial_string);

			sendDataByte1 = motorJoyHor;
			sendDataByte2 = motorJoyVer;


			cameraJoyHor = adc_read(1);
			sendDataByte3 = cameraJoyHor;



			
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
			lcd_clrscr();
      if(dataByte1 != 0){
        lcd_goto(0);
        sprintf(serial_string, "Dist: %2ucm", dataByte1);
        //serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received
        lcd_puts(serial_string);
      }
      else{
        lcd_goto(0);
        lcd_puts("Dist: TOO FAR");
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