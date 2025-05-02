/*
 * Serial.h
 *
 * Created: 2/03/2015 5:41:43 PM
 *  Author: Robert
 */ 


#ifndef SERIAL_H_
#define SERIAL_H_

//include libraries
# include <avr/io.h>

//macros
# define USART_BAUDRATE 9600
# define BAUD_PRESCALE ((((F_CPU/16)+(USART_BAUDRATE/2))/(USART_BAUDRATE))-1)

//function prototypes
void serial0_init(void);
void serial0_print_string(char * string_pointer);
void serial0_write_byte(uint8_t data_byte);

void serial1_init(void);
void serial1_print_string(char * string_pointer);
void serial1_write_byte(uint8_t data_byte);

void serial2_init(void);
void serial2_print_string(char * string_pointer);
void serial2_write_byte(uint8_t data_byte);

void serial3_init(void);
void serial3_print_string(char * string_pointer);
void serial3_write_byte(uint8_t data_byte);

#endif /* SERIAL_H_ */
