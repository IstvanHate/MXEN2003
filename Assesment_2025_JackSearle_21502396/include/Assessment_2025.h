#ifndef ASSESSMENT_2025_H
#define ASSESSMENT_2025_H

//Libraries to include
#include <avr/io.h>        // I/O port definitions
#include <avr/interrupt.h> // Interrupt handling
#include <stdbool.h>       // Boolean type definitions
#include <stdlib.h>        // Standard library functions
#include <util/delay.h>    // Utility for delay functions
#include <stdio.h>         // Standard input/output functions

//Function prototypesz
void serial0_init(void);                            // Initializes serial communication.
void serial0_print_string(char *string_pointer);    // Prints a string via USART 0.
void ping_timer_init(void);                         // Initializes the timer for the PING))) sensor.
void ping_sensor_start(void);                       // Starts the PING))) sensor process.

#endif // ASSESSMENT_2025_H
