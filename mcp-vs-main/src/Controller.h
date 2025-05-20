#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// Include <stdint.h> to define fixed-width types like uint8_t and uint16_t
#include <stdint.h>

// Include other libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>
#include "serial.h"       // Minimal serial lib
#include "adc.h"          // Minimal ADC lib
#include "milliseconds.h" // Milliseconds timekeeping lib
#include "hd44780.h"      // LCD lib

//struct
typedef struct {
    uint8_t comp_x;
    uint8_t comp_y;
    uint8_t comp_servo;
    bool autonomous;
} Inputs;

// Function prototypes
void printLCD(int sensor);
Inputs readSticks(int pin1, int pin2);   // add more pins for servo?

// Constants
#define BUILD_DATE __TIME__ " " __DATE__"\n"

#endif /* CONTROLLER_H_ */