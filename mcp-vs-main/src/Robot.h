//Example ATmega2560 Project
//File: ATmega2560Project.h
//Author: Robert Howie

#ifndef ROBOT_H_ //double inclusion guard
#define ROBOT_H_

//include standard libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

//include header files
#include "serial.h" //minimal serial lib
#include "adc.h" //minimal adc lib
#include "milliseconds.h" //milliseconds timekeeping lib
#include "hd44780.h" //LCD lib


//constants
#define BUILD_DATE __TIME__ " " __DATE__"\n"

void pwm_init(void);
void motorDrive();
void serialOutput(int32_t val);
void setupMotors();
void setupRangeSensors();
void motorDrive(int16_t *fc_ptr, int16_t *rc_ptr);
void batteryManagement(void);
void beaconFreq(void);
void servoDrive(uint8_t servoInput);


#endif /*ROBOT_H_*/