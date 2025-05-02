/*
 * milliseconds.h
 *
 * Created: 05 May 09:37:34
 *  Author: Robert
 */ 


#ifndef MILLISECONDS_H_
#define MILLISECONDS_H_

#include <avr/io.h>
#include <avr/interrupt.h>

//declare global variable
extern volatile uint32_t milliseconds;

//function prototype
void milliseconds_init(void);
uint32_t milliseconds_now(void);


#endif /* MILLISECONDS_H_ */
