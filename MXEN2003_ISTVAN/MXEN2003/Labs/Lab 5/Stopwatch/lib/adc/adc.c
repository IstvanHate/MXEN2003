/*
 * adc.c
 *
 * Created: 02 May 13:56:09
 *  Author: Robert
 */ 

#include "adc.h"

//set up the analogue to digital converter
void adc_init(void)
{
	ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // set ADC prescaler to 128
	ADMUX = (1<<REFS0); //use internal Vcc AREF, remember to ensure capacitor is attached to AREF pin
	ADCSRB = 0; //ADC0 source, single conversion mode
	ADCSRA |= (1<<ADEN); //enable ADC
}

uint16_t adc_read(uint8_t channel)
{
	if((channel>=0)&(channel<=7))
	{
		ADCSRB &= ~(1<<MUX5); //clear mux5
		ADMUX = (ADMUX&(0xE0))|(channel&0x07); //set remaining mux bits
	}
	else if((channel>=8)&(channel<=15))
	{
		ADCSRB |= (1<<MUX5); //set mux5
		ADMUX = (ADMUX&(0xE0))|(channel&0x07); //set remaining mux bits
	}
	ADCSRA |= (1<<ADSC); //start conversion
	while(ADCSRA&(1<<ADSC)){} //wait for conversion to finish
	return(ADC);
}
