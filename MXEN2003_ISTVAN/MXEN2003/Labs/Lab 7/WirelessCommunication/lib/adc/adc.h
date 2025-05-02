/*
 * adc.h
 *
 * Created: 02 May 13:56:21
 *  Author: Robert
 */ 


#ifndef ADC_H_
#define ADC_H_

# include <avr/io.h>

void adc_init(void);
uint16_t adc_read(uint8_t channel);

#endif /* ADC_H_ */
