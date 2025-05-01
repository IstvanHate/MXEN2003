// Author: Tristan Davies
// Created: Jan 2024
// An example of LCD commands reading from an ADC port

#include "Controller.h"
#include <avr/io.h> // Include for low-level register access
#include <util/delay.h>
#include <stdint.h> // Include for fixed-width integers

// Static function prototypes
void printLCD(int sensor);
void adc_init(void);
//uint16_t read_adc(uint8_t channel);
int sensorVal = 0;



int main(void)
{
    // Variable declarations
    int sensor = 0;

    // Initialisation section, runs once
    adc_init();    // Initialise ADC
    lcd_init();    // Initialise LCD
    serial0_init();
    serial2_init();
    milliseconds_init();

    _delay_ms(20);

    uint32_t current_ms;
    uint8_t joystickSendVal = 0;
    uint8_t lightVal[6] = {};

    // Main loop
    while (1)
    {
        current_ms = milliseconds_now();

        char stick_value[10] = {0};
        sensor = adc_read(1); // Read sensor value from ADC channel 0 (ADC0)
        sensor = sensor * (253.0/1024.0);
        //printLCD(sensor);
        /* sprintf(stick_value, "%u\n", sensor);
        serial0_print_string(stick_value); */

        if(current_ms >= 100)
        {
            joystickSendVal = sensor;
            serial2_write_bytes(1, joystickSendVal);
            current_ms= 0;
        }

        if (serial2_available())
        {
            serial2_get_data(lightVal, 1);
            printLCD(lightVal[0]);

            /*char stick_value[10] = {0};
            sprintf(stick_value, "%u\n", joystickVal[0]);
            serial0_print_string(stick_value); */
        }


    }
    return 0;
} // End main

void printLCD(int sensor)
{
    char lcd_string[33] = {0}; // Initialize character string for LCD formatting
    lcd_goto(0);               // Put cursor at position 0
    lcd_home();                // Same as lcd_goto(0)
    lcd_puts("Sensor value:"); // Print string to LCD first line
    lcd_goto(0x40);            // Put cursor to first character on the second line
    sprintf(lcd_string, "%u", sensor); // Format the sensor value into a string
    lcd_puts(lcd_string);      // Print string to LCD second line
    _delay_ms(200); // Add a small delay to allow readability on the LCD
    lcd_clrscr();              // Clear everything on LCD (optional; remove if unnecessary)
}

/*uint16_t read_adc(uint8_t channel)
{
    // Select ADC channel (0-7) by setting ADMUX[3:0]
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);

    // Start conversion by setting ADSC bit in ADCSRA
    ADCSRA |= (1 << ADSC);

    // Wait until the conversion is complete (ADSC bit will clear)
    while (ADCSRA & (1 << ADSC));

    // Return the 10-bit ADC result (ADCL + ADCH)
    return ADC;
} */

// Mapping function
uint8_t map_value(uint16_t value, uint16_t in_min, uint16_t in_max, uint8_t out_min, uint8_t out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}