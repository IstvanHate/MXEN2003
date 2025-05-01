#include "robot.h"
#include <avr/io.h> // Include for low-level register access
#include <util/delay.h>



void pwm_init(void)
{

    cli();
    TCCR1A = 0;
    TCCR1A |= (1<<COM1A1);
    TCCR1B = (1<<WGM13) | (1<<CS11);
    ICR1 = 20000;
    DDRB = (1<<PB5);
    OCR1A = 0;
    sei();
}

int main(void)
{
    pwm_init(); // Initialize PWM
    milliseconds_init();
    serial0_init();
    serial2_init();
    adc_init(); // init ADC
    _delay_ms(20); // delay 20 ms

    uint32_t current_ms, last_send_ms;
    uint8_t joystickVal[6] = {};
    uint16_t compValue = 0;
    uint16_t lightVal = 0;

    while (1)
    {
        // Test positions
        current_ms = milliseconds_now();
        lightVal = adc_read(0);
        lightVal = lightVal * (253.0/1024.0);



        if (serial2_available())
        {
            serial2_get_data(joystickVal, 1);
            compValue = joystickVal[0] * (8);

            OCR1A = compValue;

            char stick_value[10] = {0};
            sprintf(stick_value, "%u\n", compValue);
            serial0_print_string(stick_value);

        }

        if (current_ms >= 100)
        {
            serial2_write_bytes(1, lightVal);
            current_ms = 0;
        }
    }

    return 0;
}
