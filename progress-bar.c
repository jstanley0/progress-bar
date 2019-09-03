// IRL progress bar for attiny2313
// (c) 2019 by Jeremy Stanley
// Licensed under GNU GPL v3

// Parts list:
// 1x attiny2313 MCU
// 1x 10-LED bar
// 10x suitable resistors
// 1x momentary contact tactile switch

// port assignments:
// PB7..1 (output) = LEDs 0-6
// PD4    (output) = LED 7
// PD6    (output) = LED 8
// PD5    (output) = LED 9
// PB0    (input)  = the button

// many sorry, I originally made PORTB all LED outputs
// then swapped LED 7 with the button because I thought I needed the button on PCINTx
// so now my port assignments are a bit wonky

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

void io_init()
{
	DDRB  = 0b11111110;
	PORTB = 0b00000001;
	DDRD  = 0b01110000;
	PORTD = 0b00000000;
}

void timer_init()
{
    
}

void run_progress_bar()
{
    io_init();
    timer_init();
    
    // TODO yeah
    PORTB |= 0b11111110;
    PORTD |= 0b01110000;
    _delay_ms(200);
    PORTB &= ~0b11111110;
    PORTD &= ~0b01110000;
}

void test_led()
{
    io_init();
    
    for(unsigned char i = 0b10000000; i != 0b00000001; i >>= 1) {
        PORTB = i | 0b00000001;
        _delay_ms(50);
    }
    PORTB = 0b00000001;
    PORTD = 0b00010000;
    _delay_ms(50);
    PORTD = 0b01000000;
    _delay_ms(50);
    PORTD = 0b00100000;
    _delay_ms(50);
    PORTD = 0;
}

void wakeup_init()
{
    PCMSK |= (1 << PCINT0);
    GIMSK |= (1 << PCIE);
}

ISR(PCINT_vect)
{
}

int main(void)
{
    test_led();
    for(;;) {
        wakeup_init();
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
        run_progress_bar();
    }
}
