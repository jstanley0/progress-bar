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
#include <avr/pgmspace.h>
#include <util/delay.h>

static const uint8_t PROGMEM g_ledMask[10] = {
  0b10000000,
  0b01000000,
  0b00100000,
  0b00010000,
  0b00001000,
  0b00000100,
  0b00000010,
  0b00010000,
  0b01000000,
  0b00100000
};

volatile uint8_t *led_port(uint8_t i)
{
    return (i < 7) ? &PORTB : &PORTD;
}

void io_init()
{
    DDRB  = 0b11111110;
    PORTB = 0b00000001;
    DDRD  = 0b01110000;
    PORTD = 0b00000000;
}

void set_led(uint8_t i)
{
    *led_port(i) |= pgm_read_byte(&g_ledMask[i]);
}

void clear_led(uint8_t i)
{
    *led_port(i) &= ~pgm_read_byte(&g_ledMask[i]);
}

void clear_leds()
{
    PORTB &= 0b00000001;
    PORTD = 0;
}

void set_leds()
{
    PORTB |= 0b11111110;
    PORTD |= 0b01110000;
}

void flash_leds()
{
    for(uint8_t i = 0; i < 4; ++i)
    {
        clear_leds();
        _delay_ms(100);
        set_leds();
        _delay_ms(200);
    }
    clear_leds();
}

// timer0 provides PWM on the active LED
static volatile uint8_t g_activeLed;
ISR(TIMER0_OVF_vect)
{
    if (g_activeLed < 10) {
        set_led(g_activeLed);
    }
}
ISR(TIMER0_COMPB_vect)
{
    if (g_activeLed < 10) {
        clear_led(g_activeLed);
    }
}

// timer1 fires every 100ms to update the timer value
static volatile uint8_t g_ledIntensity;
ISR(TIMER1_COMPA_vect)
{
    // pressing the button cancels the timer
    // but don't accept an input immediately after starting, as a debouncing measure
    if (((PINB & 1) == 0) && (g_activeLed > 0 || g_ledIntensity > 4)) {
        g_activeLed = 10;
    }
    else if (g_activeLed < 10) {
        OCR0B = g_ledIntensity;
        if (g_ledIntensity++ == 255) {
            set_led(g_activeLed++);
        }
    }
}

void timer_start()
{
    TCCR0B = (1 << CS00) | (1 << CS01);    // enable timer0, 1/64 prescaler (overflows ~60Hz)
    g_ledIntensity = 0;
    TIMSK = (1 << TOIE0) | (1 << OCIE0B);  // enable overflow and output-compare B interrupts

    TCCR1B = (1 << CS11) | (1 << WGM12);   // enable timer1, 1/8 prescaler, CTC mode
    OCR1A = 2941;                          // reset every ~23.5ms, so there are 255 counts per LED
    TIMSK |= (1 << OCIE1A);                // enable output-compare A interrupt
}

void timer_stop()
{
    TCCR0B = 0;
    TCCR1B = 0;
    TIMSK = 0;
}

void run_progress_bar()
{
    g_activeLed = 0;
    io_init();
    timer_start();
    while(g_activeLed < 10) {
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_cpu();
    }
    timer_stop();
    flash_leds();
}

void test_led()
{
    io_init();

    for(uint8_t i = 0; i < 10; ++i) {
        set_led(i);
        _delay_ms(50);
        clear_led(i);
    }
}

void wakeup_init()
{
    PCMSK |= (1 << PCINT0);
    GIMSK |= (1 << PCIE);
    sei();
}

ISR(PCINT_vect)
{
}

int main(void)
{
    test_led();
    sleep_enable();
    wakeup_init();
    for(;;) {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_cpu();
        // we are awakened by a pin-change interrupt
        // run the progress bar only when the button is released
        if ((PINB & 1) == 1) {
            run_progress_bar();
        }
    }
}
