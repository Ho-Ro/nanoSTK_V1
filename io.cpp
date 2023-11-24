#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/interrupt.h>

// #include "millis.h"

int digiRead( uint8_t pin ) {
    if ( pin < 8 )
        return PIND & ( 1 << pin );
    else
        return PINB & ( 1 << ( pin - 8 ) );
}


void digiWrite( uint8_t pin, uint8_t value ) {
    if ( pin < 8 ) {
        uint8_t bit = 1 << pin;
        if ( value )
            PORTD |= bit; // digitalWrite( pin, HIGH );
        else
            PORTD &= ~bit; // digitalWrite( pin, LOW );
    } else {
        uint8_t bit = 1 << ( pin - 8 );
        if ( value )
            PORTB |= bit; // digitalWrite( pin, HIGH );
        else
            PORTB &= ~bit; // digitalWrite( pin, LOW );
    }
}


#define PTIME 30
void pulse( int8_t pin, int8_t times ) {
    do {
        digiWrite( pin, 1 );
        _delay_ms( PTIME );
        digiWrite( pin, 0 );
        _delay_ms( PTIME );
    } while ( times-- );
}


void adc_init( void )
{
    ADCSRA |= ( ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) ); // 16Mhz/128 = 125Khz the ADC reference clock
    ADMUX |= ( 1 << REFS0 );                                        // Voltage reference from Avcc (5v)
    ADCSRA |= ( 1 << ADEN );                                        // Turn on ADC
    ADCSRA |= ( 1 << ADSC ); // Do an initial conversion
}


uint16_t read_adc( uint8_t channel ) {
    ADMUX &= 0xF0;           // Clear the older channel that was read
    ADMUX |= channel;        // Defines the new ADC channel to be read
    ADCSRA |= ( 1 << ADSC ); // Starts a new conversion
    while ( ADCSRA & ( 1 << ADSC ) )
        ;        // Wait until the conversion is done
    return ADCW; // Returns the ADC value of the chosen channel
}


