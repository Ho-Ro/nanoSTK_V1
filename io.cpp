#include <avr/io.h>
#include <util/delay.h>


void pinModeInput( uint8_t pin ) {
    if ( pin < 8 ) {
        DDRD &= ~( 1 << pin ); // pinMode( pin, INPUT );
    } else {
        DDRB &= ~( 1 << ( pin - 8 ) ); // pinMode( pin, INPUT );
    }
}


void pinModeInputPullup( uint8_t pin ) {
    if ( pin < 8 ) {
        DDRD &= ~( 1 << pin ); // pinMode( pin, INPUT );
        PORTD |= ( 1 << pin ); // pinMode( pin, _PULLUP );
    } else {
        DDRB &= ~( 1 << ( pin - 8 ) ); // pinMode( pin, INPUT );
        PORTB |= ( 1 << ( pin - 8 ) ); // pinMode( pin, _PULLUP );
    }
}


void pinModeOutput( uint8_t pin ) {
    if ( pin < 8 ) {
        DDRD |= ( 1 << pin ); // pinMode( pin, OUTPUT );
    } else {
        DDRB |= ( 1 << ( pin - 8 ) ); // pinMode( pin, OUTPUT );
    }
}


int digitalRead( uint8_t pin ) {
    if ( pin < 8 )
        return PIND & ( 1 << pin );
    else
        return PINB & ( 1 << ( pin - 8 ) );
}


void digitalWrite( uint8_t pin, uint8_t value ) {
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
        digitalWrite( pin, 1 );
        _delay_ms( PTIME );
        digitalWrite( pin, 0 );
        _delay_ms( PTIME );
    } while ( times-- );
}


void adc_init( void ) {
    ADCSRA |= ( ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) ); // 16Mhz/128 = 125Khz the ADC reference clock
    ADMUX |= ( 1 << REFS0 );                                        // Voltage reference from Avcc (5v)
    ADCSRA |= ( 1 << ADEN );                                        // Turn on ADC
    ADCSRA |= ( 1 << ADSC );                                        // Do an initial conversion
}


uint16_t read_adc( uint8_t channel ) {
    ADMUX &= 0xF0;           // Clear the older channel that was read
    ADMUX |= channel;        // Defines the new ADC channel to be read
    ADCSRA |= ( 1 << ADSC ); // Starts a new conversion
    while ( ADCSRA & ( 1 << ADSC ) )
        ;        // Wait until the conversion is done
    return ADCW; // Returns the ADC value of the chosen channel
}

// set PWM frequency to 1 kHz (divide F_CPU / 16000)
static const uint16_t pwm_scale = 16000;

void pwm_init( void ) {
    // Fast PWM 1 kHz
    // Clear OC1A on Compare Match / Set OC1A at Bottom; Wave Form Generator: Fast PWM 14, Top = ICR1
    TCCR1A = ( 1 << COM1A1 ) + ( 1 << WGM11 );
    TCCR1B = ( 1 << WGM13 ) + ( 1 << WGM12 ) + ( 1 << CS10 ); // prescaler = none;
    ICR1 = pwm_scale - 1;
    OCR1A = 0;
    DDRB |= ( 1 << PB1 );
}


void pwm_out( uint8_t percent ) {
    uint16_t ocr1a = percent * (pwm_scale / 100);
    if ( ocr1a > pwm_scale - 1 )
        ocr1a = pwm_scale - 1;
    OCR1A = ocr1a;
}
