
#include "uart.h"

#define bufferSize 256
#define bufferSizeMask ( bufferSize - 1 )

static volatile uint8_t txHead, txTail, rxHead, rxTail = 0;
static volatile uint8_t txRing[ bufferSize ];
static volatile uint8_t rxRing[ bufferSize ];


void UART_class::begin( uint32_t baudrate ) {
    DDRD |= 0x02; // TX is output
    uint16_t ubrr0 = uint16_t( F_CPU / baudrate / 8 - 1 );
    UBRR0H = ubrr0 >> 8;                                         // MSB
    UBRR0L = ubrr0;                                              // LSB
    UCSR0A |= ( 1 << U2X0 );                                     // async double speed mode
    UCSR0B |= ( 1 << RXEN0 ) | ( 1 << TXEN0 ) | ( 1 << RXCIE0 ); // rx and tx enable //rx interrupt enable
    UCSR0C |= ( 1 << UCSZ00 ) | ( 1 << UCSZ01 );                 // 8 bit data size
    sei();
}


uint8_t UART_class::read( void ) {
    rxTail = ( rxTail + 1 ) & bufferSizeMask;
    return rxRing[ rxTail ];
}


void UART_class::write( uint8_t data ) {
    txHead = ( txHead + 1 ) & bufferSizeMask;
    txRing[ txHead ] = data;
    UCSR0B |= ( 1 << UDRIE0 );
}


void UART_class::print( const char *str ) {
    while ( *str > 0 )
        write( *str++ );
}


bool UART_class::available( void ) { return ( rxTail != rxHead ); }


ISR( USART_UDRE_vect ) {
    txTail = ( txTail + 1 ) & bufferSizeMask;
    UDR0 = txRing[ txTail ];
    if ( txTail == txHead )
        UCSR0B &= ~( 1 << UDRIE0 ); // exit interrupt
}


ISR( USART_RX_vect ) {
    rxHead = ( rxHead + 1 ) & bufferSizeMask;
    rxRing[ rxHead ] = UDR0;
}


UART_class UART;
