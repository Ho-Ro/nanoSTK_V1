#pragma once


#include <avr/io.h>
#include <avr/interrupt.h>


class UART_class {
  public:
    void begin( uint32_t baud );
    bool available( void );
    uint8_t read( void );
    void write( uint8_t data );
    void print( const char *str );

  private:
};


ISR( USART_UDRE_vect );

ISR( USART_RX_vect );

extern UART_class UART;
