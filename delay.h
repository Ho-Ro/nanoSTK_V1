#pragma once


#include <avr/interrupt.h>


ISR( TIMER1_COMPA_vect );
void init_millis( void );
unsigned long millis( void );

void delay_us( unsigned int us );
