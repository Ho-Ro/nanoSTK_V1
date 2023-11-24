#pragma once

int digiRead( uint8_t pin );
void digiWrite( uint8_t pin, uint8_t value );
void pulse( int8_t pin, int8_t times );
void adc_init( void );
uint16_t read_adc( uint8_t channel );
