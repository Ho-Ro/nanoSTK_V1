#pragma once

void pinModeInput( uint8_t pin );
void pinModeInputPullup( uint8_t pin );
void pinModeOutput( uint8_t pin );
int digitalRead( uint8_t pin );
void digitalWrite( uint8_t pin, uint8_t value );
void pulse( int8_t pin, int8_t times );
void adc_init( void );
uint16_t read_adc( uint8_t channel );
void pwm_init( void );
void pwm_out( uint8_t percent );
