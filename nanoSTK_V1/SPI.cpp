// SPDX-License-Identifier: GPL-3.0-or-later

// low level SPI implementation that replaces the "big" Arduino SPI library
// it provides fast HW SPI and slow bit banging SPI depending on sck_duration


#include <Arduino.h>

#include "SPI.h"


void SPIclass::init( uint8_t sck_period ) {
    // prepare SPI depending on speed
    set_sck_duration( sck_period );
    // Set MOSI and SCK output
    DDR_MOSI |= ( 1 << BIT_MOSI );
    DDR_SCK |= ( 1 << BIT_SCK );
}


void SPIclass::exit( void ) {
    // Disable SPI
    SPCR = 0;
    SPSR = 0;
    // Set MOSI and SCK input
    DDR_MOSI &= ~( 1 << BIT_MOSI );
    DDR_SCK &= ~( 1 << BIT_SCK );
}


// Configure SPI clock
//
// f_cpu < 12 MHz: SPI clock pulse low and pulse high must be 2 CPU cycles
// f_cpu >= 12 MHz: SPI clock pulse low and pulse high must be 3 CPU cycles
//
// fast clock for devices with XTAL (min. 8 MHz)
// 8 MHz / 4 -> 2 MHz
// valid also for devices with 12 MHz
// 12 MHz / 6 -> also 2 MHz
// This gives a period of 0.5 µs -> sck_duration = 1

// calculate SPI timing parameter from sck_period input (1=500ns)
// fastest useful value is 1 (=500ns) -> HW SPI divider: 8
// slowest divider for HW SPI is 128 -> 125 kHz -> 8 µs
// for slower transfer use bit banging
// return the real timing parameter
// e.g. input = 6 (=3µs), next possible value is 4µs, return 8 (=4µs)

uint8_t SPIclass::set_sck_duration( uint8_t sck_period ) {
    if ( !sck_period ) // 0: do not change sck_duration
        return sck_duration;
    sck_duration = sck_period;
    if ( sck_period > 16 ) {       // use bit banging
        SPCR = 0;                  // disable HW SPI
    } else if ( sck_period > 8 ) { // 16 MHz / 128 = 125 kHz -> 8 µs
        sck_duration = 16;
        SPCR = ( 1 << SPE ) | ( 1 << MSTR ) | 0x03;
        SPSR = 0x00;
    } else if ( sck_period > 4 ) { // 16 MHz / 64 = 250 kHz -> 4 µs
        sck_duration = 8;
        SPCR = ( 1 << SPE ) | ( 1 << MSTR ) | 0x03;
        SPSR = 0x01;
    } else if ( sck_period > 2 ) { // 16 MHz / 32 = 500 kHz -> 2 µs
        sck_duration = 4;
        SPCR = ( 1 << SPE ) | ( 1 << MSTR ) | 0x02;
        SPSR = 0x01;
    } else if ( sck_period == 2 ) { // 16 MHz / 16 = 1 MHz -> 1 µs
        sck_duration = 2;
        SPCR = ( 1 << SPE ) | ( 1 << MSTR ) | 0x01;
        SPSR = 0x00;
    } else { // 16 MHz / 8 = 2 MHz -> 500 ns
        sck_duration = 1;
        SPCR = ( 1 << SPE ) | ( 1 << MSTR ) | 0x01;
        SPSR = 0x01;
    }
    return sck_duration;
}


// shift out 8 bit data over SPI and return the received 8 bit response
uint8_t SPIclass::transfer( uint8_t data ) {
    if ( SPCR ) { // use fast HW SPI
        // Start transmission
        SPDR = data;
        // Wait for transmission complete
        while ( !( SPSR & ( 1 << SPIF ) ) )
            ;
        return SPDR;
    } else { // use slow bit banging SPI
        uint8_t response = 0;
        for ( int iii = 0; iii < 8; ++iii ) {
            response <<= 1;
            if ( PIN_MISO & ( 1 << BIT_MISO ) ) // digitalRead( MISO_IN_PIN )
                response |= 1;
            if ( data & 0x80 )
                PORT_MOSI |= ( 1 << BIT_MOSI ); // digitalWrite( MOSI_OUT_PIN, HIGH );
            else
                PORT_MOSI &= ~( 1 << BIT_MOSI ); // digitalWrite( MOSI_OUT_PIN, LOW );
            delayMicroseconds( sck_duration / 2 );
            PORT_SCK |= ( 1 << BIT_SCK ); // digitalWrite( SCK_OUT_PIN, HIGH );
            delayMicroseconds( sck_duration / 2 );
            PORT_SCK &= ~( 1 << BIT_SCK ); // digitalWrite( SCK_OUT_PIN, LOW );
            data <<= 1;
        }
        return response;
    }
}


// provide the SPI object
SPIclass SPI;
