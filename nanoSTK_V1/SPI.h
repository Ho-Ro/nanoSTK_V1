// SPDX-License-Identifier: GPL-3.0-or-later

// low level SPI implementation that replaces the "big" Arduino SPI library
// it provides fast HW SPI and slow bit banging SPI depending on sck_duration


#pragma once


// defines for low level SPI bit bang
#define DDR_MOSI DDRB
#define PORT_MOSI PORTB
#define BIT_MOSI PORTB3
#define DDR_MISO DDRB
#define PIN_MISO PINB
#define BIT_MISO PINB4
#define DDR_SCK DDRB
#define PORT_SCK PORTB
#define BIT_SCK PORTB5


class SPIclass {
  public:
    void init( uint8_t sck_period = 1 );
    void exit( void );
    uint8_t set_sck_duration( uint8_t sck_period = 1 );
    uint8_t get_sck_duration() { return sck_duration; };
    uint8_t transfer( uint8_t data );

  private:
    uint8_t spcr = 0;
    uint8_t spsr = 0;
    uint8_t sck_duration;
};

// the SPI object
extern SPIclass SPI;
