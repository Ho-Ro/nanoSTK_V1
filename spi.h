// SPDX-License-Identifier: GPL-3.0-or-later

// low level SPI implementation that replaces the "big" Arduino SPI library
// it provides fast HW SPI and slow bit banging SPI depending on sck_duration


#pragma once

class SPIclass {
  public:
    void init( uint8_t sck_period = 0 );
    void exit( void );
    uint8_t set_sck_duration( uint8_t sck_period = 0 );
    uint8_t get_sck_duration() { return sck_duration; };
    uint8_t transfer( uint8_t data );

  private:
    uint8_t spcr = 0;
    uint8_t spsr = 0;
    uint8_t sck_duration;
};

// the SPI object
extern SPIclass SPI;
