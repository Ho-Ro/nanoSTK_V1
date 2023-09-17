// SPDX-License-Identifier: GPL-3.0-or-later
//
// nanoSTK
// using arduino nano as ISP with STK500 v1 protocol
// program based on:
//
// ArduinoISP
// Copyright (c) 2008-2011 Randall Bohn
// If you require a license, see
// http://www.opensource.org/licenses/bsd-license.php
//
// This software turns the Arduino Nano into an AVR ISP using the following Arduino pins:
//
// By default, the hardware SPI pins MISO, MOSI and SCK are used to communicate
// with the target. On all Arduinos, these pins can be found on the ICSP/SPI header:
//
//               MISO ¹* * 5V (!) Avoid this pin on Due, Zero...
//               SCK   * * MOSI
//        D10 (/RESET) * * GND
//
// Pin 10 is used to reset the target microcontroller.
//
// Required HW change:
// Cut ISP header /RESET connection and connect this pin to D10
//
// Optional HW changes:
// Put an LED (with resistor) on the following pins:
// 9: Heartbeat   - Indicates that the programmer is running
// 8: Error       - Lights up if something goes wrong (use red if that makes sense)
// 7: Programming - In communication with the slave
//


// HW version 2
#define HWVER 2

// SW version 1.20
#define SWMAJ 1
#define SWMIN 20


// This program uses the original "stk500v1" protocol with byte addresses for EEPROM access.
// The modified "arduino" bootloader protocol addresses also the EEPROM word-wise.
// To enable the automatic detection of the "arduino" protocol uncomment the next line.
#define DETECT_ARDUINO_PROTOCOL


// Configure the baud rate:

// #define BAUDRATE  19200
#define BAUDRATE 115200
// #define BAUDRATE 230400


#include "Arduino.h"

#include "SPI.h"

// AVR061 - STK500 Communication Protocol
#include "command.h"

// Configure SPI clock (in Hz).
// E.g. for an ATtiny @ 128 kHz: the datasheet states that both the high and low
// SPI clock pulse must be > 2 CPU cycles, so take 3 cycles i.e. divide target
// f_cpu by 6:
//     #define SPI_CLOCK            ( 128000UL / 6 )
//

// needed for SCK calculation, avrdude uses the stk500 frequency for display
// STK500_XTAL =  7372800U

// fast clock for devices with XTAL (min. 8 MHz)
#define SPI_CLOCK_FAST      ( 8000000UL / 6 )
// This gives a fast duration of 7.5 µs -> * STK500_XTAL / 8000000 -> 6.9
#define SCK_DURATION_FAST   7

// clock slow enough for an ATtiny85 @ 1 MHz selectable with jumper
#define SPI_CLOCK_SLOW      ( 1000000UL / 6 )
// This gives a duration of 60 µs -> * STK500_XTAL / 8000000 -> 55
#define SCK_DURATION_SLOW   55


// start with slow SPI as default, will be set later in setup()
static uint32_t spi_clock = SPI_CLOCK_SLOW;
static uint8_t sck_duration = SCK_DURATION_SLOW;


// Configure which pins to use:

// The standard pin configuration.

// Use pin 10 to reset the target rather than SS
#define RESET     10

// Heartbeat LED
#define LED_HB    9
// Error LED
#define LED_ERR   8
// Programming activity LED
#define LED_PMODE 7

// switch: open = FAST, closed = SLOW
#define SWITCH_FAST   2


// By default, use hardware SPI pins:
#ifndef PIN_MOSI
#define PIN_MOSI 	MOSI
#endif

#ifndef PIN_MISO
#define PIN_MISO 	MISO
#endif

#ifndef PIN_SCK
#define PIN_SCK 	SCK
#endif


void pulse( int pin, int times );


void setup() {
    Serial.begin( BAUDRATE );

    pinMode( LED_PMODE, OUTPUT );
    pulse( LED_PMODE, 2 );
    pinMode( LED_ERR, OUTPUT );
    pulse( LED_ERR, 2 );
    pinMode(LED_HB, OUTPUT);
    pulse(LED_HB, 2);

    pinMode( SWITCH_FAST, INPUT_PULLUP );
}


static bool ISPError = false;
static bool pmode = false;
static bool use_arduino_protocol = false; // use stk500v1 as default
// address for reading and writing, set by 'U' command
static unsigned int here;
static uint8_t buff[256]; // global block storage
// signature bytes
static uint8_t sig[3];

// default wait delay before writing next EEPROM location
// can be adapted according to device signature
const int WAIT_DELAY_EEPROM_DEFAULT = 10;
static int wait_delay_EEPROM = WAIT_DELAY_EEPROM_DEFAULT;


#define beget16( addr ) ( *addr * 256 + *(addr+1) )


typedef struct param {
    uint8_t devicecode;
    uint8_t revision;
    uint8_t progtype;
    uint8_t parmode;
    uint8_t polling;
    uint8_t selftimed;
    uint8_t lockbytes;
    uint8_t fusebytes;
    uint8_t flashpoll;
    uint16_t eeprompoll;
    uint16_t pagesize;
    uint16_t eepromsize;
    uint32_t flashsize;
}
param_t;

param_t param;


// this provides a heartbeat on pin 9, so you can tell the software is running.
uint8_t hbval = 128;
int8_t hbdelta = 8;
void heartbeat() {
  static unsigned long last_time = 0;
  unsigned long now = millis();
  if ((now - last_time) < 40) {
    return;
  }
  last_time = now;
  if (hbval > 192) {
    hbdelta = -hbdelta;
  }
  if (hbval < 32) {
    hbdelta = -hbdelta;
  }
  hbval += hbdelta;
  analogWrite(LED_HB, hbval);
}

static bool rst_active_high;


void reset_target( bool reset ) {
    digitalWrite( RESET, ( ( reset && rst_active_high ) || ( !reset && !rst_active_high ) ) ? HIGH : LOW );
}


void loop( void ) {
    // is pmode active?
    if ( pmode ) {
        digitalWrite( LED_PMODE, HIGH );
    } else {
        digitalWrite( LED_PMODE, LOW );
    }
    // is there an error?
    if ( ISPError ) {
        digitalWrite( LED_ERR, HIGH );
    } else {
        digitalWrite( LED_ERR, LOW );
    }

    // light the heartbeat LED
      heartbeat();
    if ( Serial.available() ) {
        avrisp(); // process STK command
    }
}


// measure the 3V3 voltage and calculate Vcc, return 10 * Vcc
uint8_t get_V_target_10() {
    // Vref is Vcc; analogRead( Vcc ) -> 1023
    // Vcc = 3.3V * round( 1023 / v33 )
    uint16_t v33 = analogRead( A0 ); // about 700
    return ( 33 * 1023L + v33 / 2 ) / v33;
}


uint8_t getch() {
    while ( !Serial.available() )
        ;
    return Serial.read();
}


void fill( int n ) {
    for ( int x = 0; x < n; x++ ) {
        buff[x] = getch();
    }
}


#define PTIME 30
void pulse( int pin, int times ) {
    do {
        digitalWrite( pin, HIGH );
        delay( PTIME );
        digitalWrite( pin, LOW );
        delay( PTIME );
    } while ( times-- );
}


// transfer 4 bytes via ISP, returns 1 byte (last response)
// values according data sheet section "Serial Programming Instruction Set"
uint8_t spi_transaction( uint8_t a, uint8_t b, uint8_t c, uint8_t d ) {
    SPI.transfer( a );
    SPI.transfer( b );
    SPI.transfer( c );
    return SPI.transfer( d );
}


void empty_reply() {
    if ( Sync_CRC_EOP == getch() ) {
        Serial.write( Resp_STK_INSYNC );
        Serial.write( Resp_STK_OK );
    } else {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
    }
}


void byte_reply( uint8_t b ) {
    if ( Sync_CRC_EOP == getch() ) {
        Serial.write( Resp_STK_INSYNC );
        Serial.write( b );
        Serial.write( Resp_STK_OK );
    } else {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
    }
}


void get_parameter( uint8_t c ) {
    switch ( c ) {
        case Parm_STK_HW_VER:       // 0x80
            byte_reply( HWVER );
            break;
        case Parm_STK_SW_MAJOR:     // 0x81
            byte_reply( SWMAJ );
            break;
        case Parm_STK_SW_MINOR:     // 0x82
            byte_reply( SWMIN );
            break;
        case Parm_STK_VTARGET:      // 0x84
            byte_reply( get_V_target_10() );
            break;
        case Parm_STK_VADJUST:      // 0x85
            byte_reply( 0 );
            break;
        case Parm_STK_SCK_DURATION: // 0x89
            byte_reply( sck_duration );
            break;
        case Parm_STK_PROGMODE:
            byte_reply( 'S' );      // serial programmer
            break;
        case Param_STK500_TOPCARD_DETECT:
            byte_reply( 0x03 );     // no top card
            break;
        default:
            byte_reply( 0 );
    }
}


void set_parameters() {
    // call this after reading parameter packet into buff[]
    param.devicecode = buff[0];
    param.revision   = buff[1];
    param.progtype   = buff[2];
    param.parmode    = buff[3];
    param.polling    = buff[4];
    param.selftimed  = buff[5];
    param.lockbytes  = buff[6];
    param.fusebytes  = buff[7];
    param.flashpoll  = buff[8];
    // ignore buff[9] (= buff[8])
    // following are 16 bits (big endian)
    param.eeprompoll = beget16( &buff[10] );
    param.pagesize   = beget16( &buff[12] );
    param.eepromsize = beget16( &buff[14] );

    // 32 bits flashsize (big endian)
    param.flashsize = buff[16] * 0x01000000
                      + buff[17] * 0x00010000
                      + buff[18] * 0x00000100
                      + buff[19];

    // AVR devices have active low reset, AT89Sx are active high
    rst_active_high = ( param.devicecode >= 0xe0 );
}


void start_pmode() {

    // set EEPROM delay to the default value
    wait_delay_EEPROM = WAIT_DELAY_EEPROM_DEFAULT;

    // fast mode / slow mode ?
    if ( digitalRead( SWITCH_FAST ) ) { // open: FAST, closed: SLOW
        spi_clock = SPI_CLOCK_FAST;
        sck_duration = SCK_DURATION_FAST;
    } else {
        spi_clock = SPI_CLOCK_SLOW;
        sck_duration = SCK_DURATION_SLOW;
    }

    // set default stk500v1 protocol
    use_arduino_protocol = false;

    for ( uint8_t iii = 0; iii < 3; ++iii )
        sig[iii] = 0; // clear signature bytes

    // Reset target before driving PIN_SCK or PIN_MOSI

    // SPI.begin() will configure SS as output, so SPI master mode is selected.
    // We have defined RESET as pin 10, which for many Arduinos is not the SS pin.
    // So we have to configure RESET as output here,
    // (reset_target() first sets the correct level)
    reset_target( true );
    pinMode( RESET, OUTPUT );
    SPI.begin();
    SPI.beginTransaction( SPISettings( spi_clock, MSBFIRST, SPI_MODE0 ) );

    // See AVR datasheets, chapter "SERIAL_PRG Programming Algorithm":

    // Pulse RESET after PIN_SCK is low:
    digitalWrite( PIN_SCK, LOW );
    delay( 20 ); // discharge PIN_SCK, value arbitrarily chosen
    reset_target( false );
    // Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
    // speeds above 20 KHz
    delayMicroseconds( 100 );
    reset_target( true );

    // Send the enable programming command:
    delay( 50 ); // datasheet: must be > 20 msec
    spi_transaction( 0xAC, 0x53, 0x00, 0x00 );  // Programming enable
    pmode = true;
}


void end_pmode() {
    SPI.end();
    // We're about to take the target out of reset so configure SPI pins as input
    pinMode( PIN_MOSI, INPUT );
    pinMode( PIN_SCK, INPUT );
    reset_target( false );
    pinMode( RESET, INPUT );
    pmode = false;
    use_arduino_protocol = false; // switch back to default stk500v1 protocol
    for ( uint8_t iii = 0; iii < 3; ++iii )
        sig[iii] = 0; // clear signature bytes
}


void universal() {
    fill( 4 );
    uint8_t reply = spi_transaction( buff[0], buff[1], buff[2], buff[3] );
    byte_reply( reply );
    if ( 0x30 == buff[0] && buff[2] < 3 ) // read one signature byte
        sig[buff[2]] = reply;
    if ( sig[0] && sig[1] && sig[2] ) // all sig bytes available
        hack_eeprom_delay(); // shorter delay for newer parts
}


void load_prog_mem_page( uint8_t hilo, unsigned int addr, uint8_t data ) {
    spi_transaction( 0x40 + 8 * hilo,
                     addr >> 8 & 0xFF,
                     addr & 0xFF,
                     data );
}


void write_program_memory_page( unsigned int addr ) {
    spi_transaction( 0x4C, ( addr >> 8 ) & 0xFF, addr & 0xFF, 0 );
}


unsigned int current_page() {
    if ( param.pagesize == 32 ) {
        return here & 0xFFFFFFF0;
    }
    if ( param.pagesize == 64 ) {
        return here & 0xFFFFFFE0;
    }
    if ( param.pagesize == 128 ) {
        return here & 0xFFFFFFC0;
    }
    if ( param.pagesize == 256 ) {
        return here & 0xFFFFFF80;
    }
    return here;
}


void write_flash( int length ) {
    fill( length );
    if ( Sync_CRC_EOP == getch() ) {
        Serial.write( Resp_STK_INSYNC );
        Serial.write( write_flash_pages( length ) );
    } else {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
    }
}


uint8_t write_flash_pages( int length ) {
    int x = 0;
    unsigned int page = current_page();
    while ( x < length ) {
        if ( page != current_page() ) {
            write_program_memory_page( page );
            page = current_page();
        }
        load_prog_mem_page( LOW, here, buff[x++] );
        load_prog_mem_page( HIGH, here, buff[x++] );
        here++;
    }

    write_program_memory_page( page );

    return Resp_STK_OK;
}


#define EECHUNK (32)
uint8_t write_eeprom( unsigned int length ) {
    unsigned int start = here; // address of EEPROM
    if ( use_arduino_protocol ) {
        // arduino bootloader protocol sends word addresses also for EEPROM
        // calculate the EEPROM byte address
        start *= 2;
    }
    unsigned int remaining = length;
    if ( length > param.eepromsize ) {
        ISPError = true;
        return Resp_STK_FAILED;
    }
    while ( remaining > EECHUNK ) {
        write_eeprom_chunk( start, EECHUNK );
        start += EECHUNK;
        remaining -= EECHUNK;
    }
    write_eeprom_chunk( start, remaining );
    return Resp_STK_OK;
}


// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk( unsigned int start, unsigned int length ) {
    // this writes byte-by-byte, page writing may be faster (4 bytes at a time)
    fill( length );
    for ( unsigned int x = 0; x < length; x++ ) {
        unsigned int addr = start + x;
        spi_transaction( 0xC0, ( addr >> 8 ) & 0xFF, addr & 0xFF, buff[x] );
        delay( wait_delay_EEPROM );
    }
    return Resp_STK_OK;
}


void program_page() {
    char result = ( char ) Resp_STK_FAILED;
    unsigned int length = 256 * getch();
    length += getch();
    char memtype = getch();
    // flash memory @here, (length) bytes
    if ( memtype == 'F' ) {
        write_flash( length );
        return;
    }
    if ( memtype == 'E' ) {
        result = ( char )write_eeprom( length );
        if ( Sync_CRC_EOP == getch() ) {
            Serial.write( Resp_STK_INSYNC );
            Serial.print( result );
        } else {
            ISPError = true;
            Serial.write( Resp_STK_NOSYNC );
        }
        return;
    }
    Serial.write( Resp_STK_FAILED );
    return;
}


uint8_t read_program_memory( uint8_t hilo, unsigned int addr ) {
    return spi_transaction( 0x20 + hilo * 8,
                            ( addr >> 8 ) & 0xFF,
                            addr & 0xFF,
                            0 );
}


char flash_read_page( int length ) {
    for ( int x = 0; x < length; x += 2 ) {
        Serial.write( read_program_memory( LOW, here ) );
        Serial.write( read_program_memory( HIGH, here ) );
        here++;
    }
    return Resp_STK_OK;
}


char eeprom_read_page( int length ) {
    int start = here; // address of EEPROM
    if ( use_arduino_protocol ) {
        // arduino bootloader protocol sends word addresses also for EEPROM
        // calculate the EEPROM byte address
        start *= 2;
    }
    for ( int x = 0; x < length; x++ ) {
        int addr = start + x;
        // read_eeprom_memory
        Serial.write( spi_transaction( 0xA0, ( addr >> 8 ) & 0xFF, addr & 0xFF, 0xFF ) );
    }
    return Resp_STK_OK;
}


void read_page() {
    char result = ( char )Resp_STK_FAILED;
    int length = 256 * getch();
    length += getch();
    char memtype = getch();
    if ( Sync_CRC_EOP != getch() ) {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
        return;
    }
    Serial.write( Resp_STK_INSYNC );
    if ( memtype == 'F' ) {
        result = flash_read_page( length );
    }
    if ( memtype == 'E' ) {
        result = eeprom_read_page( length );
    }
    Serial.print( result );
}


void read_signature() { // used with arduino protocol, stk500v1 uses three universal() calls instead
    if ( Sync_CRC_EOP != getch() ) {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
        return;
    }

#ifdef DETECT_ARDUINO_PROTOCOL
    use_arduino_protocol = true;
#endif

    Serial.write( Resp_STK_INSYNC );
    for ( uint8_t iii = 0; iii < 3; ++iii ) {
        sig[iii] = spi_transaction( 0x30, 0x00, iii, 0x00 );   // read signature bytes
        Serial.write( sig[iii] );
    }
    Serial.write( Resp_STK_OK );
    hack_eeprom_delay();
}


void hack_eeprom_delay() {
    // HACK: set short eeprom delay 4 ms for newer devices instead of 10 ms
    wait_delay_EEPROM = WAIT_DELAY_EEPROM_DEFAULT;              // set default value
    if ( 0x1e == sig[0] ) {                                     // valid AVR parts
        if ( 0x95 == sig[1] ) {                                 // 32K flash parts
            if ( 0x0f == sig[2] || 0x14 == sig[2] )             // m328p, m328
                wait_delay_EEPROM = 4;
        } else if ( 0x94 == sig[1] ) {                          // 16K flash parts
            if ( 0x06 == sig[2] || 0x0b == sig[2] )             // m168, m168p
                wait_delay_EEPROM = 4;
        } else if ( 0x93 == sig[1] ) {                          // 8K flash parts
            if ( 0x0a == sig[2] || 0x0b == sig[2] || 0x0f == sig[2] ) // m88, t85, m88p
                wait_delay_EEPROM = 4;
        } else if ( 0x92 == sig[1] ) {                          // 4K flash parts
            if ( 0x05 == sig[2] || 0x06 == sig[2] || 0x0a == sig[2] ) // m48, t45, m48p
                wait_delay_EEPROM = 4;
        } else if ( 0x91 == sig[1] ) {                          // 2K flash parts
            if ( 0x08 == sig[2] )                               // t25
                wait_delay_EEPROM = 4;
        }
    }
}


////////////////////////////////////
////////////////////////////////////
void avrisp() {
    uint8_t ch = getch();
    switch ( ch ) {
        case Cmnd_STK_GET_SYNC:                     // 0x30 '0' signon
            ISPError = false;
            empty_reply();
            break;
        case Cmnd_STK_GET_SIGN_ON:                  // 0x31 '1'
            if ( getch() == Sync_CRC_EOP ) {
                Serial.write( Resp_STK_INSYNC );    // 0x14
                Serial.print( "AVR ISP" );
                Serial.write( Resp_STK_OK );
            } else {
                ISPError = true;
                Serial.write( Resp_STK_NOSYNC );    // 0x15
            }
            break;
        case Cmnd_STK_GET_PARAMETER:                // 0x41 'A'
            get_parameter( getch() );
            break;
        case Cmnd_STK_SET_DEVICE:                   // 0x42 'B'
            fill( 20 );
            set_parameters();
            empty_reply();
            break;

        case Cmnd_STK_SET_DEVICE_EXT:               // 0x45 'E': extended parameters - ignore for now
            fill( getch() - 1 );                    // stk500.c: buf[0] = n_extparms+1;
            empty_reply();
            break;

        case Cmnd_STK_ENTER_PROGMODE:               // 0x50 'P'
            if ( !pmode ) {
                start_pmode();
            }
            empty_reply();
            break;

        case Cmnd_STK_LEAVE_PROGMODE:               // 0x51 'Q'
            ISPError = false;
            end_pmode();
            empty_reply();
            break;

        case Cmnd_STK_LOAD_ADDRESS:                 // 0x55 'U' set address (word)
            here = getch();
            here += 256 * getch();
            empty_reply();
            break;

        case Cmnd_STK_UNIVERSAL:                    // 0x56 'V'
            universal();
            break;

        case Cmnd_STK_PROG_FLASH:                   // 0x60 '`' STK_PROG_FLASH
            getch(); // low addr
            getch(); // high addr
            empty_reply();
            break;

        case Cmnd_STK_PROG_DATA:                    // 0x61 'a'
            getch(); // data
            empty_reply();
            break;

        case Cmnd_STK_PROG_PAGE:                    // 0x64 'd'
            program_page();
            break;

        case Cmnd_STK_READ_PAGE:                    // 0x74 't'
            read_page();
            break;

        case Cmnd_STK_READ_SIGN:                    // 0x75 'u'
            read_signature();
            break;

        // expecting a command, not Sync_CRC_EOP
        // this is how we can get back in sync
        case Sync_CRC_EOP:                          // 0x20 ' '
            ISPError = true;
            Serial.write( Resp_STK_NOSYNC );
            break;

        // anything else we will return STK_UNKNOWN
        default:
            ISPError = true;
            if ( Sync_CRC_EOP == getch() ) {
                Serial.write( Resp_STK_UNKNOWN );   // 0x12
            } else {
                Serial.write( Resp_STK_NOSYNC );    // 0x15
            }
    }
}
