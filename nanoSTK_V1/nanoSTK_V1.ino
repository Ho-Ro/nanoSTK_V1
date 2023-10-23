// SPDX-License-Identifier: GPL-3.0-or-later
//
// nanoSTK_v1
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

#define RESET     10


// HW version 2
#define HWVER 2

// SW version 1.23
#define SWMAJ 1
#define SWMIN 23


// This program uses the original "stk500v1" protocol with byte addresses for EEPROM access.
// The modified "arduino" bootloader protocol addresses also the EEPROM word-wise.
// To enable the automatic detection of the "arduino" protocol uncomment the next line.
#define DETECT_ARDUINO_PROTOCOL


// Configure the baud rate:

// #define BAUDRATE  19200
#define BAUDRATE 115200
// BAUDRATE > 115200 does not work!
// #define BAUDRATE 230400


#include "Arduino.h"

#include "SPI.h"

// AVR061 - STK500 Communication Protocol
#include "AVR061_command.h"

// AVR910 - In-System Programming
#include "AVR910_ISP.h"

// Configure SPI clock (in Hz).
// E.g. for an ATtiny @ 128 kHz: the datasheet states that both the high and low
// SPI clock pulse must be > 2 CPU cycles, so take 3 cycles i.e. divide target
// f_cpu by 6:
//     #define SPI_CLOCK            ( 128000UL / 6 )
//

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

// Optional nanoSTK HW changes:
// Put an LED (with resistor) on the following pins:
// 9: Heartbeat - Indicates that the programmer is running
// the next 4 LEDs are the same as used by ScratchMonkey
// 8: Error     - An Error has occured - clear with programmer reset
// 7: Write     - Writing to the target
// 6: Read      - Reading from the targer
// 5: PMode     - Target in programming mode


#define HEARTBEAT
#ifdef HEARTBEAT
// Heartbeat LED
#define LED_HB      9
#endif

// Error LED
#define LED_ERROR   8

// Writing activity LED
#define LED_WRITE   7

// Reading activity LED
#define LED_READ    6

// Programming mode LED
#define LED_PMODE   5

// switch: open = FAST SPI mode, closed = SLOW SPI mode
#define SPI_SPEED_SELECT 2


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


////////////////////////////////////////////////////////////////////////////////


void setup() {
    Serial.begin( BAUDRATE );

    pinMode( SPI_SPEED_SELECT, INPUT_PULLUP );
    pinMode( LED_PMODE, OUTPUT );
    pulse( LED_PMODE, 2 );
    pinMode( LED_READ, OUTPUT );
    pulse( LED_READ, 2 );
    pinMode( LED_WRITE, OUTPUT );
    pulse( LED_WRITE, 2 );
    pinMode( LED_ERROR, OUTPUT );
    pulse( LED_ERROR, 2 );
#ifdef HEARTBEAT
    pinMode(LED_HB, OUTPUT);
#endif
    select_spi_speed();
}


#define PTIME 30
static void pulse( int pin, int times ) {
    do {
        digitalWrite( pin, HIGH );
        delay( PTIME );
        digitalWrite( pin, LOW );
        delay( PTIME );
    } while ( times-- );
}


static bool ISPError = false;
static bool pmode = false;
static bool use_arduino_protocol = false; // use stk500v1 as default

// address for reading and writing, set by 'U' command Cmnd_STK_LOAD_ADDRESS
static uint16_t here;

// global block storage for cmd, flash and eeprom data
static uint8_t buff[256];

// the three signature bytes
static uint8_t sig[3];

// default wait delay before writing next EEPROM location
// can be adapted according to device signature
const int WAIT_DELAY_EEPROM_DEFAULT = 10;
static int wait_delay_EEPROM = WAIT_DELAY_EEPROM_DEFAULT;


static uint16_t buff_get_16( uint16_t addr ) {
    return buff[ addr ] * 0x100
         + buff[ addr+ 1 ];
}


static uint32_t buff_get_32( uint16_t addr ) {
    return buff[ addr ] * 0x01000000L
         + buff[ addr + 1 ] * 0x00010000L
         + buff[ addr + 2 ] * 0x00000100L
         + buff[ addr + 3];
}


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
    uint8_t eeprompagesize;
    uint16_t eeprompoll;
    uint16_t pagesize;
    uint16_t eepromsize;
    uint32_t flashsize;
}
param_t;

static param_t param;


#ifdef HEARTBEAT
// this provides a heartbeat on pin 9, so you can tell the software is running.
static void heartbeat() {
    static int8_t hbdelta = 8;
    static uint8_t hbval = 128;
    static uint32_t last_time = 0;
    uint32_t now = millis();
    if ((now - last_time) < 40) {
      return;
    }
    last_time = now;
    if (hbval > 160) {
        hbdelta = -hbdelta;
    }
    else if (hbval < 16) {
        hbdelta = -hbdelta;
    }
    hbval += hbdelta;
    analogWrite(LED_HB, hbval);
}
#endif


static bool rst_active_high;



////////////////////////////////////////////////////////////////////////////////


void loop( void ) {

// is there an error?
    if ( ISPError ) {
        digitalWrite( LED_ERROR, HIGH );
    } else {
        digitalWrite( LED_ERROR, LOW );
    }

#ifdef HEARTBEAT
    // light the heartbeat LED
      heartbeat();
#endif

    if ( Serial.available() ) {
        avrisp(); // process STK command
    }
}


////////////////////////////////////////////////////////////////////////////////


static void avrisp() {
    uint8_t ch = getch();
    switch ( ch ) {
        // Use this command to try to regain synchronization when sync is lost.
        // Send this command until Resp_STK_INSYNC is received.
        // Cmnd_STK_GET_SYNC, Sync_CRC_EOP
        case Cmnd_STK_GET_SYNC:                     // 0x30 '0' signon
            ISPError = false;
            empty_reply();
            break;

        // The PC sends this command to check the communication channel.
        // Cmnd_STK_GET_SIGN_ON, Sync_CRC_EOP
        case Cmnd_STK_GET_SIGN_ON:                  // 0x31 '1'
            if ( getch() == Sync_CRC_EOP ) {
                Serial.write( Resp_STK_INSYNC );    // 0x14
                Serial.print( "nanoSTK" );
                Serial.write( Resp_STK_OK );
            } else {
                ISPError = true;
                Serial.write( Resp_STK_NOSYNC );    // 0x15
            }
            break;

        // Get the value of a valid parameter from the STK500 starterkit.
        // If the parameter is not used, the same parameter will be returned
        // together with a Resp_STK_FAILED response to indicate the error.
        // See the parameters section of AVR061 for valid parameters and their meaning.
        // Cmnd_STK_GET_PARAMETER, parameter, Sync_CRC_EOP
        case Cmnd_STK_GET_PARAMETER:                // 0x41 'A'
            stk_get_parameter( getch() );
            break;

        // Set the device Programming parameters for the current device.
        // These parameters must be set before we can enter Programming mode.
        // Cmnd_STK_SET_DEVICE, devicecode, revision, progtype, parmode, polling, selftimed,
        // lockbytes, fusebytes, flashpollval1, flashpollval2, eeprompollval1, eeprompollval2,
        // pagesizehigh, pagesizelow, eepromsizehigh, eepromsizelow, flashsize4, flashsize3,
        // flashsize2, flashsize1, Sync_CRC_EOP
        case Cmnd_STK_SET_DEVICE:                   // 0x42 'B'
            fill( 20 );
            stk_set_device();
            empty_reply();
            break;

        // Set extended programming parameters for the current device.
        // Cmnd_SET_DEVICE_EXT, commandsize, eeprompagesize, signalpagel, signalbs2, Synch_CRC_EOP
        case Cmnd_STK_SET_DEVICE_EXT:               // 0x45 'E': extended parameters; cmd[0] = n_extparms + 1;
            stk_set_device_ext();
            empty_reply();
            break;

        // Enter Programming mode for the selected device. The Programming mode and device
        // programming parameters must have been set by Cmnd_STK_SET_DEVICE prior to
        // calling this command, or the command will fail with a Resp_STK_NODEVICE response.
        // Cmnd_STK_ENTER_PROGMODE, Sync_CRC_EOP
        case Cmnd_STK_ENTER_PROGMODE:               // 0x50 'P'
            if ( !pmode ) {
                stk_enter_progmode();
            }
            empty_reply();
            break;

        // Leave programming mode.
        // Cmnd_STK_LEAVE_PROGMODE, Sync_CRC_EOP
        case Cmnd_STK_LEAVE_PROGMODE:               // 0x51 'Q'
            ISPError = false;
            stk_leave_progmode();
            empty_reply();
            break;

        // Load 16-bit address down to starterkit. This command is used to set
        // the address for the next read or write operation to FLASH or EEPROM.
        // Must always be used prior to Cmnd_STK_PROG_PAGE or Cmnd_STK_READ_PAGE.
        // Cmnd_STK_LOAD_ADDRESS, addr_low, addr_high, Sync_CRC_EOP
        case Cmnd_STK_LOAD_ADDRESS:                 // 0x55 'U' set address (word)
            here = getch();
            here += 256 * getch();
            empty_reply();
            break;


        // Universal command is used to send a generic 32-bit data/command stream
        // directly to the SPI interface of the current device. Shifting data into
        // the SPI interface at the same time shifts data out of the SPI interface.
        // The response of the last eight bits that are shifted out are returned.
        // Cmnd_STK_UNIVERSAL, byte1, byte2, byte3, byte4, Sync_CRC_EOP
        case Cmnd_STK_UNIVERSAL:                    // 0x56 'V'
            stk_universal();
            break;

#if 0
        // -- Not yet implemented -- //
        // Program one word in FLASH memory.
        // Cmnd_STK_PROG_FLASH, flash_low, flash_high, Sync_CRC_EOP
        case Cmnd_STK_PROG_FLASH:                   // 0x60 '`'; unused
            getch(); // flash_low
            getch(); // flash_high
            ISPError = true;
            if ( Sync_CRC_EOP == getch() ) {
                Serial.write( Resp_STK_UNKNOWN );   // 0x12
            } else {
                Serial.write( Resp_STK_NOSYNC );    // 0x15
            }
            break;
#endif

        // -- Not used by avrdude - not fully tested -- //
        // Program one byte in EEPROM memory.
        // Cmnd_STK_PROG_DATA, data, Sync_CRC_EOP
        case Cmnd_STK_PROG_DATA:                    // 0x61 'a'
            digitalWrite( LED_WRITE, HIGH );
            stk_prog_data();
            digitalWrite( LED_WRITE, LOW );
            break;

        // Download a block of data to the programmer and program it
        // in FLASH or EEPROM of the current target device.
        // The data block size should not be larger than 256 bytes.
        // Cmnd_STK_PROG_PAGE, bytes_high, bytes_low, memtype, data, Sync_CRC_EOP
        case Cmnd_STK_PROG_PAGE:                    // 0x64 'd'
            digitalWrite( LED_WRITE, HIGH );
            stk_prog_page();
            digitalWrite( LED_WRITE, LOW );
            break;

        // Read a block of data from FLASH or EEPROM of the current device.
        // The data block size should not be larger than 256 bytes.
        // Cmnd_STK_READ_PAGE, bytes_high, bytes_low, memtype, Sync_CRC_EOP
        case Cmnd_STK_READ_PAGE:                    // 0x74 't'
            digitalWrite( LED_READ, HIGH );
            stk_read_page();
            digitalWrite( LED_READ, LOW );
            break;

        // Read the three signature bytes.
        // Cmnd_STK_READ_SIGN, Sync_CRC_EOP
        case Cmnd_STK_READ_SIGN:                    // 0x75 'u'
            digitalWrite( LED_READ, HIGH );
            stk_read_sign();
            digitalWrite( LED_READ, LOW );
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


////////////////////////////////////////////////////////////////////////////////


static void stk_get_parameter( uint8_t c ) {
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
            select_spi_speed();
            byte_reply( sck_duration );
            break;
        case Parm_STK_PROGMODE:     // 0x93
            byte_reply( 'S' );      // serial programmer
            break;
        case Param_STK500_TOPCARD_DETECT: // 0x98
            byte_reply( 0x03 );     // no top card
            break;
        default:
            byte_reply( 0 );
    }
}

// Cmnd_STK_SET_DEVICE, devicecode, revision, progtype, parmode, polling, selftimed,
// lockbytes, fusebytes, flashpollval1, flashpollval2, eeprompollval1, eeprompollval2,
// pagesizehigh, pagesizelow, eepromsizehigh, eepromsizelow, flashsize4, flashsize3,
// flashsize2, flashsize1, Sync_CRC_EOP
static void stk_set_device() {
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
    param.eeprompoll = buff_get_16( 10 );
    param.pagesize   = buff_get_16( 12 );
    param.eepromsize = buff_get_16( 14 );

    // 32 bits flashsize (big endian)
    param.flashsize = buff_get_32( 16 );

    // AVR devices have active low reset, AT89Sx are active high
    rst_active_high = ( param.devicecode >= 0xe0 );
}


// (Cmnd_SET_DEVICE_EXT,) commandsize, eeprompagesize, signalpagel, signalbs2, Synch_CRC_EOP
static void stk_set_device_ext() {
    uint8_t n_extparms = getch() - 1;  // commandsize = n_extparms + 1;
    fill( n_extparms );
    param.eeprompagesize = buff[0];
}


// Cmnd_STK_ENTER_PROGMODE, Sync_CRC_EOP
static void stk_enter_progmode() {

    // set EEPROM delay to the default value
    wait_delay_EEPROM = WAIT_DELAY_EEPROM_DEFAULT;

    // set default stk500v1 protocol
    use_arduino_protocol = false;

    // clear signature bytes
    for ( uint8_t iii = 0; iii < 3; ++iii )
        sig[iii] = 0;

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
    spi_transaction( ISP_ENTER_PMODE_4BYTE );  // 0xAC, 0x53, 0x00, 0x00 Programming enable
    digitalWrite( LED_PMODE, HIGH );
    pmode = true;
}


// Cmnd_STK_LEAVE_PROGMODE, Sync_CRC_EOP
static void stk_leave_progmode() {
    SPI.end();
    // We're about to take the target out of reset so configure SPI pins as input
    pinMode( PIN_MOSI, INPUT );
    pinMode( PIN_SCK, INPUT );
    reset_target( false );
    pinMode( RESET, INPUT );
    digitalWrite( LED_PMODE, LOW );
    pmode = false;
    use_arduino_protocol = false; // switch back to default stk500v1 protocol
    for ( uint8_t iii = 0; iii < 3; ++iii )
        sig[iii] = 0; // clear signature bytes
}


// Cmnd_STK_UNIVERSAL, byte1, byte2, byte3, byte4, Sync_CRC_EOP
static void stk_universal() {
    fill( 4 );
    uint8_t reply = spi_transaction( buff[0], buff[1], buff[2], buff[3] );
    byte_reply( reply );
    // check if read sig byte #n: 0x30, 0, n, 0
    if ( ISP_READ_SIG == buff[0] && buff[2] < 3 ) // read one signature byte
        sig[buff[2]] = reply;
    if ( sig[0] && sig[1] && sig[2] ) // all sig bytes available
        hack_eeprom_delay(); // shorter delay for newer parts
}


// (Cmnd_STK_PROG_DATA,) data, Sync_CRC_EOP
static void stk_prog_data() {
    uint8_t data = getch();
    uint16_t addr = here; // address of EEPROM
    if ( use_arduino_protocol ) {
        // arduino bootloader protocol sends word addresses also for EEPROM
        // calculate the EEPROM byte address
        addr *= 2;
    }
    spi_transaction( ISP_WRITE_EEPROM,
                     ( addr >> 8 ) & 0xFF,
                     addr & 0xFF,
                     data ); // 0xC0
    delay( wait_delay_EEPROM );
    empty_reply();
}


// (Cmnd_STK_PROG_PAGE,) bytes_high, bytes_low, memtype, (data, Sync_CRC_EOP)
static void stk_prog_page() {
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


// (Cmnd_STK_READ_PAGE,) bytes_high, bytes_low, memtype, Sync_CRC_EOP
static void stk_read_page() {
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
        result = read_flash_page( length );
    }
    if ( memtype == 'E' ) {
        result = read_eeprom_page( length );
    }
    Serial.print( result );
}


// used with arduino protocol, stk500v1 uses three stk_universal() calls instead
// (Cmnd_STK_READ_SIGN,) Sync_CRC_EOP
static void stk_read_sign() {
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
        sig[iii] = spi_transaction( ISP_READ_SIG, 0x00, iii, 0x00 ); // 0x30 sig byte iii
        Serial.write( sig[iii] );
    }
    Serial.write( Resp_STK_OK );
    hack_eeprom_delay();
}


////////////////////////////////////////////////////////////////////////////////


static void write_flash( int length ) {
    fill( length );
    if ( Sync_CRC_EOP == getch() ) {
        Serial.write( Resp_STK_INSYNC );
        Serial.write( write_flash_pages( length ) );
    } else {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
    }
}


static uint8_t write_flash_pages( int length ) {
    int x = 0;
    unsigned int page = current_page();
    while ( x < length ) {
        if ( page != current_page() ) {
            write_flash_page( page );
            page = current_page();
        }
        load_flash_page( LOW, here, buff[x++] );
        load_flash_page( HIGH, here, buff[x++] );
        here++;
    }

    write_flash_page( page );

    return Resp_STK_OK;
}


static void load_flash_page( uint8_t hilo, unsigned int addr, uint8_t data ) {
    spi_transaction( ISP_LOAD_PROG_PAGE + 8 * hilo,
                     addr >> 8 & 0xFF,
                     addr & 0xFF,
                     data ); // 0x40
}


static void write_flash_page( unsigned int addr ) {
    spi_transaction( ISP_WRITE_PROG_PAGE,
                     ( addr >> 8 ) & 0xFF,
                     addr & 0xFF,
                     0 ); // 0x4C
}


static unsigned int current_page() {
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


static uint8_t write_eeprom( unsigned int length ) {
    unsigned int start = here; // address of EEPROM
    if ( use_arduino_protocol ) {
        // arduino bootloader protocol sends word addresses also for EEPROM
        // calculate the EEPROM byte address
        start *= 2;
    }
    fill( length );
    for ( unsigned int x = 0; x < length; x++ ) {
        load_eeprom_page( x, buff[x] );
    }

    write_eeprom_page( start );
    delay( wait_delay_EEPROM );

    return Resp_STK_OK;
}


static void load_eeprom_page( unsigned int addr, uint8_t data ) {
    spi_transaction( ISP_LOAD_EEPROM_PAGE,
                     ( addr >> 8 ) & 0xFF,
                     addr & 0xFF,
                     data ); // 0xC1
}


static void write_eeprom_page( unsigned int addr ) {
    spi_transaction( ISP_WRITE_EEPROM_PAGE,
                     ( addr >> 8 ) & 0xFF,
                     addr & 0xFF,
                     0 ); // 0xC2
}


static char read_flash_page( int length ) {
    for ( int x = 0; x < length; x += 2 ) {
        Serial.write( read_flash_byte( LOW, here ) );
        Serial.write( read_flash_byte( HIGH, here ) );
        here++;
    }
    return Resp_STK_OK;
}


static uint8_t read_flash_byte( uint8_t hilo, unsigned int addr ) {
    return spi_transaction( ISP_READ_PROG + hilo * 8,
                            ( addr >> 8 ) & 0xFF,
                            addr & 0xFF,
                            0xFF ); // 0x20
}


static char read_eeprom_page( int length ) {
    int start = here; // address of EEPROM
    if ( use_arduino_protocol ) {
        // arduino bootloader protocol sends word addresses also for EEPROM
        // calculate the EEPROM byte address
        start *= 2;
    }
    for ( int x = 0; x < length; x++ ) {
        int addr = start + x;
        Serial.write( read_eeprom_byte( addr ) );
    }
    return Resp_STK_OK;
}


static uint8_t read_eeprom_byte( unsigned int addr ) {
    return spi_transaction( ISP_READ_EEPROM,
                            ( addr >> 8 ) & 0xFF,
                            addr & 0xFF,
                            0xFF ); // 0xA0
}


// transfer 4 bytes via ISP, returns 1 byte (last response)
// values according data sheet section "Serial Programming Instruction Set"
static uint8_t spi_transaction( uint8_t a, uint8_t b, uint8_t c, uint8_t d ) {
    SPI.transfer( a );
    SPI.transfer( b );
    SPI.transfer( c );
    return SPI.transfer( d );
}


static void reset_target( bool reset ) {
    digitalWrite( RESET, ( ( reset && rst_active_high ) || ( !reset && !rst_active_high ) ) ? HIGH : LOW );
}


static uint8_t getch() {
    while ( !Serial.available() )
        ; // wait
    return Serial.read();
}


static void fill( int n ) {
    for ( int x = 0; x < n; x++ ) {
        buff[x] = getch();
    }
}


static void empty_reply() {
    if ( Sync_CRC_EOP == getch() ) {
        Serial.write( Resp_STK_INSYNC );
        Serial.write( Resp_STK_OK );
    } else {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
    }
}


static void byte_reply( uint8_t b ) {
    if ( Sync_CRC_EOP == getch() ) {
        Serial.write( Resp_STK_INSYNC );
        Serial.write( b );
        Serial.write( Resp_STK_OK );
    } else {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
    }
}


static void hack_eeprom_delay() {
    // HACK: set short eeprom delay 4 ms for newer devices instead of 10 ms
    const int WAIT_DELAY_EEPROM_FAST = 4;
    wait_delay_EEPROM = WAIT_DELAY_EEPROM_DEFAULT;              // set default value
    if ( 0x1e == sig[0] ) {                                     // valid AVR parts
        if ( 0x95 == sig[1] ) {                                 // 32K flash parts
            if ( 0x0f == sig[2] || 0x14 == sig[2] )             // m328p, m328
                wait_delay_EEPROM = WAIT_DELAY_EEPROM_FAST;
        } else if ( 0x94 == sig[1] ) {                          // 16K flash parts
            if ( 0x06 == sig[2] || 0x0b == sig[2] )             // m168, m168p
                wait_delay_EEPROM = WAIT_DELAY_EEPROM_FAST;
        } else if ( 0x93 == sig[1] ) {                          // 8K flash parts
            if ( 0x0a == sig[2] || 0x0b == sig[2] || 0x0f == sig[2] ) // m88, t85, m88p
                wait_delay_EEPROM = WAIT_DELAY_EEPROM_FAST;
        } else if ( 0x92 == sig[1] ) {                          // 4K flash parts
            if ( 0x05 == sig[2] || 0x06 == sig[2] || 0x0a == sig[2] ) // m48, t45, m48p
                wait_delay_EEPROM = WAIT_DELAY_EEPROM_FAST;
        } else if ( 0x91 == sig[1] ) {                          // 2K flash parts
            if ( 0x08 == sig[2] )                               // t25
                wait_delay_EEPROM = WAIT_DELAY_EEPROM_FAST;
        }
    }
}


// measure the 3V3 voltage and calculate Vcc, return 10 * Vcc
static uint8_t get_V_target_10() {
    // Vref is Vcc; analogRead( Vcc ) -> 1023
    // Vcc = 3.3V * round( 1023 / v33 )
    uint16_t v33 = analogRead( A0 ); // about 700
    return ( 33 * 1023L + v33 / 2 ) / v33;
}


// check switch SPI_SPEED_SELECT - open: FAST, closed: SLOW
static void select_spi_speed() {
    if ( digitalRead( SPI_SPEED_SELECT ) ) {
        spi_clock = SPI_CLOCK_FAST;
        sck_duration = SCK_DURATION_FAST;
    } else {
        spi_clock = SPI_CLOCK_SLOW;
        sck_duration = SCK_DURATION_SLOW;
    }
}
