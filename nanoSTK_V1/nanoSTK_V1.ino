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


#include "Arduino.h"

// AVR061 - STK500 Communication Protocol
#include "AVR061_command.h"

// AVR910 - In-System Programming
#include "AVR910_ISP.h"

// low level SPI implementation
#include "SPI.h"

// HW version 2
#define HWVER 2

// SW version 1.32
#define SWMAJ 1
#define SWMIN 32


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

const uint8_t RESET_ISP = 10; // /SS (PB2)

// Configure the baud rate:

// #define BAUDRATE  19200
#define BAUDRATE 115200
// BAUDRATE > 115200 does not work!

// this is the original xtal of STK500
#define STK500_XTAL 7372800UL
// this is the xtal of the nano
#define NANO_XTAL 16000000UL

// Configure which pins to use:

// Optional nanoSTK HW changes:
// Put an LED (with resistor) on the following pins:
// 9: Heartbeat - Indicates that the programmer is running
//
// the next 4 LEDs are the same as used by ScratchMonkey
// 8: Error     - An Error has occured - clear with programmer reset
// 7: Write     - Writing to the target
// 6: Read      - Reading from the targer
// 5: PMode     - Target in programming mode
//
// 3: Clk Out   - Clock source for targets that need an external clock
//
// Input to set the SPI speed
// 2: SPI speed select

//
// Analog input to measure 3.3 V, connect A0 and 3V3
// A0:          - measure 3.3 V with AREF=Vcc, calculate Vcc and show as VTARGET.
#define VTARGET

#define HEARTBEAT
#ifdef HEARTBEAT
// Heartbeat LED
const uint8_t LED_HB = 9;
#endif

// Error LED
const uint8_t LED_ERROR = 8;

// Writing activity LED
const uint8_t LED_WRITE = 7;

// Reading activity LED
const uint8_t LED_READ = 6;

// Programming mode LED
const uint8_t LED_PMODE = 5;

// 8 or 1 MHz output for targets that need an exteral clk
#define EXT_CLK
#ifdef EXT_CLK
const uint8_t EXT_CLK_OUT = 3;
#endif

// switch: open = FAST SPI mode, closed = SLOW SPI mode
const uint8_t SPI_SPEED_SELECT = 2;


// By default, use hardware SPI pins (MOSI (PB3) = 11, MISO (PB4) = 12, SCK (PB5) = 13):
#ifndef MOSI_OUT_PIN
#define MOSI_OUT_PIN MOSI
#endif

#ifndef MISO_IN_PIN
#define MISO_IN_PIN MISO
#endif

#ifndef SCK_OUT_PIN
#define SCK_OUT_PIN SCK
#endif

// This program uses the original "stk500v1" protocol with byte addresses for EEPROM access.
// The modified "arduino" bootloader protocol addresses also the EEPROM word-wise.
// To enable the automatic detection of the "arduino" protocol uncomment the next line.
#define DETECT_ARDUINO_PROTOCOL


////////////////////////////////////////////////////////////////////////////////


void setup() {
    Serial.begin( BAUDRATE );

    // start with fast SPI as default, can be changed later with term command "sck"
    pinMode( SPI_SPEED_SELECT, INPUT_PULLUP );
    if ( digitalRead( SPI_SPEED_SELECT ) ) { // pin open, default
        SPI.init( 2 );                       // default = 2 * 0.5 µs clock period -> 1 MHz SPI speed, ok for target clock >=4 MHz
#ifdef EXT_CLK
        initTimer2( 1, 0 ); // pscale = 1, cmatch = 0 -> 8 MHz
#endif
    } else {            // pin closed, slow down
        SPI.init( 16 ); // 16 * 0.5 µs = 8µs -> 125 kHz (target clock >= 500 kHz)
#ifdef EXT_CLK
        initTimer2( 1, 7 ); // pscale = 1, cmatch = 7 -> 1 MHz
#endif
    }
    pinMode( LED_PMODE, OUTPUT );
    pulse( LED_PMODE, 2 );
    pinMode( LED_READ, OUTPUT );
    pulse( LED_READ, 2 );
    pinMode( LED_WRITE, OUTPUT );
    pulse( LED_WRITE, 2 );
    pinMode( LED_ERROR, OUTPUT );
    pulse( LED_ERROR, 2 );

#ifdef HEARTBEAT
    pinMode( LED_HB, OUTPUT );
#endif
}


#define PTIME 30
static void pulse( int8_t pin, int8_t times ) {
    do {
        digitalWrite( pin, HIGH );
        delay( PTIME );
        digitalWrite( pin, LOW );
        delay( PTIME );
    } while ( times-- );
}


static bool ISPError = false;
static bool pmode = false;
static bool use_arduino_protocol = false; // use original stk500v1 as default

// address for reading and writing, set by 'U' command Cmnd_STK_LOAD_ADDRESS
static uint16_t target_addr;

// global block storage for cmd, flash and eeprom data
static uint8_t buff[ 256 ];

// the three signature bytes
static uint8_t sig[ SIG_SIZE ];

// default wait delay before writing next EEPROM location
// can be adapted according to device signature
const int16_t WAIT_DELAY_EEPROM_DEFAULT = 10;
const uint16_t WAIT_DELAY_EEPROM_FAST = 4;
static int16_t wait_delay_EEPROM = WAIT_DELAY_EEPROM_DEFAULT;


static uint16_t buff_get_16( uint16_t addr ) { return buff[ addr ] * 0x100 + buff[ addr + 1 ]; }


static uint32_t buff_get_32( uint16_t addr ) {
    return buff[ addr ] * 0x01000000L + buff[ addr + 1 ] * 0x00010000L + buff[ addr + 2 ] * 0x00000100L + buff[ addr + 3 ];
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
    uint16_t flashpoll;
    uint16_t eeprompoll;
    uint16_t pagesize;
    uint16_t eepromsize;
    uint32_t flashsize;
    uint8_t eeprompagesize;
} param_t;

static param_t param;


#ifdef HEARTBEAT
// this provides a heartbeat on pin 9, so you can see the software is running.
static void heartbeat() {
    static int8_t hbdelta = 8;
    static uint8_t hbval = 128;
    static uint32_t last_time = 0;
    uint32_t now = millis();
    if ( ( now - last_time ) < 40 ) {
        return;
    }
    last_time = now;
    if ( hbval > 160 ) {
        hbdelta = -hbdelta;
    } else if ( hbval < 16 ) {
        hbdelta = -hbdelta;
    }
    hbval += hbdelta;
    analogWrite( LED_HB, hbval );
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
    uint8_t cmd_byte = get_byte();
    switch ( cmd_byte ) {

    // Use this command to try to regain synchronization when sync is lost.
    // Send this command until Resp_STK_INSYNC is received.
    // Cmnd_STK_GET_SYNC, Sync_CRC_EOP
    case Cmnd_STK_GET_SYNC: // 0x30 '0' signon
        ISPError = false;
        empty_reply();
        break;

    // The PC sends this command to check the communication channel.
    // Cmnd_STK_GET_SIGN_ON, Sync_CRC_EOP
    case Cmnd_STK_GET_SIGN_ON: // 0x31 '1'
        if ( get_byte() == Sync_CRC_EOP ) {
            Serial.write( Resp_STK_INSYNC ); // 0x14
            Serial.print( "nanoSTK" );
            Serial.write( Resp_STK_OK );
        } else {
            ISPError = true;
            Serial.write( Resp_STK_NOSYNC ); // 0x15
        }
        break;

    // Set the value of a valid parameter in the STK500 starterkit.
    // See the parameters section for valid parameters and their meaning.
    // Cmnd_STK_SET_PARAMETER, parameter, value, Sync_CRC_EOP
    case Cmnd_STK_SET_PARAMETER: // 0x40 '@'
        stk_set_parameter( get_byte() );
        break;

    // Get the value of a valid parameter from the STK500 starterkit.
    // If the parameter is not used, the same parameter will be returned
    // together with a Resp_STK_FAILED response to indicate the error.
    // See the parameters section of AVR061 for valid parameters and their meaning.
    // Cmnd_STK_GET_PARAMETER, parameter, Sync_CRC_EOP
    case Cmnd_STK_GET_PARAMETER: // 0x41 'A'
        stk_get_parameter( get_byte() );
        break;

    // Set the device Programming parameters for the current device.
    // These parameters must be set before we can enter Programming mode.
    // Cmnd_STK_SET_DEVICE, devicecode, revision, progtype, parmode, polling, selftimed,
    // lockbytes, fusebytes, flashpollval1, flashpollval2, eeprompollval1, eeprompollval2,
    // pagesizehigh, pagesizelow, eepromsizehigh, eepromsizelow, flashsize4, flashsize3,
    // flashsize2, flashsize1, Sync_CRC_EOP
    case Cmnd_STK_SET_DEVICE: // 0x42 'B'
        stk_set_device();
        empty_reply();
        break;

    // Set extended programming parameters for the current device.
    // Cmnd_SET_DEVICE_EXT, commandsize, eeprompagesize, signalpagel, signalbs2, Synch_CRC_EOP
    case Cmnd_STK_SET_DEVICE_EXT: // 0x45 'E': extended parameters; cmd[0] = n_extparms + 1;
        stk_set_device_ext();
        empty_reply();
        break;

    // Enter Programming mode for the selected device. The Programming mode and device
    // programming parameters must have been set by Cmnd_STK_SET_DEVICE prior to
    // calling this command, or the command will fail with a Resp_STK_NODEVICE response.
    // Cmnd_STK_ENTER_PROGMODE, Sync_CRC_EOP
    case Cmnd_STK_ENTER_PROGMODE: // 0x50 'P'
        if ( !pmode ) {
            stk_enter_progmode();
        }
        empty_reply();
        break;

    // Leave programming mode.
    // Cmnd_STK_LEAVE_PROGMODE, Sync_CRC_EOP
    case Cmnd_STK_LEAVE_PROGMODE: // 0x51 'Q'
        ISPError = false;
        stk_leave_progmode();
        empty_reply();
        break;

    // Load 16-bit address down to starterkit. This command is used to set
    // the address for the next read or write operation to FLASH or EEPROM.
    // Must always be used prior to Cmnd_STK_PROG_PAGE or Cmnd_STK_READ_PAGE.
    // Cmnd_STK_LOAD_ADDRESS, addr_low, addr_high, Sync_CRC_EOP
    case Cmnd_STK_LOAD_ADDRESS:      // 0x55 'U' set address (word)
        target_addr = get_word_LH(); // little endian!
        empty_reply();
        break;


    // Universal command is used to send a generic 32-bit data/command stream
    // directly to the SPI interface of the current device. Shifting data into
    // the SPI interface at the same time shifts data out of the SPI interface.
    // The response of the last eight bits that are shifted out are returned.
    // Cmnd_STK_UNIVERSAL, byte1, byte2, byte3, byte4, Sync_CRC_EOP
    case Cmnd_STK_UNIVERSAL: // 0x56 'V'
        stk_universal();
        break;

#if 0
    // -- Not yet implemented -- //
    // Program one word in FLASH memory.
    // Cmnd_STK_PROG_FLASH, flash_low, flash_high, Sync_CRC_EOP
    case Cmnd_STK_PROG_FLASH:                   // 0x60 '`'; unused
      get_word_LH(); // flash_low, high
      ISPError = true;
      if ( Sync_CRC_EOP == get_byte() ) {
        Serial.write( Resp_STK_UNKNOWN );   // 0x12
      } else {
        Serial.write( Resp_STK_NOSYNC );    // 0x15
      }
      break;
#endif

    // -- Not used by avrdude - not fully tested -- //
    // Program one byte in EEPROM memory.
    // Cmnd_STK_PROG_DATA, data, Sync_CRC_EOP
    case Cmnd_STK_PROG_DATA: // 0x61 'a'
        digitalWrite( LED_WRITE, HIGH );
        stk_prog_data();
        digitalWrite( LED_WRITE, LOW );
        break;

    // Download a block of data to the programmer and program it
    // in FLASH or EEPROM of the current target device.
    // The data block size should not be larger than 256 bytes.
    // Cmnd_STK_PROG_PAGE, bytes_high, bytes_low, memtype, data, Sync_CRC_EOP
    case Cmnd_STK_PROG_PAGE: // 0x64 'd'
        digitalWrite( LED_WRITE, HIGH );
        stk_prog_page();
        digitalWrite( LED_WRITE, LOW );
        break;

    // Read a block of data from FLASH or EEPROM of the current device.
    // The data block size should not be larger than 256 bytes.
    // Cmnd_STK_READ_PAGE, bytes_high, bytes_low, memtype, Sync_CRC_EOP
    case Cmnd_STK_READ_PAGE: // 0x74 't'
        digitalWrite( LED_READ, HIGH );
        stk_read_page();
        digitalWrite( LED_READ, LOW );
        break;

    // Read the three signature bytes.
    // Used by arduino protocol, not used by stk500v1 protocol.
    // Cmnd_STK_READ_SIGN, Sync_CRC_EOP
    case Cmnd_STK_READ_SIGN: // 0x75 'u'
        digitalWrite( LED_READ, HIGH );
        stk_read_sign();
        digitalWrite( LED_READ, LOW );
        break;

    // expecting a command, not Sync_CRC_EOP
    // this is how we can get back in sync
    case Sync_CRC_EOP: // 0x20 ' '
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
        break;

    // anything else we will return STK_UNKNOWN
    default:
        ISPError = true;
        if ( Sync_CRC_EOP == get_byte() ) {
            Serial.write( Resp_STK_UNKNOWN ); // 0x12
        } else {
            Serial.write( Resp_STK_NOSYNC ); // 0x15
        }
    }
}


////////////////////////////////////////////////////////////////////////////////

static void stk_set_parameter( uint8_t parm ) {
    switch ( parm ) {
#ifdef EXT_CLK
    case Parm_STK_OSC_PSCALE:       // 0x86
        TCCR2B = get_byte() & 0x07; // prescaler
        empty_reply();
        break;
    case Parm_STK_OSC_CMATCH: // 0x87
        OCR2A = get_byte();   // counter match
        TCNT2 = 0;            // Reset Timer2 Counter
        empty_reply();
        break;
#endif
    case Parm_STK_SCK_DURATION: // 0x89
        SPI.set_sck_duration( get_byte() );
        empty_reply();
        break;
    default:
        get_byte();
        empty_reply();
    }
}

static void stk_get_parameter( uint8_t parm ) {
    switch ( parm ) {
    case Parm_STK_HW_VER: // 0x80
        byte_reply( HWVER );
        break;
    case Parm_STK_SW_MAJOR: // 0x81
        byte_reply( SWMAJ );
        break;
    case Parm_STK_SW_MINOR: // 0x82
        byte_reply( SWMIN );
        break;
#ifdef VTARGET
    case Parm_STK_VTARGET: // 0x84
        byte_reply( get_V_target_10() );
        break;
#endif
    case Parm_STK_VADJUST: // 0x85
        byte_reply( 0 );
        break;
#ifdef EXT_CLK
    case Parm_STK_OSC_PSCALE: // 0x86
        byte_reply( TCCR2B & 0x07 );
        break;
    case Parm_STK_OSC_CMATCH: // 0x87
        byte_reply( OCR2A );
        break;
#endif
    case Parm_STK_RESET_DURATION: // 0x88
        byte_reply( 0 );
        break;
    case Parm_STK_SCK_DURATION: // 0x89
        byte_reply( SPI.get_sck_duration() );
        break;
    case Parm_STK_PROGMODE: // 0x93
        byte_reply( 'S' );  // serial programmer
        break;
    case Param_STK500_TOPCARD_DETECT: // 0x98
        byte_reply( 0x03 );           // no top card
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
    // read parameter packet into buff[]
    fill( 20 );

    param.devicecode = buff[ 0 ];
    param.revision = buff[ 1 ];
    param.progtype = buff[ 2 ];
    param.parmode = buff[ 3 ];
    param.polling = buff[ 4 ];
    param.selftimed = buff[ 5 ];
    param.lockbytes = buff[ 6 ];
    param.fusebytes = buff[ 7 ];
    // 16 bits (big endian)
    param.flashpoll = buff_get_16( 8 );
    param.eeprompoll = buff_get_16( 10 );
    param.pagesize = buff_get_16( 12 );
    param.eepromsize = buff_get_16( 14 );
    // 32 bits flashsize (big endian)
    param.flashsize = buff_get_32( 16 );

    // AVR devices have active low reset, AT89Sx are active high
    rst_active_high = ( param.devicecode >= 0xe0 );
}


// (Cmnd_SET_DEVICE_EXT,) commandsize, eeprompagesize, signalpagel, signalbs2, reset_disposition, Synch_CRC_EOP
static void stk_set_device_ext() {
    uint8_t n_extparms = get_byte() - 1; // commandsize = n_extparms + 1;
    fill( n_extparms );
    param.eeprompagesize = buff[ 0 ];
}


// Cmnd_STK_ENTER_PROGMODE, Sync_CRC_EOP
static void stk_enter_progmode() {

    // set EEPROM delay to the default value
    wait_delay_EEPROM = WAIT_DELAY_EEPROM_DEFAULT;

    // set default stk500v1 protocol
    use_arduino_protocol = false;

    // clear signature bytes
    for ( uint8_t iii = 0; iii < SIG_SIZE; ++iii )
        sig[ iii ] = 0;

    // Reset target before driving SCK_OUT_PIN or MOSI_OUT_PIN

    // SPI.init() will configure SS as output, so SPI master mode is selected.
    // We have defined RESET_ISP as pin 10, which for many Arduinos is not the SS pin.
    // So we have to configure RESET_ISP as output here,
    // (reset_target() first sets the correct level)
    reset_target( true );
    pinMode( RESET_ISP, OUTPUT );
    SPI.init(); // HW init, do not change sck_duration

    // See AVR datasheets, chapter "SERIAL_PRG Programming Algorithm":
    // Pulse RESET_ISP after SCK_OUT_PIN is low:
    digitalWrite( SCK_OUT_PIN, LOW );
    delay( 20 ); // discharge SCK_OUT_PIN, value arbitrarily chosen
    reset_target( false );
    // Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
    // speeds above 20 KHz
    delayMicroseconds( 100 );
    reset_target( true );

    // Send the enable programming command: 0xAC, 0x53, 0x00, 0x00
    delay( 50 ); // datasheet: must be > 20 msec
    spi_transaction( ISP_ENTER_PMODE_BYTE_0, ISP_ENTER_PMODE_BYTE_1, 0, 0 );
    digitalWrite( LED_PMODE, HIGH );
    pmode = true;
}


// Cmnd_STK_LEAVE_PROGMODE, Sync_CRC_EOP
static void stk_leave_progmode() {
    // We're about to take the target out of reset so configure SPI pins as input
    SPI.exit();
    reset_target( false );
    pinMode( RESET_ISP, INPUT );
    digitalWrite( LED_PMODE, LOW );
    pmode = false;
    use_arduino_protocol = false; // switch back to default stk500v1 protocol
    for ( uint8_t iii = 0; iii < SIG_SIZE; ++iii )
        sig[ iii ] = 0; // clear signature bytes
}


// Cmnd_STK_UNIVERSAL, byte1, byte2, byte3, byte4, Sync_CRC_EOP
static void stk_universal() {
    fill( 4 );
    uint8_t reply = spi_transaction( buff[ 0 ], buff[ 1 ], buff[ 2 ], buff[ 3 ] );
    byte_reply( reply );
    // check if read sig byte #n: 0x30, 0, n, 0
    if ( ISP_READ_SIG == buff[ 0 ] && buff[ 2 ] < SIG_SIZE ) // read one signature byte
        sig[ buff[ 2 ] ] = reply;
    if ( sig[ 0 ] && sig[ 1 ] && sig[ 2 ] ) // all sig bytes available
        hack_eeprom_delay();                // set shorter delay if this is a newer parts
}


// (Cmnd_STK_PROG_DATA,) data, Sync_CRC_EOP
// not used by avrdude
static void stk_prog_data() {
    uint8_t data = get_byte();
    uint16_t ee_addr = target_addr; // address of EEPROM
    if ( use_arduino_protocol ) {
        // arduino bootloader protocol sends word addresses also for EEPROM
        ee_addr *= 2; // convert to EEPROM byte address
    }
    if ( data != read_eeprom_byte( ee_addr ) ) { // do we need to write the data?
        spi_transaction( ISP_WRITE_EEPROM, ee_addr >> 8 & 0xFF, ee_addr & 0xFF,
                         data ); // 0xC0
        start_isp_delay( wait_delay_EEPROM );
    }
    empty_reply();
}


// (Cmnd_STK_PROG_PAGE,) bytes_high, bytes_low, memtype, (data, Sync_CRC_EOP)
static void stk_prog_page() {
    uint8_t result = Resp_STK_FAILED;
    uint16_t length = get_word_HL(); // big endian!
    uint8_t memtype = get_byte();
    // flash memory @target_addr, (length) bytes
    if ( memtype == 'F' ) {
        write_flash( length );
        return;
    }
    if ( memtype == 'E' ) {
        result = write_eeprom( length );
        if ( Sync_CRC_EOP == get_byte() ) {
            Serial.write( Resp_STK_INSYNC );
            Serial.write( result );
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
    uint8_t result = Resp_STK_FAILED;
    uint16_t length = get_word_HL(); // big endian!
    uint8_t memtype = get_byte();
    if ( Sync_CRC_EOP != get_byte() ) {
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
    Serial.write( result );
}


// used with arduino protocol, stk500v1 uses three stk_universal() calls instead
// (Cmnd_STK_READ_SIGN,) Sync_CRC_EOP
static void stk_read_sign() {
    if ( Sync_CRC_EOP != get_byte() ) {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
        return;
    }

#ifdef DETECT_ARDUINO_PROTOCOL
    use_arduino_protocol = true;
#endif

    Serial.write( Resp_STK_INSYNC );
    for ( uint8_t iii = 0; iii < SIG_SIZE; ++iii ) {
        sig[ iii ] = spi_transaction( ISP_READ_SIG, 0x00, iii, 0x00 ); // 0x30 sig byte iii
        Serial.write( sig[ iii ] );
    }
    Serial.write( Resp_STK_OK );
    hack_eeprom_delay();
}


////////////////////////////////////////////////////////////////////////////////


static void write_flash( uint16_t length ) {
    fill( length );
    if ( Sync_CRC_EOP == get_byte() ) {
        Serial.write( Resp_STK_INSYNC );
        Serial.write( write_flash_pages( length ) );
    } else {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
    }
}


static uint8_t write_flash_pages( uint16_t length ) {
    uint16_t buff_idx = 0;
    uint16_t page = current_page();
    while ( buff_idx < length ) {
        if ( page != current_page() ) {
            write_flash_page( page );
            page = current_page();
        }
        load_flash_page( LOW, target_addr, buff[ buff_idx++ ] );
        load_flash_page( HIGH, target_addr, buff[ buff_idx++ ] );
        ++target_addr;
    }
    write_flash_page( page );
    return Resp_STK_OK;
}


static void load_flash_page( uint8_t hilo, uint16_t addr, uint8_t data ) {
    spi_transaction( ISP_LOAD_PROG_PAGE + 8 * hilo, addr >> 8 & 0xFF, addr & 0xFF,
                     data ); // 0x40
}


static void write_flash_page( uint16_t addr ) {
    spi_transaction( ISP_WRITE_PROG_PAGE, addr >> 8 & 0xFF, addr & 0xFF,
                     0 ); // 0x4C
}


static uint16_t current_page() {
#if 1
    if ( param.pagesize == 32 ) {
        return target_addr & 0xFFF0;
    }
    if ( param.pagesize == 64 ) {
        return target_addr & 0xFFE0;
    }
    if ( param.pagesize == 128 ) {
        return target_addr & 0xFFC0;
    }
    if ( param.pagesize == 256 ) {
        return target_addr & 0xFF80;
    }
    return target_addr;
#else
    return target_addr & ~( param.pagesize / 2 - 1 );
#endif
}


static uint8_t write_eeprom( uint16_t length ) {
    uint16_t ee_addr_wr = target_addr; // write address of EEPROM
    if ( use_arduino_protocol ) {
        // arduino bootloader protocol sends word addresses also for EEPROM
        ee_addr_wr *= 2; // convert to EEPROM byte address of page
    }
    fill( length );                   // get EEPROM page data
    uint16_t ee_addr_rd = ee_addr_wr; // read address of EEPROM
    bool page_has_changed = false;
    for ( uint16_t pg_idx = 0; pg_idx < length; ++pg_idx ) {
        uint8_t data = buff[ pg_idx ];
        if ( data != read_eeprom_byte( ee_addr_rd++ ) ) {
            load_eeprom_page( pg_idx, data ); // change this byte
            page_has_changed = true;
        }
    }
    if ( page_has_changed ) {
        write_eeprom_page( ee_addr_wr ); // only "loaded" bytes will be written
        start_isp_delay( wait_delay_EEPROM );
    }
    return Resp_STK_OK;
}


static void load_eeprom_page( uint16_t addr, uint8_t data ) {
    spi_transaction( ISP_LOAD_EEPROM_PAGE, addr >> 8 & 0xFF, addr & 0xFF,
                     data ); // 0xC1
}


static void write_eeprom_page( uint16_t addr ) {
    spi_transaction( ISP_WRITE_EEPROM_PAGE, addr >> 8 & 0xFF, addr & 0xFF,
                     0 ); // 0xC2
}


static uint8_t read_flash_page( uint16_t length ) {
    for ( uint16_t byte_idx = 0; byte_idx < length; byte_idx += 2 ) {
        Serial.write( read_flash_byte( LOW, target_addr ) );
        Serial.write( read_flash_byte( HIGH, target_addr ) );
        ++target_addr;
    }
    return Resp_STK_OK;
}


static uint8_t read_flash_byte( uint8_t hilo, uint16_t addr ) {
    return spi_transaction( ISP_READ_PROG + hilo * 8, addr >> 8 & 0xFF, addr & 0xFF,
                            0xFF ); // 0x20
}


static uint8_t read_eeprom_page( uint16_t length ) {
    uint16_t start = target_addr; // address of EEPROM
    if ( use_arduino_protocol ) {
        // arduino bootloader protocol sends word addresses also for EEPROM
        // calculate the EEPROM byte address
        start *= 2;
    }
    for ( uint16_t byte_idx = 0; byte_idx < length; ++byte_idx ) {
        uint16_t addr = start + byte_idx;
        Serial.write( read_eeprom_byte( addr ) );
    }
    return Resp_STK_OK;
}


static uint8_t read_eeprom_byte( uint16_t addr ) {
    return spi_transaction( ISP_READ_EEPROM, addr >> 8 & 0xFF, addr & 0xFF,
                            0xFF ); // 0xA0
}


static uint32_t milli_target;

// set delay target for next ISP access
static void start_isp_delay( uint8_t delay ) { milli_target = millis() + delay; }


// transfer 4 bytes via ISP, returns 1 byte (last response)
// values according data sheet section "Serial Programming Instruction Set"
static uint8_t spi_transaction( uint8_t a, uint8_t b, uint8_t c, uint8_t d ) {
    uint32_t wait = milli_target - millis();
    if ( wait < 256 )  // delay not yet finished
        delay( wait ); // wait before next SPI transfer
    SPI.transfer( a );
    SPI.transfer( b );
    SPI.transfer( c );
    return SPI.transfer( d );
}


static void reset_target( bool reset ) {
    digitalWrite( RESET_ISP, ( ( reset && rst_active_high ) || ( !reset && !rst_active_high ) ) ? HIGH : LOW );
}


static uint8_t get_byte() {
    while ( !Serial.available() )
        ; // wait
    return Serial.read();
}


static uint16_t get_word_LH() { // little endian
    uint16_t lo = get_byte();
    uint16_t hi = get_byte();
    return 0x100 * hi + lo;
}


static uint16_t get_word_HL() { // big endian
    uint16_t hi = get_byte();
    uint16_t lo = get_byte();
    return 0x100 * hi + lo;
}


static void fill( uint16_t n ) {
    for ( uint16_t idx = 0; idx < n; ++idx ) {
        buff[ idx ] = get_byte();
    }
}


static void empty_reply() {
    if ( Sync_CRC_EOP == get_byte() ) {
        Serial.write( Resp_STK_INSYNC );
        Serial.write( Resp_STK_OK );
    } else {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
    }
}


static void byte_reply( uint8_t b ) {
    if ( Sync_CRC_EOP == get_byte() ) {
        Serial.write( Resp_STK_INSYNC );
        Serial.write( b );
        Serial.write( Resp_STK_OK );
    } else {
        ISPError = true;
        Serial.write( Resp_STK_NOSYNC );
    }
}


static void hack_eeprom_delay() {
    // set short eeprom delay 4 ms for newer devices instead of 10 ms
    wait_delay_EEPROM = WAIT_DELAY_EEPROM_DEFAULT;      // set default value
    if ( 0x1e == sig[ 0 ] ) {                           // valid AVR parts
        if ( 0x95 == sig[ 1 ] ) {                       // 32K flash parts
            if ( 0x0f == sig[ 2 ] || 0x14 == sig[ 2 ] ) // m328p, m328
                wait_delay_EEPROM = WAIT_DELAY_EEPROM_FAST;
        } else if ( 0x94 == sig[ 1 ] ) {                // 16K flash parts
            if ( 0x06 == sig[ 2 ] || 0x0b == sig[ 2 ] ) // m168, m168p
                wait_delay_EEPROM = WAIT_DELAY_EEPROM_FAST;
        } else if ( 0x93 == sig[ 1 ] ) {                                    // 8K flash parts
            if ( 0x0a == sig[ 2 ] || 0x0b == sig[ 2 ] || 0x0f == sig[ 2 ] ) // m88, t85, m88p
                wait_delay_EEPROM = WAIT_DELAY_EEPROM_FAST;
        } else if ( 0x92 == sig[ 1 ] ) {                                    // 4K flash parts
            if ( 0x05 == sig[ 2 ] || 0x06 == sig[ 2 ] || 0x0a == sig[ 2 ] ) // m48, t45, m48p
                wait_delay_EEPROM = WAIT_DELAY_EEPROM_FAST;
        } else if ( 0x91 == sig[ 1 ] ) { // 2K flash parts
            if ( 0x08 == sig[ 2 ] )      // t25
                wait_delay_EEPROM = WAIT_DELAY_EEPROM_FAST;
        }
    }
}


#ifdef VTARGET

// measure the 3V3 voltage and calculate Vcc, return 10 * Vcc
static uint8_t get_V_target_10() {
    // Vref is Vcc; analogRead( Vcc ) -> 1023
    // Vcc = 3.3V * round( 1023 / v33 )
    uint16_t v33 = analogRead( A0 ); // about 700
    return ( 33 * 1023L + v33 / 2 ) / v33;
}

#endif


#ifdef EXT_CLK

//--------------------------------------------------------------------------------
// configTimer2
// output rectangle at D3 as clock signal for devices with external clock
//--------------------------------------------------------------------------------
static void initTimer2( uint8_t pscale, uint8_t cmatch ) {
    // Set OC2B for Compare Match (digital pin3)
    pinMode( EXT_CLK_OUT, OUTPUT );

    // Initialize Timer2
    TCCR2A = ( 1 << WGM21 ) | ( 1 << COM2B0 ); // CTC (MODE_2) + Toggle OC2B
    TCCR2B = pscale;                           // default: prescaler = 1
    OCR2A = cmatch;                            // default: 0 -> 8 MHz
    TCNT2 = 0;                                 // Reset Timer2 Counter
}

#endif
