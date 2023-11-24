//**** ATMEL AVR - A P P L I C A T I O N   N O T E  ************************
//*
//* Title:		AVR061 - STK500 Communication Protocol
//* Filename:		command.h
//* Version:		1.0
//* Last updated:	09.09.2002
//*
//* Support E-mail:	avr@atmel.com
//*
//**************************************************************************

// *****************[ STK Message constants ]***************************

#define STK_SIGN_ON_MESSAGE "AVR STK" // Sign on string for Cmnd_STK_GET_SIGN_ON

// *****************[ STK Response constants ]***************************

#define Resp_STK_OK 0x10            // DLE
#define Resp_STK_FAILED 0x11        // DC1
#define Resp_STK_UNKNOWN 0x12       // DC2
#define Resp_STK_NODEVICE 0x13      // DC3
#define Resp_STK_INSYNC 0x14        // DC4
#define Resp_STK_NOSYNC 0x15        // NAK
#define Resp_ADC_CHANNEL_ERROR 0x16 // SYN
#define Resp_ADC_MEASURE_OK 0x17    // ETB
#define Resp_PWM_CHANNEL_ERROR 0x18 // CAN
#define Resp_PWM_ADJUST_OK 0x19     // EM

// *****************[ STK Special constants ]***************************

#define Sync_CRC_EOP 0x20 // SPACE

// *****************[ STK Command constants ]***************************

#define Cmnd_STK_GET_SYNC 0x30    // '0'
#define Cmnd_STK_GET_SIGN_ON 0x31 // '1'

#define Cmnd_STK_SET_PARAMETER 0x40  // '@'
#define Cmnd_STK_GET_PARAMETER 0x41  // 'A'
#define Cmnd_STK_SET_DEVICE 0x42     // 'B'
#define Cmnd_STK_SET_DEVICE_EXT 0x45 // 'E'

#define Cmnd_STK_ENTER_PROGMODE 0x50 // 'P'
#define Cmnd_STK_LEAVE_PROGMODE 0x51 // 'Q'
#define Cmnd_STK_CHIP_ERASE 0x52     // 'R'
#define Cmnd_STK_CHECK_AUTOINC 0x53  // 'S'

#define Cmnd_STK_LOAD_ADDRESS 0x55    // 'U'
#define Cmnd_STK_UNIVERSAL 0x56       // 'V'
#define Cmnd_STK_UNIVERSAL_MULTI 0x57 // 'W'

#define Cmnd_STK_PROG_FLASH 0x60    // '`'
#define Cmnd_STK_PROG_DATA 0x61     // 'a'
#define Cmnd_STK_PROG_FUSE 0x62     // 'b'
#define Cmnd_STK_PROG_LOCK 0x63     // 'c'
#define Cmnd_STK_PROG_PAGE 0x64     // 'd'
#define Cmnd_STK_PROG_FUSE_EXT 0x65 // 'e'

#define Cmnd_STK_READ_FLASH 0x70      // 'p'
#define Cmnd_STK_READ_DATA 0x71       // 'q'
#define Cmnd_STK_READ_FUSE 0x72       // 'r'
#define Cmnd_STK_READ_LOCK 0x73       // 's'
#define Cmnd_STK_READ_PAGE 0x74       // 't'
#define Cmnd_STK_READ_SIGN 0x75       // 'u'
#define Cmnd_STK_READ_OSCCAL 0x76     // 'v'
#define Cmnd_STK_READ_FUSE_EXT 0x77   // 'w'
#define Cmnd_STK_READ_OSCCAL_EXT 0x78 // 'x'

// *****************[ STK Parameter constants ]***************************

#define Parm_STK_HW_VER 0x80         // ' ' - R
#define Parm_STK_SW_MAJOR 0x81       // ' ' - R
#define Parm_STK_SW_MINOR 0x82       // ' ' - R
#define Parm_STK_LEDS 0x83           // ' ' - R/W
#define Parm_STK_VTARGET 0x84        // ' ' - R/W
#define Parm_STK_VADJUST 0x85        // ' ' - R/W
#define Parm_STK_OSC_PSCALE 0x86     // ' ' - R/W
#define Parm_STK_OSC_CMATCH 0x87     // ' ' - R/W
#define Parm_STK_RESET_DURATION 0x88 // ' ' - R/W
#define Parm_STK_SCK_DURATION 0x89   // ' ' - R/W

#define Parm_STK_BUFSIZEL 0x90           // ' ' - R/W, Range {0..255}
#define Parm_STK_BUFSIZEH 0x91           // ' ' - R/W, Range {0..255}
#define Parm_STK_DEVICE 0x92             // ' ' - R/W, Range {0..255}
#define Parm_STK_PROGMODE 0x93           // ' ' - 'P' or 'S'
#define Parm_STK_PARAMODE 0x94           // ' ' - TRUE or FALSE
#define Parm_STK_POLLING 0x95            // ' ' - TRUE or FALSE
#define Parm_STK_SELFTIMED 0x96          // ' ' - TRUE or FALSE
#define Param_STK500_TOPCARD_DETECT 0x98 // ' ' - Detect top-card attached

// *****************[ STK status bit definitions ]***************************

#define Stat_STK_INSYNC 0x01     // INSYNC status bit, '1' - INSYNC
#define Stat_STK_PROGMODE 0x02   // Programming mode,  '1' - PROGMODE
#define Stat_STK_STANDALONE 0x04 // Standalone mode,   '1' - SM mode
#define Stat_STK_RESET 0x08      // RESET button,      '1' - Pushed
#define Stat_STK_PROGRAM 0x10    // Program button, '   1' - Pushed
#define Stat_STK_LEDG 0x20       // Green LED status,  '1' - Lit
#define Stat_STK_LEDR 0x40       // Red LED status,    '1' - Lit
#define Stat_STK_LEDBLINK 0x80   // LED blink ON/OFF,  '1' - Blink


// *****************************[ End Of COMMAND.H ]**************************
