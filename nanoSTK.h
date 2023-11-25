#pragma once


// the funtion prototypes for nanoSTK.cpp
//
void setup();
static uint16_t buff_get_16( uint16_t addr );
static uint32_t buff_get_32( uint16_t addr );
void loop( void );
static void heartbeat();
static void avrisp();
static void stk_set_parameter( uint8_t parm );
static void stk_get_parameter( uint8_t parm );
static void stk_set_device();
static void stk_set_device_ext();
static void stk_enter_progmode();
static void stk_leave_progmode();
static void stk_universal();
static void stk_prog_data();
static void stk_prog_page();
static void stk_read_page();
static void stk_read_sign();
static void write_flash( uint16_t length );
static uint8_t write_flash_pages( uint16_t length );
static void load_flash_page( uint8_t hilo, uint16_t addr, uint8_t data );
static void write_flash_page( uint16_t addr );
static uint16_t current_page();
static uint8_t write_eeprom( uint16_t length );
static void load_eeprom_page( uint16_t addr, uint8_t data );
static void write_eeprom_page( uint16_t addr );
static uint8_t read_flash_page( uint16_t length );
static uint8_t read_flash_byte( uint8_t hilo, uint16_t addr );
static uint8_t read_eeprom_page( uint16_t length );
static uint8_t read_eeprom_byte( uint16_t addr );
static void start_isp_delay( uint8_t delay );
static uint8_t spi_transaction( uint8_t a, uint8_t b, uint8_t c, uint8_t d );
static void reset_target( bool reset );
static uint8_t get_byte();
static uint16_t get_word_LH();
static uint16_t get_word_HL();
static void fill( uint16_t n );
static void empty_reply();
static void byte_reply( uint8_t b );
static void hack_eeprom_delay();
static uint8_t get_V_target_10();
static void initTimer2( uint8_t pscale, uint8_t cmatch );
