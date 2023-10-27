/*
Application Note AVR910: In-System Programming

Command Format

All commands have a common format consisting of four bytes. The first byte contains the com-
mand code, selecting operation and target memory. The second and third byte contain the
address of the selected memory area. The fourth byte contains the data, going in either
direction.
The data returned from the target is usually the data sent in the previous byte. Table 3 shows an
example, where two consecutive commands are sent to the target. Notice how all bytes returned
equal the bytes just received. Some commands return one byte from the target’s memory. This
byte is always returned in the last byte (byte 4). Data is always sent on MOSI and MISO lines
with most significant bit (MSB) first.

*/

// check 3 signature bytes
#define SIG_SIZE                3

#define ISP_ENTER_PMODE_BYTE_0   0xAC
#define ISP_ENTER_PMODE_BYTE_1   0x53

#define ISP_READ_PROG           0x20
// "0010.0000--00aa.aaaa--aaaa.aaaa--oooo.oooo"; (all examples are for m328)

#define ISP_READ_PROG_LOW       0x20
// "0010.0000--00aa.aaaa--aaaa.aaaa--oooo.oooo";

#define ISP_READ_PROG_HIGH      0x28
// "0010.1000--00aa.aaaa--aaaa.aaaa--oooo.oooo";


#define ISP_READ_SIG            0x30
// "0011.0000--000x.xxxx--xxxx.xxaa--oooo.oooo";


#define ISP_LOAD_PROG_PAGE      0x40
// "0100.0000--000x.xxxx--xxaa.aaaa--iiii.iiii";

#define ISP_LOAD_PROG_PAGE_LOW  0x40
// "0100.0000--000x.xxxx--xxaa.aaaa--iiii.iiii";

#define ISP_LOAD_PROG_PAGE_HIGH 0x48
// "0100.1000--000x.xxxx--xxaa.aaaa--iiii.iiii";


#define ISP_WRITE_PROG_PAGE     0x4C
// "0100.1100--00aa.aaaa--aaxx.xxxx--xxxx.xxxx";


#define ISP_READ_EEPROM         0xA0
// "1010.0000--000x.xxaa--aaaa.aaaa--oooo.oooo";

#define ISP_WRITE_EEPROM        0xC0
// "1100.0000--000x.xxaa--aaaa.aaaa--iiii.iiii";

#define ISP_LOAD_EEPROM_PAGE    0xC1
// "1100.0001--0000.0000--0000.00aa--iiii.iiii";

#define ISP_WRITE_EEPROM_PAGE   0xC2
// "1100.0010--00xx.xxaa--aaaa.aa00--xxxx.xxxx";
