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
byte is always returned in the last byte (byte 4). Data is alwa‘ys sent on MOSI and MISO lines
with most significant bit (MSB) first.

*/

#define ISP_ENTER_PMODE_4BYTE   0xAC, 0x53, 0x00, 0x00

#define ISP_READ_PROG           0x20
#define ISP_READ_PROG_LOW       0x20
#define ISP_READ_PROG_HIGH      0x28

#define ISP_READ_SIG            0x30

#define ISP_LOAD_PROG_PAGE      0x40
#define ISP_LOAD_PROG_PAGE_LOW  0x40
#define ISP_LOAD_PROG_PAGE_HIGH 0x48

#define ISP_WRITE_PROG_PAGE     0x4C

#define ISP_READ_EEPROM         0xA0
#define ISP_WRITE_EEPROM        0xC0
#define ISP_LOAD_EEPROM_PAGE    0xC1
#define ISP_WRITE_EEPROM_PAGE   0xC2
