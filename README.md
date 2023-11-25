# nanoSTK_V1

## Using Arduino Nano as AVR ISP with STK500 v1 protocol

This project enables the use of an Arduino Nano board with minor modifications (nanoSTK)
as an in-system programmer (ISP) for AVR devices like ATtiny and ATmega
together with a programming software, e.g. [AVRDUDE](https://github.com/avrdudes/avrdude/).

The program is based on:
[ArduinoISP](https://github.com/rsbohn/ArduinoISP) - Copyright (c) 2008-2011 Randall Bohn

## Programmer Hardware
The programming process uses VCC, GND and four data pins.
By default, the hardware SPI pins MISO, MOSI and SCK are used to communicate with the target.
The fourth pin (D10) from the programming microprocessor goes to the reset pin of the target.
On all Arduinos, these pins are found on the ICSP/SPI header:

```
              MISO ¹* * 5V (!)
              SCK   * * MOSI
       D10 (/RESET) * * GND
```

### HW Modifications for nanoSTK

#### Required HW Changes

- Cut ISP header /RESET connection and connect this pin to D10.
- Connect a capacitor of about 3..10 μF between the /RESET line (+) of the nano and GND (-).
  This capacitor must be removed to reprogram the nanoSTK.
  It is therefore recommended to make the capacitor pluggable.

With this setup the modified nanoSTK provides the supply voltage of 5 V to the target.

#### Optional Modifications

Put LEDs (with resistor) on the following pins:

- D9: Heartbeat - Indicates that the programmer is running
- D8: Error     - An Error has occured - clear with programmer reset
- D7: Write     - Writing to the target
- D6: Read      - Reading from the target
- D5: PMode     - Target in programming mode

D5 .. D8 are identical to the [ScratchMonkey](https://github.com/microtherion/ScratchMonkey) status LEDs.

## Firmware Development and Installation

The source code was created and tested with the *Arduino* toolchain version 1.8.19 under Linux, *Debian stable*.
Starting with version 1.50 the build process no longer depends on the *Arduino* toolchain.
You only need the *avr-gcc* toolchain.

The `Makefile` takes care of changes to the source code files and rebuilds the file
`nanoSTK.hex` accordingly, using the *avr-gcc* toolchain - type `make`

The installation process is simple and straightforward when using the `Makefile` from the command line:

- Remove the *reset capacitor* from your *nanoSTK* HW.
- Connect the device to USB.
- Enter `make upload`
- Re-install the *reset capacitor*.

Instead of removing/inserting the *reset capacitor* you can also press the reset button
as soon as the `avrdude` command is started.
The upload of the file `nanoSTK.hex` with `make upload` only requires the `avrdude` tool.
If the source code remains unchanged, the *avr-gcc* tool chain is not required.

### Other OS

You can also use the Arduino tool chain, this should work on all supported operation systems.
Check out the source and hex files (version 1.30) from the branch `arduino_toolchain`.

- Remove the *reset capacitor* from your *nanoSTK* HW.
- Connect the device to USB.
- Open `nanoSTK_V1/nanoSTK_V1.ino` with the Arduino tool chain.
- Select `Tools/Board: "Arduino Nano"`.
- Select the correct serial port.
- Execute `Sketch/Upload` from the menu.
- Reinstall the *reset capacitor*.

The upload of the file `nanoSTK_V1.hex` with `make upload` only requires the `avrdude` tool.
If the source code remains unchanged, the *Arduino* tool chain is not required.
This branch `arduino_toolchain` will not be developed further, only bugfixes will be applied.

## Usage

The communication uses the `stk500v1` protocol over serial USB with a data rate of 115200 bps, this is the default speed of `avrdude`.

### Config File .avrduderc

Put this file into your home directory (Linux) to set the nanoSTK device as the default programmer:

```
# file ~/.avrduderc

default_programmer  = "stk500v1";

default_serial = "/dev/ttyUSB0";
```

### Use with AVRDUDE

A typical call for programming the file `firmware.hex` into an ATtiny 85 looks like this:

```sh
avrdude -p t85 -U flash:w:firmware.hex:i
```

This simple call

```sh
avrdude -p t85 -v
```

will show the programmer setup and device info for the ATtiny 85:

```
avrdude-git: Version 7.2-20231105 (ded89a66)
             Copyright the AVRDUDE authors;
             see https://github.com/avrdudes/avrdude/blob/main/AUTHORS

             System wide configuration file is ~/projects/AVR/avrdude/build_linux/src/avrdude.conf
             User configuration file is ~/.avrduderc

             Using Port                    : /dev/ttyUSB0
             Using Programmer              : stk500v1
             AVR Part                      : ATtiny85
             Chip Erase delay              : 4500 us
             RESET disposition             : possible i/o
             RETRY pulse                   : SCK
             Serial program mode           : yes
             Parallel program mode         : yes
             Timeout                       : 200
             StabDelay                     : 100
             CmdexeDelay                   : 25
             SyncLoops                     : 32
             PollIndex                     : 3
             PollValue                     : 0x53
             Memory Detail                 :

                                               Block Poll               Page                       Polled
               Memory Type Alias    Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack
               ----------- -------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------
               eeprom                 65     5     4    0 no        512    4      0  4000  4500 0x00 0x00
               flash                  65    10    64    0 yes      8192   64    128  4500  4500 0x00 0x00
               lfuse                   0     0     0    0 no          1    1      0  9000  9000 0x00 0x00
               hfuse                   0     0     0    0 no          1    1      0  9000  9000 0x00 0x00
               efuse                   0     0     0    0 no          1    1      0  9000  9000 0x00 0x00
               lock                    0     0     0    0 no          1    1      0  9000  9000 0x00 0x00
               signature               0     0     0    0 no          3    1      0     0     0 0x00 0x00
               calibration             0     0     0    0 no          2    1      0     0     0 0x00 0x00
               io                      0     0     0    0 no         64    1      0     0     0 0x00 0x00

             Programmer Type : STK500
             Description     : Atmel STK500 version 1.x firmware
             Hardware Version: 2
             Firmware Version: 1.27
             Vtarget         : 4.8 V
             Varef           : 0.0 V
             Oscillator      : 1000.000 kHz
             SCK period      : 1.1 us
avrdude-git: AVR device initialized and ready to accept instructions
avrdude-git: device signature = 0x1e930b (probably t85)

avrdude-git done.  Thank you.

```

### Programming Speed

The programming algorithms are optimised for speed:

- unchanged EEPROM blocks are not programmed.
- the total serial communication overhead is taken into account for the required programming delays.

These times were measured on an ATmega328p and an ATtiny85 with random data after flash and EEPROM were filled with `0xFF`.

Device | Flash/EEPROM | Flash write | Flash verify | EEPROM write | EEPROM verify
-------|--------------|-------------|--------------|--------------|---------------
m328p  | 32K / 1024   |      5.80 s |       4.36 s |       1.79 s |        1.79 s
t85    | 8K / 512     |      1.91 s |       1.54 s |       0.90 s |        0.89 s

### Programming Slow Targets

When programming targets with a clock speed lower than 6 MHz the SPI can be slowed down from 1 MHz to 250 kHz,
this allows to communicate with targets using the default setup (internal 8 MHz oscillator divided by 8).
To slow down connect pin D2 with GND and press the reset button. You can also use the terminal command `sck 4`.

### Providing External Clock

To program a target without own clock generation, e.g. a processor on an adapter board, an external 1 MHz clock (5 V) is provided on pin D3.
The clock frequncy can be changed with the terminal command `fosc`, e.g. `fosc 100k` sets 100 kHz.

## Programming Protocol

The nanoSTK firmware uses the STK500 protocol, version 1 (`stk500v1`) as default.

**Caution!** The  Arduino bootloader uses a modified version of the `stk500v1` protocol, called `arduino`,
even if describing it as *"Arduino for bootloader using STK500 v1 protocol"*:

- **EEPROM access is handled differently:**
  - Original `stk500v1` uses byte addresses (e.g 0..1023 for a device with 1K EEPROM).
  - Modified `arduino` uses word addresses also for the EEPROM (0..511 for a 1K device).
- The `arduino` protocol does not support Vtarget and Varef.
- The `stk500v1` protocol uses three `Cmnd_STK_UNIVERSAL` (0x56 = 'V') calls to get the signature.
- The `arduino` protocol retrives the signature with the command `Cmnd_STK_READ_SIGN` (0x75 = 'u').

The firmware automatically recognises the modified `arduino` protocol by its use of the command
`Cmnd_STK_READ_SIGN` and adjusts the EEPROM addressing accordingly.

    avrdude -p t85 -c arduino -v

```
         ...
         Programmer Type : Arduino
         Description     : Arduino for bootloader using STK500 v1 protocol
         Hardware Version: 2
         Firmware Version: 1.27

avrdude: AVR device initialized and ready to accept instructions
avrdude: device signature = 0x1e930b (probably t85)

avrdude done.  Thank you.
```
