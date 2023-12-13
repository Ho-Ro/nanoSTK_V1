## Programming Speed

Comparison between the original STK500 and three FW variants for a standard Arduino Nano board with minimal HW modification:
- Cut ISP header /RESET connection (pin5) and connect this pin to D10 (PB2).
- Connect a capacitor of about 3..10 μF between the /RESET line (+) of the Nano and GND (-).

- Target: ATmega328p 8MHz internal clock - flash memory
- Data: 32K random data

Programmer      | Flash write | Flash verify
----------------|-------------|-------------
STK500          | 12.28 s     | 11.31 s
arduino as ISP  | 36.61 s     | 20.21 s
ScratchMonkey   |  8.18 s     |  7.97 s
nanoSTK         |  6.15 s     |  4.36 s
nanoSTK 500kbps |  3.84 s     |  3.33 s
nanoSTK 1Mbps   |  3.58 s     |  3.33 s

### Original Atmel STK500
```
$ avrdude-git -pm328p -c stk500v2 -U flash:w:random_K32.hex:i -B0.5 -v

avrdude-git: Version 7.2-20231201 (5501c63a)
             Copyright the AVRDUDE authors;
             see https://github.com/avrdudes/avrdude/blob/main/AUTHORS

             System wide configuration file is /home/horo/projects/AVR/avrdude/build_linux/src/avrdude.conf
             User configuration file is /home/horo/.avrduderc

             Using port            : /dev/ttyUSB0
             Using programmer      : stk500v2
             Setting bit clk period: 0.5 us
             AVR Part              : ATmega328P
             Programming modes     : ISP, HVPP, debugWIRE, SPM
             Programmer Type       : STK500V2
             Description           : Atmel STK500 version 2.x firmware
             Programmer model      : STK500
             HW version            : 2
             FW Version Controller : 2.01
             Topcard               : Unknown
             Vtarget               : 5.0 V
             Varef                 : 0.0 V
             Oscillator            : 3.686400 MHz
             SCK period            : 0.5 us
             XTAL frequency        : 7.372800 MHz
avrdude-git: AVR device initialized and ready to accept instructions
avrdude-git: device signature = 0x1e950f (probably m328p)
avrdude-git: Note: flash memory has been specified, an erase cycle will be performed.
             To disable this feature, specify the -D option.
avrdude-git: erasing chip

avrdude-git: processing -U flash:w:random_K32.hex:i
avrdude-git: reading input file random_K32.hex for flash
             with 32768 bytes in 1 section within [0, 0x7fff]
             using 256 pages and 0 pad bytes
avrdude-git: writing 32768 bytes flash ...
Writing | ################################################## | 100% 12.28 s
avrdude-git: 32768 bytes of flash written
avrdude-git: verifying flash memory against random_K32.hex
Reading | ################################################## | 100% 11.31 s
avrdude-git: 32768 bytes of flash verified

avrdude-git done.  Thank you.
```

### Arduino as ISP (Arduino Example Sketch)
```
$ avrdude-git -pm328p -c arduino_as_isp -U flash:w:random_K32.hex:i -b19200 -v

avrdude-git: Version 7.2-20231201 (5501c63a)
             Copyright the AVRDUDE authors;
             see https://github.com/avrdudes/avrdude/blob/main/AUTHORS

             System wide configuration file is /home/horo/projects/AVR/avrdude/build_linux/src/avrdude.conf
             User configuration file is /home/horo/.avrduderc

             Using port            : /dev/ttyUSB0
             Using programmer      : arduino_as_isp
             Setting baud rate     : 19200
             AVR Part              : ATmega328P
             Programming modes     : ISP, HVPP, debugWIRE, SPM
             Programmer Type       : STK500
             Description           : Arduino board as programmer using arduino as ISP firmware
             HW Version            : 2
             FW Version            : 1.18
             Topcard               : Unknown
             SCK period            : 0.0 us
             XTAL frequency        : 7.372800 MHz
avrdude-git: AVR device initialized and ready to accept instructions
avrdude-git: device signature = 0x1e950f (probably m328p)
avrdude-git: Note: flash memory has been specified, an erase cycle will be performed.
             To disable this feature, specify the -D option.
avrdude-git: erasing chip

avrdude-git: processing -U flash:w:random_K32.hex:i
avrdude-git: reading input file random_K32.hex for flash
             with 32768 bytes in 1 section within [0, 0x7fff]
             using 256 pages and 0 pad bytes
avrdude-git: writing 32768 bytes flash ...
Writing | ################################################## | 100% 36.61 s
avrdude-git: 32768 bytes of flash written
avrdude-git: verifying flash memory against random_K32.hex
Reading | ################################################## | 100% 20.22 s
avrdude-git: 32768 bytes of flash verified

avrdude-git done.  Thank you.
```

### Scratchmonkey FW
For info about HW/SW see the [ScratchMonkey on GitHub](https://github.com/microtherion/ScratchMonkey).
```
$ avrdude-git -pm328p -c scratchmonkey -U flash:w:random_K32.hex:i -v

avrdude-git: Version 7.2-20231201 (5501c63a)
             Copyright the AVRDUDE authors;
             see https://github.com/avrdudes/avrdude/blob/main/AUTHORS

             System wide configuration file is /home/horo/projects/AVR/avrdude/build_linux/src/avrdude.conf
             User configuration file is /home/horo/.avrduderc

             Using port            : /dev/ttyUSB0
             Using programmer      : scratchmonkey
             AVR Part              : ATmega328P
             Programming modes     : ISP, HVPP, debugWIRE, SPM
             Programmer Type       : STK500V2
             Description           : Atmel STK500 version 2.x firmware
             Programmer model      : SCRATCHMONKEY
             HW version            : 0
             FW Version Controller : 2.00
             Topcard               : Unknown
             Vtarget               : 5.0 V
             Varef                 : 5.0 V
             Oscillator            : Off
             SCK period            : 1.0 us
             XTAL frequency        : 16.000000 MHz
avrdude-git: AVR device initialized and ready to accept instructions
avrdude-git: device signature = 0x1e950f (probably m328p)
avrdude-git: Note: flash memory has been specified, an erase cycle will be performed.
             To disable this feature, specify the -D option.
avrdude-git: erasing chip

avrdude-git: processing -U flash:w:random_K32.hex:i
avrdude-git: reading input file random_K32.hex for flash
             with 32768 bytes in 1 section within [0, 0x7fff]
             using 256 pages and 0 pad bytes
avrdude-git: writing 32768 bytes flash ...
Writing | ################################################## | 100% 8.18 s
avrdude-git: 32768 bytes of flash written
avrdude-git: verifying flash memory against random_K32.hex
Reading | ################################################## | 100% 7.97 s
avrdude-git: 32768 bytes of flash verified

avrdude-git done.  Thank you.
```

### nanoSTK FW
```
$ avrdude-git -pm328p -c nanoSTK -U flash:w:random_K32.hex:i -v

avrdude-git: Version 7.2-20231201 (5501c63a)
             Copyright the AVRDUDE authors;
             see https://github.com/avrdudes/avrdude/blob/main/AUTHORS

             System wide configuration file is /home/horo/projects/AVR/avrdude/build_linux/src/avrdude.conf
             User configuration file is /home/horo/.avrduderc

             Using port            : /dev/ttyUSB0
             Using programmer      : nanoSTK
             AVR Part              : ATmega328P
             Programming modes     : ISP, HVPP, debugWIRE, SPM
             Programmer Type       : STK500
             Description           : nanoSTK - fast ISP using stk500v1 protocol
             HW Version            : 2
             FW Version            : 1.51
             Vtarget               : 4.8 V
             Oscillator            : 8.000000 MHz
             SCK period            : 1.0 us
             XTAL frequency        : 16.000000 MHz
avrdude-git: AVR device initialized and ready to accept instructions
avrdude-git: device signature = 0x1e950f (probably m328p)
avrdude-git: Note: flash memory has been specified, an erase cycle will be performed.
             To disable this feature, specify the -D option.
avrdude-git: erasing chip

avrdude-git: processing -U flash:w:random_K32.hex:i
avrdude-git: reading input file random_K32.hex for flash
             with 32768 bytes in 1 section within [0, 0x7fff]
             using 256 pages and 0 pad bytes
avrdude-git: writing 32768 bytes flash ...
Writing | ################################################## | 100% 6.15 s
avrdude-git: 32768 bytes of flash written
avrdude-git: verifying flash memory against random_K32.hex
Reading | ################################################## | 100% 4.36 s
avrdude-git: 32768 bytes of flash verified

avrdude-git done.  Thank you.
```

### nanoSTK FW
The serial USB communication becomes the clear bottleneck, you can speed up further with higher `BAUDRATE` setting in the FW,
e.g. a speed of 500 kbps significantly reduces the programming time.
```
$ avrdude-git -pm328p -c nanoSTK -U flash:w:random_K32.hex:i -v -b500000

avrdude-git: Version 7.2-20231201 (5501c63a)
             Copyright the AVRDUDE authors;
             see https://github.com/avrdudes/avrdude/blob/main/AUTHORS

             System wide configuration file is /home/horo/projects/AVR/avrdude/build_linux/src/avrdude.conf
             User configuration file is /home/horo/.avrduderc

             Using port            : /dev/ttyUSB0
             Using programmer      : nanoSTK
             Setting baud rate     : 500000
             AVR Part              : ATmega328P
             Programming modes     : ISP, HVPP, debugWIRE, SPM
             Programmer Type       : STK500
             Description           : nanoSTK - fast ISP using stk500v1 protocol
             HW Version            : 2
             FW Version            : 1.51
             Vtarget               : 4.8 V
             Oscillator            : 8.000000 MHz
             SCK period            : 1.0 us
             XTAL frequency        : 16.000000 MHz
avrdude-git: AVR device initialized and ready to accept instructions
avrdude-git: device signature = 0x1e950f (probably m328p)
avrdude-git: Note: flash memory has been specified, an erase cycle will be performed.
             To disable this feature, specify the -D option.
avrdude-git: erasing chip

avrdude-git: processing -U flash:w:random_K32.hex:i
avrdude-git: reading input file random_K32.hex for flash
             with 32768 bytes in 1 section within [0, 0x7fff]
             using 256 pages and 0 pad bytes
avrdude-git: writing 32768 bytes flash ...
Writing | ################################################## | 100% 3.84 s
avrdude-git: 32768 bytes of flash written
avrdude-git: verifying flash memory against random_K32.hex
Reading | ################################################## | 100% 3.33 s
avrdude-git: 32768 bytes of flash verified

avrdude-git done.  Thank you.
```
