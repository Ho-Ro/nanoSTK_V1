# Name: Makefile
# Project: NanoSTK - The fastest AVR ISP interface using STK500 v1 protocol
# Tabsize: 4


######################################
# Nothing to change below (normally) #
######################################

PROJECT = nanoSTK

F_CPU = 16000000

DEVICE = atmega328p

# if not defined use default baudrate of 115200 bps
ifdef BAUDRATE
	DEFINE_BAUDRATE = -D BAUDRATE=$(BAUDRATE)
endif

AVRDUDE = avrdude

COMPILE = avr-gcc -std=c++11 -g -Wall -Wextra -Os -I. -mmcu=$(DEVICE) -DF_CPU=$(F_CPU)

SOURCES = $(PROJECT).cpp delay.cpp io.cpp spi.cpp uart.cpp

OBJECTS = $(PROJECT).o delay.o io.o spi.o uart.o

# files that contain config information
DEPENDS = nanoSTK.h delay.h io.h spi.h uart.h AVR061_command.h AVR910_ISP.h Makefile


####################
# symbolic targets #
####################

all:	$(PROJECT).hex

lst:	$(PROJECT).lst

.PHONY: upload
upload: all
	$(AVRDUDE) -p$(DEVICE) -c arduino -U flash:w:$(PROJECT).hex:i

.PHONY: verify
verify: all
	$(AVRDUDE) -p$(DEVICE) -c arduino -U flash:v:$(PROJECT).hex:i

.PHONY: clean
clean:
	rm -f $(PROJECT).hex $(PROJECT).lst $(PROJECT).obj $(PROJECT).cof
	rm -f $(PROJECT).lss $(PROJECT).map $(PROJECT).eep.hex $(PROJECT).elf
	rm -f *.o *~ *.bak $(PROJECT).s


################
# file targets #
################

$(PROJECT).o: $(PROJECT).cpp $(DEPENDS)
	$(COMPILE) $(DEFINE_BAUDRATE) -c $< -o $@

delay.o:delay.cpp $(DEPENDS)
	$(COMPILE) -c $< -o $@

io.o:	io.cpp $(DEPENDS)
	$(COMPILE) -c $< -o $@

spi.o:	spi.cpp $(DEPENDS)
	$(COMPILE) -c $< -o $@

uart.o:	uart.cpp $(DEPENDS)
	$(COMPILE) -c $< -o $@

$(PROJECT).elf:	$(OBJECTS)
	$(COMPILE) -o $@ $^

$(PROJECT).hex:	$(PROJECT).elf
	@rm -f $@ $(PROJECT).eep.hex
	avr-objcopy -j .text -j .data -O ihex $< $@
	@#avr-objdump -d $< > $(PROJECT).lss
	avr-size $<

$(PROJECT).lst:	$(PROJECT).elf
	avr-objdump -h -S $< > $@

