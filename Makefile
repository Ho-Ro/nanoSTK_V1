# Build the *.hex file from the *.ino using arduino cli command
#
# Makefile targets:
# make hex	- build the programmer hexfile
# make flash	- upload the hex file to the nano flash (and verify automatically)
# make verify	- compare the hex file and the nano flash content
# make clean	- remove all intermediate build artifacts (keep the final *.hex file)

PROJECT = nanoSTK_V1

BUILD = build

INO = $(PROJECT)/$(PROJECT).ino
HEX = $(PROJECT).ino.hex


.PHONY:	hex
hex: $(HEX)

$(BUILD)/$(HEX): $(INO)
	arduino --pref build.path=$(BUILD) --verify $(INO) --verbose

$(HEX):	$(BUILD)/$(HEX)
	@cp $< $@

.PHONY:	flash
flash:	$(HEX)
	avrdude -p m328p -c arduino -U flash:w:$(HEX)

.PHONY:	verify
verify:	$(HEX)
	avrdude -p m328p -c arduino -U flash:v:$(HEX)

.PHONY:	clean
clean:
	rm -rf $(BUILD)

