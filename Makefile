# Build the *.hex file from the *.ino using arduino cli command
#
# Makefile targets:
# make hex	- build the programmer hexfile
# make flash	- upload the hex file to the nano flash (and verify automatically)
# make verify	- compare the hex file and the nano flash content
# make clean	- remove all intermediate build artifacts (keep the final *.hex file)

# The project
PROJECT = nanoSTK_V1

# The source code files
INO = $(PROJECT)/$(PROJECT).ino
SOURCE = $(PROJECT)/*.cpp
HEADER = $(PROJECT)/*.h

# The work space
BUILD = build

# The firmware (in work space)
INO.HEX = $(BUILD)/$(PROJECT).ino.hex

# The final firmware - ready to install
HEX = $(PROJECT).hex

# This is the default target of the Makefile
.PHONY:	hex
hex: $(HEX)

$(INO.HEX): $(INO) $(SOURCE) $(HEADER) Makefile
	arduino --pref build.path=$(BUILD) --verify $(INO) --verbose

$(HEX):	$(INO.HEX)
	@cp $< $@

.PHONY:	upload
upload:
	avrdude -p m328p -c arduino -U flash:w:$(HEX)

.PHONY:	verify
verify:
	avrdude -p m328p -c arduino -U flash:v:$(HEX)

.PHONY:	clean
clean:
	rm -rf $(BUILD)

.PHONY: format
format:
	clang-format -i $(INO) $(SOURCE)
