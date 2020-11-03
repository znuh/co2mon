
PROJECT      = co2mon
ELF_FILE     = $(PROJECT).elf
HEX_FILE     = $(PROJECT).hex
CFLAGS       = -Wall -I. -Os -o $(ELF_FILE)
COMMON_FILES = main.c sensors.c display.c utils.c sensors/*.c displays/*.c fonts/fonts.c

LINUX_ARGS   = -DPLATFORM_LINUX      -DPLATFORM_STRING='"Linux"'
DUMMY_ARGS   = -DPLATFORM_DUMMY      -DPLATFORM_STRING='"Dummy"'
AVR_ARGS     = -DPLATFORM_ATMEGA328P -DPLATFORM_STRING='"ATMega328P"' -mmcu=atmega328p
STM32_ARGS   = -DPLATFORM_STM32      -DPLATFORM_STRING='"STM32F042K"'

linux:
	gcc $(CFLAGS) $(LINUX_ARGS) $(COMMON_FILES) platform/linux.c
	
dummy:
	gcc $(CFLAGS) $(DUMMY_ARGS) $(COMMON_FILES) platform/dummy.c

avr:
	avr-gcc $(CFLAGS) $(AVR_ARGS) $(COMMON_FILES) platform/atmega328p.c
	avr-size $(ELF_FILE)
	avr-objcopy -O ihex -R .eeprom $(ELF_FILE) $(HEX_FILE)

avr_flash:
	avrdude -pm328p -carduino -b57600 -P /dev/ttyUSB0 -Uflash:w:co2mon.hex:i
