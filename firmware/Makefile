PROJECT          = co2mon
ELF_FILE         = $(PROJECT).elf
HEX_FILE         = $(PROJECT).hex
GENERIC_CFLAGS   = -Wall -I. -Os -o $(ELF_FILE)

# only used for libopencm3 stm32
BUILD_DIR = bin

LINUX_ARGS   = -DPLATFORM_LINUX      -DPLATFORM_STRING='"Linux"'
DUMMY_ARGS   = -DPLATFORM_DUMMY      -DPLATFORM_STRING='"Dummy"'
AVR_ARGS     = -DPLATFORM_ATMEGA328P -DPLATFORM_STRING='"ATMega328P"' -mmcu=atmega328p
STM32_ARGS   = -DPLATFORM_STM32      -DPLATFORM_STRING='"STM32F042K"'

COMMON_FILES = utils.c display.c sensors.c main.c
COMMON_FILES += sensors/hih6x.c sensors/lm75.c sensors/mhzxx.c sensors/scd30.c sensors/shtc3.c sensors/t67xx.c
COMMON_FILES += displays/oled.c displays/tft.c
COMMON_FILES += fonts/fonts.c

# STM32 target for libopencm3 makefile rules
CFILES = $(COMMON_FILES) platform/stm32f042k.c
TGT_CFLAGS += $(STM32_ARGS)

DEVICE=stm32f042k6t6
#OOCD_FILE = board/stm32f4discovery.cfg

#VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include stm32-rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

stm32: all
	$(PREFIX)size $(ELF_FILE)

stm32_flash: stm32 $(PROJECT).bin
	dfu-util -a 0 --dfuse-address 0x08000000 -D $(PROJECT).bin

linux:
	gcc $(GENERIC_CFLAGS) $(LINUX_ARGS) $(COMMON_FILES) platform/linux.c
	
dummy:
	gcc $(GENERIC_CFLAGS) $(DUMMY_ARGS) $(COMMON_FILES) platform/dummy.c

avr:
	avr-gcc $(GENERIC_CFLAGS) $(AVR_ARGS) $(COMMON_FILES) platform/atmega328p.c
	avr-size $(ELF_FILE)
	avr-objcopy -O ihex -R .eeprom $(ELF_FILE) $(HEX_FILE)

avr_flash: avr
	avrdude -pm328p -carduino -b57600 -P /dev/ttyUSB0 -Uflash:w:$(HEX_FILE):i
