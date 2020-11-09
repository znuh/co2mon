# building for ATMega328P target
* make sure you have *avr-gcc + libc and binutils* installed
* run **make avr**

## flashing the ATMega328P
* make sure *avrdude* is installed
* run **avrdude -pm328p -carduino -b57600 -P /dev/ttyUSBx -Uflash:w:co2mon.hex:i** (replace ttyUSBx with your device)

# building for STM32F042K target
* make sure you have the *arm-none-eabi* toolchain installed
* get *libopencm3* with **git submodule init && git submodule update**
* build libopencm3 with **make -j8 -C libopencm3**
* run **make stm32**

## flashing the STM32
* make sure *dfu-util* is installed
* set the *BOOT_SEL* jumper (JP1) before connecting the board to your computer
* run **dfu-util -a 0 --dfuse-address 0x08000000 -D co2mon.bin**
