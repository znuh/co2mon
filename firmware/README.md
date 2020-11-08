# building for ATMega328P target
* make sure you have *avr-gcc + libc and binutils* installed
* run **make avr**

# building for STM32F042K target
* make sure you have the *arm-none-eabi* toolchain installed
* get *libopencm3* with **git submodule init && git submodule update**
* build libopencm3 with **make -j8 -C libopencm3**
* run **make stm32**
