#ifndef PLATFORM_H
#define PLATFORM_H
#include <stdint.h>
#include <stddef.h>

#ifdef PLATFORM_ATMEGA328P    /* only platform with 'real' Harvard arch and special needs for ROM access */
#define F_CPU                 16000000UL
#include <avr/pgmspace.h>
#define FLASH_MEM             PROGMEM
#define flash_read_byte(a)    pgm_read_byte(a)
#else  /* non-AVR */
#define FLASH_MEM
#define flash_read_byte(a)    (*(a))
#endif

void hw_init(int argc, char **argv);
void msleep(uint16_t val);

int i2c_xfer(uint8_t addr, const void *wr_p, size_t wr_sz, void *rd_p, size_t rd_sz);

void spi_tx(const void *p, size_t n);

enum {
	GPIO_TFT_NCS       = 0,
	GPIO_TFT_NRST      = 1,
	GPIO_TFT_A0        = 2,
	GPIO_INVALID       = 3
};

void gpio_set(uint8_t gpio_id, uint8_t val);

/* UART 'channels' are used to distinguish between the sensor-UART and the USB/etc. UART
 * the UART channel is *NOT* the hardware UART peripheral ID.
 *
 * mapping between UART channels and hardware UART peripherals is done by the
 * platform-specific code. */

enum {
	UART_SYS_CH        = 0,
	UART_SENSOR_CH     = 1,
	UART_INVALID       = 2
};

void uart_config(int ch, uint32_t baudrate); /* only 8N1 supported for now */
void uart_flush(int ch);
int  uart_tx(int ch, const void *p, size_t n);
int  uart_rx(int ch, void *p, size_t n, uint16_t timeout_ms);

#endif /* PLATFORM_H */
