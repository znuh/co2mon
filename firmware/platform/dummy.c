#include "platform.h"

void msleep(uint16_t val) {
}

int i2c_xfer(uint8_t addr, const void *wr_p, size_t wr_sz, void *rd_p, size_t rd_sz) {
	return 0;
}

void uart_config(int ch, uint32_t baudrate) {
}

int uart_tx(int ch, const void *p, size_t n) {
	return n;
}

int uart_rx(int ch, void *p, size_t n, uint16_t timeout_ms) {
	return 0;
}

void uart_flush(int ch) {
}

void spi_tx(const void *p, size_t n) {
}

void gpio_setval(uint8_t gpio_id, uint8_t val) {
}

void hw_init(int argc, char **argv) {	
}
