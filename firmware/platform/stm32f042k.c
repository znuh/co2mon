#include "platform.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/i2c.h>

#include <libopencmsis/core_cm3.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <stdint.h>
#include <inttypes.h>

/* IO assignments:
 * 
 * PA1 :             ~TFT_CS
 * PA2 : TX2     AF1 MCU_TX2   - RFU
 * PA3 : RX2     AF1 MCU_RX2   - RFU
 * PA5 : SCK     AF0 TFT_CLK
 * PA7 : MOSI    AF0 TFT_DATA
 * PA9 : TX1     AF1 TXD_S - to sensor
 * PA10: RX1     AF1 RXD_S - from sensor
 * PA11: USB_DM  
 * PA12: USB_DP
 * 
 * PB6 : SCL     AF1
 * PB7 : SDA     AF1
 * 
 * PF0 :             ~TFT_RST
 * PF1 :             TFT_A0
 */

static const uint32_t gpio_map[3][2] = {
	{GPIOA, GPIO1},  /* GPIO_TFT_NCS   */
	{GPIOF, GPIO0},  /* GPIO_TFT_NRST  */
	{GPIOF, GPIO1}   /* GPIO_TFT_A0 */
};

static inline void spi_busywait(uint32_t spi) {
	while ((SPI_SR(spi) & SPI_SR_BSY));
}

void gpio_setval(uint8_t gpio_id, uint8_t val) {
	if(gpio_id >= GPIO_INVALID)
		return;
	spi_busywait(SPI1);
	if(val)
		gpio_set(gpio_map[gpio_id][0], gpio_map[gpio_id][1]);
	else
		gpio_clear(gpio_map[gpio_id][0], gpio_map[gpio_id][1]);
}

static void gpio_setup(void) {
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3 | GPIO10);          /* RXDs */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,   GPIO5 | GPIO7 | GPIO9);   /* UART1, SPI1 */
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO9);          /* TXD1 */
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO5 | GPIO7); /* SPI1 */
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9|GPIO10); /* UART1 */
	gpio_set_af(GPIOA, GPIO_AF0, GPIO5|GPIO7);  /* SPI1  */

	/* ~TFT_CS */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
	gpio_set(GPIOA, GPIO1);

	/* I2C1 */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
	gpio_set_af(GPIOB, GPIO_AF1, GPIO6|GPIO7);

	/* TFT nRST, A0 */
	gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
	gpio_set_output_options(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO0 | GPIO1);
	gpio_clear(GPIOA, GPIO1|GPIO0);
}

#define HZ      100
#define MSEC    (1000/HZ)
#define SEC     (1000*MSEC)

static volatile uint32_t jiffies = 0;

void sys_tick_handler(void) {
	jiffies++;
}

static void wait_until(uint32_t ts) {
	while(ts < jiffies) { __WFI(); } /* uint32 rollover case */
	while(jiffies < ts) { __WFI(); }
}

void msleep(uint16_t val) {
	wait_until(jiffies + ((val+(MSEC-1))/MSEC));
}

#if 0
static void udelay(uint32_t n) {
	while(n--)
		__asm__("nop");
}
#endif

static void clock_setup(void) {
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOF);
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_I2C1);
}

static void systick_setup(void) {
	systick_set_frequency(HZ, rcc_ahb_frequency);
	systick_clear();
	systick_counter_enable();
	systick_interrupt_enable();
}

static void spi_setup(void) {
	spi_reset(SPI1);
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, 
		SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_MSBFIRST);
	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);
	spi_enable(SPI1);
}

static void i2c_setup(void) {
}

static void uart_setup(void) {
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
	const uint8_t *d=p;
	for(;n;n--,d++)
		spi_send(SPI1, *d);
}

void hw_init(int argc, char **argv) {
	argc=argc; argv=argv;
	clock_setup();
	systick_setup();
	gpio_setup();
	spi_setup();
	uart_setup();
	i2c_setup();
}
