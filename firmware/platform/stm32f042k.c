#include "platform.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include <libopencmsis/core_cm3.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <stdint.h>
#include <inttypes.h>

/* IO assignments:
 * 
 * PA1 :          ~TFT_CS
 * PA2 : TX2      MCU_TX2
 * PA3 : RX2      MCU_RX2
 * PA5 : SCK      TFT_CLK
 * PA7 : MOSI     TFT_DATA
 * PA9 : TX1      TXD_S - to sensor
 * PA10: RX1      RXD_S - from sensor
 * PA11: USB_DM
 * PA12: USB_DP
 * 
 * PB6 : SCL
 * PB7 : SDA
 * 
 * PF0 :          ~TFT_RST
 * PF1 :          TFT_A0
 */

#define HZ      100
#define MSEC    (1000/HZ)
#define SEC     (1000*MSEC)

static volatile uint32_t jiffies = 0;

void sys_tick_handler(void) {
	jiffies++;
}

static void wait_until(uint32_t ts) {
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
//	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_SPI1);
}

static void systick_setup(void) {
	systick_set_frequency(HZ, rcc_ahb_frequency);
	systick_clear();
	systick_counter_enable();
	systick_interrupt_enable();
}

static void gpio_setup(void) {
#if 0
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

	/* D/C: PB6, RST: PB7 */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO6 | GPIO7);

	gpio_clear(GPIOC, GPIO13);	/* LED */
#endif
}

static void spi_setup(void) {
	//gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI1_REMAP);
#if 0
	/* SS=PA15, SCK=PB3, MISO=PB4 and MOSI=PB5 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15 );

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO3 | GPIO5 );

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO4);

	spi_reset(SPI1);
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
				  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);

	spi_enable(SPI1);
#endif
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
	clock_setup();
	systick_setup();
	gpio_setup();
	spi_setup();
}
