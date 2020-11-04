#include "platform.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>

#include <libopencmsis/core_cm3.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <stdint.h>
#include <inttypes.h>

#include "utils.h"

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
	spi_busywait(SPI1); /* ensure previous tft write has finished */
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
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO1);
	gpio_set(GPIOA, GPIO1);

	/* I2C1 */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
	gpio_set_af(GPIOB, GPIO_AF1, GPIO6|GPIO7);

	/* TFT nRST, A0 */
	gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
	gpio_set_output_options(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO0 | GPIO1);
	gpio_clear(GPIOF, GPIO1|GPIO0);
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
	wait_until(jiffies + ((val+(MSEC-1))/MSEC) + 1);
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
	/* ST7735S TSCYCW: min 66ns (~16MHz), so 48MHz/4=12MHz should be fine */
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_4, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, 
		SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_MSBFIRST);
	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);
	spi_enable(SPI1);
}

static void i2c_setup(void) {
	rcc_set_i2c_clock_sysclk(I2C1);
	i2c_peripheral_disable(I2C1);
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, 0);
	i2c_set_speed(I2C1, i2c_speed_sm_100k, 48);
	i2c_peripheral_enable(I2C1);
	i2c_disable_autoend(I2C1); /* disable auto STOP generation */
}

/* I2C_ISR_BUSY: Bus busy
 * 
 * I2C_ISR_TIMEOUT: Timeout or t_LOW detection flag
 * I2C_ISR_PECERR: PEC Error in reception
 * I2C_ISR_OVR: Overrun/Underrun (slave mode)
 * I2C_ISR_ARLO: Arbitration lost
 * I2C_ISR_BERR: Bus error
 * 
 * I2C_ISR_TCR: Transfer Complete Reload
 * I2C_ISR_TC: Transfer Complete (master mode)
 * I2C_ISR_STOPF: Stop detection flag
 * I2C_ISR_NACKF: Not Acknowledge received flag
 * 
 * I2C_ISR_RXNE: Receive data register not empty (receivers)
 * I2C_ISR_TXIS: Transmit interrupt status (transmitters)
 * I2C_ISR_TXE: Transmit data register empty (transmitters)
 */
#define I2C_ISR_ERRORMASK      (I2C_ISR_TIMEOUT|I2C_ISR_PECERR|I2C_ISR_OVR|I2C_ISR_ARLO|I2C_ISR_BERR)
#define I2C_ICR_CLEARMASK      (I2C_ICR_ALERTCF|I2C_ICR_TIMOUTCF|I2C_ICR_PECCF|I2C_ICR_OVRCF|I2C_ICR_ARLOCF| \
                                I2C_ICR_BERRCF|I2C_ICR_STOPCF|I2C_ICR_NACKCF|I2C_ICR_ADDRCF)
#define I2C_SOFT_TIMEOUT       65536 /* assuming 48MHz, 5 cycles per loop: 65536*5*20.83ns = ~6.8ms */
static uint32_t i2c_status_wait(uint32_t flags) {
	uint32_t sr, soft_timeout=I2C_SOFT_TIMEOUT;
	for(;soft_timeout;soft_timeout--) {
		sr = I2C_ISR(I2C1);
		if(sr&I2C_ISR_ERRORMASK)
			break;
		if(sr&I2C_ISR_BUSY)
			continue;
		if((sr&flags) || (!flags)) /* return if at least 1 flag matches */
			break;
	}
	return sr;
}

#define I2C_READ    1
#define I2C_WRITE   0

static void i2c_start(uint8_t addr, uint8_t read_nwrite, size_t n) {
	i2c_set_7bit_address(I2C1, addr);
	if(read_nwrite == I2C_READ)
		i2c_set_read_transfer_dir(I2C1);
	else
		i2c_set_write_transfer_dir(I2C1);
	i2c_set_bytes_to_transfer(I2C1, MIN(n,255));
	if(n>255)
		I2C_CR2(I2C1) |= I2C_CR2_RELOAD;
	else
		I2C_CR2(I2C1) &= ~I2C_CR2_RELOAD;
	i2c_send_start(I2C1);
}

#define I2C_TX_STATUSMASK     (I2C_ISR_NACKF | I2C_ISR_TXIS)

/* this function should resemble the flow-charts from figures 231, 232 of RM0091 */
static int i2c_write(uint8_t addr, const uint8_t *d, size_t n) {
	uint32_t nbytes = MIN(n,255), reload = n>255;

	i2c_start(addr,I2C_WRITE,n);

	while(n) {
		uint32_t sr = i2c_status_wait(I2C_TX_STATUSMASK);
		if(sr != I2C_ISR_TXIS)
			return 0;
		i2c_send_data(I2C1, *d++);
		n--;
		nbytes--;
		if(nbytes) /* continue until current chunk complete */
			continue;
		else { /* current chunk complete */
			sr = i2c_status_wait(I2C_ISR_TC|I2C_ISR_TCR);
			if(!(sr & I2C_ISR_TC)) /* TC not set -> error */
				return 0;
			if(!reload) /* last chunk -> done */
				return 1;
			/* not last chunk - expect TCR set */
			if(!(sr & I2C_ISR_TCR)) /* TCR not set -> error */
				return 0;
			/* next chunk */
			nbytes =  MIN(n,255);
			i2c_set_bytes_to_transfer(I2C1, nbytes);
			reload = n>255;
			if(reload)
				I2C_CR2(I2C1) |= I2C_CR2_RELOAD;
			else
				I2C_CR2(I2C1) &= ~I2C_CR2_RELOAD;
		} /* current nbytes chunk complete */
	} /* foreach byte to write */
	
	return 0; /* shouldn't be reached */
}

static int i2c_read(uint8_t addr, uint8_t *d, size_t n) {
	uint32_t nbytes = MIN(n,255), reload = n>255;

	i2c_start(addr,I2C_READ,n);
	
	/* TBD */
	return 0;
}

int i2c_xfer(uint8_t addr, const void *wr_p, size_t wr_sz, void *rd_p, size_t rd_sz) {
	const uint8_t *wr_d = wr_p;
	uint8_t *rd_d = rd_p;
	int n=0;

	/* wait if i2c peripheral still busy */
	i2c_status_wait(0);

	/* do a sort-of soft reset if error flag(s) set
	 * resets state machine and all error flags */
	if(I2C_ISR(I2C1) & I2C_ISR_ERRORMASK) {
		int i;
		i2c_peripheral_disable(I2C1);
		/* RM0091: PE must be kept low for at least 3 APB clock cycles */
		for(i=5;i;i--) {
			uint32_t dummy = I2C_ISR(I2C1);
			dummy=dummy; /* silence the compiler warning */
		}
		i2c_peripheral_enable(I2C1);
	}
	
	/* clear all flags */
	I2C_ICR(I2C1) = I2C_ICR_CLEARMASK;

	/* write cycle */
	if((wr_d) && (wr_sz>0)) {
		int res = i2c_write(addr, wr_p, wr_sz);
		if(!res)
			goto out;
		/* no stop here b/c repeated start follows or stop before function return */
		n++;
	}

	/* read cycle */
	if((rd_d) && (rd_sz>0)) {
		int res = i2c_read(addr, rd_p, rd_sz);
		if(!res)
			goto out;
		n++;
	}

out:
	i2c_send_stop(I2C1);
	return n;
}

static uint32_t uart1_baudrate = 9600;

static void uart_setup(void) {
	usart_set_baudrate(USART1, uart1_baudrate);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    nvic_enable_irq(NVIC_USART1_IRQ);
	USART_CR1(USART1) |= USART_CR1_RXNEIE;
	usart_enable(USART1);
}

void uart_config(int ch, uint32_t baudrate) {
	if(ch != UART_SENSOR_CH)
		return;
	if(baudrate == uart1_baudrate)
		return;
	uart1_baudrate = baudrate;
	usart_set_baudrate(USART1, uart1_baudrate);
}

int uart_tx(int ch, const void *p, size_t n) {
	const uint8_t *d=p;
	if(ch != UART_SENSOR_CH)
		return n;
	for(;n;n--,d++)
		usart_send_blocking(USART1, *d);
	return n;
}

#define RXBUF_SIZE      32

static volatile uint8_t rxbuf[RXBUF_SIZE];
static volatile uint32_t rx_put = 0;
static volatile uint32_t rx_get = 0;

void usart1_isr(void) {
	if (USART_ISR(USART1) & USART_ISR_RXNE) {
		uint8_t d = usart_recv(USART1);
		rxbuf[rx_put++]=d;
		rx_put&=(RXBUF_SIZE-1);
		if(rx_put == rx_get) { /* ringbuffer overrun - drop oldest byte */
			rx_get++;
			rx_get&=(RXBUF_SIZE-1);
		}
	}
}

int uart_rx(int ch, void *p, size_t n, uint16_t timeout_ms) {
	uint32_t put, get, timeout=timeout_ms;
	int res=0, can_read;
	uint8_t *d=p;

	if(ch != UART_SENSOR_CH) /* not (yet) implemented */
		return 0;

	timeout+=(MSEC>>1); /* round up */
	timeout/=MSEC;
	timeout+=jiffies;

	while(((unsigned)res<n) && (jiffies != timeout)) { /* TODO: more robust jiffies rollover handling? */
		nvic_disable_irq(NVIC_USART1_IRQ);
		put=rx_put; get=rx_get;
		can_read = (get != put);
		if(can_read) {
			*d = rxbuf[get++];
			get&=(RXBUF_SIZE-1);
			rx_get = get;
		}
		nvic_enable_irq(NVIC_USART1_IRQ);
		res+=can_read;
		d+=can_read;
	}
	return res;
}

void uart_flush(int ch) {
	if(ch == UART_SENSOR_CH) {
		nvic_disable_irq(NVIC_USART1_IRQ);
		rx_put = rx_get = 0;
		nvic_enable_irq(NVIC_USART1_IRQ);
	}
}

void spi_tx(const void *p, size_t n) {
	const uint8_t *d=p;
	for(;n;n--,d++)
		spi_send8(SPI1, *d);
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
