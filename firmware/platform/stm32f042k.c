/*
 * This file is part of the co2mon project.
 *
 * Copyright (C) 2020 Benedikt Heinz <hunz@mailbox.org>
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this code.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "platform.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>

#include <libopencmsis/core_cm3.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <stdint.h>
#include <inttypes.h>

#include "utils.h"

extern int  usb_tx(const void *p, size_t n);
extern void usb_setup(void);
void start_bootloader(void);

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

#define I2C_PORT     GPIOB
#define I2C_SCL      GPIO6
#define I2C_SDA      GPIO7

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
	gpio_set(I2C_PORT, I2C_SCL|I2C_SDA);
	gpio_set_output_options(I2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, I2C_SCL|I2C_SDA);
	gpio_mode_setup(I2C_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C_SCL|I2C_SDA);

	/* TFT nRST, A0 */
	gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
	gpio_set_output_options(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO0 | GPIO1);
	gpio_clear(GPIOF, GPIO1|GPIO0);
}

#define HZ                100
#define MSEC             (1000/HZ)
#define SEC              (1000*MSEC)
#define MS_TO_TICKS(a)   (((a)+(MSEC-1))/MSEC)

static volatile uint32_t jiffies = 0;

void sys_tick_handler(void) {
	jiffies++;
}

typedef struct timeout_s {
	uint32_t start;
	uint32_t end;
	int need_rollover;
	int expired;
	int rollover;
} timeout_t;

static void timeout_set(timeout_t *to, uint32_t ticks) {
	to->start = jiffies;
	to->expired = ticks == 0;
	to->end = to->start + ticks + 1;          /* need to add 1 timer cycle b/c current cycle already started */
	to->need_rollover = to->start >= to->end; /* ticks is at least 1 so equal case means a rollover too */
	to->rollover = 0;
}

static int timeout(timeout_t *to) {
	uint32_t now = jiffies;
	to->rollover |= now < to->start;
	to->expired  |= ((now >= to->end) && (to->rollover >= to->need_rollover)) || (to->rollover > to->need_rollover);
	return to->expired;
}

void msleep(uint16_t val) {
	timeout_t to;
	timeout_set(&to, MS_TO_TICKS(val));
	while(!timeout(&to))
		__WFI();
}

static void udelay(uint32_t n) {
	while(n--)
		__asm__("nop");
}

static void clock_setup(void) {
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOF);
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_USART1);
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

#define I2C_READ              1
#define I2C_WRITE             0
#define I2C_HALFCYCLE_DELAY   21   /* measured and tuned for 99kHz @48MHz F_CPU */
#define I2C_TIMEOUT           40   /* maximum of 40 msec for clock stretching   */

/* expects SCL low 
 * returns w/ SCL low */
static inline int i2c_cycle(int sda) {
	int res;

	if(sda)
		gpio_set(I2C_PORT, I2C_SDA);
	else
		gpio_clear(I2C_PORT, I2C_SDA);
	udelay(I2C_HALFCYCLE_DELAY>>1);

	/* SCL high */
	gpio_set(I2C_PORT, I2C_SCL);
	udelay(I2C_HALFCYCLE_DELAY);

	/* clock stretching active? */
	if(!gpio_get(I2C_PORT, I2C_SCL)) {
		timeout_t to;
		timeout_set(&to, MS_TO_TICKS(I2C_TIMEOUT));
		while((!(res=gpio_get(I2C_PORT, I2C_SCL))) && (!timeout(&to))) {}
		if(!res)
			return -1;
	}

	/* sample SDA */
	res = gpio_get(I2C_PORT, I2C_SDA) != 0;

	/* SCL low */
	gpio_clear(I2C_PORT, I2C_SCL);
	udelay(I2C_HALFCYCLE_DELAY>>1);

	return res; /* return sampled value */
}

/* expects SCL low 
 * returns w/ SCL low */
static int i2c_9bit_xfer(uint32_t txd) {
	int rxd=0, res=0, i=9;
	for(;i;i--,txd<<=1) {
		res = i2c_cycle(txd&0x100);
		if(res < 0) return res;
		rxd<<=1;
		rxd|=res;
	}
	return rxd;
}

static int i2c_sendbyte(uint8_t b) {
	int res = i2c_9bit_xfer((b<<1)|1); /* additional 9th bit needes to be hi for ACK rx */
	return (res >= 0) && (!(res&1));   /* check if no error and ACK received */
}

/* expects nothing
 * result: SCL low, SDA low */
static int i2c_start(uint8_t addr) {
	/* repeated start? */
	if(!gpio_get(I2C_PORT, I2C_SDA)) { /* ensure SDA high */
		gpio_set(I2C_PORT, I2C_SDA);
		udelay(I2C_HALFCYCLE_DELAY>>1);
	}
	if(!gpio_get(I2C_PORT, I2C_SCL)) { /* ensure SCL high */
		gpio_set(I2C_PORT, I2C_SCL);
		udelay(I2C_HALFCYCLE_DELAY);
	}

	/* SDA -> low */
	gpio_clear(I2C_PORT, I2C_SDA);
	udelay(I2C_HALFCYCLE_DELAY);

	/* SCL -> low */
	gpio_clear(I2C_PORT, I2C_SCL);
	udelay(I2C_HALFCYCLE_DELAY>>1);

	/* send addr. */
	return i2c_sendbyte(addr);
}

static inline int i2c_rcvbyte(uint8_t *b, uint8_t ack) {
	int res = i2c_9bit_xfer((0xff<<1)|(ack^1));
	if(res>=0) *b = res>>1;
	return res >= 0;
}

/* expects SCL low
 * returns w/ SCL hi, SDA hi */
static void i2c_stop(void) {
	gpio_clear(I2C_PORT, I2C_SDA);
	udelay(I2C_HALFCYCLE_DELAY);

	gpio_set(I2C_PORT, I2C_SCL);
	udelay(I2C_HALFCYCLE_DELAY);

	gpio_set(I2C_PORT, I2C_SDA);
	udelay(I2C_HALFCYCLE_DELAY);
}

/* Errata sheet ES0243 rev 3 lists several horrible errata for the hardware I2C peripheral
 * therefor a software I2C implementation is used */
int i2c_xfer(uint8_t addr, const void *wr_p, size_t wr_sz, void *rd_p, size_t rd_sz) {
	const uint8_t *wr_d = wr_p;
	uint8_t *rd_d = rd_p;
	int n=0;

	/* write cycle */
	if((wr_d) && (wr_sz>0)) {
		int res = i2c_start((addr<<1)|I2C_WRITE);
		if(!res)
			goto out;
		for(;wr_sz;wr_sz--,wr_d++) {
			res = i2c_sendbyte(*wr_d);
			if(!res)
				goto out;
		}
		/* no stop here b/c repeated start follows or stop before function return */
		n++;
	}

	/* read cycle */
	if((rd_d) && (rd_sz>0)) {
		int res = i2c_start((addr<<1)|I2C_READ);
		if(!res)
			goto out;
		for(;rd_sz;rd_sz--,rd_d++) {
			res = i2c_rcvbyte(rd_d, rd_sz > 1);
			if(!res)
				goto out;
		}
		n++;
	}

out:
	i2c_stop();
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
		return usb_tx(p,n);
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
	uint32_t put, get;
	int res=0, can_read;
	timeout_t to;
	uint8_t *d=p;

	if(ch != UART_SENSOR_CH) /* not (yet) implemented */
		return 0;

	timeout_set(&to, MS_TO_TICKS(timeout_ms));
	while(((unsigned)res<n) && (!timeout(&to))) {
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

void bl_ram_func(void);

__attribute__( ( long_call, section(".data#") ) ) void bl_ram_func(void) {   /* extra # after section name mutes the asm warning m) */
	FLASH_CR |= FLASH_CR_PER;
	FLASH_AR = 0x08000000; /* erase the page of the vetor table to enforce bootloader mode */
	FLASH_CR |= FLASH_CR_STRT;

	while(FLASH_SR & FLASH_SR_BSY) {} /* busywait */
	FLASH_CR &= ~FLASH_CR_PER;

	SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ; /* trigger system reset via SCB */
	while(1) {}
}

void start_bootloader(void) {
	udelay(1024*1024*4);                /* wait a bit after USB disconnect - msleep would require the timer interrupt */
	cm_disable_interrupts();
	flash_unlock();
	flash_wait_for_last_operation();
	bl_ram_func();
}

void hw_init(int argc, char **argv) {
	argc=argc; argv=argv;
	clock_setup();
	systick_setup();
	gpio_setup();
	spi_setup();
	uart_setup();
	usb_setup();
}
