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
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

/* IO assignments:
 * 
 * D0 : PD0 RXD   <- from host
 * D1 : PD1 TXD   -> to host
 * D2 : PD2       MCU_RX2
 * 
 * D8 : PB0 ICP1  RXD_S - from sensor
 * D9 : PB1 OC1A  TXD_S - to sensor
 * D10: PB2 nSS   ~TFT_CS
 * D11: PB3 MOSI  TFT_DATA
 * D13: PB5 SCK   TFT_CLK
 * 
 * A2 : PC2       ~TFT_RST
 * A3 : PC3       TFT_A0
 * A4 : PC4       SDA
 * A5 : PC5       SCL
 */

static volatile uint16_t t1ovf = 0; /* ~244.14 Hz (~4.098ms) */
#define MS_TO_TICKS(a)   (((a)+3)>>2) /* round up */

ISR(TIMER1_OVF_vect) {
	t1ovf++;
}

typedef struct timeout_s {
	uint16_t start;
	uint16_t end;
	uint8_t need_rollover;
	uint8_t expired;
} timeout_t;

static void timeout_set(timeout_t *to, uint16_t ticks) {
	to->start = t1ovf;
	to->expired = ticks == 0;
	to->end = to->start + ticks + 1; /* need to add 1 timer cycle b/c current cycle already started */
	to->need_rollover = to->start >= to->end; /* ticks is at least 1 so equal case means a rollover too */
}

static int timeout(timeout_t *to) {
	uint16_t now = t1ovf;
	to->need_rollover &= now >= to->start; /* if set, keep bit set until now < start */
	to->expired |= (now >= to->end) && (!to->need_rollover);
	return to->expired;
}

void msleep(uint16_t val) {
	timeout_t to;
	timeout_set(&to, MS_TO_TICKS(val));
	while(!timeout(&to))
		sleep_cpu();
}

static void twi_init(void) {
	/* f_SCL = F_CPU / (16 + 2(TWBR) * PRESCALER) */
	TWBR = 5; /* ~91kHz */
	TWDR = 0xff;
	TWSR = (1<<TWPS1); /* prescaler: 16 */
	TWCR = (1<<TWEN);
}

static int twi_wait(void) {
	uint8_t timeout;
	for(timeout=0xff; (!(TWCR & _BV(TWINT))) && (timeout); timeout--) {}
	return timeout ? (TWSR & 0xF8) : -1;
}

static int twi_action(uint8_t flags) {
	TWCR = _BV(TWINT) | _BV(TWEN) | flags;
	return twi_wait();
}

static int twi_start(uint8_t addr) {
	int res = twi_action(_BV(TWSTA));
	if((res != TW_START) && (res != TW_REP_START))
		return 0;
	TWDR = addr;
	res = twi_action(0);
	return res == ((addr&TW_READ) ? TW_MR_SLA_ACK : TW_MT_SLA_ACK);
}

static int twi_sendbyte(uint8_t b) {
	TWDR = b;
	return twi_action(0) == TW_MT_DATA_ACK;
}

static int twi_rcvbyte(uint8_t *b, uint8_t ack) {
	int res = twi_action(ack ? _BV(TWEA) : 0);
	if((res == TW_MR_DATA_ACK) || (res == TW_MR_DATA_NACK))
		*b = TWDR;
	return res;
}

static void twi_stop(void) {
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}

int i2c_xfer(uint8_t addr, const void *wr_p, size_t wr_sz, void *rd_p, size_t rd_sz) {
	const uint8_t *wr_d = wr_p;
	uint8_t *rd_d = rd_p;
	int n=0;

	/* write cycle */
	if((wr_d) && (wr_sz>0)) {
		int res = twi_start((addr<<1)|TW_WRITE);
		if(!res)
			goto out;
		for(;wr_sz;wr_sz--,wr_d++) {
			res = twi_sendbyte(*wr_d);
			if(!res)
				goto out;
		}
		/* no stop here b/c repeated start follows or stop before function return */
		n++;
	}

	/* read cycle */
	if((rd_d) && (rd_sz>0)) {
		int res = twi_start((addr<<1)|TW_READ);
		if(!res)
			goto out;
		for(;rd_sz;rd_sz--,rd_d++) {
			res = twi_rcvbyte(rd_d, rd_sz > 1);
			if(!res)
				goto out;
		}
		n++;
	}

out:
	twi_stop();
	return n;
}

static uint16_t uart_etu          = 1667; /* value for 9600 baud */

#define RXBUF_SIZE      32

static volatile uint8_t rxbuf[RXBUF_SIZE];
static volatile uint8_t rx_put = 0;
static volatile uint8_t rx_get = 0;

/* softuart RX - startbit detection */
ISR(TIMER1_CAPT_vect) {
	OCR1B = ICR1 + uart_etu + (uart_etu>>1);
	TIMSK1 ^= (1<<OCIE1B)|(1<<ICIE1);
	TIFR1 = (1<<OCF1B);
}

/* softuart RX - baud sampling */
ISR(TIMER1_COMPB_vect) {
	static uint8_t sreg  = 0xff;
	static uint8_t bitcnt = 9;
	uint8_t sample = PINB<<7;
	OCR1B += uart_etu; /* setup next sample */
	
	/* sample 8 bits */
	if(--bitcnt) {
		sreg>>=1;
		sreg|=sample;
		return;
	}
	
	/* byte complete */

	TIMSK1 ^= (1<<OCIE1B)|(1<<ICIE1); /* wait for next stopbit */
	TIFR1 = (1<<ICF1);

	if(sample) { /* stopbit correct? */
		rxbuf[rx_put++]=sreg;
		//UDR0 = sreg; /* debug */
		rx_put&=(RXBUF_SIZE-1);
		if(rx_put == rx_get) { /* ringbuffer overrun - drop oldest byte */
			rx_get++;
			rx_get&=(RXBUF_SIZE-1);
		}
	}
	sreg = 0xff;
	bitcnt = 9;
}

static volatile uint16_t txbits   = 0;

#define TX_BUSY()   (TCCR1A)

/* softuart TX */
ISR(TIMER1_COMPA_vect) {
	uint16_t tb = txbits;
	OCR1A += uart_etu; /* setup next match */
	TCCR1A = (1<<COM1A1) | ((tb&1)<<COM1A0); /* setup bit for next match */
	tb>>=1;
	txbits = tb;

	if(!tb) { /* last bit is always 1 (stopbit) */
		TIMSK1 &= ~(1<<OCIE1A); /* disable OC1A interrupt */
		TCCR1A = 0; /* disable compare match output */
	}
}

static void soft_tx(uint8_t d) {
	txbits = (7<<8) | d; /* additional stopbits won't be sent but needed for timing */

	cli();
	OCR1A = TCNT1 + 128;
	TCCR1A = (1<<COM1A1); /* startbit: clear on match */
	TIMSK1 |= (1<<OCIE1A);
	TIFR1 = (1<<OCF1A);
	sei();

	while(TX_BUSY()) {}
}

int uart_tx(int ch, const void *p, size_t n) {
	const uint8_t *d=p;
	size_t ret = n;

	if((!d) || (n<1))
		return 0;

	for(;n;d++,n--) {
		if(ch == UART_SENSOR_CH)
			soft_tx(*d);
		else {
			loop_until_bit_is_set(UCSR0A, UDRE0);
			UDR0=*d;
		}
	}
	return ret;
}

int uart_rx(int ch, void *p, size_t n, uint16_t timeout_ms) {
	uint8_t *d=p;
	uint8_t put, get, can_read;
	timeout_t to;
	int res=0;

	if(ch != UART_SENSOR_CH) /* not (yet) implemented */
		return 0;

	timeout_set(&to, MS_TO_TICKS(timeout_ms));
	while((res<n) && (!timeout(&to))) {
		cli();
		put=rx_put; get=rx_get;
		can_read = (get != put);
		if(can_read) {
			*d = rxbuf[get++];
			get&=(RXBUF_SIZE-1);
			rx_get = get;
		}
		sei();
		res+=can_read;
		d+=can_read;
	}
	return res;
}

void uart_flush(int ch) {
	if(ch == UART_SENSOR_CH) {
		cli();
		rx_put = rx_get = 0;
		sei();
	}
}

void uart_config(int ch, uint32_t baudrate) {
	if(ch == UART_SENSOR_CH) {
		uint32_t tmp = F_CPU, tmp2 = baudrate;
		tmp2>>=1;
		tmp+=tmp2;
		tmp/=baudrate;
		uart_etu = tmp;
	}
}

static void t1_init(void) {
	/* drive TXD high, enable pullup for RXD */
	PORTB |= (1<<PB1) | (1<<PB0);
	DDRB  |= (1<<PB1);
	
	TCCR1B = (1<<ICNC1)|(1<<CS10);   /* enable T1, set to F_CPU */
	TIMSK1 = (1<<ICIE1)|(1<<TOV1);   /* enable T1 input capture & overflow interrupts */
	TIFR1  = (1<<ICF1)|(1<<OCF1B)|(1<<OCF1A)|(1<<TOV1);
}

static void sysuart_init(void) {
	/* enable TXD output */
	PORTD |= (1<<PD1);
	DDRD  |= (1<<PD1);

	UBRR0H = 0;
	UBRR0L = 25; /* 38.4 kBaud */

	UCSR0A = 0;
	UCSR0B = (1<<TXEN0);
	UCSR0C = (3<<UCSZ00); /* 8 bit, 1 stopbit */
}

static void io_init(void) {
	/* PB2: nSS / ~TFT_CS
	 * PB3: MOSI / TFT_DATA
	 * PB5: SCK  / TFT_CLK */
	PORTB |= (1<<PB2);
	DDRB  |= (1<<PB5)|(1<<PB3)|(1<<PB2);
	/* PC2: ~TFT_RST
	 * PC3: TFT_A0 */
	DDRC  |= (1<<PC3)|(1<<PC2);	
	PORTC &= ~((1<<PC3)|(1<<PC2));
}

static void spi_init(void) {
	SPSR = (1<<SPI2X); /* double f_SCK (result: F_CPU/2) */
	SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA);
}

void spi_tx(const void *p, size_t n) {
	const uint8_t *d=p;
	uint8_t dummy;
	for(;n;n--,d++) {
		SPDR = *d;
		loop_until_bit_is_set(SPSR, SPIF);
		dummy = SPDR;
	}
	dummy=dummy; /* get rid of compiler warning */
}

/* GPIO_TFT_NCS : PB2
 * GPIO_TFT_NRST: PC2
 * GPIO_TFT_A0  : PC3 */
void gpio_setval(uint8_t gpio_id, uint8_t val) {
	uint8_t bpos[] = {PB2, PC2, PC3};
	uint8_t bit = bpos[gpio_id];

	/* PORT registers are never written in interrupt context
	 * therefor no cli/sei needed here for atomic access */

	if(gpio_id == GPIO_TFT_NCS)
		PORTB = (~(1<<bit) & PORTB) | (val<<bit);
	else
		PORTC = (~(1<<bit) & PORTC) | (val<<bit);
}

void hw_init(int argc, char **argv) {
	sysuart_init();
	t1_init();
	sei();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	twi_init();
	io_init();
	spi_init();
}
