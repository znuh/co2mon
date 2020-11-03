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
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <alloca.h>
#include <assert.h>
#include "platform.h"
#include "utils.h"

static int i2c_fd = -1;
static int uart_fd = -1;

void msleep(uint16_t val) {
	usleep(val*1000);
}

int i2c_xfer(uint8_t addr, const void *wr_p, size_t wr_sz, void *rd_p, size_t rd_sz) {
	const uint8_t *wr_d = wr_p;
	uint8_t *rd_d = rd_p;
	//int bus_fd = *((int*)priv);
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data io_data;
	//size_t wr_sz = wr_n ? *wr_n : 0;
	//size_t rd_sz = rd_n ? *rd_n : 0;
	int idx=0, res;
	uint8_t *txcopy; /* neede b/c of const *wr_d */

	if(i2c_fd < 0)
		return 0;

	if(((wr_sz > 0) && (!wr_d)) || ((rd_sz > 0) && (!rd_d)))
		return -EINVAL;

	txcopy = wr_sz > 0 ? alloca(wr_sz) : NULL;

	if(wr_sz > 0) {
		msgs[idx].addr  = addr;
		msgs[idx].flags = 0;
		msgs[idx].len   = wr_sz;
		msgs[idx].buf   = txcopy;
		memcpy(txcopy, wr_d, wr_sz);
		idx++;
	}
	if(rd_sz > 0) {
		msgs[idx].addr  = addr;
		msgs[idx].flags = I2C_M_RD;
		msgs[idx].len   = rd_sz;
		msgs[idx].buf   = rd_d;
		idx++;
	}
	io_data.msgs  = msgs;
	io_data.nmsgs = idx;
	res = ioctl(i2c_fd, I2C_RDWR, &io_data);
	return res;
}

static const uint32_t baudtbl[] = {
	9600  , B9600,
	19200 , B19200,
	38400 , B38400,
	57600 , B57600,
	115200, B115200,
	0     , 0
};

static uint32_t uart_baudrate = 0;
static uint32_t uart_vtime = 0;

void uart_config(int ch, uint32_t baudrate) {
	struct termios termio;
	int i, res;

	if((ch != UART_SENSOR_CH) || (uart_fd < 0))
		return;

	if(uart_baudrate == baudrate)
		return;

	res = tcgetattr(uart_fd, &termio);
	if(res < 0)
		return;
	
	for(i=0;baudtbl[i];i+=2) {
		if(baudtbl[i] == baudrate)
			break;
	}

	if(!baudtbl[i])
		return;

	termio.c_cflag = baudtbl[i+1] | CS8 | CREAD | CLOCAL;
	termio.c_iflag = IGNBRK | IGNPAR;
	termio.c_oflag = 0;
	termio.c_lflag = 0;

	res = tcsetattr(uart_fd, TCSANOW, &termio);
	if(res < 0)
		return;

	uart_baudrate = baudrate;
}

static void uart_set_vtime(int ch, uint32_t vtime) {
	struct termios termio;
	int res;

	if((ch != UART_SENSOR_CH) || (uart_fd < 0))
		return;

	if(uart_vtime == vtime)
		return;

	res = tcgetattr(uart_fd, &termio);
	if(res < 0)
		return;

	termio.c_cc[VMIN] = 0;
	termio.c_cc[VTIME] = vtime;
	//printf("set vtime %d\n",vtime);

	res = tcsetattr(uart_fd, TCSANOW, &termio);
	if(res < 0)
		return;

	uart_vtime = vtime;
}

int uart_tx(int ch, const void *p, size_t n) {
	return uart_fd < 0 ? 0 : write(uart_fd, p, n);
}

int uart_rx(int ch, void *p, size_t n, uint16_t timeout_ms) {
	if(uart_fd < 0)
		return 0;
	uart_set_vtime(ch, MAX((timeout_ms+99)/100, 1));
	return read(uart_fd, p, n);
}

void uart_flush(int ch) {
	uint8_t foo[128];
	int x;
	if(uart_fd < 0)
		return;
	x=read(uart_fd, foo, sizeof(foo));
	x=x;
}

void spi_tx(const void *p, size_t n) {
	/* not (yet) implemented */
}

void gpio_setval(uint8_t gpio_id, uint8_t val) {
	/* not (yet) implemented */
}

void hw_init(int argc, char **argv) {	
	if(argc>1)
		i2c_fd = open(argv[1], O_RDWR);
	if(argc>2) {
		uart_fd = open(argv[2], O_RDWR);
		uart_set_vtime(UART_SENSOR_CH, 1);
		uart_config(UART_SENSOR_CH, 9600);
	}
}
