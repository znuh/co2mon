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
#include <string.h>
#include "sensor.h"
#include "platform.h"
//#define DEBUG
#ifdef DEBUG
#include <stdio.h>
#endif

enum {
	CONT_START    = 0x0010,
	CONT_STOP     = 0x0104,
	GET_DATARDY   = 0x0202,
	GET_DATA      = 0x0300,
	SET_INTERVAL  = 0x4600,
	SET_AUTOCAL   = 0x5306,
	FORCE_RECAL   = 0x5204,
	SET_TEMPOFS   = 0x5403,
	SET_ALTCOMP   = 0x5102,
	
};

#ifdef DEBUG
static void hexdump(const uint8_t *d, size_t n) {
	for(;n;n--,d++)
		printf("%02x ",*d);
	printf("\n");
}
#endif

static uint8_t crc8(const uint8_t *d, size_t n) {
	uint8_t crc = 0xff;
	uint8_t i;
	for(;n;n--,d++) {
		crc ^= *d;
		for(i=8;i;i--) {
			crc = (crc & 0x80) ? (crc<<1)^0x31 : crc<<1;
		}
	}
	return crc;
}

static int scd30_command(uint8_t addr, uint16_t cmd, uint8_t argc, uint16_t argv0, uint8_t *rd, size_t rd_n) {
	uint8_t txbuf[5] = {cmd>>8, cmd&0xff, 0, 0, 0};
	int res, n=2;
	if(argc > 0) {
		txbuf[2] = argv0>>8;
		txbuf[3] = argv0&0xff;
		txbuf[4] = crc8(txbuf+2,2);
		n=5;
	}
#ifdef DEBUG
	//hexdump(txbuf,n);
#endif
	res = i2c_xfer(addr, txbuf, n, NULL, 0);
	if((res < 1) || (rd_n < 1))
		return res;
	res = i2c_xfer(addr, NULL, 0, rd, rd_n);
	if(res < 1)
		return res;
	/* verify checksum */
	for(n=0;(n+2)<rd_n;n+=3) {
		if(crc8(rd+n,2) != rd[n+2])
			return 0;
	}
	return 1;
}

int scd30_init(uint8_t addr) {
	uint8_t rxbuf[3] = {0, 0, 0};
	int res;
	res = scd30_command(addr, SET_AUTOCAL, 1, 1, NULL, 0);
	if(res < 1)
		goto out;
	res = scd30_command(addr, SET_INTERVAL, 1, 5, NULL, 0);
	if(res < 1)
		goto out;
	res = scd30_command(addr, CONT_START, 1, 0, NULL, 0);
	if(res < 1)
		goto out;
	res = scd30_command(addr, GET_DATARDY, 0, 0, rxbuf, sizeof(rxbuf));
	if(res < 1)
		return 0;
	res = rxbuf[2] == crc8(rxbuf,2);
out:
	return res;
}

static int16_t conv_val(const uint8_t *d) {
	uint32_t v = d[0];
	float vf;
	v<<=8;
	v|=d[1];
	v<<=8;
	v|=d[3];
	v<<=8;
	v|=d[4];
	memcpy(&vf, &v, sizeof(vf));
	vf+=0.5;
	return (int16_t)vf;
}

#define POLL_DELAY      20
#define READY_TIMEOUT   20

int scd30_read(uint8_t addr, readings_t *vals) {
	uint8_t rxbuf[18], timeout = READY_TIMEOUT;
	int res;

	do {
		res = scd30_command(addr, GET_DATARDY, 0, 0, rxbuf, 3);
		if(res < 1)
			goto out;
		if(rxbuf[1] == 1)
			break;
		msleep(POLL_DELAY);
	} while(--timeout);
#ifdef DEBUG
//		printf("get datardy %d %02x %02x %02x\n",res,rxbuf[0],rxbuf[1],rxbuf[2]);
#endif
#ifdef DEBUG
	if(timeout < READY_TIMEOUT)
		printf("SCD30 poll timeout %d res %d rdy %d\n",timeout,res,rxbuf[1]);
#endif
	if(rxbuf[1] != 1)
		goto out;
	res = scd30_command(addr, GET_DATA, 0, 0, rxbuf, sizeof(rxbuf));
#ifdef DEBUG
	//printf("get data %d ",res);
	//hexdump(rxbuf,sizeof(rxbuf));
#endif
	if(res < 1)
		goto out;
	vals->co2=conv_val(rxbuf);
	vals->temperature=conv_val(rxbuf+2*3);
	vals->relh=conv_val(rxbuf+2*3*2);
out:
	return res;
}
