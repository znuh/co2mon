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
#include "sensor.h"
#include "platform.h"
//#define DEBUG
#ifdef DEBUG
#include <stdio.h>
#include <sys/time.h>
#include <stdint.h>
#include <inttypes.h>
#endif

enum {
	/* MH-Z14A & MH-Z19B */
	READ_CO2      = 0x86,
	AUTOCAL_CFG   = 0x79,
	DET_RANGE     = 0x99,
	/* MH-Z19B only */
	ZERO_PT_CAL   = 0x87,
	SPAN_PT_CAL   = 0x88
};

int mhzxx_init(uint8_t addr);
int mhzxx_read(uint8_t addr, readings_t *vals);

static uint8_t cksum(const uint8_t *d, size_t n) {
	uint8_t res;
	for(res=0;n;n--,d++)
		res+=*d;
	return (0xff - res)+1;
}

static const uint8_t read_cmd[] = {0xff, 1, READ_CO2, 0, 0, 0, 0, 0, 0x79};

int mhzxx_read(uint8_t addr, readings_t *vals) {
	uint8_t resp[9];
	int res;
	addr=addr; /* prevent compiler warning */
	uart_config(UART_SENSOR_CH, 9600);
	uart_flush(UART_SENSOR_CH);
	uart_tx(UART_SENSOR_CH, read_cmd, sizeof(read_cmd));
	msleep(50); /* according to logic analyzer tests response should be complete after approx. 30ms */
#ifdef DEBUG
	struct timeval tv1, tv2;
	uint64_t ts1, ts2;
	gettimeofday(&tv1, NULL);
#endif
	res = uart_rx(UART_SENSOR_CH, resp, sizeof(resp), 100);
#ifdef DEBUG
	gettimeofday(&tv2, NULL);
	ts1 = tv1.tv_sec * 1000000 + tv1.tv_usec;
	ts2 = tv2.tv_sec * 1000000 + tv2.tv_usec;
	ts2-=ts1;
	ts2+=999;
	ts2/=1000;
	printf("res: %d after %"PRIu64" ms\n",res,ts2);
#endif
	if(res != sizeof(resp))
		return 0;
	res = (resp[8] == cksum(resp+1,7)) && (resp[0] == 0xff) && (resp[1] == 0x86);
	if(res)
		vals->co2 = resp[2]<<8 | resp[3];
	return res;
}

int mhzxx_init(uint8_t addr) {
	uint8_t res, retries=5;
	readings_t dummy; /* try a dummy-read */
	for(;((res=mhzxx_read(addr, &dummy)) != 1) && retries;retries--)
		msleep(50);
	return res;
}
