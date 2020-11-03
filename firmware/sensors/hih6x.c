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

#define STATUS(x)   ((x[0])>>6)

enum {
	STATUS_NORMAL  = 0,
	STATUS_STALE   = 1,
	STATUS_CMDMODE = 2,
	STATUS_INVALID = 3
};

int hih6x_read(uint8_t addr, readings_t *vals) {
	uint8_t rptr=0, rdata[4];
	uint8_t res = i2c_xfer(addr, &rptr, sizeof(rptr), rdata, sizeof(rdata));
	res = (res == 2) && (STATUS(rdata) < STATUS_CMDMODE);
	if(res && vals) {
		uint16_t tmp = ((rdata[0]&0x3f)<<8) | rdata[1]; /* 14 bit */
		int16_t stmp;
		tmp>>=6; /* 8 bit */
		tmp*=25; /* scale to 100% - mult */
		tmp+=(1<<5); /* round */
		vals->relh = tmp>>6; /* scale to 100% - div */
		
		tmp = (rdata[2]<<6) | (rdata[3]>>2); /* 14 bit */
		tmp>>=6; /* 8 bit */
		tmp*=165; /* scale mult */
		tmp+=(1<<7); /* round */
		tmp>>=8; /* scale div */
		stmp=tmp;
		stmp-=40; /* offset adjust */
		vals->temperature = stmp;
	}
	return res;
}

int hih6x_init(uint8_t addr) {
	return hih6x_read(addr, NULL); /* attempt a dummy-read */
}
