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
#ifdef DEBUG
#include <stdio.h>
#endif

enum {
	FW_REV                 = 5001,
	STATUS                 = 5002,
	GAS_PPM	               = 5003,
	RESET_DEVICE           = 1000,
	START_SINGLE_POINT_CAL = 1004,
	SLAVE_ADDRESS          = 4005,
	ABC_LOGIC_EN_DISABLE   = 1006
};

int t67xx_init(uint8_t addr);
int t67xx_read(uint8_t addr, readings_t *vals);

static int read_reg(uint8_t addr, uint16_t reg, uint16_t *dst) {
	uint8_t tdata[5] = {4, reg>>8, reg&0xff, 0, 1};
	uint8_t rdata[4] = {0x23, 0x23, 0x23, 0x23};
	int valid, res = i2c_xfer(addr, tdata, sizeof(tdata), NULL, 0);
	if(res != 1)
		goto out;
	msleep(10);
	res = i2c_xfer(addr, NULL, 0, rdata, 4);
out:
	valid = (res == 1) && (rdata[0] == 4) && (rdata[1] == 2);
#ifdef DEBUG
	printf("t67xx read %d res %d %02x %02x %02x%02x\n",reg,res,rdata[0],rdata[1],rdata[2],rdata[3]);
#endif
	*dst = valid ? (rdata[2]<<8 | rdata[3]) : UINT16_MAX;
	return valid;
}

int t67xx_init(uint8_t addr) {
	uint16_t fw = 0xffff, status = 0xffff;
	int res = read_reg(addr, FW_REV, &fw);
	if(res < 1)
		goto out;
	res = read_reg(addr, STATUS, &status);
	if(res < 1)
		goto out;
out:
	return res;
}

int t67xx_read(uint8_t addr, readings_t *vals) {
	return read_reg(addr, GAS_PPM, &vals->co2);
}
