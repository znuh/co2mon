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
#include "utils.h"

enum {
	CMD_SLEEP     = 0xB098,
	CMD_WAKEUP    = 0x3517,
	CMD_READ_ID   = 0xEFC8,
	CMD_RESET     = 0x805D,
	CMD_MEASURE   = 0x7866, /* no clock stretching, temp first, normal mode */
};

int shtc3_init(uint8_t addr);
int shtc3_read(uint8_t addr, readings_t *vals);

#define CRC8_INIT    0xff
#define CRC8_POLY    0x31

static int crc_check_word(const uint8_t *w) {
	return crc8(CRC8_INIT, CRC8_POLY, w, 2) == w[2];
}

static int cmd(uint8_t addr, uint16_t cval, uint8_t *rxd, uint8_t rx_sz) {
	uint8_t buf[2] = {cval>>8, cval&0xff};
	return i2c_xfer(addr, buf, 2, rxd, rx_sz);
}

int shtc3_init(uint8_t addr) {
	uint8_t rxd[3]={0,0,0};
	int res = cmd(addr, CMD_READ_ID, rxd, sizeof(rxd));
	if (res == 1)
		res = (rxd[0]&8) && ((rxd[1]&0x3f) == 0x7) && crc_check_word(rxd);
	return res;
}

int shtc3_read(uint8_t addr, readings_t *vals) {
	int32_t tmp;
	uint8_t rxbuf[6];
	int res = cmd(addr, CMD_MEASURE, NULL, 0);
	if(res < 1)
		goto out;
	msleep(15);
	res = i2c_xfer(addr, NULL, 0, rxbuf, sizeof(rxbuf));
	if(res < 1)
		goto out;

	/* temperature */
	res = crc_check_word(rxbuf);
	if(!res)
		goto out;
	tmp = rxbuf[0];
	tmp<<=8;
	tmp|=rxbuf[1];
	tmp*=175;
	tmp+=(1<<15); /* round */
	tmp>>=16;
	tmp-=45;
	vals->temperature = tmp;

	/* humidity */
	res = crc_check_word(rxbuf+3);
	if(!res)
		goto out;
	tmp = rxbuf[3];
	tmp<<=8;
	tmp|=rxbuf[4];
	tmp*=100;
	tmp>>=16;
	vals->relh = tmp;

out:
	return res;
}
