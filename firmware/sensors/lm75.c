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

int lm75_init(uint8_t addr);
int lm75_read(uint8_t addr, readings_t *vals);

int lm75_read(uint8_t addr, readings_t *vals) {
	uint8_t rptr=0;
	int8_t temp;
	uint8_t res = (i2c_xfer(addr, &rptr, sizeof(rptr), &temp, sizeof(temp)) == 2);
	if(res && vals)
		vals->temperature = temp;
	return res;
}

int lm75_init(uint8_t addr) {
	return lm75_read(addr, NULL); /* try a dummy-read */
}
