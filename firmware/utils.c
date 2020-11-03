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
#include <stdint.h>
#include "utils.h"

/* val:          value
 * buf:          dst buffer
 * n:            number of digits plus decimal point if applicable
 * point_ofs:    decimal point offset (-1 for none) relative from least significant digit
 * zeropad:      prepend leading zeroes all the way
 * 
 * returns:      pointer to first digit
 */
char *i16_to_dec(int16_t val, char *buf, uint8_t n, int8_t point_ofs, uint8_t zeropad) {
	char *res;
	uint8_t i;
	uint8_t neg = val < 0;
	val = neg ? -val : val;
	buf+= n - 1;
	res=buf;
	for(i=0;i<n;i++,buf--) {
		char c = '0';
		if(i == point_ofs)
			c = '.';
		else if(val) {
			uint16_t new = val/10;
			c = val - (new * 10) + '0';		
			val = new;
		}
		else if((i > (point_ofs+1)) && (!zeropad))
			c = ' ';
		*buf = c;
		res = (c == ' ') ? res : buf;
	}
	if(neg)
		*(--res)='-';
	return res;
}
