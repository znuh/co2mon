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
#include "fonts.h"
#include "platform.h"

#include "font_8x8.h"
//#include "font_12x16.h"

//#include "digits_27x40.h"
#include "digits_28x40.h"
//#include "digits_30x40.h"

#include "12x27-1bpp-hpacked-92chars.h"
#include "digits_38x50-rle.h"

const char font_digits[13] = " 0123456789?";

const font_t fonts[] = {
	{.data = font_8x8  , .w = 8 , .h = 8 , .flags = FONTFLG_1BPP|FONTFLG_VPACK, .n_chars = 95, .index = NULL, .char_ofs = ' '}, /* OLED vertical byte-packed 1bpp, glyph data:  760 Bytes */
//	{.data = font_12x16, .w = 12, .h = 16, .flags = FONTFLG_1BPP|FONTFLG_VPACK, .n_chars = 95, .index = NULL, .char_ofs = ' '}, /* OLED vertical byte-packed 1bpp, glyph data: 2280 Bytes */
	
//	{.data = font_27x40, .w = 27, .h = 40, .flags = FONTFLG_1BPP|FONTFLG_VPACK, .n_chars = 12, .index = font_digits, .char_ofs = 0}, /* OLED vertical byte-packed 1bpp, glyph data: 1620 Bytes */
	{.data = font_28x40, .w = 28, .h = 40, .flags = FONTFLG_1BPP|FONTFLG_VPACK, .n_chars = 12, .index = font_digits, .char_ofs = 0}, /* OLED vertical byte-packed 1bpp, glyph data: 1680 Bytes */
//	{.data = font_30x40, .w = 30, .h = 40, .flags = FONTFLG_1BPP|FONTFLG_VPACK, .n_chars = 12, .index = font_digits, .char_ofs = 0}, /* OLED vertical byte-packed 1bpp, glyph data: 1800 Bytes */

	{.data = font_12x27_hpacked, .w = 12, .h = 27, .flags = FONTFLG_1BPP, .n_chars = 92, .index = NULL, .char_ofs = ' '}, /* TFT horizontal byte-packed 1bpp, glyph data: 3726 Bytes */
	{.data = digits_38x50_rle,   .w = 38, .h = 50, .flags = FONTFLG_RLE, .n_chars = 12, .index = font_digits, .char_ofs = 0} /* TFT run-lenth encoded grayscale, rl encoded: 5043 Bytes */
};

uint8_t font_index(const font_t *f, char c) {
	uint8_t res;
	if(!f->index) /* no index table */
		return c-(f->char_ofs);
	for(res=0;f->index[res] && (f->index[res] != c);res++) {}
	return res;
}
