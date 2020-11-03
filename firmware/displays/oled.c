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
#include "display.h"
#include "platform.h"
#include "utils.h"
#include "ssd1306.h"
#include "fonts.h"

typedef struct display_ctx_s {
	uint8_t x, y;
} display_ctx_t;

static display_ctx_t disp;

static int cmd(uint8_t cmd) {
	uint8_t txbuf[] = {SSD1306_CTL_CMD, cmd};
	return i2c_xfer(SSD1306_I2C_ADDR, txbuf, sizeof(txbuf), NULL, 0);
}

static const uint8_t init_cmds[] FLASH_MEM = {
	SSD1306_DISPLAYOFF,
	SSD1306_SETDISPLAYCLOCKDIV,
	0x80,
	
	SSD1306_SETMULTIPLEX,
	SSD1306_LCDHEIGHT - 1,
	
	SSD1306_SETDISPLAYOFFSET,
	0x00,
	SSD1306_SETSTARTLINE | 0x0,
	SSD1306_CHARGEPUMP,
	0x14,
	
	SSD1306_MEMORYMODE,
	0x00,
	SSD1306_SEGREMAP | 0x1,
	SSD1306_COMSCANDEC,
	
	SSD1306_SETCOMPINS,
	0x12,
	SSD1306_SETCONTRAST,
	0xcf,
	
	SSD1306_SETPRECHARGE,
	0xf1,
	SSD1306_SETVCOMDETECT,
	0x40,
	SSD1306_DISPLAYALLON_RESUME,
	SSD1306_NORMALDISPLAY,
	SSD1306_DEACTIVATE_SCROLL,
	
	SSD1306_DISPLAYON,
};

static void goto_x(uint8_t x) {
	cmd(SSD1306_COLUMNADDR);
	cmd(x);
	cmd(SSD1306_LCDWIDTH-1);
	disp.x=x;
}

static void goto_y(uint8_t y) {
	cmd(SSD1306_PAGEADDR);
	cmd(y);
	cmd(7);
	disp.y=y;
}

static void goto_xy(uint8_t x, uint8_t y) {
	goto_x(x); goto_y(y);
}

static void clear(void) {
	uint8_t buf[128+1] = {0x40};
	uint16_t i;
	memset(buf+1, 0, sizeof(buf)-1);
	goto_xy(0, 0);
	for(i=1024;i;i-=(sizeof(buf)-1))
		i2c_xfer(SSD1306_I2C_ADDR, buf, sizeof(buf), NULL, 0);
}

#define PUTS_NORMAL		0
#define PUTS_INVERT		0xff

static void oled_puts(uint8_t font, const char *str, uint8_t invert) {
	const font_t *f = fonts+font;
	uint8_t c_mult = f->w * (f->h>>3); /* multiplier for char offset */
	uint8_t c_stride = f->h>>3; /* column stride */
	uint8_t y_cnt;
	uint8_t x_save = disp.x;
	uint8_t *bp, i2cbuf[128+1] = { SSD1306_CTL_DATA };
	const char *p;

	/* y-loop for font->h > 8 */
	for(y_cnt=0;y_cnt<c_stride;y_cnt++,disp.y++) {

		/* for y>0: set cursor position */
		if(y_cnt)
			goto_xy(x_save, disp.y);

		bp = i2cbuf + 1;

		/* foreach char */
		for(p=str;*p;p++) {
			uint8_t char_idx = font_index(f, *p);
			const uint8_t *cp = f->data + (char_idx * c_mult) + y_cnt;
			uint8_t i;

			for(i=f->w;i;i--,cp+=c_stride,bp++)
				*bp = flash_read_byte(&(cp[0])) ^ invert; /* char loop for each column of font */

			disp.x+=f->w;
		} /* string loop foreach char */

		i2c_xfer(SSD1306_I2C_ADDR, i2cbuf, bp-i2cbuf, NULL, 0);	

	} /* y-loop for font->h > 8 */
}

uint8_t oled_init(void) {
	uint8_t i;
	for(i=0;i<sizeof(init_cmds);i++) {
		uint8_t val = flash_read_byte(&(init_cmds[i]));
		int res = cmd(val);
		if(!res)
			return 0;
	}
	clear();
	return 1;
}

uint8_t oled_update(readings_t *vals) {
	static uint8_t first_run = 1;
	char txtbuf[17]="????";
	uint8_t x_ofs=0;

	txtbuf[16]=0;

	/* no need to redraw static 'ppm CO2' string every time */
	if(first_run) {
		clear();
		goto_xy(128-(8*7), 5);
		oled_puts(OLED_TEXT_8x8, "ppm CO2", PUTS_NORMAL);
		first_run=0;
	}

	/* CO2 */
	if(CO2_VALID(vals))
		i16_to_dec(vals->co2,txtbuf,4,-1,0);
	goto_xy(6,0);
	oled_puts(OLED_DIGITS_28x40, txtbuf, PUTS_NORMAL);

	memset(txtbuf,' ',16);

	/* temperature */
	if(TEMP_VALID(vals)) {
		uint8_t txbuf[9] = { SSD1306_CTL_DATA };
		const uint8_t *cp = fonts[OLED_TEXT_8x8].data+(('o'-' ')*8);
		uint8_t i;

		/* draw value */
		i16_to_dec(vals->temperature, txtbuf, 4, -1, 0);
		txtbuf[5]=0;

		goto_xy(0,7);
		oled_puts(OLED_TEXT_8x8, txtbuf, PUTS_NORMAL);

		/* derive degree sign by shifting 'o' up 2 pixels */
		for(i=0;i<8;cp++)
			txbuf[++i]=flash_read_byte(&(cp[0]))>>2;
		i2c_xfer(SSD1306_I2C_ADDR, txbuf, sizeof(txbuf), NULL, 0);
		
		txtbuf[6]='C';
		x_ofs=6;
	}

	/* relative humidity */
	if(RELH_VALID(vals)) {
		i16_to_dec(vals->relh, txtbuf+9, 3, -1, 0);
		txtbuf[9+3]='%';
		txtbuf[9+5]='r';
		txtbuf[9+6]='h';
	}

	goto_xy(x_ofs*8,7);
	oled_puts(OLED_TEXT_8x8, txtbuf+x_ofs, PUTS_NORMAL);

	return 1;
}

uint8_t oled_showtext(char **lines, uint8_t n) {
	uint8_t h=fonts[OLED_TEXT_8x8].h, l;
	n=MIN(n,SSD1306_LCDHEIGHT/h);
	clear();
	for(l=0;l<n;l++) {
		goto_xy(0,l);
		oled_puts(OLED_TEXT_8x8, lines[l], PUTS_NORMAL);
	}
	return l;
}
