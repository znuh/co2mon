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
#include "fonts.h"
#include "st77.h"

/* command: A0 low,
 * data   : A0 high */
enum {
	MODE_CMD     = 0,
	MODE_DATA    = 1,
	MODE_UNDEF   = 0xff
};

void    tft_init(void);
uint8_t tft_update(readings_t *vals);
uint8_t tft_showtext(char **lines, uint8_t n);

static inline void tft_mode(uint8_t new_mode) {
	static uint8_t current_mode = MODE_UNDEF;
	if(current_mode != new_mode) {
		gpio_set(GPIO_TFT_A0, new_mode);
		current_mode = new_mode;
	}
}

static void tft_write(int16_t cmd, const uint8_t *d, size_t data_n) {
	if(cmd>=0) {
		uint8_t v=cmd&0xff;
		tft_mode(MODE_CMD);
		spi_tx(&v, 1);
	}
	if((d) && (data_n>0)) {
		tft_mode(MODE_DATA);
		spi_tx(d, data_n);
	}
}

#define DISPLAY_WIDTH   160
#define DISPLAY_HEIGHT  128

static const uint8_t st77_init[] FLASH_MEM = {
	/* Panel on */
	//ST77_CMD_SWRESET, 0,
	//ST77_CMD_SLPOUT,  0,

	/* Panel settings */
	ST77_CMD_FRMCTR1, 3, 0x01, 0x2C, 0x2D,
	ST77_CMD_FRMCTR2, 3, 0x01, 0x2C, 0x2D,
	ST77_CMD_FRMCTR3, 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D,
//	ST77_CMD_INVCTR,  1, 0x07, // default ok?
	ST77_CMD_PWCTR1,  3, 0xA2, 0x02, 0x84,
	ST77_CMD_PWCTR2,  1, 0xC5,
//	ST77_CMD_PWCTR3,  2, 0x0A, 0x00, // default ok?
	ST77_CMD_PWCTR4,  2, 0x8A, 0x2A,
//	ST77_CMD_PWCTR5,  2, 0x8A, 0xEE, // default ok?
	ST77_CMD_VMCTR1,  1, 0x0E,

	/* System settings */
//	ST77_CMD_CASET,   4, 0, 0, 0, 80-1,
//	ST77_CMD_RASET,   4, 0, 0, 0, 160-1,
	ST77_CMD_COLMOD,  1, 0x05, /* 16-bit/pixel */
	//ST77_CMD_MADCTL,  1, 0xC8, // default value ok?
//	ST77_CMD_INVOFF,  0, // default value ok?
//	ST77_CMD_MADCTL,  1, 0xa0, // test

//	ST77_CMD_NORON,  0, // default ok?
//	ST77_CMD_DISPON, 0,

	ST77_CMD_INVALID
};

static const uint8_t madctl_lut[] FLASH_MEM = {
	ST77_MADCTL_MX | ST77_MADCTL_MY,
	ST77_MADCTL_MY | ST77_MADCTL_MV,
	0,
	ST77_MADCTL_MX | ST77_MADCTL_MV,
};

static void display_orientation(uint32_t i) {
	uint8_t val = flash_read_byte(&(madctl_lut[i&3]));
	tft_write(ST77_CMD_MADCTL, &val, 1);
}

static void display_blit_start(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2) {
	uint8_t ca_data[] = {x1>>8, x1&0xff, x2>>8, x2&0xff};
	uint8_t ra_data[] = {y1>>8, y1&0xff, y2>>8, y2&0xff};
	tft_write(ST77_CMD_CASET, ca_data, sizeof(ca_data));
	tft_write(ST77_CMD_RASET, ra_data, sizeof(ra_data));
	tft_write(ST77_CMD_RAMWR, NULL, 0);
}

static void display_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t col) {
	uint32_t n=w*h;
	uint8_t buf[] = {col>>8, col&0xff};
	display_blit_start(x,y,x+w-1,y+h-1);
	while(n--)
		tft_write(-1, buf, sizeof(buf));
}

static uint16_t hsv2rgb(uint8_t h, uint8_t s, uint8_t v) {
	uint8_t region, remainder, p, q, t, r, g, b;

	if(!s)
		return 0;

    region = h / 43;
    remainder = (h - (region * 43)) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0:
            r = v; g = t; b = p;
            break;
        case 1:
            r = q; g = v; b = p;
            break;
        case 2:
            r = p; g = v; b = t;
            break;
        case 3:
            r = p; g = q; b = v;
            break;
        case 4:
            r = t; g = p; b = v;
            break;
        default:
            r = v; g = p; b = q;
            break;
    }
    r>>=3;
    g>>=2;
    b>>=3;

    return (r<<(5+6)) | (g<<5) | b;
}

static void rle_decode(const uint8_t **rp, uint8_t *buf, uint16_t n) {
	const uint8_t *r=*rp;
	uint8_t col=0, rl=0, v;
	for(;n;n--,buf++) {
		if(rl) 
			rl--;
		else {
			v = flash_read_byte(r++);
			if(v&0x80) 
				col=(v<<1)|1;
			else
				rl=v-1;
		}
		*buf=col;
	}
	*rp=r;
}

#define BSWAP(x)    ((x>>8)|(x<<8))

static inline uint16_t blend(uint16_t r, uint16_t g, uint16_t b, uint16_t alpha) {
	r*=++alpha;
	g*=alpha;
	b*=alpha;
	r>>=8;
	g>>=8;
	b>>=8;
	return (r<<(5+6)) | (g<<5) | b;
}

/* warning: stack usage: ~840 Bytes for RLE digits */
static void draw_text(uint8_t xs, uint8_t ys, uint8_t font, const char *txt, uint16_t fg, uint16_t bg) {
	uint16_t rbuf[DISPLAY_WIDTH], *cp;
	const font_t *f = &fonts[font];
	const uint8_t *fd = f->data;
	uint16_t colors[] = {BSWAP(bg), BSWAP(fg)};
	uint16_t char_offsets[16];
	uint8_t cw = f->w, ch = f->h;
	uint8_t rle = (f->flags&FONTFLG_RLE) ? 1 : 0;
	uint16_t row_stride = rle ? 0 : (cw*f->n_chars)>>3;
	uint16_t r=fg>>(5+6), g=(fg>>5)&0x3f, b=fg&0x1f;
	uint16_t char_ofs, bits, row_len;
	uint8_t shift, x, y, i, clen;
	uint16_t font_width = cw*f->n_chars;
	uint8_t rlebuf[font_width*rle]; /* only needed for RLE - holds one decoded font row */

	/* determine glyph offsets and strlen along the way */
	for(i=0;(i<16) && txt[i];i++)
		char_offsets[i] = font_index(f, txt[i])*cw;
	clen = i;

	row_len = clen*cw; /* words to write for each row */
	display_blit_start(xs,ys,xs+row_len-1,ys+ch-1);
	row_len<<=1; /* bytes to write for each row */

	for(y=0;y<ch;y++) {
		if(rle)
			rle_decode(&fd, rlebuf, font_width);
		for(cp=rbuf,i=0;i<clen;i++) { /* foreach char */
			char_ofs = char_offsets[i];
			if(rle) { /* run-length encoded greyscale */
				uint8_t *rp=rlebuf+char_ofs;
				for(x=cw;x;x--,cp++,rp++)
					*cp = BSWAP(blend(r,g,b,*rp));
			}
			else { /* 1bpp horizontal packed */
				shift = char_ofs&7; /* bits offset within bytes */
				char_ofs>>=3; /* char offset in bytes */
				bits = flash_read_byte(&(fd[char_ofs++]))>>shift;
				bits|= flash_read_byte(&(fd[char_ofs]))<<(8-shift);
				for(x=0;x<cw;x++,bits>>=1,cp++)
					*cp = colors[bits&1];
			}
		} /* foreach char */
		tft_write(-1, (uint8_t*)rbuf, row_len);
		fd+=row_stride;
	} /* foreach row */
}

/* display layout:
 * total: 160x128
 * 
 * 0,0: CO2 value - w: 38*4=152, h: 50   - remaining h: 128-50 = 78
 * => h=27 for remaining text (2 lines)
 */

/* 90: green -> <800
 * 70: still greenish
 * 60: greenish-orange
 * 50: greenish-orange -> 900
 * 40: yellow w/ greenish touch -> 1000
 * 30: yellow -> 1100
 * 20: orange-ish -> 1200
 * 10: deep orange -> 1300
 * 1: deep red -> 1400
 * 0: green!??
 * */

static uint16_t co2_color(uint16_t ppm) {
	uint16_t tmp = MAX(ppm, 800);
	tmp = MIN(tmp, 1400);
	if(tmp <= 800)
		tmp = 90;
	else if(tmp >= 900) {
		tmp-=900; /* value now: 0..500 */
		tmp/=10; /* value now: 0..50 */
		tmp=50-tmp; /* value now: 50..0 */
		tmp = MAX(tmp,1); /* value now: 50..1 */
	}
	else { /* tmp: 800..899: green to slight yellow/orange */
		tmp-=800; /* value now: 0..99 */
		tmp<<=2; /* value now: 0..396 */
		tmp/=10; /* value now: 0..40 */
		tmp=MIN(tmp,38); /* value now: 0..38 */
		tmp=38-tmp; /* 38..0 */
		tmp+=51; /* 89..51 */
	}
	return hsv2rgb(tmp, 255, 255);
};

void tft_init(void) {
	uint8_t databuf[8];
	uint8_t i, j, cmd;

	/* do a hardware reset first */
	gpio_set(GPIO_TFT_NCS, 1);
	gpio_set(GPIO_TFT_NRST, 0);
	msleep(1);
	gpio_set(GPIO_TFT_NRST, 1);
	msleep(ST77_INIT_DELAY);

	/* select display and send sleep-out */
	gpio_set(GPIO_TFT_NCS, 0);
	tft_write(ST77_CMD_SLPOUT, NULL, 0);
	msleep(ST77_INIT_DELAY);

	/* process list of init commands */
	for(i=0; (cmd = flash_read_byte(&(st77_init[i]))) != ST77_CMD_INVALID; i++) {
		uint8_t dlen = flash_read_byte(&(st77_init[++i]));
		for(j=0;j<dlen;j++)
			databuf[j] = flash_read_byte(&(st77_init[++i]));
		tft_write(cmd, databuf, dlen);
	}

	display_orientation(1); // use 1 or 3
	display_rect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, 0x0000);

	tft_write(ST77_CMD_DISPON, NULL, 0);
}

uint8_t tft_update(readings_t *vals) {
	static uint8_t first_run = 1;
	char txtbuf[14]="????";
	uint16_t ppm_color = 0xffff;

	txtbuf[13]=0;

	/* no need to redraw static 'ppm CO2' string every time */
	if(first_run) {
		display_rect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, 0x0000);
		draw_text(70, 55, TFT_TEXT_12x27, "ppm CO2", 0xffff, 0);
		first_run=0;
	}

	/* CO2 */
	if(CO2_VALID(vals)) {
		i16_to_dec(vals->co2,txtbuf,4,-1,0);
		ppm_color = co2_color(vals->co2);
	}
	draw_text(0, 0, TFT_DIGITS_38x50, txtbuf, ppm_color, 0);

	memset(txtbuf,' ',13);

	/* temperature */
	if(TEMP_VALID(vals)) {
		/* draw value */
		i16_to_dec(vals->temperature, txtbuf, 3, -1, 0);
		txtbuf[3]='{'; // { replaced w/ degree sign in font *cough*
		txtbuf[4]='C';
	}

	/* relative humidity */
	if(RELH_VALID(vals)) {
		i16_to_dec(vals->relh, txtbuf+6, 3, -1, 0);
		txtbuf[6+3]='%';
		txtbuf[6+5]='r';
		txtbuf[6+6]='h';
	}

	draw_text(0, DISPLAY_HEIGHT - fonts[TFT_TEXT_12x27].h, TFT_TEXT_12x27, txtbuf, 0xffff, 0);

	/* issue a dummy command w/o data to set A0 low and reduce current
	 * through voltage divider resistors for ATMEGA328P platform */
	tft_write(ST77_CMD_NOP, NULL, 0);

	return 1;
}

uint8_t tft_showtext(char **lines, uint8_t n) {
	uint8_t h=fonts[TFT_TEXT_12x27].h, l, y;
	n=MIN(n,DISPLAY_HEIGHT/h);
	display_rect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, 0x0000);
	for(l=0,y=0;l<n;l++,y+=h)
		draw_text(0, y, TFT_TEXT_12x27, lines[l], 0xffff, 0);
	return l;
}
