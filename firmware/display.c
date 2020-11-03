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
#include "display.h"
#include "platform.h"
#include <string.h>

//#define DEBUG
#ifdef DEBUG
#include <stdio.h>
#endif

enum {
	DISPLAY_NONE        = 0,
	DISPLAY_OLED_128x64 = 1,
	DISPLAY_TFT_160x120 = 2
};

uint8_t oled_init(void);
uint8_t oled_update(readings_t *vals);
uint8_t oled_showtext(char **lines, uint8_t n);

void    tft_init(void);
uint8_t tft_update(readings_t *vals);
uint8_t tft_showtext(char **lines, uint8_t n);

static uint8_t display_type = DISPLAY_NONE;

void display_init(void) {
	display_type = oled_init() ? DISPLAY_OLED_128x64 : DISPLAY_TFT_160x120;
	if(display_type == DISPLAY_TFT_160x120)
		tft_init();
#ifdef DEBUG
	puts(display_type == DISPLAY_TFT_160x120 ? "using TFT display" : "using OLED display");
#endif
}

uint8_t display_update(readings_t *vals) {
	return display_type == DISPLAY_OLED_128x64 ? oled_update(vals) : tft_update(vals);
}

void display_getname(char *dst) {
	strcpy(dst, display_type == DISPLAY_OLED_128x64 ? "OLED" : "TFT");
}

uint8_t display_showtext(char **lines, uint8_t n) {
	return display_type == DISPLAY_OLED_128x64 ? oled_showtext(lines,n) : tft_showtext(lines,n);
}
