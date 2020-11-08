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
#include "platform.h"
#include "sensors.h"
#include "display.h"
#include "utils.h"
#include <string.h>

/* maximum string length for both displays is 13 chars 
 * maximum number of lines for both displays is *4* */
#define PROJECT_STRING    "znu.nz/co2mon"
/* length of platform string (defined in Makefile) must be <= 10,
 * firmware string length must be <= 10 */
#define FIRMWARE_STRING   "v0.1-rc1"

static void print_info(void) {
	char display[14]="display: ";
	char sensors[3][14]={"","",""};
	char *lines[8]={
		PROJECT_STRING,
		"hw:" PLATFORM_STRING,
		"fw:" FIRMWARE_STRING,
		display,
		"sensors:",
		sensors[0],
		sensors[1],
		sensors[2]
	};
	const char *sn;
	uint8_t i;
	display_getname(display+9);
	/* get list of sensors */
	for(i=0;(i<3)&&(sn=sensors_getname(i));i++) {
		strncpy(sensors[i]+1,sn,12);
		sensors[i][0]='+';
		sensors[i][13]=0;
	}
	/* display - split into multiple pages if necessary */
	for(i=0;i<8;) {
		i+=display_showtext(lines+i, 8-i);
		msleep(5000);
	}
	/* also output info via system UART */
	for(i=0;i<8;i++) {
		uart_tx(UART_SYS_CH, lines[i], strlen(lines[i]));
		uart_tx(UART_SYS_CH, "\r\n", 2);
	}
}

static void serial_printvals(readings_t *v) {
	if(CO2_VALID(v)) {
		char buf[16]="CO2: xxxx ppm ";
		i16_to_dec(v->co2,buf+5,4,-1,0);
		uart_tx(UART_SYS_CH, buf, strlen(buf));
	}
	if(TEMP_VALID(v)) {
		char buf[16]="temp: +99°C ";
		i16_to_dec(v->temperature,buf+6,3,-1,0);
		uart_tx(UART_SYS_CH, buf, strlen(buf));
	}
	if(RELH_VALID(v)) {
		char buf[16]="relh: xxx% ";
		i16_to_dec(v->relh,buf+6,3,-1,0);
		uart_tx(UART_SYS_CH, buf, strlen(buf));
	}
	uart_tx(UART_SYS_CH, "\r\n", 2);
}

/* TODOs:
 * - use watchdog
 * - STM32: send values via USB to host
 */
int main(int argc, char **argv) {
	readings_t vals;

	hw_init(argc, argv);
	msleep(100); /* wait a bit for sensors to come up after poweron */

	display_init();
	sensors_init();

	print_info();

	while(1) {
		sensors_read(&vals);
		display_update(&vals);
		serial_printvals(&vals);
		msleep(5000);
	}

	return 0;
}
