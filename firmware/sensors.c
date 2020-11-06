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
#include "sensor.h"
#include "utils.h"
#include <string.h>

//#define DEBUG

#ifdef DEBUG
#include <stdio.h>
#endif

int t67xx_init(uint8_t addr);
int t67xx_read(uint8_t addr, readings_t *vals);

int hih6x_init(uint8_t addr);
int hih6x_read(uint8_t addr, readings_t *vals);

int lm75_init(uint8_t addr);
int lm75_read(uint8_t addr, readings_t *vals);

int scd30_init(uint8_t addr);
int scd30_read(uint8_t addr, readings_t *vals);

int shtc3_init(uint8_t addr);
int shtc3_read(uint8_t addr, readings_t *vals);

int mhzxx_init(uint8_t addr);
int mhzxx_read(uint8_t addr, readings_t *vals);

/* if values for temp/rhel/co2 are provided by multiple
 * sensors, sensors later in this list "win"
 * b/c they overwrite the values provided by
 * previous sensors. */
static const sensor_t sensors[] = {
	{
		.iface = IFACE_UART,
		.addr  = 0,
		.name  = "MH-Zxx",    /* MH-Z14A, MH-Z19B: CO2 only */
		.init  = mhzxx_init,
		.read  = mhzxx_read,
	},
	/* temperature of SCD30 doesn't seem to be too accurate
	 * therefor we move it up in this list */
	{
		.iface = IFACE_I2C,
		.addr  = 0x61,
		.name  = "SCD30",     /* CO2, temperature, humidity */
		.init  = scd30_init,
		.read  = scd30_read,
	},
	/* move SHTC3 up in this list
	 * b/c it's heated a bit by the co2mon PCB
	 * when mounted directly on the PCB */
	{
		.iface = IFACE_I2C,
		.addr  = 0x70,
		.name  = "SHTC3",     /* temperature, humidity */
		.init  = shtc3_init,
		.read  = shtc3_read,
	},
	{
		.iface = IFACE_I2C,
		.addr  = 0x15,
		.name  = "T67xx",     /* CO2 only */
		.init  = t67xx_init,
		.read  = t67xx_read,
	},
	{
		.iface = IFACE_I2C,
		.addr  = 0x27,
		.name  = "HIH6x",     /* temperature, humidity */
		.init  = hih6x_init,
		.read  = hih6x_read,
	},
	{
		.iface = IFACE_I2C,
		.addr  = 0x48,
		.name  = "LM75",     /* temperature */
		.init  = lm75_init,
		.read  = lm75_read,
	},
	{
		.iface = 0,
		.addr  = 0,
		.name  = "",
		.init  = NULL,
		.read  = NULL,
	}
};

#define MAX_SENSORS        3 /* CO2, temp, humidity */
static uint8_t avail_sensors[MAX_SENSORS];
static uint8_t n_sensors = 0;

int sensors_init(void) {
	int i;
	for(i=0;(n_sensors<MAX_SENSORS) && sensors[i].iface;i++) {
		const sensor_t *s = sensors+i;
		int res = 0;
		if(s->iface == IFACE_I2C) {
			uint8_t ping;
			res = i2c_xfer(s->addr, NULL, 0, &ping, 1);
#ifdef DEBUG
			printf("i2c probe %s@%02x: %d\n",s->name,s->addr,res);
#endif
			if(res < 1)
				continue;
		}
		res = s->init(s->addr);
#ifdef DEBUG
		printf("%s@%02x: init: %d\n",s->name,s->addr,res);
#endif
		if(res < 1)
			continue;
#ifdef DEBUG
		printf("sensor %s found\n",s->name);
#endif /* DEBUG */
		avail_sensors[n_sensors++] = i;
	}
	return n_sensors;
}

int sensors_read(readings_t *vals) {
	int i;

	vals->co2         = CO2_INVALID;
	vals->temperature = TEMP_INVALID;
	vals->relh        = RELH_INVALID;
	
	for(i=0;i<n_sensors;i++) {
		const sensor_t *s = sensors + avail_sensors[i];
		s->read(s->addr, vals);
	}

#ifdef DEBUG
	printf("CO2: %d ppm, temp: %d°C, rh: %d%%\n", vals->co2, vals->temperature, vals->relh);
#endif /* DEBUG */

	return 0;
}

const char *sensors_getname(uint8_t i) {
	return (i<n_sensors) ? sensors[avail_sensors[i]].name : NULL;
}
