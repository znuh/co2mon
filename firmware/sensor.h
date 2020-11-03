#ifndef SENSOR_H
#include <stdint.h>
#include "sensors.h"

#define IFACE_INVALID	0
#define IFACE_NONE      1
#define IFACE_I2C       2
#define IFACE_UART      3

typedef struct sensor_s {
	uint8_t iface;
	uint8_t addr;
	const char *name;
	int (*init) (uint8_t addr);
	int (*read) (uint8_t addr, readings_t *vals);
} sensor_t;

#endif /* SENSOR_H */
