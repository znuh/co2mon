#ifndef SENSORS_H
#define SENSORS_H
#include <stdint.h>

#define CO2_INVALID    UINT16_MAX
#define TEMP_INVALID   INT8_MIN
#define RELH_INVALID   INT8_MIN

#define CO2_VALID(v)   (((v)->co2 > 0) && ((v)->co2 < INT16_MAX))
#define TEMP_VALID(v)  ((v)->temperature > INT8_MIN)
#define RELH_VALID(v)  ((v)->relh > INT8_MIN)

typedef struct readings_s {
	uint16_t co2;
	int8_t  temperature;
	int8_t  relh;
} readings_t;

int sensors_init(void);
int sensors_read(readings_t *vals);
const char *sensors_getname(uint8_t i);

#endif /* SENSORS_H */
