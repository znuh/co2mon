#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stddef.h> /* size_t */

#ifndef MAX
#define MAX(a,b)    ((a)>=(b)?(a):(b))
#endif

#ifndef MIN
#define MIN(a,b)    ((a)<=(b)?(a):(b))
#endif

char *i16_to_dec(int16_t val, char *buf, uint8_t n, int8_t point_ofs, uint8_t zeropad);
uint8_t crc8(uint8_t init, uint8_t poly, const uint8_t *d, size_t n);

#endif /* UTILS_H */
