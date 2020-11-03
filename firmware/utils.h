#ifndef UTILS_H
#define UTILS_H

#ifndef MAX
#define MAX(a,b)    ((a)>=(b)?(a):(b))
#endif

#ifndef MIN
#define MIN(a,b)    ((a)<=(b)?(a):(b))
#endif

char *i16_to_dec(int16_t val, char *buf, uint8_t n, int8_t point_ofs, uint8_t zeropad);

#endif /* UTILS_H */
