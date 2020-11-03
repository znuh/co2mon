#ifndef DISPLAY_H
#define DISPLAY_H

#include "sensors.h"

void display_init(void);
uint8_t display_update(readings_t *vals);
void display_getname(char *dst);
uint8_t display_showtext(char **lines, uint8_t n);

#endif /* DISPLAY_H */
