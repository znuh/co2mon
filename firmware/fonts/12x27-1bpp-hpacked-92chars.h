#include "platform.h"
#include <stdint.h>

static const uint8_t font_12x27_hpacked[] PLATFORM_FLASH_MEM = {
	0x00, 0x00, 0x00, 0x98, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00,
	0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x98, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x98, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
	0x80, 0xc1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
	0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x0c, 0xc0,
	0x0f, 0xf8, 0x01, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x18, 0x00, 0x00,
	0x06, 0x98, 0x01, 0x33, 0x60, 0xc0, 0x3f, 0x70, 0x00, 0x06, 0xc0, 0xc1, 0x01, 0x60, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xf8, 0x00, 0x06, 0xf8, 0x80, 0x0f, 0x80, 0xe1, 0x3f, 0xe0,
	0xe1, 0x7f, 0xf8, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x06, 0x80, 0x0f, 0x00, 0x00,
	0x06, 0xfe, 0x80, 0x1f, 0xfe, 0xe0, 0x3f, 0xfe, 0x83, 0x1f, 0x06, 0x06, 0x0f, 0x00, 0x63, 0x30,
	0x06, 0x30, 0x60, 0x06, 0x86, 0x0f, 0xfe, 0x80, 0x0f, 0xfe, 0x80, 0x1f, 0xfe, 0x67, 0x60, 0x06,
	0x36, 0x60, 0x06, 0x63, 0x60, 0xfe, 0x87, 0x1f, 0x0c, 0xc0, 0x0f, 0x9c, 0x03, 0x00, 0x00, 0x00,
	0x00, 0x06, 0x00, 0x00, 0x00, 0x06, 0x00, 0xe0, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0xc0, 0x00,
	0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x18, 0x00, 0x00, 0x06, 0x00, 0x00, 0x33, 0x60, 0xe0,
	0x3f, 0xf8, 0x00, 0x00, 0x60, 0x00, 0x03, 0x6c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
	0xfe, 0x03, 0x06, 0xfc, 0xc3, 0x3f, 0x80, 0xe1, 0x3f, 0xf8, 0xe1, 0x7f, 0xfe, 0xc3, 0x3f, 0x00,
	0x00, 0x00, 0x00, 0x07, 0x00, 0x0e, 0xc0, 0x3f, 0x00, 0x00, 0x06, 0xfe, 0xe3, 0x3f, 0xfe, 0xe3,
	0x3f, 0xfe, 0xe3, 0x3f, 0x06, 0x06, 0x0f, 0x00, 0x63, 0x30, 0x06, 0x30, 0x60, 0x06, 0xe6, 0x3f,
	0xfe, 0xe3, 0x3f, 0xfe, 0xc3, 0x3f, 0xfe, 0x67, 0x60, 0x06, 0x36, 0x60, 0x06, 0x63, 0x60, 0xfe,
	0x83, 0x01, 0x0c, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x06,
	0x00, 0xf0, 0x03, 0x00, 0x06, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xc0, 0x18, 0x00, 0x00, 0x06, 0x00, 0x00, 0x33, 0x60, 0x30, 0x33, 0xcc, 0x00, 0x00, 0x60, 0x00,
	0x03, 0x6c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x06, 0x03, 0x07, 0x06, 0x63, 0x30,
	0xc0, 0x61, 0x00, 0x1c, 0x00, 0x60, 0x06, 0xc3, 0x30, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x1c,
	0x60, 0x30, 0xf8, 0x00, 0x06, 0x06, 0x63, 0x60, 0x06, 0x63, 0x00, 0x06, 0x60, 0x60, 0x06, 0x06,
	0x06, 0x00, 0x63, 0x18, 0x06, 0x70, 0x70, 0x06, 0x66, 0x30, 0x06, 0x63, 0x30, 0x06, 0x63, 0x60,
	0x60, 0x60, 0x60, 0x06, 0x36, 0x60, 0x8c, 0x61, 0x60, 0x00, 0x83, 0x01, 0x18, 0x00, 0x0c, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x06, 0x00, 0x18, 0x06, 0x00, 0x06, 0x00,
	0x06, 0x80, 0xc1, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x06, 0x00,
	0x00, 0x33, 0xf8, 0x31, 0x1b, 0xcc, 0x00, 0x00, 0x30, 0x00, 0x06, 0xf8, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x18, 0x03, 0x07, 0x07, 0x06, 0x66, 0x60, 0xc0, 0x61, 0x00, 0x0c, 0x00, 0x60,
	0x03, 0x66, 0x60, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x38, 0x60, 0x60, 0xfe, 0x03, 0x06, 0x06,
	0x36, 0x60, 0x06, 0x66, 0x00, 0x06, 0x30, 0x60, 0x06, 0x06, 0x06, 0x00, 0x63, 0x18, 0x06, 0x70,
	0x70, 0x0e, 0x36, 0x60, 0x06, 0x36, 0x60, 0x06, 0x66, 0x60, 0x60, 0x60, 0x60, 0x06, 0x36, 0x60,
	0x8c, 0xc1, 0x30, 0x80, 0x81, 0x01, 0x18, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
	0x00, 0x00, 0x00, 0x06, 0x00, 0x18, 0x06, 0x00, 0x06, 0x00, 0x06, 0x80, 0xc1, 0x00, 0x60, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x06, 0x00, 0x00, 0x33, 0xfc, 0x33, 0x1b, 0xcc,
	0x00, 0x00, 0x30, 0x00, 0x06, 0xfe, 0x07, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x83, 0x07,
	0x06, 0x00, 0x06, 0x60, 0xe0, 0x61, 0x00, 0x06, 0x00, 0x30, 0x03, 0x66, 0x60, 0x00, 0x00, 0x00,
	0xe0, 0x00, 0x00, 0x70, 0x00, 0x60, 0x06, 0x03, 0x0f, 0x06, 0x36, 0x00, 0x06, 0x66, 0x00, 0x06,
	0x30, 0x00, 0x06, 0x06, 0x06, 0x00, 0x63, 0x0c, 0x06, 0xf0, 0x78, 0x0e, 0x36, 0x60, 0x06, 0x36,
	0x60, 0x06, 0x66, 0x00, 0x60, 0x60, 0x60, 0x0c, 0x33, 0x60, 0x8c, 0xc1, 0x30, 0x80, 0x81, 0x01,
	0x18, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x06, 0x00, 0x18,
	0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x06, 0x00, 0xe0, 0xff, 0x66, 0x36, 0x1b, 0x6c, 0x00, 0x00, 0x30, 0x00, 0x06, 0xfe,
	0x07, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x83, 0x07, 0x06, 0x00, 0x06, 0x60, 0xe0, 0x61,
	0x00, 0x06, 0x00, 0x30, 0x03, 0x66, 0x60, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0xe0, 0x00, 0x60,
	0x03, 0x06, 0x0f, 0x06, 0x36, 0x00, 0x06, 0x66, 0x00, 0x06, 0x30, 0x00, 0x06, 0x06, 0x06, 0x00,
	0x63, 0x0c, 0x06, 0xf0, 0x78, 0x1e, 0x36, 0x60, 0x06, 0x36, 0x60, 0x06, 0x66, 0x00, 0x60, 0x60,
	0x60, 0x0c, 0x33, 0x60, 0xd8, 0xc0, 0x30, 0xc0, 0x80, 0x01, 0x30, 0x00, 0x0c, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x06, 0x00, 0x18, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00,
	0xc0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0xe0, 0xff,
	0x66, 0xe6, 0x0d, 0x78, 0x00, 0x00, 0x18, 0x00, 0x0c, 0xf8, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x0c, 0xc3, 0x06, 0x06, 0x00, 0x06, 0x60, 0xb0, 0x61, 0x00, 0x06, 0x00, 0x18, 0x03, 0x66,
	0x60, 0x70, 0x00, 0x07, 0x38, 0xe0, 0x3f, 0xc0, 0x01, 0x60, 0xe3, 0x06, 0x0f, 0x06, 0x36, 0x00,
	0x06, 0x66, 0x00, 0x06, 0x30, 0x00, 0x06, 0x06, 0x06, 0x00, 0x63, 0x06, 0x06, 0xb0, 0x6d, 0x1e,
	0x36, 0x60, 0x06, 0x36, 0x60, 0x06, 0x66, 0x00, 0x60, 0x60, 0x60, 0x0c, 0x33, 0x60, 0xd8, 0x80,
	0x19, 0xc0, 0x80, 0x01, 0x30, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f, 0xf6, 0x81, 0x1f,
	0xf8, 0x86, 0x1f, 0x18, 0x80, 0x3f, 0xf6, 0x00, 0x07, 0xc0, 0xc1, 0x60, 0x60, 0x60, 0x1b, 0xf6,
	0x00, 0x0f, 0xfc, 0x81, 0x3f, 0xec, 0x81, 0x0f, 0xfe, 0x61, 0x30, 0x06, 0x66, 0x66, 0x0c, 0x63,
	0x30, 0xfc, 0x07, 0x00, 0x00, 0x00, 0x06, 0x00, 0x80, 0x19, 0x66, 0xc0, 0x0c, 0x38, 0x00, 0x00,
	0x18, 0x00, 0x0c, 0x6c, 0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0xc3, 0x06, 0x06, 0x00,
	0x03, 0x30, 0xb0, 0xe1, 0x0f, 0xf6, 0x00, 0x18, 0x06, 0x63, 0x60, 0x70, 0x00, 0x07, 0x1c, 0xe0,
	0x3f, 0x80, 0x03, 0x30, 0xf3, 0x87, 0x19, 0x06, 0x33, 0x00, 0x06, 0x66, 0x00, 0x06, 0x30, 0x00,
	0x06, 0x06, 0x06, 0x00, 0x63, 0x06, 0x06, 0xb0, 0x6d, 0x36, 0x36, 0x60, 0x06, 0x36, 0x60, 0x06,
	0x66, 0x00, 0x60, 0x60, 0x60, 0x0c, 0x33, 0x62, 0xd8, 0x80, 0x19, 0x60, 0x80, 0x01, 0x30, 0x00,
	0x0c, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0xfe, 0xc3, 0x3f, 0xfc, 0xc7, 0x3f, 0xfe, 0xc0, 0x3f,
	0xfe, 0x01, 0x07, 0xc0, 0xc1, 0x30, 0x60, 0xe0, 0x3f, 0xfe, 0xc1, 0x3f, 0xfc, 0xc3, 0x3f, 0xfc,
	0xc3, 0x1f, 0xfe, 0x61, 0x30, 0x06, 0x66, 0x66, 0x0c, 0x63, 0x30, 0xfc, 0x07, 0x00, 0x00, 0x00,
	0x06, 0x00, 0x80, 0x19, 0x66, 0x00, 0x0c, 0x38, 0x00, 0x00, 0x18, 0x00, 0x0c, 0x6c, 0x03, 0x06,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x63, 0x06, 0x06, 0x00, 0x03, 0x1f, 0x98, 0xe1, 0x3f, 0xfe,
	0x03, 0x0c, 0xfc, 0x61, 0x60, 0x70, 0x00, 0x07, 0x0e, 0x00, 0x00, 0x00, 0x07, 0x30, 0x1b, 0x87,
	0x19, 0xfe, 0x31, 0x00, 0x06, 0xe6, 0x0f, 0x7e, 0x30, 0x00, 0xfe, 0x07, 0x06, 0x00, 0xe3, 0x03,
	0x06, 0x30, 0x67, 0x36, 0x36, 0x60, 0x06, 0x36, 0x60, 0x06, 0xc3, 0x1f, 0x60, 0x60, 0x60, 0x98,
	0x31, 0x62, 0x70, 0x80, 0x19, 0x60, 0x80, 0x01, 0x60, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x60, 0x0e, 0x66, 0x60, 0x06, 0x67, 0x60, 0xfe, 0x60, 0x30, 0x0e, 0x03, 0x06, 0x80, 0xc1, 0x18,
	0x60, 0x60, 0x66, 0x0e, 0xc3, 0x30, 0x0c, 0x66, 0x30, 0x1c, 0x63, 0x30, 0x18, 0x60, 0x30, 0x06,
	0x66, 0x66, 0x98, 0x61, 0x30, 0x00, 0x06, 0x00, 0x00, 0x00, 0x06, 0x00, 0x80, 0x19, 0xfc, 0x01,
	0x06, 0x3c, 0x00, 0x00, 0x18, 0x00, 0x0c, 0x60, 0xe0, 0x7f, 0x00, 0xe0, 0x7f, 0x00, 0x00, 0x06,
	0x63, 0x06, 0x06, 0x80, 0x01, 0x1f, 0x98, 0x01, 0x30, 0x0e, 0x03, 0x0c, 0xfc, 0xc1, 0x70, 0x00,
	0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x07, 0x18, 0x1b, 0x86, 0x19, 0xfe, 0x31, 0x00, 0x06, 0xe6,
	0x0f, 0x7e, 0x30, 0x7e, 0xfe, 0x07, 0x06, 0x00, 0xe3, 0x03, 0x06, 0x30, 0x67, 0x66, 0x36, 0x60,
	0x06, 0x33, 0x60, 0xfe, 0x83, 0x3f, 0x60, 0x60, 0x60, 0x98, 0x31, 0x67, 0x70, 0x00, 0x0f, 0x30,
	0x80, 0x01, 0x60, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x66, 0x60, 0x06, 0x66,
	0x60, 0x18, 0x60, 0x30, 0x06, 0x03, 0x06, 0x80, 0xc1, 0x0c, 0x60, 0x60, 0x66, 0x06, 0x63, 0x60,
	0x0c, 0x66, 0x30, 0x0c, 0x60, 0x30, 0x18, 0x60, 0x30, 0x0c, 0x63, 0x66, 0x98, 0x61, 0x30, 0x00,
	0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x80, 0x19, 0xf8, 0x03, 0x06, 0x6c, 0x00, 0x00, 0x18, 0x00,
	0x0c, 0x00, 0xe0, 0x7f, 0x00, 0xe0, 0x7f, 0x00, 0x00, 0x06, 0x33, 0x06, 0x06, 0xc0, 0x00, 0x30,
	0x8c, 0x01, 0x60, 0x06, 0x06, 0x06, 0x06, 0xc3, 0x7f, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x80,
	0x03, 0x0c, 0x1b, 0x86, 0x1f, 0x06, 0x33, 0x00, 0x06, 0x66, 0x00, 0x06, 0x30, 0x7e, 0x06, 0x06,
	0x06, 0x00, 0xe3, 0x03, 0x06, 0x30, 0x62, 0x66, 0x36, 0x60, 0xfe, 0x33, 0x60, 0xfe, 0x00, 0x60,
	0x60, 0x60, 0x60, 0x98, 0x31, 0x67, 0x70, 0x00, 0x0f, 0x30, 0x80, 0x01, 0x60, 0x00, 0x0c, 0x00,
	0x00, 0x00, 0x00, 0x80, 0x6f, 0x06, 0x66, 0x00, 0x06, 0xe6, 0x7f, 0x18, 0x60, 0x30, 0x06, 0x03,
	0x06, 0x80, 0xc1, 0x06, 0x60, 0x60, 0x66, 0x06, 0x63, 0x60, 0x0c, 0x66, 0x30, 0x0c, 0x60, 0x00,
	0x18, 0x60, 0x30, 0x0c, 0x63, 0x66, 0xf0, 0x60, 0x30, 0x80, 0x01, 0x00, 0x00, 0x00, 0x06, 0x00,
	0x80, 0x19, 0x60, 0x06, 0x06, 0x66, 0x00, 0x00, 0x18, 0x00, 0x0c, 0x00, 0x00, 0x06, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x03, 0x33, 0x06, 0x06, 0x60, 0x00, 0x60, 0x8c, 0x01, 0x60, 0x06, 0x06, 0x06,
	0x03, 0x06, 0x6f, 0x00, 0x00, 0x00, 0x38, 0xe0, 0x3f, 0xc0, 0x01, 0x06, 0x1b, 0xc6, 0x3f, 0x06,
	0x36, 0x00, 0x06, 0x66, 0x00, 0x06, 0x30, 0x60, 0x06, 0x06, 0x06, 0x00, 0x63, 0x06, 0x06, 0x30,
	0x62, 0xc6, 0x36, 0x60, 0xfe, 0x30, 0x60, 0xc6, 0x00, 0x60, 0x60, 0x60, 0x60, 0x98, 0xb1, 0x6d,
	0xd8, 0x00, 0x06, 0x18, 0x80, 0x01, 0xc0, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x7f, 0x06,
	0x66, 0x00, 0x06, 0xe6, 0x7f, 0x18, 0x60, 0x30, 0x06, 0x03, 0x06, 0x80, 0xc1, 0x03, 0x60, 0x60,
	0x66, 0x06, 0x63, 0x60, 0x0c, 0x66, 0x30, 0x0c, 0xc0, 0x0f, 0x18, 0x60, 0x30, 0x0c, 0x63, 0x66,
	0xf0, 0x60, 0x30, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0xf0, 0x7f, 0x60, 0x06, 0x03, 0xc6,
	0x06, 0x00, 0x18, 0x00, 0x0c, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1b, 0x06,
	0x06, 0x30, 0x00, 0x60, 0x86, 0x01, 0x60, 0x06, 0x06, 0x03, 0x03, 0x06, 0x60, 0x00, 0x00, 0x00,
	0x70, 0xe0, 0x3f, 0xe0, 0x00, 0x06, 0xf3, 0xc3, 0x30, 0x06, 0x36, 0x00, 0x06, 0x66, 0x00, 0x06,
	0x30, 0x60, 0x06, 0x06, 0x06, 0x00, 0x63, 0x06, 0x06, 0x30, 0x60, 0xc6, 0x36, 0x60, 0x06, 0x30,
	0x60, 0xc6, 0x00, 0x60, 0x60, 0x60, 0x60, 0xf0, 0xb0, 0x6d, 0xd8, 0x00, 0x06, 0x18, 0x80, 0x01,
	0xc0, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x60, 0x70, 0x06, 0x66, 0x00, 0x06, 0x66, 0x00, 0x18,
	0x60, 0x30, 0x06, 0x03, 0x06, 0x80, 0xc1, 0x01, 0x60, 0x60, 0x66, 0x06, 0x63, 0x60, 0x0c, 0x66,
	0x30, 0x0c, 0x80, 0x1f, 0x18, 0x60, 0x30, 0x98, 0x61, 0x66, 0x60, 0x60, 0x30, 0x60, 0x00, 0x00,
	0x00, 0x00, 0x06, 0x00, 0xf0, 0x7f, 0x66, 0x06, 0x1b, 0xc3, 0x06, 0x00, 0x18, 0x00, 0x0c, 0x00,
	0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1b, 0x06, 0x06, 0x18, 0x00, 0x60, 0xfe, 0x07,
	0x60, 0x06, 0x06, 0x03, 0x03, 0x06, 0x60, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x70, 0x00, 0x03,
	0xe3, 0xc1, 0x30, 0x06, 0x36, 0x00, 0x06, 0x66, 0x00, 0x06, 0x30, 0x60, 0x06, 0x06, 0x06, 0x00,
	0x63, 0x0c, 0x06, 0x30, 0x60, 0x86, 0x37, 0x60, 0x06, 0x30, 0x60, 0x86, 0x01, 0x60, 0x60, 0x60,
	0x60, 0xf0, 0xf0, 0x78, 0xd8, 0x00, 0x06, 0x0c, 0x80, 0x01, 0xc0, 0x00, 0x0c, 0x00, 0x00, 0x00,
	0x00, 0x60, 0x60, 0x06, 0x66, 0x00, 0x06, 0x66, 0x00, 0x18, 0x60, 0x30, 0x06, 0x03, 0x06, 0x80,
	0xc1, 0x03, 0x60, 0x60, 0x66, 0x06, 0x63, 0x60, 0x0c, 0x66, 0x30, 0x0c, 0x00, 0x30, 0x18, 0x60,
	0x30, 0x98, 0x61, 0x66, 0xf0, 0x60, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0xc0, 0x0c,
	0x66, 0x06, 0x3f, 0x83, 0x07, 0x00, 0x18, 0x00, 0x0c, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
	0x80, 0x01, 0x0f, 0x06, 0x06, 0x0c, 0x00, 0x60, 0xfe, 0x07, 0x60, 0x06, 0x06, 0x03, 0x03, 0x06,
	0x60, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x38, 0x00, 0x03, 0x03, 0xc0, 0x30, 0x06, 0x36, 0x00,
	0x06, 0x66, 0x00, 0x06, 0x30, 0x60, 0x06, 0x06, 0x06, 0x00, 0x63, 0x0c, 0x06, 0x30, 0x60, 0x86,
	0x37, 0x60, 0x06, 0x30, 0x60, 0x86, 0x01, 0x60, 0x60, 0x60, 0x60, 0xf0, 0xf0, 0x78, 0x8c, 0x01,
	0x06, 0x0c, 0x80, 0x01, 0x80, 0x01, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x06, 0x66, 0x00,
	0x06, 0x66, 0x00, 0x18, 0x60, 0x30, 0x06, 0x03, 0x06, 0x80, 0xc1, 0x06, 0x60, 0x60, 0x66, 0x06,
	0x63, 0x60, 0x0c, 0x66, 0x30, 0x0c, 0x00, 0x30, 0x18, 0x60, 0x30, 0x98, 0x61, 0x66, 0xf0, 0x60,
	0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0xc0, 0x0c, 0xfc, 0x83, 0x67, 0x83, 0x03, 0x00,
	0x30, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x0f, 0x06, 0x06, 0x06,
	0x60, 0x60, 0x80, 0x61, 0x60, 0x06, 0x86, 0x01, 0x03, 0x06, 0x30, 0x00, 0x00, 0x00, 0x80, 0x03,
	0x00, 0x1c, 0x00, 0x03, 0x03, 0x66, 0x60, 0x06, 0x36, 0x60, 0x06, 0x66, 0x00, 0x06, 0x30, 0x60,
	0x06, 0x06, 0x06, 0x06, 0x63, 0x18, 0x06, 0x30, 0x60, 0x06, 0x37, 0x60, 0x06, 0x30, 0x60, 0x06,
	0x63, 0x60, 0x60, 0x60, 0x60, 0xf0, 0x70, 0x70, 0x8c, 0x01, 0x06, 0x06, 0x80, 0x01, 0x80, 0x01,
	0x0c, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x06, 0x66, 0x60, 0x06, 0x66, 0x60, 0x18, 0x60, 0x30,
	0x06, 0x03, 0x06, 0x80, 0xc1, 0x0c, 0x60, 0x60, 0x66, 0x06, 0x63, 0x60, 0x0c, 0x66, 0x30, 0x0c,
	0x60, 0x30, 0x18, 0x60, 0x30, 0xf0, 0x60, 0x66, 0x98, 0x61, 0x30, 0x0c, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xc0, 0x0c, 0xf8, 0x81, 0x67, 0xc7, 0x03, 0x00, 0x30, 0x00, 0x06, 0x00, 0x00, 0x00,
	0x70, 0x00, 0x00, 0x70, 0x80, 0x01, 0x06, 0x03, 0x06, 0x06, 0x60, 0x30, 0x80, 0x61, 0x30, 0x0c,
	0x83, 0x01, 0x06, 0x03, 0x38, 0x70, 0x00, 0x07, 0x00, 0x07, 0x00, 0x0e, 0x00, 0x00, 0x06, 0x66,
	0x60, 0x06, 0x63, 0x60, 0x06, 0x63, 0x00, 0x06, 0x60, 0x70, 0x06, 0x06, 0x06, 0x06, 0x63, 0x18,
	0x06, 0x30, 0x60, 0x06, 0x67, 0x30, 0x06, 0x60, 0x33, 0x06, 0x63, 0x60, 0x60, 0xc0, 0x30, 0x60,
	0x70, 0x70, 0x8c, 0x01, 0x06, 0x06, 0x80, 0x01, 0x80, 0x01, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x60,
	0x60, 0x06, 0x66, 0x60, 0x06, 0x66, 0x60, 0x18, 0x60, 0x38, 0x06, 0x03, 0x06, 0x80, 0xc1, 0x18,
	0x60, 0x60, 0x66, 0x06, 0xc3, 0x30, 0x1c, 0x66, 0x38, 0x0c, 0x60, 0x30, 0x18, 0x60, 0x38, 0xf0,
	0x60, 0x66, 0x98, 0x61, 0x38, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0c, 0x60, 0x80,
	0x67, 0xfe, 0x07, 0x00, 0x30, 0x00, 0x06, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0xc0, 0x00,
	0xfe, 0x03, 0x0f, 0xfe, 0xc7, 0x3f, 0x80, 0xc1, 0x3f, 0xfc, 0x83, 0x01, 0xfe, 0x83, 0x1f, 0x70,
	0x00, 0x07, 0x00, 0x06, 0x00, 0x06, 0x00, 0x03, 0xfe, 0x63, 0x60, 0xfe, 0xe3, 0x3f, 0xfe, 0xe3,
	0x3f, 0x06, 0xe0, 0x7f, 0x06, 0x06, 0x0f, 0xfc, 0x61, 0x30, 0xfe, 0x33, 0x60, 0x06, 0xe6, 0x3f,
	0x06, 0xe0, 0x3f, 0x06, 0xc6, 0x3f, 0x60, 0xc0, 0x3f, 0x60, 0x30, 0x60, 0x06, 0x03, 0x06, 0xff,
	0x87, 0x01, 0x00, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x7f, 0xfe, 0xc3, 0x3f, 0xfc, 0xc7,
	0x3f, 0x18, 0xc0, 0x3f, 0x06, 0x03, 0x0f, 0x80, 0xc1, 0x30, 0xf0, 0x60, 0x66, 0x06, 0xc3, 0x3f,
	0xfc, 0xc3, 0x3f, 0x0c, 0xc0, 0x1f, 0xf0, 0xc1, 0x3f, 0x60, 0xe0, 0x3f, 0x0c, 0xc3, 0x3f, 0xfe,
	0x07, 0x00, 0x00, 0x00, 0x06, 0x00, 0xc0, 0x0c, 0x60, 0xc0, 0x66, 0x7c, 0x06, 0x00, 0x60, 0x00,
	0x03, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0xc0, 0x00, 0xf8, 0x00, 0x0f, 0xfe, 0x87, 0x0f,
	0x80, 0x81, 0x0f, 0xf0, 0x80, 0x01, 0xf8, 0x80, 0x07, 0x70, 0x00, 0x07, 0x00, 0x04, 0x00, 0x02,
	0x00, 0x03, 0xf8, 0x61, 0x60, 0xfe, 0x80, 0x1f, 0xfe, 0xe0, 0x3f, 0x06, 0x80, 0x6f, 0x06, 0x06,
	0x0f, 0xf8, 0x60, 0x30, 0xfe, 0x33, 0x60, 0x06, 0x86, 0x0f, 0x06, 0x80, 0x0f, 0x06, 0x86, 0x1f,
	0x60, 0x00, 0x0f, 0x60, 0x30, 0x60, 0x06, 0x03, 0x06, 0xff, 0x87, 0x01, 0x00, 0x03, 0x0c, 0x00,
	0x00, 0x00, 0x00, 0x80, 0x7f, 0xfe, 0x81, 0x1f, 0xf8, 0x87, 0x1f, 0x18, 0x80, 0x37, 0x06, 0x03,
	0x0f, 0x80, 0xc1, 0x60, 0xf0, 0x60, 0x66, 0x06, 0x03, 0x0f, 0xec, 0x81, 0x37, 0x0c, 0x80, 0x0f,
	0xe0, 0x81, 0x37, 0x60, 0xe0, 0x19, 0x0c, 0x83, 0x37, 0xfe, 0x07, 0x00, 0x00, 0x00, 0x06, 0x00,
	0xc0, 0x0c, 0x60, 0xc0, 0x3c, 0x00, 0x00, 0x00, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00, 0x60, 0x00,
	0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x00, 0xc3, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x18, 0x00,
	0x00, 0x00, 0xc0, 0xc1, 0x01, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f,
	0x00, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x30, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00,
	0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc1, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xc0,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x0c, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xf0, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07,
	0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x30, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00
};
