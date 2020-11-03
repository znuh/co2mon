#ifndef FONTS_H
#define FONTS_H

#include <stdint.h>

enum {
	OLED_TEXT_8x8,
//	OLED_TEXT_12x16,
	
//	OLED_DIGITS_27x40,
	OLED_DIGITS_28x40,
//	OLED_DIGITS_30x40,

	TFT_TEXT_12x27,
	TFT_DIGITS_38x50,

	MAX_FONTS
};

#define FONTFLG_1BPP    (1<<0) /* otherwise: greyscale */
#define FONTFLG_VPACK   (1<<1) /* otherwise: horizontal byte-packing (only 1bpp fonts) */
#define FONTFLG_RLE     (1<<2) /* run-length encoding */

typedef struct font_s {
	const uint8_t *data;
	const uint8_t h;
	const uint8_t w;
	const uint8_t flags;
	const uint8_t n_chars;
	const char *index;
	const uint8_t char_ofs;
} font_t;

const char font_digits[13];
const font_t fonts[MAX_FONTS];

uint8_t font_index(const font_t *f, char c);

#endif /* FONTS_H */
