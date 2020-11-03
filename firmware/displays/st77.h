#ifndef ST77_H
#define ST77_H

/* command definitions for ST7735S */
enum {
	/* System Function Command List */
	ST77_CMD_NOP		= 0x00,	/* No Operation */
	ST77_CMD_SWRESET	= 0x01, /* Software Reset - needs 120msec wait */
	/* read commands aren't used */
	ST77_CMD_SLPIN		= 0x10, /* Sleep In & Booster Off */
	ST77_CMD_SLPOUT		= 0x11, /* Sleep Out & Booster On - needs 120 msec delay */
	ST77_CMD_PTLON		= 0x12, /* Partial Mode On */
	ST77_CMD_NORON		= 0x13, /* Partial Off (Normal) - */
	ST77_CMD_INVOFF		= 0x20, /* Display Inversion Off (Normal) */
	ST77_CMD_INVON		= 0x21, /* Display Inversion On */
	ST77_CMD_GAMSET		= 0x26, /* Gamma Curve Select */
	ST77_CMD_DISPOFF	= 0x28, /* Display Off */
	ST77_CMD_DISPON		= 0x29, /* Display On */
	ST77_CMD_CASET		= 0x2A, /* Column Address Set */
	ST77_CMD_RASET		= 0x2B, /* Row Address Set */
	ST77_CMD_RAMWR		= 0x2C, /* Memory Write */
	ST77_CMD_RGBSET		= 0x2D, /* LUT for 4k,65k,262k Color display */
	ST77_CMD_PTLAR		= 0x30, /* Partial Start/End Address Set */
	ST77_CMD_SCRLAR		= 0x33, /* Scroll area set */
	ST77_CMD_TEOFF		= 0x34, /* Tearing effect line off */
	ST77_CMD_TEON		= 0x35, /* Tearing Effect Mode Set & on */
	ST77_CMD_MADCTL		= 0x36, /* Memory Control */
	ST77_CMD_VSCSAD		= 0x37, /* Scroll Start Address of RAM */
	ST77_CMD_IDMOFF		= 0x38, /* Idle Mode Off */
	ST77_CMD_IDMON		= 0x39, /* Idle Mode On */
	ST77_CMD_COLMOD		= 0x3A, /* Interface Pixel Format */
	
	/* Panel Function Command List */
	ST77_CMD_FRMCTR1	= 0xB1, /* Normal mode Set 1-line period */
	ST77_CMD_FRMCTR2	= 0xB2, /* Idle Mode Set 1-line period */
	ST77_CMD_FRMCTR3	= 0xB3, /* Partial mode + Full Colors Set 1-line period */
	ST77_CMD_INVCTR		= 0xB4, /* Display Inversion Control */
	ST77_CMD_PWCTR1		= 0xC0, /* Power Control Setting - VRH */
	ST77_CMD_PWCTR2		= 0xC1, /* Power Control Setting - VGH/VGL */
	ST77_CMD_PWCTR3		= 0xC2, /* OPA/Boost adj. normal mode */
	ST77_CMD_PWCTR4		= 0xC3, /* OPA/Boost adj. idle mode */
	ST77_CMD_PWCTR5		= 0xC4, /* OPA/Boost adj Partial mode + Full Colors */
	ST77_CMD_VMCTR1		= 0xC5, /* VCOM Control 1 */
	ST77_CMD_VMOFCTR	= 0xC7, /* Set VCOM Offset control */
	ST77_CMD_WRID2		= 0xD1, /* Set LCM Version Code */
	ST77_CMD_WRID3		= 0xD2, /* Set the Project Code at ID3 */
	ST77_CMD_NVCTR1		= 0xD9, /* NVM Control Status */
	ST77_CMD_NVCTR2		= 0xDE, /* NVM Read Command Action Code */
	ST77_CMD_NVCTR3		= 0xDF, /* NVM Write Command Action Code */
	ST77_CMD_GAMCTRP1	= 0xE0, /* Gamma Adjustment (+ Polarity) */
	ST77_CMD_GAMCTRN1	= 0xE1, /* Gamma Adjustment (- Polarity) */
	ST77_CMD_GCV		= 0xFC, /* Gate clock Variable */
	
	/* invalid command marker */
	ST77_CMD_INVALID	= 0xFF
};

#define ST77_MADCTL_MY  (1<<7)
#define ST77_MADCTL_MX  (1<<6)
#define ST77_MADCTL_MV  (1<<5)
#define ST77_MADCTL_ML  (1<<4)
#define ST77_MADCTL_RGB (1<<3)
#define ST77_MADCTL_MH  (1<<2)

/* hw reset of display needs >=10usec low pulse, 120msec wait */
#define ST77_INIT_DELAY    120 /* milliseconds */

#endif /* ST77_H */
