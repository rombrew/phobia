#ifndef _H_SPI_
#define _H_SPI_

#include "libc.h"

#define GPIO_SPI_EXT_NSS			XGPIO_DEF4('A', 4, 4, 5)
#define GPIO_SPI_EXT_SCK			XGPIO_DEF4('A', 5, 5, 5)
#define GPIO_SPI_EXT_MISO			XGPIO_DEF4('A', 6, 6, 5)
#define GPIO_SPI_EXT_MOSI			XGPIO_DEF4('A', 7, 7, 5)

enum {
	SPI_ID_ON_PCB			= 0,
	SPI_ID_EXT
};

enum {
	SPI_MODE_LOW_RISING		= 0,
	SPI_MODE_HIGH_FALLING,
	SPI_MODE_LOW_FALLING,
	SPI_MODE_HIGH_RISING,

	SPI_MODE_SIZE_8			= 0,
	SPI_MODE_SIZE_16		= 4,
};

void SPI_startup(int bus_ID, int freq_hz, int mode);
void SPI_halt(int bus_ID);

u16_t SPI_transfer(int bus_ID, u16_t txbuf);

#endif /* _H_SPI_ */

