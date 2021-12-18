#ifndef _H_SPI_
#define _H_SPI_

#include "libc.h"

#define GPIO_SPI_NSS			XGPIO_DEF4('A', 4, 4, 5)
#define GPIO_SPI_SCK			XGPIO_DEF4('A', 5, 5, 5)
#define GPIO_SPI_MISO			XGPIO_DEF4('A', 6, 6, 5)
#define GPIO_SPI_MOSI			XGPIO_DEF4('A', 7, 7, 5)

enum {
	SPI_MODE_LOW_RISING	= 0U,
	SPI_MODE_HIGH_FALLING,
	SPI_MODE_LOW_FALLING,
	SPI_MODE_HIGH_RISING
};

void SPI_startup(int *freq, int mode);
void SPI_halt();

void SPI_transfer(const u8_t *txbuf, u8_t *rxbuf, int len);

#endif /* _H_SPI_ */

