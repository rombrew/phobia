#ifndef _H_SPI_
#define _H_SPI_

#include "libc.h"

#ifndef GPIO_SPI1_NSS
#define GPIO_SPI1_NSS			XGPIO_DEF4('A', 4, 4, 5)
#endif /* GPIO_SPI1_NSS */

#ifndef GPIO_SPI1_SCK
#define GPIO_SPI1_SCK			XGPIO_DEF4('A', 5, 5, 5)
#define GPIO_SPI1_MISO			XGPIO_DEF4('A', 6, 6, 5)
#define GPIO_SPI1_MOSI			XGPIO_DEF4('A', 7, 7, 5)
#endif /* GPIO_SPI1_SCK */

#ifndef GPIO_SPI2_NSS
#define GPIO_SPI2_NSS			XGPIO_DEF2('B', 12)
#endif /* GPIO_SPI2_NSS */

#ifndef GPIO_SPI2_SCK
#define GPIO_SPI2_SCK			XGPIO_DEF4('B', 13, 0, 5)
#define GPIO_SPI2_MISO			XGPIO_DEF4('B', 14, 0, 5)
#define GPIO_SPI2_MOSI			XGPIO_DEF4('B', 15, 0, 5)
#endif /* GPIO_SPI2_SCK */

#ifndef GPIO_SPI3_NSS
#define GPIO_SPI3_NSS			XGPIO_DEF2('C', 9)
#endif /* GPIO_SPI3_NSS */

#ifndef GPIO_SPI3_SCK
#define GPIO_SPI3_SCK			XGPIO_DEF4('C', 10, 0, 6)
#define GPIO_SPI3_MISO			XGPIO_DEF4('C', 11, 0, 6)
#define GPIO_SPI3_MOSI			XGPIO_DEF4('C', 12, 0, 6)
#endif /* GPIO_SPI3_SCK */

enum {
	SPI_LOW_RISING		= 0,
	SPI_HIGH_FALLING,
	SPI_LOW_FALLING,
	SPI_HIGH_RISING,

	SPI_SIZE_16		= 4,
};

void SPI_startup(int bus_ID, int freq_hz, int mode);
void SPI_halt(int bus_ID);

uint16_t SPI_transfer(int bus_ID, uint16_t txbuf, int nss_hold);

#endif /* _H_SPI_ */

