#ifndef _H_SPI_
#define _H_SPI_

#include "libc.h"

#ifndef HW_SPI_EXT_ID
#define HW_SPI_EXT_ID			BUS_ID_SPI1
#endif /* HW_SPI_EXT_ID */

#ifndef GPIO_SPI1_NSS
#define GPIO_SPI1_NSS			XGPIO_DEF2('A', 4)
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

	SPI_DMA			= 4,
	SPI_DATA_8		= 8,
	SPI_NSS_ON		= 32
};

enum {
	BUS_ID_SPI1		= 0,
	BUS_ID_SPI2,
	BUS_ID_SPI3
};

void SPI_startup(int bus, int freq_hz, int mode);
void SPI_halt(int bus);

uint16_t SPI_transfer(int bus, uint16_t txbuf);
void SPI_transfer_dma(int bus, const uint16_t *txbuf, uint16_t *rxbuf, int len);

#endif /* _H_SPI_ */

