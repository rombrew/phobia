#include "hal.h"
#include "libc.h"

#include "cmsis/stm32xx.h"

void SPI_startup(int *freq, int mode)
{
	int		BR, HZ, CPOLHA;

	/* Enable SPI1 clock.
	 * */
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	for (BR = 0; BR < 8; ++BR) {

		HZ = CLOCK_APB2_HZ / (1UL << (BR + 1));

		if (HZ <= *freq)
			break;
	}

	CPOLHA =  (mode == SPI_MODE_HIGH_FALLING) ? SPI_CR1_CPOL
		: (mode == SPI_MODE_LOW_FALLING)  ? SPI_CR1_CPHA
		: (mode == SPI_MODE_HIGH_RISING)  ? SPI_CR1_CPOL | SPI_CR1_CPHA : 0;

	/* Configure SPI1.
	 * */
	SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE
			| (BR << 3) | SPI_CR1_MSTR | CPOLHA;
	SPI1->CR2 = 0;

	/* Enable SPI pins.
	 * */
	GPIO_set_mode_OUTPUT(GPIO_SPI_NSS);
	GPIO_set_HIGH(GPIO_SPI_NSS);

	GPIO_set_mode_FUNCTION(GPIO_SPI_SCK);
	GPIO_set_mode_FUNCTION(GPIO_SPI_MISO);
	GPIO_set_mode_FUNCTION(GPIO_SPI_MOSI);

	GPIO_set_mode_SPEED_FAST(GPIO_SPI_NSS);
	GPIO_set_mode_SPEED_FAST(GPIO_SPI_SCK);
	GPIO_set_mode_SPEED_FAST(GPIO_SPI_MISO);
	GPIO_set_mode_SPEED_FAST(GPIO_SPI_MOSI);

	/* Output the actual frequency.
	 * */
	*freq = HZ;
}

void SPI_halt()
{
	/* Disable SPI pins.
	 * */
	GPIO_set_mode_INPUT(GPIO_SPI_NSS);
	GPIO_set_mode_INPUT(GPIO_SPI_SCK);
	GPIO_set_mode_INPUT(GPIO_SPI_MISO);
	GPIO_set_mode_INPUT(GPIO_SPI_MOSI);

	/* Disable SPI1.
	 * */
	SPI1->CR1 = 0;
	SPI1->CR2 = 0;

	/* Disable SPI1 clock.
	 * */
	RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
}

void SPI_transfer(const u8_t *txbuf, u8_t *rxbuf, int len)
{
	int		txlen, rxlen;

	GPIO_set_LOW(GPIO_SPI_NSS);

	hal_futile_ns(20);

	txlen = 0;
	rxlen = 0;

	while (rxlen < len) {

		if (SPI1->SR & SPI_SR_TXE) {

			if (txlen < len) {

				SPI1->DR = (u8_t) *txbuf++;
				txlen++;
			}
		}

		if (SPI1->SR & SPI_SR_RXNE) {

			if (rxlen < len) {

				*rxbuf++ = (u8_t) SPI1->DR;
				rxlen++;
			}
		}

		__DSB();
		__NOP();
	}

	hal_futile_ns(20);

	GPIO_set_HIGH(GPIO_SPI_NSS);
}

