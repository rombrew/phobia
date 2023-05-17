#include "hal.h"
#include "cmsis/stm32xx.h"

void SPI_startup(int bus_ID, int freq, int mode)
{
	SPI_TypeDef		*SPI;

	int			clock, baud, dframe, cpolha;

	switch (bus_ID) {

		default:
		case 1:
			SPI = SPI1;

			/* Enable SPI clock.
			 * */
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
			break;

		case 2:
			SPI = SPI2;

			RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
			break;

		case 3:
			SPI = SPI3;

			RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
			break;
	}

	switch (bus_ID) {

		default:
		case 1:
			clock = CLOCK_APB2_HZ;
			break;

		case 2:
		case 3:
			clock = CLOCK_APB1_HZ;
			break;
	}

	for (baud = 0; baud < 8; ++baud) {

		if (clock / (1U << (baud + 1)) <= freq)
			break;
	}

	switch (mode & 3U) {

		default:
		case SPI_LOW_RISING:
			cpolha = 0;
			break;

		case SPI_HIGH_FALLING:
			cpolha = SPI_CR1_CPOL;
			break;

		case SPI_LOW_FALLING:
			cpolha = SPI_CR1_CPHA;
			break;

		case SPI_HIGH_RISING:
			cpolha = SPI_CR1_CPOL | SPI_CR1_CPHA;
			break;
	}

#if defined(STM32F4)

	dframe = (mode & SPI_SIZE_16) ? SPI_CR1_DFF : 0;

	/* Configure SPI.
	 * */
	SPI->CR1 = dframe | SPI_CR1_SSM | SPI_CR1_SSI
		| (baud << SPI_CR1_BR_Pos) | SPI_CR1_MSTR | cpolha;
	SPI->CR2 = 0;

#elif defined(STM32F7)

	dframe = (mode & SPI_SIZE_16) ? (15U << SPI_CR2_DS_Pos)
			: (7U << SPI_CR2_DS_Pos);

	/* Configure SPI.
	 * */
	SPI->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | (baud << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR | cpolha;
	SPI->CR2 = dframe;

#endif /* STM32Fx */

	switch (bus_ID) {

		default:
		case 1:
			/* Enable SPI pins.
			 * */
			GPIO_set_mode_OUTPUT(GPIO_SPI1_NSS);
			GPIO_set_HIGH(GPIO_SPI1_NSS);

			GPIO_set_mode_FUNCTION(GPIO_SPI1_SCK);
			GPIO_set_mode_FUNCTION(GPIO_SPI1_MISO);
			GPIO_set_mode_FUNCTION(GPIO_SPI1_MOSI);

			GPIO_set_mode_SPEED_FAST(GPIO_SPI1_NSS);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI1_SCK);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI1_MISO);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI1_MOSI);
			break;

		case 2:
			GPIO_set_mode_OUTPUT(GPIO_SPI2_NSS);
			GPIO_set_HIGH(GPIO_SPI2_NSS);

			GPIO_set_mode_FUNCTION(GPIO_SPI2_SCK);
			GPIO_set_mode_FUNCTION(GPIO_SPI2_MISO);
			GPIO_set_mode_FUNCTION(GPIO_SPI2_MOSI);

			GPIO_set_mode_SPEED_FAST(GPIO_SPI2_NSS);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI2_SCK);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI2_MISO);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI2_MOSI);
			break;

		case 3:
			GPIO_set_mode_OUTPUT(GPIO_SPI3_NSS);
			GPIO_set_HIGH(GPIO_SPI3_NSS);

			GPIO_set_mode_FUNCTION(GPIO_SPI3_SCK);
			GPIO_set_mode_FUNCTION(GPIO_SPI3_MISO);
			GPIO_set_mode_FUNCTION(GPIO_SPI3_MOSI);

			GPIO_set_mode_SPEED_FAST(GPIO_SPI3_NSS);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI3_SCK);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI3_MISO);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI3_MOSI);
			break;
	}

	/* Enable SPI.
	 * */
	SPI->CR1 |= SPI_CR1_SPE;
}

void SPI_halt(int bus_ID)
{
	SPI_TypeDef		*SPI;

	switch (bus_ID) {

		default:
		case 1:
			SPI = SPI1;

			/* Disable SPI pins.
			 * */
			GPIO_set_mode_INPUT(GPIO_SPI1_NSS);
			GPIO_set_mode_INPUT(GPIO_SPI1_SCK);
			GPIO_set_mode_INPUT(GPIO_SPI1_MISO);
			GPIO_set_mode_INPUT(GPIO_SPI1_MOSI);
			break;

		case 2:
			SPI = SPI2;

			GPIO_set_mode_INPUT(GPIO_SPI2_NSS);
			GPIO_set_mode_INPUT(GPIO_SPI2_SCK);
			GPIO_set_mode_INPUT(GPIO_SPI2_MISO);
			GPIO_set_mode_INPUT(GPIO_SPI2_MOSI);
			break;

		case 3:
			SPI = SPI3;

			GPIO_set_mode_INPUT(GPIO_SPI3_NSS);
			GPIO_set_mode_INPUT(GPIO_SPI3_SCK);
			GPIO_set_mode_INPUT(GPIO_SPI3_MISO);
			GPIO_set_mode_INPUT(GPIO_SPI3_MOSI);
			break;
	}

	/* Disable SPI.
	 * */
	SPI->CR1 = 0;
	SPI->CR2 = 0;

	switch (bus_ID) {

		default:
		case 1:
			/* Disable SPI clock.
			 * */
			RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
			break;

		case 2:
			RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
			break;

		case 3:
			RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
			break;
	}
}

uint16_t SPI_transfer(int bus_ID, uint16_t txbuf, int nss_hold)
{
	SPI_TypeDef		*SPI;

	int			gpio_NSS, N = 0;

	switch (bus_ID) {

		default:
		case 1:
			SPI = SPI1;

			gpio_NSS = GPIO_SPI1_NSS;
			break;

		case 2:
			SPI = SPI3;

			gpio_NSS = GPIO_SPI2_NSS;
			break;

		case 3:
			SPI = SPI3;

			gpio_NSS = GPIO_SPI3_NSS;
			break;
	}

	while ((SPI->SR & SPI_SR_TXE) == 0) {

		__NOP();
		__NOP();

		if (N > 70000U)
			return 0U;

		N++;
	}

	GPIO_set_LOW(gpio_NSS);
	TIM_wait_ns(nss_hold);

	SPI->DR = txbuf;

	while ((SPI->SR & SPI_SR_RXNE) == 0) {

		__NOP();
		__NOP();
	}

	txbuf = SPI->DR;

	while (SPI->SR & SPI_SR_BSY) {

		__NOP();
		__NOP();
	}

	TIM_wait_ns(nss_hold);
	GPIO_set_HIGH(gpio_NSS);

	TIM_wait_ns(nss_hold);

	return txbuf;
}

