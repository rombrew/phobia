#include "hal.h"
#include "cmsis/stm32xx.h"

void SPI_startup(int bus_ID, int freq_hz, int mode)
{
	SPI_TypeDef		*SPI;
	int			clock_HZ, BR, HZ, DFRAME, CPOLHA;

	/* Enable SPI clock.
	 * */
	switch (bus_ID) {

#ifdef HW_HAVE_SPI_ON_PCB
		case SPI_ID_ON_PCB:

			SPI = SPI3;

			RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
			clock_HZ = CLOCK_APB1_HZ;
			break;
#endif /* HW_HAVE_SPI_ON_PCB */

		default:
		case SPI_ID_EXT:

			SPI = SPI1;

			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
			clock_HZ = CLOCK_APB2_HZ;
			break;
	}

	for (BR = 0; BR < 8; ++BR) {

		HZ = clock_HZ / (1U << (BR + 1));

		if (HZ <= freq_hz)
			break;
	}

	switch (mode & 3U) {

		case SPI_MODE_HIGH_FALLING:
			CPOLHA = SPI_CR1_CPOL;
			break;

		case SPI_MODE_LOW_FALLING:
			CPOLHA = SPI_CR1_CPHA;
			break;

		case SPI_MODE_HIGH_RISING:
			CPOLHA = SPI_CR1_CPOL | SPI_CR1_CPHA;
			break;

		default:
		case SPI_MODE_LOW_RISING:
			CPOLHA = 0;
			break;
	}

#if defined(STM32F4)

	DFRAME = (mode & SPI_MODE_SIZE_16) ? SPI_CR1_DFF : 0;

	/* Configure SPI.
	 * */
	SPI->CR1 = DFRAME | SPI_CR1_SSM | SPI_CR1_SSI
		| (BR << SPI_CR1_BR_Pos) | SPI_CR1_MSTR | CPOLHA;
	SPI->CR2 = 0;

#elif defined(STM32F7)

	DFRAME = (mode & SPI_MODE_SIZE_16) ? (15U << SPI_CR2_DS_Pos)
			: (7U << SPI_CR2_DS_Pos);

	/* Configure SPI.
	 * */
	SPI->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | (BR << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR | CPOLHA;
	SPI->CR2 = DFRAME;

#endif /* STM32Fx */

	/* Enable SPI pins.
	 * */
	switch (bus_ID) {

#ifdef HW_HAVE_SPI_ON_PCB
		case SPI_ID_ON_PCB:

			GPIO_set_mode_OUTPUT(GPIO_SPI_ON_PCB_NSS);
			GPIO_set_HIGH(GPIO_SPI_ON_PCB_NSS);

			GPIO_set_mode_FUNCTION(GPIO_SPI_ON_PCB_SCK);
			GPIO_set_mode_FUNCTION(GPIO_SPI_ON_PCB_MISO);
			GPIO_set_mode_FUNCTION(GPIO_SPI_ON_PCB_MOSI);

			GPIO_set_mode_SPEED_FAST(GPIO_SPI_ON_PCB_NSS);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI_ON_PCB_SCK);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI_ON_PCB_MISO);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI_ON_PCB_MOSI);
			break;
#endif /* HW_HAVE_SPI_ON_PCB */

		default:
		case SPI_ID_EXT:

			GPIO_set_mode_OUTPUT(GPIO_SPI_EXT_NSS);
			GPIO_set_HIGH(GPIO_SPI_EXT_NSS);

			GPIO_set_mode_FUNCTION(GPIO_SPI_EXT_SCK);
			GPIO_set_mode_FUNCTION(GPIO_SPI_EXT_MISO);
			GPIO_set_mode_FUNCTION(GPIO_SPI_EXT_MOSI);

			GPIO_set_mode_SPEED_FAST(GPIO_SPI_EXT_NSS);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI_EXT_SCK);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI_EXT_MISO);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI_EXT_MOSI);
			break;
	}

	/* Enable SPI.
	 * */
	SPI->CR1 |= SPI_CR1_SPE;
}

void SPI_halt(int bus_ID)
{
	SPI_TypeDef		*SPI;

	/* Disable SPI pins.
	 * */
	switch (bus_ID) {

#ifdef HW_HAVE_SPI_ON_PCB
		case SPI_ID_ON_PCB:

			SPI = SPI3;

			GPIO_set_mode_INPUT(GPIO_SPI_ON_PCB_NSS);
			GPIO_set_mode_INPUT(GPIO_SPI_ON_PCB_SCK);
			GPIO_set_mode_INPUT(GPIO_SPI_ON_PCB_MISO);
			GPIO_set_mode_INPUT(GPIO_SPI_ON_PCB_MOSI);
			break;
#endif /* HW_HAVE_SPI_ON_PCB */

		default:
		case SPI_ID_EXT:

			SPI = SPI1;

			GPIO_set_mode_INPUT(GPIO_SPI_EXT_NSS);
			GPIO_set_mode_INPUT(GPIO_SPI_EXT_SCK);
			GPIO_set_mode_INPUT(GPIO_SPI_EXT_MISO);
			GPIO_set_mode_INPUT(GPIO_SPI_EXT_MOSI);
			break;
	}

	/* Disable SPI.
	 * */
	SPI->CR1 = 0;
	SPI->CR2 = 0;

	/* Disable SPI clock.
	 * */
	switch (bus_ID) {

#ifdef HW_HAVE_SPI_ON_PCB
		case SPI_ID_ON_PCB:

			RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
			break;
#endif /* HW_HAVE_SPI_ON_PCB */

		default:
		case SPI_ID_EXT:

			RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
			break;
	}
}

uint16_t SPI_transfer(int bus_ID, uint16_t txbuf)
{
	SPI_TypeDef		*SPI;
	uint16_t		rxbuf;
	int			gpio_NSS, hold_NSS, wait_N;

	switch (bus_ID) {

#ifdef HW_HAVE_SPI_ON_PCB
		case SPI_ID_ON_PCB:

			SPI = SPI3;
			gpio_NSS = GPIO_SPI_ON_PCB_NSS;
			hold_NSS = 100;
			break;
#endif /* HW_HAVE_SPI_ON_PCB */

		default:
		case SPI_ID_EXT:

			SPI = SPI1;
			gpio_NSS = GPIO_SPI_EXT_NSS;
			hold_NSS = 200;
			break;
	}

	wait_N = 0;

	while ((SPI->SR & SPI_SR_TXE) == 0) {

		__NOP();

		if (wait_N > 70000U) {

			/* Timeout.
			 * */
			return 0;
		}

		wait_N++;
	}

	GPIO_set_LOW(gpio_NSS);

	/* Wait after NSS hold.
	 * */
	TIM_wait_ns(hold_NSS);

	SPI->DR = txbuf;

	while ((SPI->SR & SPI_SR_RXNE) == 0) {

		__NOP();
	}

	rxbuf = SPI->DR;

	while (SPI->SR & SPI_SR_BSY) {

		__NOP();
	}

	/* Wait before NSS release.
	 * */
	TIM_wait_ns(hold_NSS);

	GPIO_set_HIGH(gpio_NSS);

	/* Ensure minimal NSS pulse.
	 * */
	TIM_wait_ns(hold_NSS);

	return rxbuf;
}

