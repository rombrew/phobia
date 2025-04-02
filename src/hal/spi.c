#include "hal.h"
#include "cmsis/stm32xx.h"

typedef struct {

	SPI_TypeDef	*SPI;

	int		gpio_NSS;

	int		clock;
	int		hold;

	uint32_t	dmabuf[8] LD_DMA;
}
priv_SPI_t;

static priv_SPI_t		priv_SPI[3];

int SPI_is_halted(int bus)
{
	return (priv_SPI[bus].SPI == (SPI_TypeDef *) 0) ? HAL_OK : HAL_FAULT;
}

void SPI_startup(int bus, int freq, int mode)
{
	int			clock, baud, dsize, cpol, N;

	switch (bus) {

		default:
		case BUS_ID_SPI1:

			priv_SPI[bus].SPI = SPI1;
			priv_SPI[bus].gpio_NSS = GPIO_SPI1_NSS;

			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
			break;

		case BUS_ID_SPI2:

			priv_SPI[bus].SPI = SPI2;
			priv_SPI[bus].gpio_NSS = GPIO_SPI2_NSS;

			RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
			break;

		case BUS_ID_SPI3:

			priv_SPI[bus].SPI = SPI3;
			priv_SPI[bus].gpio_NSS = GPIO_SPI3_NSS;

			RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
			break;
	}

	switch (bus) {

		default:
		case BUS_ID_SPI1:
			clock = CLOCK_APB2_HZ;
			break;

		case BUS_ID_SPI2:
		case BUS_ID_SPI3:
			clock = CLOCK_APB1_HZ;
			break;
	}

	baud = 7U;

	for (N = 0; N < 8; ++N) {

		priv_SPI[bus].clock = clock / (1U << (N + 1));

		if (priv_SPI[bus].clock <= freq) {

			baud = N;
			break;
		}
	}

	priv_SPI[bus].hold = 1U * 1000000U / (priv_SPI[bus].clock / 1000U);

	switch (mode & 3U) {

		default:
		case SPI_LOW_RISING:
			cpol = 0;
			break;

		case SPI_HIGH_FALLING:
			cpol = SPI_CR1_CPOL;
			break;

		case SPI_LOW_FALLING:
			cpol = SPI_CR1_CPHA;
			break;

		case SPI_HIGH_RISING:
			cpol = SPI_CR1_CPOL | SPI_CR1_CPHA;
			break;
	}

#if defined(STM32F4)
	dsize = (mode & SPI_DATA_8) ? 0U : 1U;

	/* Configure SPI.
	 * */
	priv_SPI[bus].SPI->CR1 = (dsize << SPI_CR1_DFF_Pos) | SPI_CR1_SSM
		| SPI_CR1_SSI | (baud << SPI_CR1_BR_Pos) | SPI_CR1_MSTR | cpol;
	priv_SPI[bus].SPI->CR2 = 0;

#elif defined(STM32F7)
	dsize = (mode & SPI_DATA_8) ? 7U : 15U;

	/* Configure SPI.
	 * */
	priv_SPI[bus].SPI->CR1 = SPI_CR1_SSM | SPI_CR1_SSI
		| (baud << SPI_CR1_BR_Pos) | SPI_CR1_MSTR | cpol;
	priv_SPI[bus].SPI->CR2 = dsize << SPI_CR2_DS_Pos;
#endif /* STM32Fx */

	GPIO_set_mode_OUTPUT(priv_SPI[bus].gpio_NSS);
	GPIO_set_HIGH(priv_SPI[bus].gpio_NSS);

	GPIO_set_mode_SPEED_FAST(priv_SPI[bus].gpio_NSS);

	switch (bus) {

		default:
		case BUS_ID_SPI1:

			GPIO_set_mode_FUNCTION(GPIO_SPI1_SCK);
			GPIO_set_mode_FUNCTION(GPIO_SPI1_MISO);
			GPIO_set_mode_FUNCTION(GPIO_SPI1_MOSI);

			GPIO_set_mode_SPEED_FAST(GPIO_SPI1_SCK);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI1_MISO);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI1_MOSI);
			break;

		case BUS_ID_SPI2:

			GPIO_set_mode_FUNCTION(GPIO_SPI2_SCK);
			GPIO_set_mode_FUNCTION(GPIO_SPI2_MISO);
			GPIO_set_mode_FUNCTION(GPIO_SPI2_MOSI);

			GPIO_set_mode_SPEED_FAST(GPIO_SPI2_SCK);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI2_MISO);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI2_MOSI);
			break;

		case BUS_ID_SPI3:

			GPIO_set_mode_FUNCTION(GPIO_SPI3_SCK);
			GPIO_set_mode_FUNCTION(GPIO_SPI3_MISO);
			GPIO_set_mode_FUNCTION(GPIO_SPI3_MOSI);

			GPIO_set_mode_SPEED_FAST(GPIO_SPI3_SCK);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI3_MISO);
			GPIO_set_mode_SPEED_FAST(GPIO_SPI3_MOSI);
			break;
	}

	if (mode & SPI_DMA) {

		/* Enable TIM8 clock.
		 * */
		RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

		TIM8->CR1 = TIM_CR1_OPM;
		TIM8->CR2 = 0;
		TIM8->SMCR = 0;
		TIM8->DIER = TIM_DIER_CC3DE | TIM_DIER_CC2DE;
		TIM8->CCMR1 = 0;
		TIM8->CCMR2 = 0;
		TIM8->CCER = 0;
		TIM8->PSC = CLOCK_TIM8_HZ / 2U / priv_SPI[bus].clock - 1U;
		TIM8->ARR = 44;
		TIM8->CCR1 = 2;		/* NSS = 0 */
		TIM8->CCR2 = 5;		/* DR = txbuf */
		TIM8->CCR3 = 42;	/* rxbuf = DR */
		TIM8->CCR4 = 42;	/* NSS = 1 */

		dsize = (mode & SPI_DATA_8) ? 0U : 1U;

		/* Enable DMA on TIM8_CH3.
		 * */
		DMA2_Stream4->CR = (7U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1
			| (dsize << DMA_SxCR_MSIZE_Pos) | (dsize << DMA_SxCR_PSIZE_Pos)
			| DMA_SxCR_MINC;
		DMA2_Stream4->PAR = (uint32_t) &priv_SPI[bus].SPI->DR;
		DMA2_Stream4->FCR = DMA_SxFCR_DMDIS;

		/* Enable DMA on TIM8_CH2.
		 * */
		DMA2_Stream3->CR = (7U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1
			| (dsize << DMA_SxCR_MSIZE_Pos) | (dsize << DMA_SxCR_PSIZE_Pos)
			| DMA_SxCR_MINC | DMA_SxCR_DIR_0;
		DMA2_Stream3->PAR = (uint32_t) &priv_SPI[bus].SPI->DR;
		DMA2_Stream3->FCR = DMA_SxFCR_DMDIS;

		if (mode & SPI_NSS_ON) {

			TIM8->DIER |= TIM_DIER_CC4DE | TIM_DIER_CC1DE;

#define XGPIO_GET_BSRR(xGPIO)	(GPIOA_BASE + 0x0400U * XGPIO_GET_PORT(xGPIO) + 0x18U)

			/* Enable DMA on TIM8_CH1.
			 * */
			DMA2_Stream2->CR = (7U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1
				| (2U << DMA_SxCR_MSIZE_Pos) | (2U << DMA_SxCR_PSIZE_Pos)
				| DMA_SxCR_DIR_0 | DMA_SxCR_CIRC;
			DMA2_Stream2->NDTR = 1;
			DMA2_Stream2->PAR = (uint32_t) XGPIO_GET_BSRR(priv_SPI[bus].gpio_NSS);
			DMA2_Stream2->M0AR = (uint32_t) &priv_SPI[bus].dmabuf[0];
			DMA2_Stream2->FCR = DMA_SxFCR_DMDIS;

			/* Enable DMA on TIM8_CH4.
			 * */
			DMA2_Stream7->CR = (7U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1
				| (2U << DMA_SxCR_MSIZE_Pos) | (2U << DMA_SxCR_PSIZE_Pos)
				| DMA_SxCR_DIR_0 | DMA_SxCR_CIRC;
			DMA2_Stream7->NDTR = 1;
			DMA2_Stream7->PAR = (uint32_t) XGPIO_GET_BSRR(priv_SPI[bus].gpio_NSS);
			DMA2_Stream7->M0AR = (uint32_t) &priv_SPI[bus].dmabuf[1];
			DMA2_Stream7->FCR = DMA_SxFCR_DMDIS;

			N = XGPIO_GET_N(priv_SPI[bus].gpio_NSS);

			priv_SPI[bus].dmabuf[0] = (1U << (N + 16));
			priv_SPI[bus].dmabuf[1] = (1U << N);

			__DSB();

#ifdef STM32F7
			/* D-Cache Clean.
			 * */
			SCB->DCCMVAC = (uint32_t) &priv_SPI[bus].dmabuf[0];

			__DSB();
			__ISB();
#endif /* STM32F7 */

			/* Enable DMA2.
			 * */
			DMA2_Stream2->CR |= DMA_SxCR_EN;
			DMA2_Stream7->CR |= DMA_SxCR_EN;
		}
	}

	/* Enable SPI.
	 * */
	priv_SPI[bus].SPI->CR1 |= SPI_CR1_SPE;
}

void SPI_halt(int bus)
{
	if (priv_SPI[bus].SPI == 0)
		return ;

	GPIO_set_mode_INPUT(priv_SPI[bus].gpio_NSS);

	switch (bus) {

		default:
		case BUS_ID_SPI1:

			GPIO_set_mode_INPUT(GPIO_SPI1_SCK);
			GPIO_set_mode_INPUT(GPIO_SPI1_MISO);
			GPIO_set_mode_INPUT(GPIO_SPI1_MOSI);
			break;

		case BUS_ID_SPI2:

			GPIO_set_mode_INPUT(GPIO_SPI2_SCK);
			GPIO_set_mode_INPUT(GPIO_SPI2_MISO);
			GPIO_set_mode_INPUT(GPIO_SPI2_MOSI);
			break;

		case BUS_ID_SPI3:

			GPIO_set_mode_INPUT(GPIO_SPI3_SCK);
			GPIO_set_mode_INPUT(GPIO_SPI3_MISO);
			GPIO_set_mode_INPUT(GPIO_SPI3_MOSI);
			break;
	}

	if (priv_SPI[bus].dmabuf[0] != 0) {

		int		N = 0;

		/* Disable TIM8.
		 * */
		TIM8->CR1 = 0;
		TIM8->CR2 = 0;

		/* Disable DMA2.
		 * */
		DMA2_Stream4->CR = 0;
		DMA2_Stream3->CR = 0;
		DMA2_Stream2->CR = 0;
		DMA2_Stream7->CR = 0;

		while (		   (DMA2_Stream4->CR & DMA_SxCR_EN)
				|| (DMA2_Stream3->CR & DMA_SxCR_EN)
				|| (DMA2_Stream2->CR & DMA_SxCR_EN)
				|| (DMA2_Stream7->CR & DMA_SxCR_EN)) {

			__NOP();

			if (N > 70000U)
				break;

			N++;
		}

		/* Disable TIM8 clock.
		 * */
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM8EN;

		priv_SPI[bus].dmabuf[0] = 0U;
		priv_SPI[bus].dmabuf[1] = 0U;
	}

	/* Disable SPI.
	 * */
	priv_SPI[bus].SPI->CR1 = 0;
	priv_SPI[bus].SPI->CR2 = 0;

	switch (bus) {

		default:
		case BUS_ID_SPI1:
			RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
			break;

		case BUS_ID_SPI2:
			RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
			break;

		case BUS_ID_SPI3:
			RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
			break;
	}

	priv_SPI[bus].SPI = (SPI_TypeDef *) 0;
}

uint16_t SPI_transfer(int bus, uint16_t txbuf)
{
	int			N = 0;

	if (priv_SPI[bus].SPI == 0)
		return 0U;

	while ((priv_SPI[bus].SPI->SR & SPI_SR_TXE) != SPI_SR_TXE) {

		__NOP();

		if (N > 70000U)
			return 0U;

		N++;
	}

	GPIO_set_LOW(priv_SPI[bus].gpio_NSS);
	TIM_wait_ns(priv_SPI[bus].hold);

	priv_SPI[bus].SPI->DR = txbuf;

	while ((priv_SPI[bus].SPI->SR & SPI_SR_RXNE) != SPI_SR_RXNE) {

		__NOP();
	}

	txbuf = priv_SPI[bus].SPI->DR;

	while ((priv_SPI[bus].SPI->SR & SPI_SR_BSY) == SPI_SR_BSY) {

		__NOP();
	}

	TIM_wait_ns(priv_SPI[bus].hold);

	GPIO_set_HIGH(priv_SPI[bus].gpio_NSS);
	TIM_wait_ns(priv_SPI[bus].hold);

	return txbuf;
}

void SPI_transfer_dma(int bus, const uint16_t *txbuf, uint16_t *rxbuf, int len)
{
	DMA2_Stream4->CR &= ~DMA_SxCR_EN;
	DMA2_Stream3->CR &= ~DMA_SxCR_EN;

	__DSB();

#ifdef STM32F7
	/* Clean D-Cache on TXBUF.
	 * */
	SCB->DCCMVAC = (uint32_t) txbuf;

	/* Invalidate D-Cache on RXBUF.
	 * */
	SCB->DCIMVAC = (uint32_t) rxbuf;

	__DSB();
	__ISB();
#endif /* STM32F7 */

	DMA2_Stream4->NDTR = len;
	DMA2_Stream3->NDTR = len;

	DMA2_Stream4->M0AR = (uint32_t) rxbuf;
	DMA2_Stream3->M0AR = (uint32_t) txbuf;

	DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3
		| DMA_LIFCR_CTEIF3 | DMA_LIFCR_CFEIF3;
	DMA2->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4
		| DMA_HIFCR_CTEIF4 | DMA_HIFCR_CFEIF4;

	/* Enable DMA2.
	 * */
	DMA2_Stream4->CR |= DMA_SxCR_EN;
	DMA2_Stream3->CR |= DMA_SxCR_EN;

	TIM8->CNT = 0;
	TIM8->RCR = len - 1U;

	/* Start TIM8.
	 * */
	TIM8->CR1 |= TIM_CR1_CEN;
}

