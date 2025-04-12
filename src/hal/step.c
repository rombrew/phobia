#include "hal.h"
#include "cmsis/stm32xx.h"

#define STEP_DMA_MAX		64

typedef struct {

	uint16_t	dmabuf[STEP_DMA_MAX] LD_DMA;

	int		dma_rp;

	int		wire_STEP;
	int		wire_DIR;

	uint16_t	NCNT;
}
priv_STEP_t;

static priv_STEP_t		priv_STEP;

void STEP_mode_DMA()
{
	/* Enable TIM8 clock.
	 * */
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

	hal.STEP_frequency = (hal.STEP_frequency < 50000U) ? 50000U
		: (hal.STEP_frequency > 2000000U) ? 2000000U : hal.STEP_frequency;

	TIM8->CR1 = 0;
	TIM8->CR2 = 0;
	TIM8->SMCR = 0;
	TIM8->DIER = TIM_DIER_UDE;
	TIM8->CCMR1 = 0;
	TIM8->CCMR2 = 0;
	TIM8->CCER = 0;
	TIM8->CNT = 0;
	TIM8->PSC = 0;
	TIM8->ARR = CLOCK_TIM8_HZ / hal.STEP_frequency;
	TIM8->CCR1 = 0;
	TIM8->CCR2 = 0;
	TIM8->CCR3 = 0;
	TIM8->CCR4 = 0;

	hal.STEP_frequency = CLOCK_TIM8_HZ / TIM8->ARR;

#define XGPIO_GET_IDR(xGPIO)	(GPIOA_BASE + 0x0400U * XGPIO_GET_PORT(xGPIO) + 0x10U)

	/* DMA on TIM8_UP.
	 * */
	DMA2_Stream1->CR = (7U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1
		| (1U << DMA_SxCR_MSIZE_Pos) | (1U << DMA_SxCR_PSIZE_Pos)
		| DMA_SxCR_MINC | DMA_SxCR_CIRC;
	DMA2_Stream1->NDTR = STEP_DMA_MAX;
	DMA2_Stream1->PAR = (uint32_t) XGPIO_GET_IDR(GPIO_STEP);
	DMA2_Stream1->M0AR = (uint32_t) priv_STEP.dmabuf;
	DMA2_Stream1->FCR = DMA_SxFCR_DMDIS;

	DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1
		| DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

	priv_STEP.dma_rp = 0;

	priv_STEP.wire_STEP = GPIO_get_STATE(GPIO_STEP);
	priv_STEP.wire_DIR = GPIO_get_STATE(GPIO_DIR);

	priv_STEP.NCNT = 32767;

	/* Start TIM8.
	 * */
	TIM8->CR1 |= TIM_CR1_CEN;

	/* Start DMA2.
	 * */
	DMA2_Stream1->CR |= DMA_SxCR_EN;

	/* Enable STEP/DIR pins.
	 * */
	GPIO_set_mode_INPUT(GPIO_STEP);
	GPIO_set_mode_INPUT(GPIO_DIR);

	GPIO_set_mode_PULL_DOWN(GPIO_STEP);
	GPIO_set_mode_PULL_DOWN(GPIO_DIR);
}

void STEP_startup()
{
	if (		hal.STEP_mode == STEP_ON_STEP_DIR
			|| hal.STEP_mode == STEP_ON_CW_CCW) {

		STEP_mode_DMA();
	}
}

static void
STEP_halt()
{
	int		N = 0;

	/* Disable STEP/DIR pins.
	 * */
	GPIO_set_mode_PULL_NONE(GPIO_STEP);
	GPIO_set_mode_PULL_NONE(GPIO_DIR);

	/* Disable TIM8.
	 * */
	TIM8->CR1 = 0;

	/* Disable DMA2.
	 * */
	DMA2_Stream1->CR = 0;

	while (DMA2_Stream1->CR & DMA_SxCR_EN) {

		__NOP();

		if (N > 70000)
			break;

		N++;
	}

	/* Disable TIM8 clock.
	 * */
	RCC->APB2ENR &= ~RCC_APB2ENR_TIM8EN;
}

void STEP_configure()
{
	if (hal.STEP_mode != STEP_DISABLED) {

		STEP_halt();
		STEP_startup();
	}
	else {
		STEP_halt();
	}
}

int STEP_get_POSITION()
{
	int		wp = (STEP_DMA_MAX - DMA2_Stream1->NDTR) & (STEP_DMA_MAX - 1);
	int		rp = priv_STEP.dma_rp;

#ifdef STM32F7
	/* Invalidate D-Cache.
	 * */
	SCB_InvalidateDCache_by_Addr((volatile void *) priv_STEP.dmabuf, sizeof(priv_STEP.dmabuf));
#endif /* STM32F7 */

#define XGPIO_GET_STATE(xGPIO, IDR)	((IDR & (1U << XGPIO_GET_N(xGPIO))) ? 1 : 0)

	if (hal.STEP_mode == STEP_ON_STEP_DIR) {

		while (rp != wp) {

			uint16_t	IDR = priv_STEP.dmabuf[rp];
			int		wire_STEP, wire_DIR;

			wire_STEP = XGPIO_GET_STATE(GPIO_STEP, IDR);
			wire_DIR = XGPIO_GET_STATE(GPIO_DIR, IDR);

			if (unlikely(		priv_STEP.wire_STEP == 0
						&& wire_STEP != 0)) {

				if (wire_DIR != 0) {

					priv_STEP.NCNT++;
				}
				else {
					priv_STEP.NCNT--;
				}
			}

			priv_STEP.wire_STEP = wire_STEP;
			priv_STEP.wire_DIR = wire_DIR;

			rp = (rp + 1) & (STEP_DMA_MAX - 1);
		}
	}
	else if (hal.STEP_mode == STEP_ON_CW_CCW) {

		while (rp != wp) {

			uint16_t	IDR = priv_STEP.dmabuf[rp];
			int		wire_CW, wire_CCW;

			wire_CW = XGPIO_GET_STATE(GPIO_STEP, IDR);
			wire_CCW = XGPIO_GET_STATE(GPIO_DIR, IDR);

			if (unlikely(		priv_STEP.wire_STEP == 0
						&& wire_CW != 0)) {

				priv_STEP.NCNT++;
			}

			if (unlikely(		priv_STEP.wire_DIR == 0
						&& wire_CCW != 0)) {

				priv_STEP.NCNT--;
			}

			priv_STEP.wire_STEP = wire_CW;
			priv_STEP.wire_DIR = wire_CCW;

			rp = (rp + 1) & (STEP_DMA_MAX - 1);
		}
	}

	priv_STEP.dma_rp = rp;

	return priv_STEP.NCNT;
}

