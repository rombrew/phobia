#include "hal.h"
#include "cmsis/stm32xx.h"

#define CLOCK_TIM7_HZ			(CLOCK_APB1_HZ * 2UL)

typedef struct {

	uint32_t		tick_CNT;
}
priv_TIM_t;

static priv_TIM_t		priv_TIM;

void irq_TIM7()
{
	TIM7->SR = ~TIM_SR_UIF;

	priv_TIM.tick_CNT += 1U;
}

void TIM_startup()
{
	/* Enable TIM7 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

	TIM7->CR1 = 0;
	TIM7->CR2 = 0;
	TIM7->DIER = TIM_DIER_UIE;
	TIM7->CNT = 0;
	TIM7->PSC = 0;
	TIM7->ARR = 65535;

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(TIM7_IRQn, 15);
	NVIC_EnableIRQ(TIM7_IRQn);

	/* Start TIM3.
	 * */
	TIM7->CR1 |= TIM_CR1_CEN;
}

void TIM_wait_ns(int ns)
{
	uint16_t		xCNT;
	int			elapsed, hold;

	xCNT = TIM7->CNT;

	hold = ns * (CLOCK_TIM7_HZ / 1000000UL) / 1000UL;

	do {
		elapsed = (int) ((uint16_t) TIM7->CNT - xCNT);

		if (elapsed >= hold)
			break;

		__NOP();
		__NOP();
	}
	while (1);
}

void TIM_wait_ms(int ms)
{
	uint32_t		xCNT;
	int			elapsed, hold;

	xCNT = priv_TIM.tick_CNT;

	hold = ms * (CLOCK_TIM7_HZ / 65536UL) / 1000UL;

	do {
		__DSB();
		__WFI();

		elapsed = (int) (priv_TIM.tick_CNT - xCNT);

		if (elapsed >= hold)
			break;
	}
	while (1);
}

