#include "hal.h"
#include "cmsis/stm32xx.h"

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
	TIM7->ARR = 65535U;

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
	uint32_t		END, CNT;

	END = TIM7->CNT + (uint32_t) (ns)
		* (CLOCK_TIM7_HZ / 1000000UL) / 1000UL;

	do {
		CNT = TIM7->CNT;

		if (CNT < END)
			CNT += 65535U;

		if (CNT >= END)
			break;

		__NOP();
	}
	while (1);
}

int TIM_get_CNT()
{
	return TIM7->CNT;
}

