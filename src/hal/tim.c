#include "hal.h"
#include "cmsis/stm32xx.h"

static void
TIM_mode_DRIVE_ABI()
{
	/* Enable TIM3 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM3->CR1 = 0;
	TIM3->CR2 = 0;
	TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM3->DIER = 0;
	TIM3->CCMR1 = TIM_CCMR1_IC2F_1 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F_1
		| TIM_CCMR1_CC1S_0;
	TIM3->CCMR2 = 0;
	TIM3->CCER = 0;
	TIM3->CNT = 32767;
	TIM3->PSC = 0;
	TIM3->ARR = 65535;
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	/* Start TIM3.
	 * */
	TIM3->CR1 |= TIM_CR1_CEN;

	/* Enable TIM3 pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_TIM3_CH1);
	GPIO_set_mode_FUNCTION(GPIO_TIM3_CH2);
}

void TIM_startup()
{
	if (hal.TIM_mode == TIM_DRIVE_ABI) {

		TIM_mode_DRIVE_ABI();
	}
}

static void
TIM_halt()
{
	/* Disable TIM3 pins.
	 * */
	GPIO_set_mode_INPUT(GPIO_TIM3_CH1);
	GPIO_set_mode_INPUT(GPIO_TIM3_CH2);

	/* Disable TIM3.
	 * */
	TIM3->CR1 = 0;

	/* Disable TIM3 clock.
	 * */
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
}

void TIM_configure()
{
	if (hal.TIM_mode != TIM_DISABLED) {

		TIM_halt();
		TIM_startup();
	}
	else {
		TIM_halt();
	}
}

int TIM_get_EP()
{
	return TIM3->CNT;
}

