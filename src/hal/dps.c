#include "hal.h"
#include "cmsis/stm32xx.h"

static void
DPS_mode_DRIVE_HALL()
{
	/* Enable HALL pins.
	 * */
	GPIO_set_mode_INPUT(GPIO_HALL_A);
	GPIO_set_mode_INPUT(GPIO_HALL_B);
	GPIO_set_mode_INPUT(GPIO_HALL_C);
}

static void
DPS_mode_DRIVE_ABI()
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

void DPS_startup()
{
	if (hal.DPS_mode == DPS_DRIVE_ABI) {

		DPS_mode_DRIVE_HALL();
	}
	else if (hal.DPS_mode == DPS_DRIVE_ABI) {

		DPS_mode_DRIVE_ABI();
	}
}

static void
DPS_halt()
{
	/* Disable TIM3 pins.
	 * */
	GPIO_set_mode_INPUT(GPIO_TIM3_CH1);
	GPIO_set_mode_INPUT(GPIO_TIM3_CH2);
	GPIO_set_mode_INPUT(GPIO_TIM3_CH3);

	/* Disable TIM3.
	 * */
	TIM3->CR1 = 0;

	/* Disable TIM3 clock.
	 * */
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
}

void DPS_configure()
{
	if (hal.DPS_mode != DPS_DISABLED) {

		DPS_halt();
		DPS_startup();
	}
	else {
		DPS_halt();
	}
}

int DPS_get_HALL()
{
	GPIO_TypeDef	*GPIO = (GPIO_TypeDef *) (GPIOA_BASE
				+ 0x0400 * XGPIO_GET_PORT(GPIO_HALL_A));
	int		IDR, HALL;

	IDR = GPIO->IDR;

	HALL  = (IDR & (1UL << XGPIO_GET_N(GPIO_HALL_A))) ? LEG_A : 0UL;
	HALL |= (IDR & (1UL << XGPIO_GET_N(GPIO_HALL_B))) ? LEG_B : 0UL;
	HALL |= (IDR & (1UL << XGPIO_GET_N(GPIO_HALL_C))) ? LEG_C : 0UL;

	return HALL;
}

int DPS_get_EP()
{
	return TIM3->CNT;
}

