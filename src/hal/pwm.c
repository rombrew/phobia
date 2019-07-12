#include "cmsis/stm32f4xx.h"
#include "hal.h"

#define GPIO_TIM1_CH1N			XGPIO_DEF4('B', 13, 0, 1)
#define GPIO_TIM1_CH2N			XGPIO_DEF4('B', 14, 0, 1)
#define GPIO_TIM1_CH3N			XGPIO_DEF4('B', 15, 0, 1)
#define GPIO_TIM1_CH1			XGPIO_DEF4('A', 8, 0, 1)
#define GPIO_TIM1_CH2			XGPIO_DEF4('A', 9, 0, 1)
#define GPIO_TIM1_CH3			XGPIO_DEF4('A', 10, 0, 1)

#define CLOCK_TIM1_HZ			(CLOCK_APB2_HZ * 2UL)
#define TIM_ADC_ADVANCE			60

void irq_TIM1_UP_TIM10() { }

static int
PWM_calculate_R()
{
	int		R;

	R = (int) ((float) CLOCK_TIM1_HZ / 2.f / hal.PWM_frequency + .5f);
	hal.PWM_frequency = (float) CLOCK_TIM1_HZ / 2.f / (float) R;
	hal.PWM_resolution = R;

	return R;
}

static int
PWM_calculate_D()
{
	int		D;

	D = (int) ((float) CLOCK_TIM1_HZ * (float) hal.PWM_deadtime / 1000000000.f + .5f);
	D = (D < 127) ? D : 127;
	hal.PWM_deadtime = (float) D * 1000000000.f / (float) CLOCK_TIM1_HZ;

	return D;
}

void PWM_startup()
{
	int		R, D;

	R = PWM_calculate_R();
	D = PWM_calculate_D();

	/* Enable TIM1 clock.
	 * */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	/* Configure TIM1.
	 * */
	TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CMS_1;
	TIM1->CR2 = TIM_CR2_MMS_1 | TIM_CR2_CCPC;
	TIM1->SMCR = 0;
	TIM1->DIER = 0;
	TIM1->CCMR1 = TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE
		| TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
	TIM1->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4PE
		| TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
	TIM1->CCER = TIM_CCER_CC4E;
	TIM1->CNT = 0;
	TIM1->PSC = 0;
	TIM1->ARR = R;
	TIM1->RCR = 0;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = R - TIM_ADC_ADVANCE;
	TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_OSSR | D;

	/* Start TIM1.
	 * */
	TIM1->EGR |= TIM_EGR_COMG | TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->RCR = 1;

	/* Enable TIM1 pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH1N);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH2N);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH3N);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH1);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH2);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH3);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH1N);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH2N);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH3N);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH1);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH2);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH3);
}

void PWM_set_configuration()
{
	int		R, D;

	R = PWM_calculate_R();
	D = PWM_calculate_D();

	TIM1->ARR = R;
	TIM1->CCR4 = R - TIM_ADC_ADVANCE;

	MODIFY_REG(TIM1->BDTR, 0xFF, D);
}

void PWM_set_DC(int A, int B, int C)
{
	TIM1->CCR1 = A;
	TIM1->CCR2 = B;
	TIM1->CCR3 = C;
}

void PWM_set_Z(int Z)
{
	if (Z & LEG_A) {

		TIM1->CCER &= ~(TIM_CCER_CC1NE | TIM_CCER_CC1E);
	}
	else {
		TIM1->CCER |= (TIM_CCER_CC1NE | TIM_CCER_CC1E);
	}

	if (Z & LEG_B) {

		TIM1->CCER &= ~(TIM_CCER_CC2NE | TIM_CCER_CC2E);
	}
	else {
		TIM1->CCER |= (TIM_CCER_CC2NE | TIM_CCER_CC2E);
	}

	if (Z & LEG_C) {

		TIM1->CCER &= ~(TIM_CCER_CC3NE | TIM_CCER_CC3E);
	}
	else {
		TIM1->CCER |= (TIM_CCER_CC3NE | TIM_CCER_CC3E);
	}

	TIM1->EGR |= TIM_EGR_COMG;
}

void PWM_halt_Z()
{
	TIM1->CCER = TIM_CCER_CC4E;
	TIM1->EGR |= TIM_EGR_COMG;
}

