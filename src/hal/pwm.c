#include "hal.h"
#include "cmsis/stm32xx.h"

#define CLOCK_TIM1_HZ			(CLOCK_APB2_HZ * 2UL)
#define TIM_ADC_ADVANCE			80

void irq_TIM1_UP_TIM10() { }

static int
PWM_build()
{
	int		resolution, dtick;

	resolution = (int) ((float) CLOCK_TIM1_HZ / 2.f / hal.PWM_frequency + .5f);
	dtick = (int) ((float) CLOCK_TIM1_HZ * (float) hal.PWM_deadtime / 1000000000.f + .5f);
	dtick = (dtick < 127) ? dtick : 127;

	hal.PWM_frequency = (float) CLOCK_TIM1_HZ / 2.f / (float) resolution;
	hal.PWM_resolution = resolution;
	hal.PWM_deadtime = (float) dtick * 1000000000.f / (float) CLOCK_TIM1_HZ;

	return dtick;
}

void PWM_startup()
{
	int		dtick;

	dtick = PWM_build();

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
	TIM1->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0
		| TIM_CCMR2_OC4PE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
	TIM1->CCER = TIM_CCER_CC4E;
	TIM1->CNT = 0;
	TIM1->PSC = 0;
	TIM1->ARR = hal.PWM_resolution;
	TIM1->RCR = 0;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = hal.PWM_resolution - TIM_ADC_ADVANCE;
	TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_OSSR | dtick;

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

void PWM_configure()
{
	int		dtick;

	dtick = PWM_build();

	TIM1->ARR = hal.PWM_resolution;
	TIM1->CCR4 = hal.PWM_resolution - TIM_ADC_ADVANCE;

	MODIFY_REG(TIM1->BDTR, 0xFFUL, dtick);
}

void PWM_set_DC(int A, int B, int C)
{
#ifdef HW_HAVE_REVERSED_PWM
	TIM1->CCR1 = C;
	TIM1->CCR2 = B;
	TIM1->CCR3 = A;
#else /* HW_HAVE_REVERSED_PWM */
	TIM1->CCR1 = A;
	TIM1->CCR2 = B;
	TIM1->CCR3 = C;
#endif
}

void PWM_set_Z(int Z)
{
#ifdef HW_HAVE_REVERSED_PWM
	if (Z & LEG_C) {
#else /* HW_HAVE_REVERSED_PWM */
	if (Z & LEG_A) {
#endif

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

#ifdef HW_HAVE_REVERSED_PWM
	if (Z & LEG_A) {
#else /* HW_HAVE_REVERSED_PWM */
	if (Z & LEG_C) {
#endif
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

