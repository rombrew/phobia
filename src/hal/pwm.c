#include "hal.h"
#include "cmsis/stm32xx.h"

void irq_TIM1_UP_TIM10() { }

static int
PWM_build()
{
	int		resolution, DTG;

	resolution = (int) ((float) (CLOCK_TIM1_HZ / 2U) / hal.PWM_frequency + 0.5f);
	DTG = (int) ((float) CLOCK_TIM1_HZ * hal.PWM_deadtime / 1000000000.f + 0.5f);
	DTG = (DTG < 127) ? DTG : 127;

	hal.PWM_frequency = (float) (CLOCK_TIM1_HZ / 2U) / (float) resolution;
	hal.PWM_resolution = resolution;
	hal.PWM_deadtime = (float) DTG * 1000000000.f / (float) CLOCK_TIM1_HZ;

	return DTG;
}

void PWM_startup()
{
	int		DTG;

	DTG = PWM_build();

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
	TIM1->CCR4 = hal.PWM_resolution - hal.ADC_sample_advance;
	TIM1->BDTR = TIM_BDTR_MOE
#ifdef HW_HAVE_PWM_STOP
		| (0U << TIM_BDTR_BKP_Pos) | TIM_BDTR_BKE
#endif /* HW_HAVE_PWM_STOP */
		| TIM_BDTR_OSSR | (DTG << TIM_BDTR_DTG_Pos);

	/* Start TIM1.
	 * */
	TIM1->EGR |= TIM_EGR_COMG | TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->RCR = 1;

#ifdef HW_HAVE_PWM_THREE_WIRE
	GPIO_set_mode_OUTPUT(GPIO_TIM1_CH1N);
	GPIO_set_mode_OUTPUT(GPIO_TIM1_CH2N);
	GPIO_set_mode_OUTPUT(GPIO_TIM1_CH3N);

	/* Only three wire PWM.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH1);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH2);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH3);
#else /* HW_HAVE_PWM_THREE_WIRE */

	/* Enable TIM1 pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH1N);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH2N);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH3N);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH1);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH2);
	GPIO_set_mode_FUNCTION(GPIO_TIM1_CH3);
#endif

	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH1N);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH2N);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH3N);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH1);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH2);
	GPIO_set_mode_SPEED_HIGH(GPIO_TIM1_CH3);

#ifdef HW_HAVE_PWM_STOP
	GPIO_set_mode_FUNCTION(GPIO_TIM1_BKIN);
#endif /* HW_HAVE_PWM_STOP */
}

void PWM_configure()
{
	int		DTG;

	DTG = PWM_build();

	TIM1->ARR = hal.PWM_resolution;
	TIM1->CCR4 = hal.PWM_resolution - hal.ADC_sample_advance;

	MODIFY_REG(TIM1->BDTR, 0xFFU, DTG);
}

void PWM_set_DC(int A, int B, int C)
{
#ifdef HW_HAVE_PWM_REVERSED
	TIM1->CCR1 = C;
	TIM1->CCR2 = B;
	TIM1->CCR3 = A;
#else /* HW_HAVE_PWM_REVERSED */
	TIM1->CCR1 = A;
	TIM1->CCR2 = B;
	TIM1->CCR3 = C;
#endif

	hal.CNT_raw[2] = TIM7->CNT;
}

void PWM_set_Z(int Z)
{
#ifdef HW_HAVE_PWM_REVERSED
	if (Z & LEG_C) {
#else /* HW_HAVE_PWM_REVERSED */
	if (Z & LEG_A) {
#endif

#ifdef HW_HAVE_PWM_THREE_WIRE
		GPIO_set_LOW(GPIO_TIM1_CH1N);
#endif /* HW_HAVE_PWM_THREE_WIRE */

#ifdef HW_HAVE_PWM_POLARITY
		TIM1->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC1NE
				| TIM_CCER_CC1P | TIM_CCER_CC1E);
#else /* HW_HAVE_PWM_POLARITY */
		TIM1->CCER &= ~(TIM_CCER_CC1NE | TIM_CCER_CC1E);
#endif
	}
	else {

#ifdef HW_HAVE_PWM_THREE_WIRE
		GPIO_set_HIGH(GPIO_TIM1_CH1N);
#endif /* HW_HAVE_PWM_THREE_WIRE */

#ifdef HW_HAVE_PWM_POLARITY
		TIM1->CCER |= (TIM_CCER_CC1NP | TIM_CCER_CC1NE
				| TIM_CCER_CC1P | TIM_CCER_CC1E);
#else /* HW_HAVE_PWM_POLARITY */
		TIM1->CCER |= (TIM_CCER_CC1NE | TIM_CCER_CC1E);
#endif
	}

	if (Z & LEG_B) {

#ifdef HW_HAVE_PWM_THREE_WIRE
		GPIO_set_LOW(GPIO_TIM1_CH2N);
#endif /* HW_HAVE_PWM_THREE_WIRE */

#ifdef HW_HAVE_PWM_POLARITY
		TIM1->CCER &= ~(TIM_CCER_CC2NP | TIM_CCER_CC2NE
				| TIM_CCER_CC2P | TIM_CCER_CC2E);
#else /* HW_HAVE_PWM_POLARITY */
		TIM1->CCER &= ~(TIM_CCER_CC2NE | TIM_CCER_CC2E);
#endif
	}
	else {

#ifdef HW_HAVE_PWM_THREE_WIRE
		GPIO_set_HIGH(GPIO_TIM1_CH2N);
#endif /* HW_HAVE_PWM_THREE_WIRE */

#ifdef HW_HAVE_PWM_POLARITY
		TIM1->CCER |= (TIM_CCER_CC2NP | TIM_CCER_CC2NE
				| TIM_CCER_CC2P | TIM_CCER_CC2E);
#else /* HW_HAVE_PWM_POLARITY */
		TIM1->CCER |= (TIM_CCER_CC2NE | TIM_CCER_CC2E);
#endif
	}

#ifdef HW_HAVE_PWM_REVERSED
	if (Z & LEG_A) {
#else /* HW_HAVE_PWM_REVERSED */
	if (Z & LEG_C) {
#endif

#ifdef HW_HAVE_PWM_THREE_WIRE
		GPIO_set_LOW(GPIO_TIM1_CH3N);
#endif /* HW_HAVE_PWM_THREE_WIRE */

#ifdef HW_HAVE_PWM_POLARITY
		TIM1->CCER &= ~(TIM_CCER_CC3NP | TIM_CCER_CC3NE
				| TIM_CCER_CC3P | TIM_CCER_CC3E);
#else /* HW_HAVE_PWM_POLARITY */
		TIM1->CCER &= ~(TIM_CCER_CC3NE | TIM_CCER_CC3E);
#endif
	}
	else {

#ifdef HW_HAVE_PWM_THREE_WIRE
		GPIO_set_HIGH(GPIO_TIM1_CH3N);
#endif /* HW_HAVE_PWM_THREE_WIRE */

#ifdef HW_HAVE_PWM_POLARITY
		TIM1->CCER |= (TIM_CCER_CC3NP | TIM_CCER_CC3NE
				| TIM_CCER_CC3P | TIM_CCER_CC3E);
#else /* HW_HAVE_PWM_POLARITY */
		TIM1->CCER |= (TIM_CCER_CC3NE | TIM_CCER_CC3E);
#endif
	}

	TIM1->EGR |= TIM_EGR_COMG;
}

int PWM_fault()
{
#ifdef HW_HAVE_PWM_STOP
	if (TIM1->SR & TIM_SR_BIF) {

		TIM1->SR &= ~TIM_SR_BIF;

		return HAL_FAULT;
	}
#endif /* HW_HAVE_PWM_STOP */

	return HAL_OK;
}

