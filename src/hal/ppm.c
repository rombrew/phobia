#include "hal.h"
#include "cmsis/stm32xx.h"

typedef struct {

	float			clock_PSC;
}
priv_PPM_t;

static priv_PPM_t		priv_PPM;

void irq_TIM4()
{
	int		SR;

	SR = TIM4->SR;

	if (likely(SR & TIM_SR_CC2IF)) {

		TIM4->SR = ~TIM_SR_CC2IF;
		TIM4->DIER = TIM_DIER_CC4IE;
	}
	else if (SR & TIM_SR_CC4IF) {

		TIM4->SR = ~TIM_SR_CC4IF;
		TIM4->DIER = TIM_DIER_CC2IE;
	}
}

static void
PPM_mode_PULSE_WIDTH()
{
	/* Enable TIM4 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	hal.PPM_frequency = (hal.PPM_frequency < 1000000U) ? 1000000U
		: (hal.PPM_frequency > CLOCK_TIM4_HZ) ? CLOCK_TIM4_HZ : hal.PPM_frequency;

	TIM4->CR1 = 0;
	TIM4->CR2 = 0;
	TIM4->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_TS_0 | TIM_SMCR_SMS_2;
	TIM4->DIER = TIM_DIER_CC2IE;
	TIM4->CCMR1 = TIM_CCMR1_CC2S_1 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_CC1S_0;
	TIM4->CCMR2 = TIM_CCMR2_OC4M_0;
	TIM4->CCER = TIM_CCER_CC2P | TIM_CCER_CC2E | TIM_CCER_CC1E;
	TIM4->CNT = 0;
	TIM4->PSC = (CLOCK_TIM4_HZ / hal.PPM_frequency) - 1U;
	TIM4->ARR = 65535;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 65535;

	priv_PPM.clock_PSC = (float) (TIM4->PSC + 1U) / (float) CLOCK_TIM4_HZ;
	hal.PPM_frequency = CLOCK_TIM4_HZ / (TIM4->PSC + 1U);

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(TIM4_IRQn, 3);
	NVIC_EnableIRQ(TIM4_IRQn);

	/* Start TIM4.
	 * */
	TIM4->CR1 |= TIM_CR1_CEN;

	/* Enable TIM4 pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_TIM4_CH1);
	GPIO_set_mode_PULL_DOWN(GPIO_TIM4_CH1);
}

void PPM_startup()
{
	if (hal.PPM_mode == PPM_PULSE_WIDTH) {

		PPM_mode_PULSE_WIDTH();
	}
	else if (hal.PPM_mode == PPM_PULSE_OUTPUT) {

		/* TODO */
	}
}

static void
PPM_halt()
{
	/* Disable TIM4 pins.
	 * */
	GPIO_set_mode_INPUT(GPIO_TIM4_CH1);
	GPIO_set_mode_PULL_NONE(GPIO_TIM4_CH1);

	/* Disable TIM4.
	 * */
	TIM4->CR1 = 0;

	/* Disable IRQ.
	 * */
	NVIC_DisableIRQ(TIM4_IRQn);

	/* Disable TIM4 clock.
	 * */
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
}

void PPM_configure()
{
	if (hal.PPM_mode != PPM_DISABLED) {

		PPM_halt();
		PPM_startup();
	}
	else {
		PPM_halt();
	}
}

float PPM_get_PULSE()
{
	return (float) TIM4->CCR2 * priv_PPM.clock_PSC;
}

float PPM_get_PERIOD()
{
	return (float) TIM4->CCR1 * priv_PPM.clock_PSC;
}

