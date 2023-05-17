#include "hal.h"
#include "cmsis/stm32xx.h"

void DAC_startup(int mode)
{
	/* Enable DAC clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	/* Enable DAC output buffer.
	 * */
	DAC->CR = DAC_CR_BOFF2 | DAC_CR_BOFF1;

	if (mode & DAC_OUT1) {

		/* Enable DAC pin.
		 * */
		GPIO_set_mode_ANALOG(GPIO_DAC_OUT1);

		/* Enable DAC.
		 * */
		DAC->CR |= DAC_CR_EN1;
	}

	if (mode & DAC_OUT2) {

		/* Enable DAC pin.
		 * */
		GPIO_set_mode_ANALOG(GPIO_DAC_OUT2);

		/* Enable DAC.
		 * */
		DAC->CR |= DAC_CR_EN2;
	}
}

void DAC_halt()
{
	/* Disable DAC.
	 * */
	DAC->CR = 0;

	/* Disable DAC clock.
	 * */
	RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
}

void DAC_set_OUT1(int xOUT)
{
	DAC->DHR12R1 = xOUT;
}

void DAC_set_OUT2(int xOUT)
{
	DAC->DHR12R2 = xOUT;
}

