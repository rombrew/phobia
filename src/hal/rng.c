#include "cmsis/stm32f4xx.h"
#include "hal.h"

void RNG_startup()
{
	/* Enable RNG clock.
	 * */
	RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;

	/* Enable RNG.
	 * */
	RNG->CR = RNG_CR_RNGEN;
}

unsigned long RNG_urand()
{
	unsigned long 		urand = 0UL;
	int			N = 0;

	do {
		/* Wait till RNG is ready.
		 * */
		if (RNG->SR & RNG_SR_DRDY) {

			urand = RNG->DR;
			break;
		}

		N++; __NOP();
	}
	while (N < 70000UL);

	return urand;
}

