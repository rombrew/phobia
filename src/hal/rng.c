#include "hal.h"
#include "cmsis/stm32xx.h"

void RNG_startup()
{
	/* Enable RNG clock.
	 * */
	RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;

	/* Enable RNG.
	 * */
	RNG->CR |= RNG_CR_RNGEN;
}

uint32_t RNG_urand()
{
	uint32_t 		urand = 0U;
	int			N = 0;

	do {
		/* Check that no error occured.
		 * */
		if (RNG->SR & (RNG_SR_SEIS | RNG_SR_CEIS)) {

			RNG->SR &= ~(RNG_SR_SEIS | RNG_SR_CEIS);

			RNG->CR &= ~(RNG_CR_RNGEN);
			RNG->CR |= RNG_CR_RNGEN;
		}

		/* Wait till RNG is ready.
		 * */
		if (RNG->SR & RNG_SR_DRDY) {

			urand = RNG->DR;
			break;
		}

		__NOP();

		N++;
	}
	while (N < 700000);

	return urand;
}

uint32_t RNG_make_UID()
{
	uint32_t		UID;

#if defined(STM32F4)
	UID = crc32u((const void *) 0x1FFF7A10U, 12);
#elif defined(STM32F7)
	UID = crc32u((const void *) 0x1FF07A10U, 12);
#endif /* STM32Fx */

	return UID;
}

