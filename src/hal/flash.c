#include "hal.h"
#include "libc.h"

#include "cmsis/stm32xx.h"

#if defined(STM32F4)

const FLASH_config_t	FLASH_config = {

	.begin = 8,
	.total = 4,

	.flash = 0x08000000U,

	.map = {

		0x08080000U,
		0x080A0000U,
		0x080C0000U,
		0x080E0000U,
		0x08100000U,
		0x08100000U
	}
};

#elif defined(STM32F7)

const FLASH_config_t	FLASH_config = {

	.begin = 6,
	.total = 2,

	.flash = 0x08000000U,

	.map = {

		0x08040000U,
		0x08060000U,
		0x08080000U,
		0x08080000U,
		0x08080000U,
		0x08080000U
	}
};

#endif /* STM32Fx */

static void
FLASH_unlock()
{
	if (FLASH->CR & FLASH_CR_LOCK) {

		FLASH->KEYR = 0x45670123U;
		FLASH->KEYR = 0xCDEF89ABU;
	}
}

static void
FLASH_lock()
{
	FLASH->CR |= FLASH_CR_LOCK;
}

static void
FLASH_wait_BSY()
{
	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

		__NOP();
	}
}

LD_RAMFUNC static void
FLASH_erase_on_IWDG(int N)
{
	/* Disable all IRQs to be sure that no code execution from flash will
	 * occur while erase operation is in progress.
	 * */
	__disable_irq();

	__DSB();
	__ISB();

	FLASH->CR = FLASH_CR_PSIZE_1 | (N << FLASH_CR_SNB_Pos)
		| FLASH_CR_SER | FLASH_CR_STRT;

	__DSB();

	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

		/* Kick IWDG during a long wait.
		 * */
		IWDG->KR = 0xAAAA;

		__NOP();
	}

	FLASH->CR = 0;

	__DSB();

	__enable_irq();
}

void *FLASH_erase(uint32_t *flash)
{
	int		N, native_N = 0;

	for (N = 0; N < FLASH_config.total; ++N) {

		if (		(uint32_t) flash >= FLASH_config.map[N]
				&& (uint32_t) flash < FLASH_config.map[N + 1]) {

			flash = (void *) FLASH_config.map[N];
			native_N = N + FLASH_config.begin;

			break;
		}
	}

	if (native_N != 0) {

		FLASH_unlock();
		FLASH_wait_BSY();

		/* Call the func from RAM because flash will busy.
		 * */
		FLASH_erase_on_IWDG(native_N);

		FLASH_lock();

#if defined(STM32F4)
		/* Reset D-Cache.
		 * */
		FLASH->ACR &= ~FLASH_ACR_DCEN;
		FLASH->ACR |= FLASH_ACR_DCRST;
		FLASH->ACR &= ~FLASH_ACR_DCRST;
		FLASH->ACR |= FLASH_ACR_DCEN;

#elif defined(STM32F7)
		/* Invalidate D-Cache.
		 * */
		SCB_InvalidateDCache_by_Addr((volatile void *) FLASH_config.map[N],
				FLASH_config.map[N + 1] - FLASH_config.map[N]);
#endif /* STM32Fx */
	}

	return flash;
}

void FLASH_prog_u32(uint32_t *flash, uint32_t val)
{
#ifdef STM32F7
	if (		(uint32_t) flash >= 0x00200000U
			&& (uint32_t) flash < 0x00280000U) {

		flash = (uint32_t *) ((uint32_t) flash + 0x07E00000U);
	}
#endif /* STM32F7 */

	if (		(uint32_t) flash >= FLASH_config.flash
			&& (uint32_t) flash < FLASH_config.map[FLASH_config.total]) {

		FLASH_unlock();
		FLASH_wait_BSY();

		FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;

		/* Program flash memory.
		 * */
		*flash = val;

		__DSB();

#ifdef STM32F7
		/* D-Cache Clean and Invalidate.
		 * */
		SCB->DCCIMVAC = (uint32_t) flash;

		__DSB();
		__ISB();

#endif /* STM32F7 */

		FLASH_wait_BSY();

		FLASH->CR = 0;

		FLASH_lock();
	}
}

