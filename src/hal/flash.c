#include "hal.h"
#include "libc.h"

#include "cmsis/stm32xx.h"

#if defined(STM32F4)

const FLASH_config_t	FLASH_config = {

	.begin = 8,
	.total = 4,

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

	FLASH->CR = FLASH_CR_PSIZE_1 | (N << 3)
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

void *FLASH_erase(void *flash)
{
	int		N, raw_N = 0;

	for (N = 0; N < FLASH_config.total; ++N) {

		if (		(uint32_t) flash >= FLASH_config.map[N]
				&& (uint32_t) flash < FLASH_config.map[N + 1]) {

			flash = (void *) FLASH_config.map[N];
			raw_N = N + FLASH_config.begin;

			break;
		}
	}

	if (raw_N != 0) {

		FLASH_unlock();
		FLASH_wait_BSY();

		/* Call the func from RAM because flash will busy.
		 * */
		FLASH_erase_on_IWDG(raw_N);

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

void FLASH_prog(void *flash, uint32_t value)
{
	uint32_t			*ld_flash = (uint32_t *) flash;

	if (		(uint32_t) ld_flash >= fw.ld_end
			&& (uint32_t) ld_flash < FLASH_config.map[FLASH_config.total]) {

		FLASH_unlock();
		FLASH_wait_BSY();

		FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;

		/* Program flash memory.
		 * */
		*ld_flash = value;

		__DSB();

#ifdef STM32F7
		/* D-Cache Clean and Invalidate.
		 * */
		SCB->DCCIMVAC = (uint32_t) ld_flash;

		__DSB();
		__ISB();

#endif /* STM32F7 */

		FLASH_wait_BSY();

		FLASH->CR = 0;

		FLASH_lock();
	}
}

