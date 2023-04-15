#include <stddef.h>

#include "hal.h"
#include "libc.h"

#include "cmsis/stm32xx.h"

#if defined(STM32F4)

const FLASH_config_t	FLASH_config = { 8, 4 };

const uint32_t FLASH_map[] = {

	0x08080000U,
	0x080A0000U,
	0x080C0000U,
	0x080E0000U,
	0x08100000U
};

#elif defined(STM32F7)

const FLASH_config_t	FLASH_config = { 6, 2 };

const uint32_t FLASH_map[] = {

	0x08040000U,
	0x08060000U,
	0x08080000U
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
	int		N, sector_N = 0;

	for (N = 0; N < FLASH_config.s_total; ++N) {

		if (		(uint32_t) flash >= FLASH_map[N]
				&& (uint32_t) flash < FLASH_map[N + 1]) {

			flash = (void *) FLASH_map[N];
			sector_N = N + FLASH_config.s_first;

			break;
		}
	}

	if (sector_N != 0) {

		FLASH_unlock();
		FLASH_wait_BSY();

		/* Call the func from RAM because flash will busy.
		 * */
		FLASH_erase_on_IWDG(sector_N);

		FLASH_lock();

#if defined(STM32F4)

		/* Reset D-Cache.
		 * */
		FLASH->ACR &= ~FLASH_ACR_DCEN;
		FLASH->ACR |= FLASH_ACR_DCRST;
		FLASH->ACR &= ~FLASH_ACR_DCRST;
		FLASH->ACR |= FLASH_ACR_DCEN;

#elif defined(STM32F7)

		/* Invalidate D-Cache on the erased sector.
		 * */
		SCB_InvalidateDCacheByAddr((void *) FLASH_map[N],
				FLASH_map[N + 1] - FLASH_map[N]);

#endif /* STM32Fx */
	}

	return flash;
}

void FLASH_prog(void *flash, uint32_t value)
{
	uint32_t			*long_flash = flash;

	if (		(uint32_t) long_flash >= fw.ld_end
			&& (uint32_t) long_flash < FLASH_map[FLASH_config.s_total]) {

		FLASH_unlock();
		FLASH_wait_BSY();

		FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;

		/* Program flash memory.
		 * */
		*long_flash = value;

		__DSB();

#ifdef STM32F7
		/* D-Cache Clean and Invalidate.
		 * */
		SCB->DCCIMVAC = (uint32_t) long_flash;

		__DSB();
		__ISB();

#endif /* STM32F7 */

		FLASH_wait_BSY();

		FLASH->CR = 0;

		FLASH_lock();
	}
}

