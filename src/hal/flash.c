#include <stddef.h>

#include "cmsis/stm32f4xx.h"
#include "hal.h"
#include "libc.h"

const unsigned long flash_ram_map[] = {

	0x08080000,
	0x080A0000,
	0x080C0000,
	0x080E0000,
	0x08100000
};

static void
FLASH_unlock()
{
	if (FLASH->CR & FLASH_CR_LOCK) {

		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
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
	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY)
		__NOP();
}

LD_RAMFUNC static void
FLASH_erase_on_IWDG(int N)
{
	/* Disable all IRQs to be sure that no code execution from flash will
	 * occur while erase operation is in progress.
	 * */
	__disable_irq();

	FLASH->CR = FLASH_CR_PSIZE_1 | (N << 3)
		| FLASH_CR_SER | FLASH_CR_STRT;

	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

		/* Kick IWDG during long wait.
		 * */
		IWDG->KR = 0xAAAA;

		__NOP();
	}

	FLASH->CR = 0;

	__enable_irq();
}

void *FLASH_erase(void *flash)
{
	int		N, sector_N = 0;

	for (N = 0; N < FLASH_SECTOR_MAX; ++N) {

		if (		(u32_t) flash >= flash_ram_map[N]
				&& (u32_t) flash < flash_ram_map[N + 1]) {

			flash = (void *) flash_ram_map[N];
			sector_N = N + 8;

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

		/* Flush DATA caches.
		 * */
		FLASH->ACR &= ~FLASH_ACR_DCEN;
		FLASH->ACR |= FLASH_ACR_DCRST;
		FLASH->ACR &= ~FLASH_ACR_DCRST;
		FLASH->ACR |= FLASH_ACR_DCEN;
	}

	return flash;
}

void *FLASH_prog(void *flash, const void *s, int n)
{
	u32_t			*long_flash = flash;
	const u32_t		*long_s = s;

	if (		(u32_t) flash >= flash_ram_map[0]
			&& (u32_t) flash < flash_ram_map[FLASH_SECTOR_MAX]) {

		FLASH_unlock();
		FLASH_wait_BSY();

		FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;

		while (n >= 4) {

			*long_flash++ = *long_s++;
			n += - 4;
		}

		FLASH_wait_BSY();
		FLASH->CR = 0;

		FLASH_lock();
	}

	return flash;
}

