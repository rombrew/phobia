#include <stddef.h>

#include "cmsis/stm32f4xx.h"
#include "hal.h"

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

void *FLASH_erase(void *flash)
{
	int		N, sector_N = 0;

	for (N = 0; N < FLASH_SECTOR_MAX; ++N) {

		if (		(unsigned long) flash >= flash_ram_map[N]
				&& (unsigned long) flash < flash_ram_map[N + 1]) {

			flash = (void *) flash_ram_map[N];
			sector_N = N + 8;

			break;
		}
	}

	if (sector_N != 0) {

		FLASH_unlock();
		FLASH_wait_BSY();

		FLASH->CR = FLASH_CR_PSIZE_1 | (sector_N << 3) | FLASH_CR_SER;
		FLASH->CR |= FLASH_CR_STRT;

		FLASH_wait_BSY();
		FLASH->CR &= ~(FLASH_CR_SER);

		FLASH_lock();
	}

	return flash;
}

void *FLASH_prog(void *flash, const void *s, int sz)
{
	long			*ld = flash;
	const long		*ls = s;

	if (		(unsigned long) flash >= flash_ram_map[0]
			&& (unsigned long) flash < flash_ram_map[FLASH_SECTOR_MAX]) {

		FLASH_unlock();
		FLASH_wait_BSY();

		FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;

		while (sz >= sizeof(long)) {

			*ld++ = *ls++;
			sz -= sizeof(long);
		}

		FLASH_wait_BSY();
		FLASH->CR &= ~(FLASH_CR_PG);

		FLASH_lock();
	}

	return flash;
}

