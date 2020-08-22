#include <stddef.h>

#include "cmsis/stm32f4xx.h"
#include "hal.h"
#include "libc.h"

const unsigned long flash_ram_map[] = {

	0x08080000UL,
	0x080A0000UL,
	0x080C0000UL,
	0x080E0000UL,
	0x08100000UL
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

	FLASH->CR = FLASH_CR_PSIZE_1 | (N << 3)
		| FLASH_CR_SER | FLASH_CR_STRT;

	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

		/* Kick IWDG during a long wait.
		 * */
		IWDG->KR = 0xAAAA;

		__NOP();
	}

	FLASH->CR = 0;

	__enable_irq();
}

LD_RAMFUNC static void
FLASH_selfupdate_on_IWDG()
{
	u32_t			*long_s = (u32_t *) flash_ram_map[0];
	u32_t			*long_flash, *long_end;
	int			N;

	/* Disable all IRQs to be sure that no code execution from flash will
	 * occur while flash operations is in progress.
	 * */
	__disable_irq();

	for (N = 0; N < 7; ++N) {

		FLASH->CR = FLASH_CR_PSIZE_1 | (N << 3)
			| FLASH_CR_SER | FLASH_CR_STRT;

		while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

			IWDG->KR = 0xAAAA;
			__NOP();
		}
	}

	FLASH->CR = 0;

	/* Get load ADDRESS and SIZE.
	 * */
	long_flash = (u32_t *) *(long_s + 7);
	long_end   = (u32_t *) *(long_s + 8);

	FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;

	while (long_flash < long_end) {

		*long_flash++ = *long_s++;

		while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

			__NOP();
		}

		IWDG->KR = 0xAAAA;
	}

	FLASH->CR = 0;
	FLASH->CR |= FLASH_CR_LOCK;

	__DSB();

	SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)
			| (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk)
			| SCB_AIRCR_SYSRESETREQ_Msk);

	while (1);
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

			FLASH_wait_BSY();
		}

		FLASH->CR = 0;

		FLASH_lock();
	}

	return flash;
}

void FLASH_selfupdate()
{
	GPIO_set_HIGH(GPIO_LED);

	FLASH_unlock();
	FLASH_wait_BSY();

	/* Call the func from RAM because flash will be erased.
	 * */
	FLASH_selfupdate_on_IWDG();
}

