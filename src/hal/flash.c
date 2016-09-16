/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2016 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stddef.h>

#include "cmsis/stm32f4xx.h"
#include "flash.h"
#include "hal.h"

#define FLASH_SECTOR_OFFSET		8

void * const flash_sector_map[] = {

	/* Base addresses.
	 * */
	(void *) 0x08080000,
	(void *) 0x080A0000,
	(void *) 0x080C0000,
	(void *) 0x080E0000,

	/* The End.
	 * */
	(void *) 0x08100000,
};

static void
flash_unlock()
{
	if (FLASH->CR & FLASH_CR_LOCK) {

		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
}

static void
flash_lock()
{
	FLASH->CR |= FLASH_CR_LOCK;
}

static void
flash_wait_for_complete()
{
	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) ;
}

void flash_erase(int snb)
{
	flash_unlock();
	flash_wait_for_complete();

	snb = (snb + FLASH_SECTOR_OFFSET) & 0x0F;

	FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER | (snb << 3) | FLASH_CR_SER;
	FLASH->CR |= FLASH_CR_STRT;

	flash_wait_for_complete();
	FLASH->CR &= ~(FLASH_CR_SER);

	flash_lock();
}

void flash_write(void *d, const void *s, int sz)
{
	unsigned int		*ud = d;
	const unsigned int	*us = s;

	flash_unlock();
	flash_wait_for_complete();

	FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;

	while (sz >= 4) {

		*ud++ = *us++;
		sz -= 4;
	}

	flash_wait_for_complete();
	FLASH->CR &= ~(FLASH_CR_PG);

	flash_lock();
}

