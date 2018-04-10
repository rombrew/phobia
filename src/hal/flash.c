/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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
#include "hal.h"

#define FLASH_FIRST_SECTOR		8

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
	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) ;
}

void FLASH_erase(int n)
{
	FLASH_unlock();
	FLASH_wait_BSY();

	n = (n + FLASH_FIRST_SECTOR) & 0x0F;

	FLASH->CR = FLASH_CR_PSIZE_1 | (n << 3) | FLASH_CR_SER;
	FLASH->CR |= FLASH_CR_STRT;

	FLASH_wait_BSY();
	FLASH->CR &= ~(FLASH_CR_SER);

	FLASH_lock();
}

void FLASH_write(void *d, const void *s, unsigned long sz)
{
	long			*ldst = d;
	const long		*lsrc = s;

	FLASH_unlock();
	FLASH_wait_BSY();

	FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;

	while (sz >= sizeof(long)) {

		*ldst++ = *lsrc++;
		sz -= sizeof(long);
	}

	FLASH_wait_BSY();
	FLASH->CR &= ~(FLASH_CR_PG);

	FLASH_lock();
}

