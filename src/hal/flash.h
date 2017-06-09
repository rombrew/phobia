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

#ifndef _H_FLASH_
#define _H_FLASH_

#define FLASH_STORAGE_BASE__		0x08080000
#define FLASH_SECTOR_SIZE		0x00020000
#define FLASH_SECTOR_MAX		4

#define FLASH_STORAGE_END		(FLASH_STORAGE_BASE + FLASH_SECTOR_SIZE * FLASH_SECTOR_MAX)

void flash_erase(int N);
void flash_write(void *d, const void *s, int sz);

#endif /* _H_FLASH_ */

