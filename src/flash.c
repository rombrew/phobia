/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

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

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

typedef struct {

	unsigned long		number;
	unsigned long		version;
	unsigned long		content[1021];
	unsigned long		crc32;
}
flash_block_t;

static flash_block_t *
flash_block_scan()
{
	flash_block_t			*block, *last;

	block = (void *) flash_ram_map[0];
	last = NULL;

	do {
		if (block->version == REG_CONFIG_VERSION) {

			if (crc32b(block, sizeof(flash_block_t) - 4) == block->crc32) {

				if (last != NULL) {

					last = (block->number > last->number)
						? block : last;
				}
				else
					last = block;
			}
		}

		block += 1;

		if ((unsigned long) block >= flash_ram_map[FLASH_SECTOR_MAX])
			break;
	}
	while (1);

	return last;
}

int flash_block_load()
{
	const reg_t		*reg;
	flash_block_t		*block;
	unsigned long		*content;
	int			rc = -1;

	block = flash_block_scan();

	if (block != NULL) {

		content = block->content;

		for (reg = regfile; reg->sym != NULL; ++reg) {

			if (reg->mode & REG_CONFIG) {

				* (unsigned long *) reg->link = *content++;
			}
		}

		rc = 0;
	}

	return rc;
}

static int
flash_is_block_dirty(const flash_block_t *block)
{
	const unsigned long	*lsrc, *lend;
	int			dirty = 0;

	lsrc = (const unsigned long *) block;
	lend = (const unsigned long *) (block + 1);

	while (lsrc < lend) {

		if (*lsrc++ != 0xFFFFFFFFUL) {

			dirty = 1;
			break;
		}
	}

	return dirty;
}

static int
flash_block_write()
{
	const reg_t		*reg;
	flash_block_t		*block, *temp;
	unsigned int		number;
	int			n, rc;

	block = flash_block_scan();

	if (block != NULL) {

		number = block->number + 1;
		block += 1;

		if ((unsigned long) block >= flash_ram_map[FLASH_SECTOR_MAX])
			block = (void *) flash_ram_map[0];
	}
	else {
		number = 1;
		block = (void *) flash_ram_map[0];
	}

	if (flash_is_block_dirty(block)) {

		FLASH_sector_erase(block);
	}

	temp = pvPortMalloc(sizeof(flash_block_t));

	if (temp != NULL) {

		temp->number = number;
		temp->version = REG_CONFIG_VERSION;

		for (reg = regfile, n = 0; reg->sym != NULL; ++reg) {

			if (reg->mode & REG_CONFIG) {

				temp->content[n++] = * (unsigned long *) reg->link;
			}
		}

		for (; n < sizeof(temp->content) / sizeof(unsigned long); ++n)
			temp->content[n] = 0xFFFFFFFFUL;

		temp->crc32 = crc32b(temp, sizeof(flash_block_t) - 4);

		FLASH_write(block, temp, sizeof(flash_block_t));

		vPortFree(temp);
	}

	rc = (crc32b(block, sizeof(flash_block_t) - 4) == block->crc32) ? 0 : -1;

	return rc;
}

SH_DEF(flash_write)
{
	int			rc;

	if (pm.lu_mode != PM_LU_DISABLED)
		return ;

	printf("Flash ... ");

	rc = flash_block_write();

	printf("%s" EOL, (rc == 0) ? "Done" : "Failed");
}

SH_DEF(flash_info_map)
{
	flash_block_t			*block;
	int				sector_N, info_sym;

	block = (void *) flash_ram_map[0];
	sector_N = 0;

	do {
		if (flash_is_block_dirty(block)) {

			info_sym = 'x';

			if (block->version == REG_CONFIG_VERSION) {

				if (crc32b(block, sizeof(flash_block_t) - 4) == block->crc32) {

					info_sym = 'a';
				}
			}
		}
		else {
			info_sym = '.';
		}

		iodef->putc(info_sym);

		block += 1;

		if ((unsigned long) block >= flash_ram_map[sector_N + 1]) {

			puts(EOL);

			sector_N += 1;

			if (sector_N >= FLASH_SECTOR_MAX)
				break;
		}
	}
	while (1);
}

