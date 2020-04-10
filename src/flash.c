#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

typedef struct {

	u32_t			number;
	u32_t			version;
	u32_t			content[1021];
	u32_t			crc32;
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

		if ((u32_t) block >= flash_ram_map[FLASH_SECTOR_MAX])
			break;
	}
	while (1);

	return last;
}

int flash_block_load()
{
	const reg_t		*reg;
	flash_block_t		*block;
	u32_t			*content;
	int			rc = 0;

	block = flash_block_scan();

	if (block != NULL) {

		content = block->content;

		for (reg = regfile; reg->sym != NULL; ++reg) {

			if (reg->mode & REG_CONFIG) {

				* (u32_t *) reg->link = *content++;
			}
		}

		rc = 1;
	}

	return rc;
}

static int
flash_is_block_dirty(const flash_block_t *block)
{
	const u32_t		*lsrc, *lend;
	int			dirty = 0;

	lsrc = (const u32_t *) block;
	lend = (const u32_t *) (block + 1);

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

		if ((u32_t) block >= flash_ram_map[FLASH_SECTOR_MAX])
			block = (void *) flash_ram_map[0];
	}
	else {
		number = 1;
		block = (void *) flash_ram_map[0];
	}

	if (flash_is_block_dirty(block) != 0) {

		FLASH_erase(block);
	}

	temp = pvPortMalloc(sizeof(flash_block_t));

	if (temp != NULL) {

		temp->number = number;
		temp->version = REG_CONFIG_VERSION;

		for (reg = regfile, n = 0; reg->sym != NULL; ++reg) {

			if (reg->mode & REG_CONFIG) {

				temp->content[n++] = * (u32_t *) reg->link;
			}
		}

		for (; n < sizeof(temp->content) / sizeof(u32_t); ++n)
			temp->content[n] = 0xFFFFFFFFUL;

		temp->crc32 = crc32b(temp, sizeof(flash_block_t) - 4);

		FLASH_prog(block, temp, sizeof(flash_block_t));

		vPortFree(temp);
	}

	rc = (crc32b(block, sizeof(flash_block_t) - 4) == block->crc32) ? 0 : -1;

	return rc;
}

SH_DEF(flash_write)
{
	int			rc;

	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

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
		if (flash_is_block_dirty(block) != 0) {

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

		if ((u32_t) block >= flash_ram_map[sector_N + 1]) {

			puts(EOL);

			sector_N += 1;

			if (sector_N >= FLASH_SECTOR_MAX)
				break;
		}
	}
	while (1);
}

SH_DEF(flash_cleanup)
{
	flash_block_t			*block;
	u32_t				lz = 0;

	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	block = (void *) flash_ram_map[0];

	do {
		if (block->version == REG_CONFIG_VERSION) {

			if (crc32b(block, sizeof(flash_block_t) - 4) == block->crc32) {

				FLASH_prog(&block->crc32, &lz, sizeof(u32_t));
			}
		}

		block += 1;

		if ((u32_t) block >= flash_ram_map[FLASH_SECTOR_MAX])
			break;
	}
	while (1);
}

