#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

typedef struct {

	u32_t			number;
	u32_t			regs_N;
	u32_t			content[4093];
	u32_t			crc32;
}
flash_block_t;

static u32_t
flash_block_crc32(flash_block_t *block)
{
	return crc32b(block, sizeof(flash_block_t) - sizeof(u32_t));
}

static flash_block_t *
flash_block_scan()
{
	flash_block_t			*block, *last;

	block = (void *) flash_ram_map[0];
	last = NULL;

	do {
		if (flash_block_crc32(block) == block->crc32) {

			if (last != NULL) {

				last = (block->number > last->number)
					? block : last;
			}
			else {
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
	char			*lsyms;
	int			N, rc = 0;

	block = flash_block_scan();

	if (block != NULL) {

		lsyms = (char *) (block->content + block->regs_N);

		for (N = 0; N < block->regs_N; ++N) {

			/* Search for an exact match.
			 * */
			reg = reg_search(lsyms);

			if (reg != NULL && reg->mode & REG_CONFIG) {

				/* Load content of the REGISTER.
				 * */
				* (u32_t *) reg->link = block->content[N];
			}

			lsyms += strlen(lsyms) + 1;
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
flash_block_store_content(flash_block_t *block)
{
	const reg_t		*reg;
	char			*lsyms;
	int			N;

	for (reg = regfile, N = 0; reg->sym != NULL; ++reg) {

		if (reg->mode & REG_CONFIG) {

			/* Store binary VALUE of register.
			 * */
			block->content[N++] = * (u32_t *) reg->link;
		}
	}

	block->regs_N = N;

	lsyms = (char *) (block->content + N);

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (reg->mode & REG_CONFIG) {

			/* Store symbolic NAME of register.
			 * */
			strcpy(lsyms, reg->sym);
			lsyms += strlen(reg->sym) + 1;

			if (lsyms >= (char *) &block->crc32) {

				/* Block is full.
				 * */
				return 0;
			}
		}
	}

	N = lsyms - (char *) block->content;
	N = (N & 3UL) ? N / 4 + 1 : N / 4;

	/* Fill the tail.
	 * */
	for (; N < sizeof(block->content) / sizeof(u32_t); ++N)
		block->content[N] = 0xFFFFFFFFUL;

	block->crc32 = flash_block_crc32(block);

	return 1;
}

static int
flash_block_write()
{
	flash_block_t		*block, *temp;
	u32_t			number;
	int			rc = 0;

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

	temp = block;

	while (flash_is_block_dirty(block) != 0) {

		block += 1;

		if ((u32_t) block >= flash_ram_map[FLASH_SECTOR_MAX])
			block = (void *) flash_ram_map[0];

		if (block == temp) {

			/* All flash is dirty.
			 * */
			block = FLASH_erase(block);
			break;
		}
	}

	temp = pvPortMalloc(sizeof(flash_block_t));

	if (temp != NULL) {

		temp->number = number;

		rc = flash_block_store_content(temp);

		if (rc != 0) {

			FLASH_prog(block, temp, sizeof(flash_block_t));
		}

		vPortFree(temp);

		if (rc != 0) {

			rc = (flash_block_crc32(block) == block->crc32) ? 1 : 0;
		}
	}

	return rc;
}

int flash_block_relocate()
{
	flash_block_t		*block, *reloc;
	u32_t			number, crc32;
	int			rc = 1;

	block = flash_block_scan();
	reloc = (void *) flash_ram_map[FLASH_SECTOR_MAX - 1];

	if (block != NULL && block < reloc) {

		while (flash_is_block_dirty(reloc) != 0) {

			reloc += 1;

			if ((u32_t) reloc >= flash_ram_map[FLASH_SECTOR_MAX]) {

				reloc = (void *) flash_ram_map[FLASH_SECTOR_MAX - 1];
				reloc = FLASH_erase(reloc);
				break;
			}
		}

		number = block->number + 1;

		FLASH_prog(&reloc->number, &number, sizeof(u32_t));
		FLASH_prog(&reloc->regs_N, &block->regs_N, sizeof(u32_t));
		FLASH_prog(&reloc->content, &block->content, sizeof(block->content));

		crc32 = flash_block_crc32(reloc);

		FLASH_prog(&reloc->crc32, &crc32, sizeof(u32_t));

		rc = (flash_block_crc32(reloc) == reloc->crc32) ? 1 : 0;
	}

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

	printf("%s" EOL, (rc != 0) ? "Done" : "Failed");
}

SH_DEF(flash_info_map)
{
	flash_block_t			*block;
	int				N, info_sym;

	block = (void *) flash_ram_map[0];
	N = 0;

	do {
		if (flash_is_block_dirty(block) != 0) {

			info_sym = 'x';

			if (flash_block_crc32(block) == block->crc32) {

				info_sym = 'a';
			}
		}
		else {
			info_sym = '.';
		}

		printf(" %c", info_sym);

		block += 1;

		if ((u32_t) block >= flash_ram_map[N + 1]) {

			puts(EOL);

			N += 1;

			if (N >= FLASH_SECTOR_MAX)
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
		if (flash_block_crc32(block) == block->crc32) {

			FLASH_prog(&block->crc32, &lz, sizeof(u32_t));
		}

		block += 1;

		if ((u32_t) block >= flash_ram_map[FLASH_SECTOR_MAX])
			break;
	}
	while (1);
}

