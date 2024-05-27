#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

#define REGS_SYM_MAX				79

typedef struct {

	uint32_t		number;
	uint32_t		content[4094];
	uint32_t		crc32;
}
flash_block_t;

typedef struct {

	uint32_t		*flash;

	union {
		uint32_t	l;
		uint8_t		b[4];
	}
	packed;

	int			index;
	int			total;
}
flash_prog_t;

static int
flash_prog_u8(flash_prog_t *pg, uint8_t u)
{
	int			rc = 0;

	if (pg->total >= 4) {

		pg->packed.b[pg->index++] = u;

		if (pg->index >= 4) {

			FLASH_prog_u32(pg->flash, pg->packed.l);

			pg->flash += 1;
			pg->total -= 4;

			pg->index = 0;
		}

		rc = 1;
	}

	return rc;
}

static void
flash_prog_u32(flash_prog_t *pg, uint32_t l)
{
	union {
		uint32_t	l;
		uint8_t		b[4];
	}
	packed = { l };

	flash_prog_u8(pg, packed.b[0]);
	flash_prog_u8(pg, packed.b[1]);
	flash_prog_u8(pg, packed.b[2]);
	flash_prog_u8(pg, packed.b[3]);
}

static const char *
flash_strcpyn(char *d, const char *s, int len)
{
	do {
		if (len < 1) {

			*d = 0;
			break;
		}

		if (		   *s == 0xFF
				|| *s == 0xBF) {

			*d = 0;
			break;
		}

		*d++ = *s++;
		--len;
	}
	while (1);

	return s;
}

static uint32_t
flash_block_crc32(const flash_block_t *block)
{
	return crc32u(block, sizeof(flash_block_t) - sizeof(uint32_t));
}

static flash_block_t *
flash_block_scan()
{
	flash_block_t		*block, *last;

	block = (flash_block_t *) FLASH_config.map[0];
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

		if ((uint32_t) block >= FLASH_config.map[FLASH_config.total])
			break;
	}
	while (1);

	return last;
}

int flash_block_regs_load()
{
	const flash_block_t	*block;
	const reg_t		*reg, *linked;
	const char		*lsym;

	char			symbuf[REGS_SYM_MAX + 1];
	int			rc = 0;

	block = flash_block_scan();

	if (block == NULL) {

		/* No valid configuration block found.
		 * */
		return rc;
	}

	lsym = (const char *) block->content;

	while (*lsym != 0xFF) {

		lsym = flash_strcpyn(symbuf, lsym, REGS_SYM_MAX);

		/* Search for an exact match of symbolic name.
		 * */
		reg = reg_search(symbuf);

		if (*lsym == 0xBF) {

			lsym = flash_strcpyn(symbuf, lsym + 1, REGS_SYM_MAX);

			if (reg != NULL && reg->mode & REG_CONFIG
					&& reg->mode & REG_LINKED) {

				linked = reg_search(symbuf);

				if (linked != NULL) {

					reg->link->i = (int) (linked - regfile);
				}
			}
		}
		else if (*lsym == 0xFF) {

			if (reg != NULL && reg->mode & REG_CONFIG) {

				memcpy((void *) reg->link, lsym + 1, sizeof(uint32_t));
			}

			lsym += 5;
		}

		if (*lsym != 0xFF) {

			rc = 1;
			break;
		}

		lsym++;
	}

	return rc;
}

static int
flash_is_block_dirty(const flash_block_t *block)
{
	const uint32_t		*lsrc, *lend;
	int			dirty = 0;

	lsrc = (const uint32_t *) block;
	lend = (const uint32_t *) (block + 1);

	while (lsrc < lend) {

		if (*lsrc++ != 0xFFFFFFFFU) {

			dirty = 1;
			break;
		}
	}

	return dirty;
}

static int
flash_prog_config_regs(flash_block_t *block)
{
	const reg_t		*reg;
	const char		*lsym;

	flash_prog_t		pg;
	int			rc = 0;

	pg.flash = block->content;
	pg.index = 0;
	pg.total = sizeof(block->content);

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (reg->mode & REG_CONFIG) {

			lsym = reg->sym;

			/* Store symbolic name of the register.
			 * */
			while (*lsym != 0) { flash_prog_u8(&pg, *lsym++); }

			if (reg->mode & REG_LINKED) {

				lsym = regfile[reg->link->i].sym;

				flash_prog_u8(&pg, 0xBF);

				while (*lsym != 0) { flash_prog_u8(&pg, *lsym++); }
			}
			else {
				flash_prog_u8(&pg, 0xFF);
				flash_prog_u32(&pg, reg->link->i);
			}

			if ((rc = flash_prog_u8(&pg, 0xFF)) == 0)
				break;
		}
	}

	if (rc != 0) {

		while (pg.index != 0) {

			/* Flush the tail contents.
			 * */
			flash_prog_u8(&pg, 0xFF);
		}
	}

	return rc;
}

static int
flash_block_prog()
{
	flash_block_t		*block, *origin;
	uint32_t		number, crc32;
	int			rc = 0;

	block = flash_block_scan();

	if (block != NULL) {

		number = block->number + 1;
		block += 1;

		if ((uint32_t) block >= FLASH_config.map[FLASH_config.total])
			block = (flash_block_t *) FLASH_config.map[0];
	}
	else {
		number = 1;
		block = (flash_block_t *) FLASH_config.map[0];
	}

	origin = block;

	while (flash_is_block_dirty(block) != 0) {

		block += 1;

		if ((uint32_t) block >= FLASH_config.map[FLASH_config.total])
			block = (flash_block_t *) FLASH_config.map[0];

		if (block == origin) {

			/* All flash storage is dirty.
			 * */
			block = FLASH_erase((uint32_t *) block);
			break;
		}
	}

	FLASH_prog_u32(&block->number, number);

	if ((rc = flash_prog_config_regs(block)) != 0) {

		crc32 = flash_block_crc32(block);

		FLASH_prog_u32(&block->crc32, crc32);

		rc = (flash_block_crc32(block) == block->crc32) ? 1 : 0;
	}

	return rc;
}

SH_DEF(flash_prog)
{
	int			rc;

	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	printf("Flash ... ");

	rc = flash_block_prog();

	printf("%s" EOL, (rc != 0) ? "Done" : "Fail");
}

SH_DEF(flash_info)
{
	flash_block_t			*block;
	int				N, info_sym;

	block = (flash_block_t *) FLASH_config.map[0];
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

		if ((uint32_t) block >= FLASH_config.map[N + 1]) {

			puts(EOL);

			N += 1;

			if (N >= FLASH_config.total)
				break;
		}
	}
	while (1);
}

SH_DEF(flash_wipe)
{
	flash_block_t			*block;
	uint32_t			lz = 0;

	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	block = (flash_block_t *) FLASH_config.map[0];

	do {
		if (flash_block_crc32(block) == block->crc32) {

			FLASH_prog_u32(&block->crc32, lz);
		}

		block += 1;

		if ((uint32_t) block >= FLASH_config.map[FLASH_config.total])
			break;
	}
	while (1);
}

