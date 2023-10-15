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

	int			bspin;
	int			bleft;
}
flash_prog_t;

static int
flash_prog_putc(flash_prog_t *pg, int c)
{
	int			rc = 0;

	if (pg->bleft > 0) {

		pg->packed.b[pg->bspin++] = (uint8_t) c;

		if (pg->bspin >= 4) {

			FLASH_prog(pg->flash, pg->packed.l);

			pg->flash += 1;
			pg->bspin = 0;

			pg->bleft -= 4;
		}

		rc = 1;
	}

	return rc;
}

static void
flash_prog_long(flash_prog_t *pg, uint32_t l)
{
	union {
		uint32_t	l;
		uint8_t		b[4];
	}
	packed = { l };

	flash_prog_putc(pg, packed.b[0]);
	flash_prog_putc(pg, packed.b[1]);
	flash_prog_putc(pg, packed.b[2]);
	flash_prog_putc(pg, packed.b[3]);
}

static const char *
flash_strcpyn(char *d, const char *s, int n)
{
	do {
		if (n < 1) {

			*d = 0;
			break;
		}

		if (*s == 0xFF || *s == 0xFE) {

			*d = 0;
			break;
		}

		*d++ = *s++;

		--n;
	}
	while (1);

	return s;
}

static uint32_t
flash_block_crc32(const flash_block_t *block)
{
	return crc32b(block, sizeof(flash_block_t) - sizeof(uint32_t));
}

static flash_block_t *
flash_block_scan()
{
	flash_block_t		*block, *last;

	block = (void *) FLASH_map[0];
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

		if ((uint32_t) block >= FLASH_map[FLASH_config.total])
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

	char			*symbuf;
	int			rc = 0;

	block = flash_block_scan();

	if (block == NULL) {

		/* No valid configuration block found.
		 * */
		return rc;
	}

	symbuf = pvPortMalloc(REGS_SYM_MAX + 1);
	lsym = (const char *) block->content;

	while (*lsym != 0xFF) {

		lsym = flash_strcpyn(symbuf, lsym, REGS_SYM_MAX);

		/* Search for an exact match of symbolic NAME.
		 * */
		reg = reg_search(symbuf);

		if (*lsym == 0xFE) {

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

				/* Load binary VALUE.
				 * */
				memcpy(reg->link, lsym + 1, sizeof(uint32_t));
			}

			lsym += 5;
		}
		else {
			rc = 1;
			break;
		}

		if (*lsym != 0xFF) {

			rc = 1;
			break;
		}

		lsym++;
	}

	vPortFree(symbuf);

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
	pg.bspin = 0;
	pg.bleft = sizeof(block->content);

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (reg->mode & REG_CONFIG) {

			lsym = reg->sym;

			/* Store symbolic NAME of register.
			 * */
			while (*lsym != 0) { flash_prog_putc(&pg, *lsym++); }

			if (reg->mode & REG_LINKED) {

				lsym = regfile[reg->link->i].sym;

				flash_prog_putc(&pg, 0xFE);

				while (*lsym != 0) { flash_prog_putc(&pg, *lsym++); }
			}
			else {
				/* Store binary VALUE.
				 * */
				flash_prog_putc(&pg, 0xFF);
				flash_prog_long(&pg, reg->link->i);
			}

			if ((rc = flash_prog_putc(&pg, 0xFF)) == 0)
				break;
		}
	}

	if (rc != 0) {

		/* Fill the tail.
		 * */
		while (flash_prog_putc(&pg, 0xFF) != 0) ;
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

		if ((uint32_t) block >= FLASH_map[FLASH_config.total])
			block = (void *) FLASH_map[0];
	}
	else {
		number = 1;
		block = (void *) FLASH_map[0];
	}

	origin = block;

	while (flash_is_block_dirty(block) != 0) {

		block += 1;

		if ((uint32_t) block >= FLASH_map[FLASH_config.total])
			block = (void *) FLASH_map[0];

		if (block == origin) {

			/* All flash is dirty.
			 * */
			block = FLASH_erase(block);
			break;
		}
	}

	FLASH_prog(&block->number, number);

	if ((rc = flash_prog_config_regs(block)) != 0) {

		crc32 = flash_block_crc32(block);

		FLASH_prog(&block->crc32, crc32);

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

	block = (void *) FLASH_map[0];
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

		if ((uint32_t) block >= FLASH_map[N + 1]) {

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

	block = (void *) FLASH_map[0];

	do {
		if (flash_block_crc32(block) == block->crc32) {

			FLASH_prog(&block->crc32, lz);
		}

		block += 1;

		if ((uint32_t) block >= FLASH_map[FLASH_config.total])
			break;
	}
	while (1);
}

