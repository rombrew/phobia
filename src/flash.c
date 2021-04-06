#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

#define REGS_SYM_MAX				79

typedef struct {

	u32_t			number;
	u32_t			content[4094];
	u32_t			crc32;
}
flash_block_t;

typedef struct {

	u32_t			*flash;

	union {
		u32_t		l;
		u8_t		b[4];
	}
	packed;

	int			pack_i;
	int			bleft;
}
FE_prog_t;

static int
FE_prog_putc(FE_prog_t *pg, int c)
{
	int			rc = 0;

	if (pg->bleft > 0) {

		pg->packed.b[pg->pack_i++] = (u8_t) c;

		if (pg->pack_i >= 4) {

			FLASH_prog(pg->flash, &pg->packed.l, sizeof(u32_t));

			pg->flash += 1;
			pg->pack_i = 0;

			pg->bleft -= 4;
		}

		rc = 1;
	}

	return rc;
}

static void
FE_prog_long(FE_prog_t *pg, u32_t l)
{
	union {
		u32_t		l;
		u8_t		b[4];
	}
	packed = { l };

	FE_prog_putc(pg, packed.b[0]);
	FE_prog_putc(pg, packed.b[1]);
	FE_prog_putc(pg, packed.b[2]);
	FE_prog_putc(pg, packed.b[3]);
}

static const char *
FE_strcpyn(char *d, const char *s, int n)
{
	do {
		if (n <= 0) {

			*d = 0;
			break;
		}

		if (*s == 0xFF || *s == 0xFE) {

			*d = 0;
			break;
		}

		*d++ = *s++;

		n--;
	}
	while (1);

	return s;
}

static u32_t
flash_block_crc32(const flash_block_t *block)
{
	return crc32b(block, sizeof(flash_block_t) - sizeof(u32_t));
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

		if ((u32_t) block >= FLASH_map[FLASH_config.s_total])
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

	char			*temp_s;
	int			FE_fail = 0;

	block = flash_block_scan();

	if (block == NULL) {

		/* No valid configuration block found.
		 * */
		return FE_fail;
	}

	temp_s = pvPortMalloc(REGS_SYM_MAX + 1);
	lsym = (const char *) block->content;

	while (*lsym != 0xFF) {

		lsym = FE_strcpyn(temp_s, lsym, REGS_SYM_MAX);

		/* Search for an exact match of symbolic NAME.
		 * */
		reg = reg_search(temp_s);

		if (*lsym == 0xFE) {

			lsym = FE_strcpyn(temp_s, lsym + 1, REGS_SYM_MAX);

			if (reg != NULL && reg->mode & REG_CONFIG
					&& reg->mode & REG_LINKED) {

				linked = reg_search(temp_s);

				if (linked != NULL) {

					reg->link->i = (int) (linked - regfile);
				}
			}
		}
		else if (*lsym == 0xFF) {

			if (reg != NULL && reg->mode & REG_CONFIG) {

				/* Load binary VALUE.
				 * */
				memcpy(reg->link, lsym + 1, sizeof(u32_t));
			}

			lsym += 5;
		}
		else {
			FE_fail = 1;
			break;
		}

		if (*lsym != 0xFF) {

			FE_fail = 1;
			break;
		}

		lsym++;
	}

	vPortFree(temp_s);

	return FE_fail;
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
flash_regs_FE_prog(flash_block_t *block)
{
	const reg_t		*reg;
	const char		*lsym;

	FE_prog_t		pg;
	int			rc = 0;

	pg.flash = block->content;
	pg.pack_i = 0;
	pg.bleft = sizeof(block->content);

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (reg->mode & REG_CONFIG) {

			lsym = reg->sym;

			/* Store symbolic NAME of register.
			 * */
			while (*lsym != 0) { FE_prog_putc(&pg, *lsym++); }

			if (reg->mode & REG_LINKED) {

				lsym = regfile[reg->link->i].sym;

				FE_prog_putc(&pg, 0xFE);

				while (*lsym != 0) { FE_prog_putc(&pg, *lsym++); }
			}
			else {
				/* Store binary VALUE.
				 * */
				FE_prog_putc(&pg, 0xFF);
				FE_prog_long(&pg, reg->link->i);
			}

			rc = FE_prog_putc(&pg, 0xFF);

			if (rc == 0) {

				break;
			}
		}
	}

	if (rc != 0) {

		/* Fill the tail.
		 * */
		while (FE_prog_putc(&pg, 0xFF) != 0) ;
	}

	return rc;
}

static int
flash_block_prog()
{
	flash_block_t		*block, *origin;
	u32_t			number, crc32;
	int			rc = 0;

	block = flash_block_scan();

	if (block != NULL) {

		number = block->number + 1;
		block += 1;

		if ((u32_t) block >= FLASH_map[FLASH_config.s_total])
			block = (void *) FLASH_map[0];
	}
	else {
		number = 1;
		block = (void *) FLASH_map[0];
	}

	origin = block;

	while (flash_is_block_dirty(block) != 0) {

		block += 1;

		if ((u32_t) block >= FLASH_map[FLASH_config.s_total])
			block = (void *) FLASH_map[0];

		if (block == origin) {

			/* All flash is dirty.
			 * */
			block = FLASH_erase(block);
			break;
		}
	}

	FLASH_prog(&block->number, &number, sizeof(u32_t));

	rc = flash_regs_FE_prog(block);

	if (rc != 0) {

		crc32 = flash_block_crc32(block);

		FLASH_prog(&block->crc32, &crc32, sizeof(u32_t));

		rc = (flash_block_crc32(block) == block->crc32) ? 1 : 0;
	}

	return rc;
}

int flash_block_relocate(u32_t len)
{
	flash_block_t		*block, *reloc, *temp;
	int			N, erase_N, rc = 1;

	block = flash_block_scan();

	if (block != NULL) {

		for (erase_N = 1; erase_N < FLASH_config.s_total + 1; ++erase_N) {

			if (FLASH_map[erase_N] - FLASH_map[0] >= len)
				break;
		}

		/* Check if we have at least one unaffected flash sector. If so
		 * we can safely relocate the configuration block to it.
		 * */
		if (erase_N < FLASH_config.s_total) {

			reloc = (void *) FLASH_map[erase_N];

			while (flash_is_block_dirty(reloc) != 0) {

				reloc += 1;

				if ((u32_t) reloc >= FLASH_map[FLASH_config.s_total]) {

					reloc = (void *) FLASH_map[FLASH_config.s_total - 1];
					reloc = FLASH_erase(reloc);
					break;
				}
			}

			FLASH_prog(reloc, block, sizeof(flash_block_t));

			rc = (flash_block_crc32(reloc) == reloc->crc32) ? 1 : 0;

			for (N = 0; N < erase_N; ++N)
				FLASH_erase((void *) FLASH_map[N]);
		}
		else {
			temp = pvPortMalloc(sizeof(flash_block_t));

			if (temp != NULL) {

				memcpy(temp, block, sizeof(flash_block_t));

				/* NOTE: If power goes down on the erase before
				 * flashprog we will loss the configuration block.
				 * */

				for (N = 0; N < erase_N; ++N)
					FLASH_erase((void *) FLASH_map[N]);

				reloc = (void *) FLASH_map[FLASH_config.s_total];
				reloc -= 1;

				FLASH_prog(reloc, temp, sizeof(flash_block_t));

				rc = (flash_block_crc32(reloc) == reloc->crc32) ? 1 : 0;

				vPortFree(temp);
			}
			else {
				/* We are failed to relocate the configuration
				 * block in this case.
				 * */
				rc = 0;
			}
		}
	}

	return rc;
}

SH_DEF(flash_prog)
{
	int			rc;

	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	printf("Flash ... ");

	rc = flash_block_prog();

	printf("%s" EOL, (rc != 0) ? "Done" : "Fail");
}

SH_DEF(flash_info_map)
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

		if ((u32_t) block >= FLASH_map[N + 1]) {

			puts(EOL);

			N += 1;

			if (N >= FLASH_config.s_total)
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

	block = (void *) FLASH_map[0];

	do {
		if (flash_block_crc32(block) == block->crc32) {

			FLASH_prog(&block->crc32, &lz, sizeof(u32_t));
		}

		block += 1;

		if ((u32_t) block >= FLASH_map[FLASH_config.s_total])
			break;
	}
	while (1);
}

