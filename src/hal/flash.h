#ifndef _H_FLASH_
#define _H_FLASH_

#include "libc.h"

typedef struct {

	int		begin;
	int		total;

	uint32_t	flash;
	uint32_t	map[6];
}
FLASH_config_t;

extern const FLASH_config_t	FLASH_config;

void *FLASH_erase(uint32_t *flash);
void FLASH_prog_u32(uint32_t *flash, uint32_t val);

#endif /* _H_FLASH_ */

