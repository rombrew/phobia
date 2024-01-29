#ifndef _H_FLASH_
#define _H_FLASH_

#include "libc.h"

typedef struct {

	int		begin;
	int		total;

	uint32_t	map[6];
}
FLASH_config_t;

extern const FLASH_config_t	FLASH_config;

void *FLASH_erase(void *flash);
void FLASH_prog(void *flash, uint32_t value);

#endif /* _H_FLASH_ */

