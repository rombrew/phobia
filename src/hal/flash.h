#ifndef _H_FLASH_
#define _H_FLASH_

#include "libc.h"

typedef struct {

	int		s_first;
	int		s_total;
}
FLASH_config_t;

extern const FLASH_config_t	FLASH_config;
extern const uint32_t		FLASH_map[];

void *FLASH_erase(void *flash);
void FLASH_prog(void *flash, uint32_t value);

#endif /* _H_FLASH_ */

