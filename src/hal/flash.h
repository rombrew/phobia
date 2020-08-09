#ifndef _H_FLASH_
#define _H_FLASH_

#define FLASH_SECTOR_MAX		4

extern const unsigned long flash_ram_map[];

void *FLASH_erase(void *flash);
void *FLASH_prog(void *flash, const void *s, int n);

void FLASH_selfupdate();

#endif /* _H_FLASH_ */

