#ifndef _H_FLASH_
#define _H_FLASH_

#define FLASH_SECTOR_MAX		4

extern const unsigned long flash_ram_map[];

void *FLASH_sector_erase(void *flash);
void *FLASH_write(void *flash, const void *s, unsigned long sz);

#endif /* _H_FLASH_ */

