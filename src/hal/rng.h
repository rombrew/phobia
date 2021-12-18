#ifndef _H_RNG_
#define _H_RNG_

#include "libc.h"

void RNG_startup();
u32_t RNG_urand();
u32_t RNG_make_UID();

#endif /* _H_RNG_ */

