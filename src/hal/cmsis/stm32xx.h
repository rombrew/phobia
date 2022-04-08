#ifndef _H_STM32XX_
#define _H_STM32XX_

#if defined(_HW_STM32F405)
#include "stm32f4xx.h"
#elif defined(_HW_STM32F722)
#include "stm32f7xx.h"
#else
#error HW macro _HW_STM32Fxx must be defined
#endif /* _HW_STM32Fxx */

#endif /* _H_STM32XX_ */

