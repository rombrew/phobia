#ifndef _H_STM32XX_
#define _H_STM32XX_

#if defined(HW_MCU_STM32F405)
#include "stm32f4xx.h"
#elif defined(HW_MCU_STM32F722)
#include "stm32f7xx.h"
#else
#error HW macro HW_MCU_XX must be defined
#endif /* HW_MCU_XX */

#endif /* _H_STM32XX_ */

