/**
  ******************************************************************************
  * @file    stm32f7xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32F7xx Device Peripheral Access Layer Header File.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32F7xx device used in the target application
  *              - To use or not the peripheral’s drivers in application code(i.e.
  *                code will be based on direct access to peripheral’s registers
  *                rather than drivers API), this option is controlled by
  *                "#define USE_HAL_DRIVER"
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f7xx
  * @{
  */

#ifndef __STM32F7xx_H
#define __STM32F7xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief STM32 Family
  */
#if !defined  (STM32F7)
#define STM32F7
#endif /* STM32F7 */

/**
  * @brief CMSIS Device version number V1.2.6
  */
#define __STM32F7_CMSIS_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32F7_CMSIS_VERSION_SUB1   (0x02) /*!< [23:16] sub1 version */
#define __STM32F7_CMSIS_VERSION_SUB2   (0x06) /*!< [15:8]  sub2 version */
#define __STM32F7_CMSIS_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32F7_CMSIS_VERSION        ((__STM32F7_CMSIS_VERSION_MAIN << 24)\
                                       |(__STM32F7_CMSIS_VERSION_SUB1 << 16)\
                                       |(__STM32F7_CMSIS_VERSION_SUB2 << 8 )\
                                       |(__STM32F7_CMSIS_VERSION_RC))
/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */
#include "stm32f722xx.h"

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */
typedef enum
{
  RESET = 0U,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0U,
  ENABLE = !DISABLE
} FunctionalState;

typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;

/**
  * @}
  */

/** @addtogroup Exported_macro
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F7xx_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

