#ifndef _H_ADC_
#define _H_ADC_

#include "gpio.h"

#define ADC_RESOLUTION			4096

#ifdef _HW_REV2

#define GPIO_ADC_CURRENT_A		XGPIO_DEF3('C', 3, 13)
#define GPIO_ADC_CURRENT_B		XGPIO_DEF3('A', 1, 1)
#define GPIO_ADC_VOLTAGE_U		XGPIO_DEF3('A', 2, 2)
#define GPIO_ADC_VOLTAGE_A		XGPIO_DEF3('A', 3, 3) // not impl
#define GPIO_ADC_VOLTAGE_B		XGPIO_DEF3('A', 3, 3) // not impl
#define GPIO_ADC_VOLTAGE_C		XGPIO_DEF3('A', 3, 3) // not impl
#define GPIO_ADC_PCB_NTC		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_EXT_NTC		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_ANALOG_ANG		XGPIO_DEF3('A', 0, 0) // not impl
#define GPIO_ADC_ANALOG_BRK		XGPIO_DEF3('A', 0, 0) // not impl
#define GPIO_ADC_INTERNAL_TEMP		XGPIO_DEF3('J', 0, 1)

#endif /* _HW_REV2 */

#ifdef _HW_REV4B

#define GPIO_ADC_CURRENT_A		XGPIO_DEF3('A', 3, 3)
#define GPIO_ADC_CURRENT_B		XGPIO_DEF3('A', 2, 2)
#define GPIO_ADC_VOLTAGE_U		XGPIO_DEF3('A', 1, 1)
#define GPIO_ADC_VOLTAGE_A		XGPIO_DEF3('C', 3, 13)
#define GPIO_ADC_VOLTAGE_B		XGPIO_DEF3('C', 0, 10)
#define GPIO_ADC_VOLTAGE_C		XGPIO_DEF3('C', 1, 11)
#define GPIO_ADC_PCB_NTC		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_EXT_NTC		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_ANALOG_ANG		XGPIO_DEF3('A', 0, 0)
#define GPIO_ADC_ANALOG_BRK		XGPIO_DEF3('A', 0, 0)
#define GPIO_ADC_INTERNAL_TEMP		XGPIO_DEF3('J', 0, 1)

#endif /* _HW_REV4B */

#ifdef _HW_REV4C

#define GPIO_ADC_CURRENT_A		XGPIO_DEF3('A', 3, 3)
#define GPIO_ADC_CURRENT_B		XGPIO_DEF3('A', 2, 2)
#define GPIO_ADC_VOLTAGE_U		XGPIO_DEF3('A', 1, 1)
#define GPIO_ADC_VOLTAGE_A		XGPIO_DEF3('C', 3, 13)
#define GPIO_ADC_VOLTAGE_B		XGPIO_DEF3('C', 0, 10)
#define GPIO_ADC_VOLTAGE_C		XGPIO_DEF3('C', 1, 11)
#define GPIO_ADC_PCB_NTC		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_EXT_NTC		XGPIO_DEF3('A', 0, 0)
#define GPIO_ADC_ANALOG_ANG		XGPIO_DEF3('B', 1, 9)
#define GPIO_ADC_ANALOG_BRK		XGPIO_DEF3('C', 4, 14)
#define GPIO_ADC_INTERNAL_TEMP		XGPIO_DEF3('J', 0, 1)

#endif /* _HW_REV4C */

void ADC_irq_lock();
void ADC_irq_unlock();

void ADC_startup();
float ADC_get_VALUE(int xGPIO);

extern void ADC_IRQ();

#endif /* _H_ADC_ */

