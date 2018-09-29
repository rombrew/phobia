/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _H_ADC_
#define _H_ADC_

#include "gpio.h"

#define GPIO_ADC_CURRENT_A		XGPIO_DEF3('A', 3, 3)
#define GPIO_ADC_CURRENT_B		XGPIO_DEF3('A', 2, 2)
#define GPIO_ADC_VOLTAGE_U		XGPIO_DEF3('A', 1, 1)
#define GPIO_ADC_VOLTAGE_A		XGPIO_DEF3('C', 3, 13)
#define GPIO_ADC_VOLTAGE_B		XGPIO_DEF3('C', 0, 10)
#define GPIO_ADC_VOLTAGE_C		XGPIO_DEF3('C', 1, 11)
#define GPIO_ADC_PCB_NTC		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_EXT_NTC		XGPIO_DEF3('A', 0, 0)

#define GPIO_ADC_INTERNAL_TEMP		XGPIO_DEF3('J', 0, 0)

void ADC_irq_lock();
void ADC_irq_unlock();

void ADC_startup();
float ADC_get_VALUE(int xGPIO);

extern void ADC_IRQ();

#endif /* _H_ADC_ */

