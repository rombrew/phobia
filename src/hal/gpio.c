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

#include "cmsis/stm32f4xx.h"
#include "hal.h"

#define XGPIO_DECODE(xGPIO)	\
	GPIO_TypeDef	*GPIO = (GPIO_TypeDef *) (GPIOA_BASE + 0x0400 * XGPIO_GET_PORT(xGPIO)); \
	int		N = XGPIO_GET_N(xGPIO);

void GPIO_set_mode_INPUT(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	CLEAR_BIT(GPIO->MODER, 3UL << (N * 2));
}

void GPIO_set_mode_OUTPUT(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->MODER, 3UL << (N * 2), 1UL << (N * 2));
}

void GPIO_set_mode_ANALOG(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->MODER, 3UL << (N * 2), 3UL << (N * 2));
}

void GPIO_set_mode_FUNCTION(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->AFR[(N > 7) ? 1 : 0], 15UL << ((N & 7) * 4),
			(XGPIO_GET_FUNC(xGPIO) & 0xF) << ((N & 7) * 4));

	MODIFY_REG(GPIO->MODER, 3UL << (N * 2), 2UL << (N * 2));
}

void GPIO_set_mode_PUSH_PULL(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	GPIO->OTYPER &= ~(1UL << N);
}

void GPIO_set_mode_OPEN_DRAIN(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	GPIO->OTYPER |= (1UL << N);
}

void GPIO_set_mode_SPEED_LOW(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	CLEAR_BIT(GPIO->OSPEEDR, 3UL << (N * 2));
}

void GPIO_set_mode_SPEED_HIGH(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->OSPEEDR, 3UL << (N * 2), 1UL << (N * 2));
}

void GPIO_set_mode_PULL_NONE(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	CLEAR_BIT(GPIO->PUPDR, 3UL << (N * 2));
}

void GPIO_set_mode_PULL_UP(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->PUPDR, 3UL << (N * 2), 1UL << (N * 2));
}

void GPIO_set_mode_PULL_DOWN(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->PUPDR, 3UL << (N * 2), 2UL << (N * 2));
}

void GPIO_set_HIGH(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	GPIO->BSRRL = (1UL << N);
}

void GPIO_set_LOW(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	GPIO->BSRRH = (1UL << N);
}

int GPIO_get_VALUE(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	return (GPIO->IDR & (1UL << N)) ? 1 : 0;
}

