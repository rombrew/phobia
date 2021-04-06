#include "hal.h"
#include "cmsis/stm32xx.h"

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

	GPIO->BSRR = (1UL << N);
}

void GPIO_set_LOW(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	GPIO->BSRR = (1UL << (N + 16));
}

int GPIO_get_VALUE(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	return (GPIO->IDR & (1UL << N)) ? 1 : 0;
}

int GPIO_get_HALL()
{
	GPIO_TypeDef	*GPIO = (GPIO_TypeDef *) (GPIOA_BASE + 0x0400 * XGPIO_GET_PORT(GPIO_HALL_A));
	int		IDR, HALL;

	IDR = GPIO->IDR;

	HALL  = (IDR & (1UL << XGPIO_GET_N(GPIO_HALL_A))) ? LEG_A : 0UL;
	HALL |= (IDR & (1UL << XGPIO_GET_N(GPIO_HALL_B))) ? LEG_B : 0UL;
	HALL |= (IDR & (1UL << XGPIO_GET_N(GPIO_HALL_C))) ? LEG_C : 0UL;

	return HALL;
}

