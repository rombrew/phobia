#include "hal.h"
#include "cmsis/stm32xx.h"

#define XGPIO_DECODE(xGPIO)	\
	GPIO_TypeDef	*GPIO = (GPIO_TypeDef *) (GPIOA_BASE		\
				+ 0x0400U * XGPIO_GET_PORT(xGPIO));	\
	int		N = XGPIO_GET_N(xGPIO);

void GPIO_set_mode_INPUT(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	CLEAR_BIT(GPIO->MODER, 3U << (N * 2));
}

void GPIO_set_mode_OUTPUT(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->MODER, 3U << (N * 2), 1U << (N * 2));

	if (xGPIO & XGPIO_OPEN_DRAIN) {

		GPIO->OTYPER |= (1U << N);
	}
}

void GPIO_set_mode_ANALOG(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->MODER, 3U << (N * 2), 3U << (N * 2));
}

void GPIO_set_mode_FUNCTION(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->AFR[(N > 7) ? 1 : 0], 15U << ((N & 7) * 4),
			(XGPIO_GET_FUNC(xGPIO) & 0xF) << ((N & 7) * 4));

	MODIFY_REG(GPIO->MODER, 3U << (N * 2), 2U << (N * 2));
}

void GPIO_set_mode_PUSH_PULL(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	GPIO->OTYPER &= ~(1U << N);
}

void GPIO_set_mode_OPEN_DRAIN(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	GPIO->OTYPER |= (1U << N);
}

void GPIO_set_mode_SPEED_LOW(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	CLEAR_BIT(GPIO->OSPEEDR, 3U << (N * 2));
}

void GPIO_set_mode_SPEED_HIGH(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->OSPEEDR, 3U << (N * 2), 1U << (N * 2));
}

void GPIO_set_mode_SPEED_FAST(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->OSPEEDR, 3U << (N * 2), 2U << (N * 2));
}

void GPIO_set_mode_PULL_NONE(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	CLEAR_BIT(GPIO->PUPDR, 3U << (N * 2));
}

void GPIO_set_mode_PULL_UP(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->PUPDR, 3U << (N * 2), 1U << (N * 2));
}

void GPIO_set_mode_PULL_DOWN(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	MODIFY_REG(GPIO->PUPDR, 3U << (N * 2), 2U << (N * 2));
}

void GPIO_set_HIGH(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	if ((xGPIO & XGPIO_OPEN_DRAIN) == 0U) {

		GPIO->BSRR = (1U << N);
	}
	else {
		GPIO->BSRR = (1U << (N + 16));
	}
}

void GPIO_set_LOW(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	if ((xGPIO & XGPIO_OPEN_DRAIN) == 0U) {

		GPIO->BSRR = (1U << (N + 16));
	}
	else {
		GPIO->BSRR = (1U << N);
	}
}

int GPIO_get_STATE(int xGPIO)
{
	XGPIO_DECODE(xGPIO);

	return (GPIO->IDR & (1U << N)) ? 1 : 0;
}

