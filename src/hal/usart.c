/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32f4xx.h"
#include "hal.h"

typedef struct {

	QueueHandle_t	xRX;
	QueueHandle_t	xTX;
}
HAL_USART_t;

static HAL_USART_t	hal_USART;

void irqUSART3()
{
	BaseType_t		xWoken = pdFALSE;
	unsigned int 		SR;
	char			xC;

	SR = USART3->SR;

	if (SR & USART_SR_RXNE) {

		xC = USART3->DR;
		xQueueSendToBackFromISR(hal_USART.xRX, &xC, &xWoken);
	}

	if (SR & USART_SR_TXE) {

		if (xQueueReceiveFromISR(hal_USART.xTX, &xC, &xWoken) == pdTRUE) {

			USART3->DR = xC;
		}
		else {
			USART3->CR1 &= ~USART_CR1_TXEIE;
		}
	}

	portYIELD_FROM_ISR(xWoken);
}

void USART_enable()
{
	/* Alloc queues.
	 */
	hal_USART.xRX = xQueueCreate(20, sizeof(char));
	hal_USART.xTX = xQueueCreate(40, sizeof(char));

	/* Enable USART3 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	/* Enable PC10 (TX), PC11 (RX) pins.
	 * */
	MODIFY_REG(GPIOC->AFR[1], (15UL << 8) | (15UL << 12),
			(7UL << 8) | (7UL << 12));
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER10 | GPIO_MODER_MODER11,
			GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);

	/* Configure USART.
	 * */
	USART3->BRR = HAL_APB1_HZ / hal.USART_baud_rate;
	USART3->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE
		| USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	USART3->CR2 = 0;
	USART3->CR3 = 0;

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(USART3_IRQn, 11);
	NVIC_EnableIRQ(USART3_IRQn);
}

void USART_disable()
{
	/* Disable IRQ.
	 * */
	NVIC_DisableIRQ(USART3_IRQn);

	/* Disable USART.
	 * */
	USART3->CR1 = 0;

	/* Disable PC10 PC11 pins.
	 * */
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER10 | GPIO_MODER_MODER11, 0);

	/* Disable USART3 clock.
	 * */
	RCC->AHB1ENR &= ~RCC_APB1ENR_USART3EN;

	/* Free queues.
	 * */
	vQueueDelete(hal_USART.xRX);
	vQueueDelete(hal_USART.xTX);
}

int USART_getc()
{
	char		xC;

	xQueueReceive(hal_USART.xRX, &xC, portMAX_DELAY);

	return (int) xC;
}

void USART_putc(int c)
{
	char		xC = (char) c;

	if (xQueueSendToBack(hal_USART.xTX, &xC, portMAX_DELAY) == pdTRUE) {

		USART3->CR1 |= USART_CR1_TXEIE;
	}
}

void USART_debug_putc(int c)
{
	while (!(USART3->SR & USART_SR_TXE)) ;

	USART3->DR = c;
}

