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

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32f4xx.h"
#include "hal.h"

#define GPIO_USART_TX			XGPIO_DEF4('C', 10, 0, 7)
#define GPIO_USART_RX			XGPIO_DEF4('C', 11, 0, 7)

typedef struct {

	QueueHandle_t		xRX;
	QueueHandle_t		xTX;
}
HAL_USART_t;

static HAL_USART_t		hal_USART;

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

void USART_startup()
{
	/* Enable USART3 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	/* Enable USART3 pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_USART_TX);
	GPIO_set_mode_FUNCTION(GPIO_USART_RX);

	/* Alloc queues.
	 * */
	hal_USART.xRX = xQueueCreate(20, sizeof(char));
	hal_USART.xTX = xQueueCreate(40, sizeof(char));

	/* Configure USART.
	 * */
	USART3->BRR = CLOCK_APB1_HZ / hal.USART_baud_rate;
	USART3->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE
		| USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	USART3->CR2 = 0;
	USART3->CR3 = 0;

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(USART3_IRQn, 11);
	NVIC_EnableIRQ(USART3_IRQn);
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
	while ((USART3->SR & USART_SR_TXE) == 0) ;

	USART3->DR = c;
}

