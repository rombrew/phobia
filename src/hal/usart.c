/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

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

halUSART_t	 		halUSART;

void irqUSART3() { }

void irqDMA1_Stream3()
{
	DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
}

void usartEnable()
{
	/* Enable USART3 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	/* Enable DMA1 clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	/* Enable PC10 (TX), PC11 (RX) pins.
	 * */
	MODIFY_REG(GPIOC->AFR[1], (15UL << 8) | (15UL << 12),
			(7UL << 8) | (7UL << 12));
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER10 | GPIO_MODER_MODER11,
			GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);

	/* Configure USART.
	 * */
	USART3->BRR = HZ_APB1 / halUSART.baudRate;
	USART3->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE
		| USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	USART3->CR2 = 0;
	USART3->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;

	/* Flush RX buffer.
	 * */
	halUSART.rN = 0;

	/* Configure DMA for RX.
	 * */
	DMA1->LIFCR |= DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1
		| DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
	DMA1_Stream1->PAR = (unsigned int) &USART3->DR;
	DMA1_Stream1->M0AR = (unsigned int) halUSART.RX;
	DMA1_Stream1->NDTR = USART_RXBUF_SZ;
	DMA1_Stream1->FCR = 0;
	DMA1_Stream1->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_PL_0 | DMA_SxCR_MINC
		| DMA_SxCR_CIRC;

	DMA1_Stream1->CR |= DMA_SxCR_EN;

	/* Configure DMA for TX.
	 * */
	DMA1->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3
		| DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
	DMA1_Stream3->PAR = (unsigned int) &USART3->DR;
	DMA1_Stream3->M0AR = (unsigned int) halUSART.TX;
	DMA1_Stream3->NDTR = 0;
	DMA1_Stream3->FCR = 0;
	DMA1_Stream3->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_PL_0 | DMA_SxCR_MINC
		| DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;

	/* Enable IRQs.
	 * */
	NVIC_SetPriority(DMA1_Stream3_IRQn, 11);
	NVIC_SetPriority(USART3_IRQn, 11);
	NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	NVIC_EnableIRQ(USART3_IRQn);
}

void uartDisable()
{
	/* Disable IRQs.
	 * */
	NVIC_DisableIRQ(DMA1_Stream3_IRQn);
	NVIC_DisableIRQ(USART3_IRQn);

	/* Disable DMA.
	 * */
	DMA1_Stream1->CR = 0;

	/* Disable USART.
	 * */
	USART3->CR1 = 0;

	/* Disable PC10, PC11 pins.
	 * */
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER10 | GPIO_MODER_MODER11, 0);

	/* Disable DMA1 clock.
	 * */
	RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;

	/* Disable USART3 clock.
	 * */
	RCC->AHB1ENR &= ~RCC_APB1ENR_USART3EN;
}

int usartRecv()
{
	int	rN, rW, xC;

	rN = halUSART.rN;
	rW = USART_RXBUF_SZ - DMA1_Stream1->NDTR;

	if (rN != rW) {

		/* There are data.
		 * */
		xC = halUSART.RX[rN];
		halUSART.rN = (rN < (USART_RXBUF_SZ - 1)) ? rN + 1 : 0;
	}
	else
		xC = -1;

	return xC;
}

int usartPoll()
{
	return (DMA1_Stream3->CR & DMA_SxCR_EN) ? 0 : 1;
}

void usartPushAll(int N)
{
	DMA1->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3
		| DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
	DMA1_Stream3->NDTR = N;
	DMA1_Stream3->CR |= DMA_SxCR_EN;
}

