/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2014 Roman Belov <romblv@gmail.com>

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

#define UART_RXBUF_SZ		12

typedef struct {

	char		rBuf[UART_RXBUF_SZ];
	int		rN;
}
halUART_TypeDef;

static halUART_TypeDef		halUART;

void irqUSART3() { }

void uartEnable(int baudR)
{
	/* Enable USART3 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	/* Enable DMA1 clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	/* Enable PC10 (TX) and PC11 (RX) pins.
	 * */
	MODIFY_REG(GPIOC->AFR[1], (15UL << 8) | (15UL << 12),
			(7UL << 8) | (7UL << 12));
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER10 | GPIO_MODER_MODER11,
			GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);

	/* Configure USART.
	 * */
	USART3->BRR = FREQ_APB1_HZ / baudR;
	USART3->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE
		| USART_CR1_TE | USART_CR1_RE;
	USART3->CR2 = 0;
	USART3->CR3 = USART_CR3_DMAR;

	/* Configure DMA for RX.
	 * */
	DMA1->LIFCR |= DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1
		| DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
	DMA1_Stream1->PAR = (unsigned int) &USART3->DR;
	DMA1_Stream1->M0AR = (unsigned int) halUART.rBuf;
	DMA1_Stream1->NDTR = UART_RXBUF_SZ;
	DMA1_Stream1->FCR = 0;
	DMA1_Stream1->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_PL_0 | DMA_SxCR_MINC
		| DMA_SxCR_CIRC;

	DMA1_Stream1->CR |= DMA_SxCR_EN;

	/* Enable IRQ.
	 * */
	/*NVIC_SetPriority(USART3_IRQn, 1);
	NVIC_EnableIRQ(USART3_IRQn);*/
}

void uartDisable()
{
	/* Disable IRQ.
	 * */
	/*NVIC_DisableIRQ(USART3_IRQn);*/

	/* Disable DMA.
	 * */
	DMA1_Stream1->CR = 0;

	/* Disable USART.
	 * */
	USART3->CR1 = 0;

	/* Disable PC10 and PC11 pins.
	 * */
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER10 | GPIO_MODER_MODER11, 0);

	/* Disable DMA1 clock.
	 * */
	RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;

	/* Disable USART3 clock.
	 * */
	RCC->AHB1ENR &= ~RCC_APB1ENR_USART3EN;
}

int uartReceive()
{
	int	rN, rW, xC;

	rN = halUART.rN;
	rW = UART_RXBUF_SZ - DMA1_Stream1->NDTR;

	if (rN != rW) {

		/* There are data.
		 * */
		xC = halUART.rBuf[rN];
		halUART.rN = (rN < (UART_RXBUF_SZ - 1)) ? rN + 1 : 0;
	}
	else
		xC = -1;

	return xC;
}

void uartSend(char xC)
{
	/* Wait for TXE.
	 * */
	while (!(USART3->SR & USART_SR_TXE))
		__WFI();

	USART3->DR = xC;
}

