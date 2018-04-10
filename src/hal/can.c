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

#include "cmsis/stm32f4xx.h"
#include "hal.h"

#define CAN_TIMEOUT			70000UL

void irqCAN1_TX()
{
}

void irqCAN1_RX0()
{
}

void irqCAN1_RX1()
{
}

void irqCAN1_SCE()
{
}

void canEnable()
{
	int		INAK, N = 0;

	/* Enable CAN1 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

	/* Enable PA11 (RX), PA12 (TX) pins.
	 * */
	MODIFY_REG(GPIOA->AFR[1], (15UL << 12) | (15UL << 16),
			(9UL << 12) | (9UL << 16));
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER11 | GPIO_MODER_MODER12,
			GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);

	/* Mode initialization.
	 * */
	CAN1->MCR = CAN_MCR_INRQ;

	do {
		INAK = CAN1->MSR & CAN_MSR_INAK;
		N++;
	}
	while (!INAK && N < CAN_TIMEOUT);

	CAN1->IER = CAN_IER_FMPIE0 | CAN_IER_FMPIE1;
	CAN1->BTR = (5UL << 20) | (6UL << 16) | (2UL);

	/* Enable IRQs.
	 * */
	NVIC_SetPriority(CAN1_RX0_IRQn, 7);
	NVIC_SetPriority(CAN1_RX1_IRQn, 7);
	NVIC_SetPriority(CAN1_SCE_IRQn, 7);
	NVIC_EnableIRQ(CAN1_RX0_IRQn);
	NVIC_EnableIRQ(CAN1_RX1_IRQn);
	NVIC_EnableIRQ(CAN1_SCE_IRQn);

	/* Mode normal.
	 * */
	CAN1->MCR &= ~CAN_MCR_INRQ;
}

void canDisable()
{
	/* Disable IRQs.
	 * */
	NVIC_DisableIRQ(CAN1_RX0_IRQn);
	NVIC_DisableIRQ(CAN1_RX1_IRQn);
	NVIC_DisableIRQ(CAN1_SCE_IRQn);

	/* Disable pins.
	 * */
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER11 | GPIO_MODER_MODER12, 0);

	/* Disable CAN1 clock.
	 * */
	RCC->APB1ENR &= ~RCC_APB1ENR_CAN1EN;
}

void canFilter(int nFilter, int bID, int bMask, int nFifo)
{
	int		BIT;

	CAN1->FMR |= CAN_FMR_FINIT;

	BIT = 1UL << nFilter;
	CAN1->FA1R &= ~BIT;

	if (bID != 0) {

		CAN1->FM1R &= ~BIT;
		CAN1->FS1R |= BIT;

		CAN1->FFA1R &= ~BIT;
		CAN1->FFA1R |= (nFifo == 1) ? BIT : 0;

		CAN1->sFilterRegister[nFilter].FR1 = (bID << 21);
		CAN1->sFilterRegister[nFilter].FR2 = (bMask << 21);

		CAN1->FA1R |= BIT;
	}

	CAN1->FMR &= ~CAN_FMR_FINIT;
}

int canEmptyMailBox()
{
	int		nEmpty = 0;

	if (CAN1->TSR & CAN_TSR_TME0)
		nEmpty++;
	if (CAN1->TSR & CAN_TSR_TME1)
		nEmpty++;
	if (CAN1->TSR & CAN_TSR_TME2)
		nEmpty++;

	return nEmpty;
}

int canTransmit(int bID, int nBytes, const char bData[8])
{
	int		nMailBox;

	if (CAN1->TSR & CAN_TSR_TME0)
		nMailBox = 0;
	else if (CAN1->TSR & CAN_TSR_TME1)
		nMailBox = 1;
	else if (CAN1->TSR & CAN_TSR_TME2)
		nMailBox = 2;
	else
		return -1;

	CAN1->sTxMailBox[nMailBox].TIR = (bID << 21);
	CAN1->sTxMailBox[nMailBox].TDTR = nBytes;

	CAN1->sTxMailBox[nMailBox].TDLR =
		((unsigned int) bData[0])
		| ((unsigned int) bData[1] << 8)
		| ((unsigned int) bData[2] << 16)
		| ((unsigned int) bData[3] << 24);

	CAN1->sTxMailBox[nMailBox].TDHR =
		((unsigned int) bData[4])
		| ((unsigned int) bData[5] << 8)
		| ((unsigned int) bData[6] << 16)
		| ((unsigned int) bData[7] << 24);

	CAN1->sTxMailBox[nMailBox].TIR |= CAN_TI1R_TXRQ;

	return 0;
}

int canMsgSend(int bID, int nBytes, const char bData[8], int xWay)
{
	/*int		nEmpty;

	nEmpty = canEmptyMailBox();*/

	return 0;
}

