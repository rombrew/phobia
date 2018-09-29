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

#define GPIO_CAN_RX			XGPIO_DEF4('B', 8, 0, 9)
#define GPIO_CAN_TX			XGPIO_DEF4('B', 9, 0, 9)

void irqCAN1_TX() { }

static void
irqCAN1_RX(int fifo)
{
	unsigned long		temp;

	hal.CAN_msg_ID = (unsigned long) (CAN1->sFIFOMailBox[fifo].RIR >> 21);
	hal.CAN_msg_len = (int) (CAN1->sFIFOMailBox[fifo].RDTR & 0xFUL);

	temp = CAN1->sFIFOMailBox[fifo].RDLR;

	hal.CAN_msg_payload[0] = (unsigned char) (temp & 0xFFUL);
	hal.CAN_msg_payload[1] = (unsigned char) ((temp >> 8) & 0xFFUL);
	hal.CAN_msg_payload[2] = (unsigned char) ((temp >> 16) & 0xFFUL);
	hal.CAN_msg_payload[3] = (unsigned char) ((temp >> 24) & 0xFFUL);

	if (hal.CAN_msg_len > 4) {

		temp = CAN1->sFIFOMailBox[fifo].RDHR;

		hal.CAN_msg_payload[4] = (unsigned char) (temp & 0xFFUL);
		hal.CAN_msg_payload[5] = (unsigned char) ((temp >> 8) & 0xFFUL);
		hal.CAN_msg_payload[6] = (unsigned char) ((temp >> 16) & 0xFFUL);
		hal.CAN_msg_payload[7] = (unsigned char) ((temp >> 24) & 0xFFUL);
	}

	if (fifo == 0) CAN1->RF0R |= CAN_RF0R_RFOM0;
	else CAN1->RF1R |= CAN_RF1R_RFOM1;

	CAN_IRQ();
}

void irqCAN1_RX0() { irqCAN1_RX(0); }
void irqCAN1_RX1() { irqCAN1_RX(1); }
void irqCAN1_SCE() { }

void CAN_startup()
{
	int		INAK, N = 0;

	/* Enable CAN1 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

	/* Enable CAN1 pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_CAN_RX);
	GPIO_set_mode_FUNCTION(GPIO_CAN_TX);

	/* Mode Initialization.
	 * */
	CAN1->MCR = CAN_MCR_ABOM | CAN_MCR_NART | CAN_MCR_INRQ;

	do {
		INAK = CAN1->MSR & CAN_MSR_INAK;
		N++;
	}
	while (INAK == 0 && N < 70000UL);

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

	/* Go to Normal mode.
	 * */
	CAN1->MCR &= ~CAN_MCR_INRQ;
}

void CAN_set_filter(int nfilt, int fifo, unsigned long ID, unsigned long mID)
{
	unsigned long	bfilt = (1UL << nfilt);

	CAN1->FMR |= CAN_FMR_FINIT;

	CAN1->FM1R &= ~bfilt;
	CAN1->FS1R |= bfilt;

	CAN1->FFA1R &= ~bfilt;
	CAN1->FFA1R |= (fifo == 1) ? bfilt : 0UL;

	CAN1->sFilterRegister[nfilt].FR1 = (ID << 21);
	CAN1->sFilterRegister[nfilt].FR2 = (mID << 21) + 6UL;

	CAN1->FA1R |= bfilt;

	CAN1->MCR &= ~CAN_MCR_INRQ;
}

void CAN_send_msg(unsigned long ID, int len, const unsigned char payload[8])
{
	int		mailbox;

	if (CAN1->TSR & CAN_TSR_TME0)
		mailbox = 0;
	else if (CAN1->TSR & CAN_TSR_TME1)
		mailbox = 1;
	else if (CAN1->TSR & CAN_TSR_TME2)
		mailbox = 2;
	else {
		/* Message is discarded.
		 * */
		return ;
	}

	CAN1->sTxMailBox[mailbox].TIR = (ID << 21);
	CAN1->sTxMailBox[mailbox].TDTR = len;

	CAN1->sTxMailBox[mailbox].TDLR =
		((unsigned long) payload[0])
		| ((unsigned long) payload[1] << 8)
		| ((unsigned long) payload[2] << 16)
		| ((unsigned long) payload[3] << 24);

	if (len > 4) {

		CAN1->sTxMailBox[mailbox].TDHR =
			((unsigned long) payload[4])
			| ((unsigned long) payload[5] << 8)
			| ((unsigned long) payload[6] << 16)
			| ((unsigned long) payload[7] << 24);
	}

	CAN1->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;
}

