#include "hal.h"
#include "cmsis/stm32xx.h"

void irq_CAN1_TX() { }

static void
irq_CAN1_RX(int mb)
{
	hal.CAN_msg.ID = (uint16_t) (CAN1->sFIFOMailBox[mb].RIR >> CAN_RI0R_STID_Pos);
	hal.CAN_msg.len = (uint16_t) (CAN1->sFIFOMailBox[mb].RDTR & CAN_RDT0R_DLC_Msk);

	hal.CAN_msg.payload.l[0] = CAN1->sFIFOMailBox[mb].RDLR;
	hal.CAN_msg.payload.l[1] = CAN1->sFIFOMailBox[mb].RDHR;
}

void irq_CAN1_RX0()
{
	irq_CAN1_RX(0);

	CAN1->RF0R |= CAN_RF0R_RFOM0;

	CAN_IRQ();
}

void irq_CAN1_RX1()
{
	irq_CAN1_RX(1);

	CAN1->RF1R |= CAN_RF1R_RFOM1;

	CAN_IRQ();
}

void irq_CAN1_SCE() { }

void CAN_startup()
{
	/* Enable CAN1 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

	/* Enable CAN1 pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_CAN_RX);
	GPIO_set_mode_FUNCTION(GPIO_CAN_TX);
	GPIO_set_mode_SPEED_HIGH(GPIO_CAN_RX);
	GPIO_set_mode_SPEED_HIGH(GPIO_CAN_TX);

	/* Force a master RESET.
	 * */
	CAN1->MCR = CAN_MCR_RESET;

	/* Enable IRQs.
	 * */
	NVIC_SetPriority(CAN1_RX0_IRQn, 7);
	NVIC_SetPriority(CAN1_RX1_IRQn, 7);
	NVIC_EnableIRQ(CAN1_RX0_IRQn);
	NVIC_EnableIRQ(CAN1_RX1_IRQn);

	/* Configure CAN1.
	 * */
	CAN_configure();
}

static int
CAN_wait_MSR(uint32_t xBITS, uint32_t xSET)
{
	uint32_t		xMSR;
	int			N = 0;

	do {
		xMSR = CAN1->MSR & xBITS;

		if (xMSR == xSET) {

			return HAL_OK;
		}

		__NOP();

		N++;
	}
	while (N < 70000U);

	return HAL_FAULT;
}

void CAN_configure()
{
	int		BRP, TS1, TS2;

	/* Exit mode SLEEP.
	 * */
	CAN1->MCR &= ~CAN_MCR_SLEEP;

	if (CAN_wait_MSR(CAN_MSR_SLAK, 0) != HAL_OK) {

		log_TRACE("CAN no SLEEP fault" EOL);
	}

	/* Go to mode INIT.
	 * */
	CAN1->MCR |= CAN_MCR_INRQ;

	if (CAN_wait_MSR(CAN_MSR_INAK, CAN_MSR_INAK) != HAL_OK) {

		log_TRACE("CAN to INIT fault" EOL);
	}

	CAN1->MCR |= CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;

	/* Enable message pending IRQs.
	 * */
	CAN1->IER = CAN_IER_FMPIE0 | CAN_IER_FMPIE1;

#if defined(STM32F4)
	TS1 = 7;
	TS2 = 6;
#elif defined(STM32F7)
	TS1 = 9;
	TS2 = 8;
#endif /* STM32Fx */

	BRP = (CLOCK_APB1_HZ / hal.CAN_bitfreq + 5U) / (1U + TS1 + TS2);

	/* Update bit timing.
	 * */
	CAN1->BTR = ((TS2 - 1U) << CAN_BTR_TS2_Pos)
		| ((TS1 - 1U) << CAN_BTR_TS1_Pos) | (BRP - 1U);

	hal.CAN_bitfreq = CLOCK_APB1_HZ / BRP / (1U + TS1 + TS2);

#ifdef STM32F4
	CAN1->FMR |= CAN_FMR_FINIT;

	/* Enable all 28 filters to CAN1.
	 * */
	MODIFY_REG(CAN1->FMR, CAN_FMR_CAN2SB_Msk, 28U << CAN_FMR_CAN2SB_Pos);
#endif /* STM32F4 */

	CAN1->FMR &= ~CAN_FMR_FINIT;

	/* Go to mode NORMAL.
	 * */
	CAN1->MCR &= ~CAN_MCR_INRQ;

	if (CAN_wait_MSR(CAN_MSR_INAK, CAN_MSR_INAK) != HAL_OK) {

		log_TRACE("CAN to NORMAL fault" EOL);
	}
}

void CAN_bind_ID(int fs, int mb, int ID, int mask_ID)
{
	uint32_t		BFS = (1U << fs);

	CAN1->FMR |= CAN_FMR_FINIT;
	CAN1->FA1R &= ~BFS;

	if (ID != 0) {

		CAN1->FM1R &= ~BFS;
		CAN1->FS1R |= BFS;

		CAN1->FFA1R &= ~BFS;
		CAN1->FFA1R |= (mb == 1) ? BFS : 0U;

		CAN1->sFilterRegister[fs].FR1 = (ID << CAN_F0R1_FB21_Pos);
		CAN1->sFilterRegister[fs].FR2 = (mask_ID << CAN_F0R2_FB21_Pos) + 6U;

		CAN1->FA1R |= BFS;
	}

	CAN1->FMR &= ~CAN_FMR_FINIT;
}

int CAN_send_msg(const CAN_msg_t *msg)
{
	uint32_t		xTSR;
	int			mb, irq;

	irq = hal_lock_irq();

	xTSR = CAN1->TSR;

	if (likely(xTSR & CAN_TSR_TME_Msk)) {

		mb = (xTSR & CAN_TSR_CODE_Msk) >> CAN_TSR_CODE_Pos;
	}
	else {
		hal_unlock_irq(irq);

		return HAL_FAULT;
	}

	CAN1->sTxMailBox[mb].TIR = (uint32_t) msg->ID << CAN_TI0R_STID_Pos;
	CAN1->sTxMailBox[mb].TDTR = (uint32_t) msg->len;

	CAN1->sTxMailBox[mb].TDLR = msg->payload.l[0];
	CAN1->sTxMailBox[mb].TDHR = msg->payload.l[1];

	CAN1->sTxMailBox[mb].TIR |= CAN_TI0R_TXRQ;

	hal_unlock_irq(irq);

	return HAL_OK;
}

int CAN_errate()
{
	int		errate;

	errate =  ((CAN1->ESR & CAN_ESR_REC_Msk) >> (CAN_ESR_REC_Pos - 8U))
		| ((CAN1->ESR & CAN_ESR_TEC_Msk) >> (CAN_ESR_TEC_Pos - 3U))
		| ((CAN1->ESR & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos);

	return errate;
}

