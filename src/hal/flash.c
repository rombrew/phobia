#include <stddef.h>

#include "hal.h"
#include "ifcan.h"
#include "libc.h"

#include "cmsis/stm32xx.h"

#if defined(STM32F4)

const FLASH_config_t	FLASH_config = { 8, 4 };

const u32_t FLASH_map[] = {

	0x08080000UL,
	0x080A0000UL,
	0x080C0000UL,
	0x080E0000UL,
	0x08100000UL
};

#elif defined(STM32F7)

const FLASH_config_t	FLASH_config = { 6, 2 };

const u32_t FLASH_map[] = {

	0x08040000UL,
	0x08060000UL,
	0x08080000UL
};

#endif /* STM32Fx */

static void
FLASH_unlock()
{
	if (FLASH->CR & FLASH_CR_LOCK) {

		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
}

static void
FLASH_lock()
{
	FLASH->CR |= FLASH_CR_LOCK;
}

static void
FLASH_wait_BSY()
{
	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

		__NOP();
	}
}

LD_RAMFUNC static void
FLASH_erase_on_IWDG(int N)
{
	/* Disable all IRQs to be sure that no code execution from flash will
	 * occur while erase operation is in progress.
	 * */
	__disable_irq();

	__DSB();
	__ISB();

	FLASH->CR = FLASH_CR_PSIZE_1 | (N << 3)
		| FLASH_CR_SER | FLASH_CR_STRT;

	__DSB();

	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

		/* Kick IWDG during a long wait.
		 * */
		IWDG->KR = 0xAAAA;

		__NOP();
	}

	FLASH->CR = 0;

	__DSB();

	__enable_irq();
}

#ifdef HW_HAVE_NETWORK_CAN

#define self_INQ_SIZE		30

LD_RAMFUNC static void
self_IFCAN_node_ACK(int node_ACK)
{
	CAN_msg_t		msg;
	int			mb;

	msg.ID = IFCAN_ID(net.node_ID, IFCAN_NODE_ACK);
	msg.len = 2;

	msg.payload[0] = net.node_ID;
	msg.payload[1] = node_ACK;

	mb = (CAN1->TSR & CAN_TSR_CODE_Msk) >> CAN_TSR_CODE_Pos;

	CAN1->sTxMailBox[mb].TIR = (u32_t) msg.ID << 21;
	CAN1->sTxMailBox[mb].TDTR = (u32_t) msg.len;
	CAN1->sTxMailBox[mb].TDLR = ((u32_t) msg.payload[0]) | ((u32_t) msg.payload[1] << 8);

	CAN1->sTxMailBox[mb].TIR |= CAN_TI0R_TXRQ;
}

LD_RAMFUNC static u32_t
self_crc32a(const void *s, int n)
{
	const u8_t		*bs = (const u8_t *) s;
	u32_t			crc, mask;

	crc = 0xFFFFFFFFUL;

	while (n >= 1) {

		int		j;

		crc = crc ^ (u32_t) (*bs++);

		for (j = 0; j < 8; ++j) {

			mask = - (crc & 1UL);
			crc = (crc >> 1) ^ (0xEDB88320UL & mask);
		}

		n += - 1;
	}

	return crc ^ 0xFFFFFFFFUL;
}

LD_RAMFUNC static void
self_futile_on_IWDG(int us)
{
	u32_t			xVAL, xLOAD;
	int			elapsed, hold;

	xVAL = SysTick->VAL;
	xLOAD = SysTick->LOAD + 1UL;

	hold = us * (clock_cpu_hz / 1000000UL);

	do {
		elapsed = (int) (xVAL - SysTick->VAL);
		elapsed += (elapsed < 0) ? xLOAD : 0;

		if (elapsed >= hold)
			break;

		/* Kick IWDG.
		 * */
		IWDG->KR = 0xAAAA;

		__NOP();
	}
	while (1);
}

LD_RAMFUNC static void
self_flash_prog(void *flash, u32_t value)
{
	u32_t			*long_flash = flash;

	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

		/* Kick IWDG.
		 * */
		IWDG->KR = 0xAAAA;

		__NOP();
	}

	/* Program flash memory.
	 * */
	*long_flash = value;

	__DSB();

#ifdef STM32F7

	/* D-Cache Clean and Invalidate.
	 * */
	SCB->DCCIMVAC = (u32_t) long_flash;

	__DSB();
	__ISB();

#endif /* STM32F7 */
}

LD_RAMFUNC static void
FLASH_selfupdate_on_IWDG(u32_t INIT_sizeof, u32_t INIT_crc32)
{
	struct {

		CAN_msg_t		MSG[self_INQ_SIZE];
		int			wp, rp;
	}
	queue_IN;

	u32_t			*flash, accepted;
	int			paused, length, N;

	/* Disable all IRQs to be sure that no code execution from flash will
	 * occur while flash operations is in progress.
	 * */
	__disable_irq();

	__DSB();
	__ISB();

	self_IFCAN_node_ACK(IFCAN_ACK_FLASH_WAIT_FOR_ERASE);

	for (N = 0; N < FLASH_config.s_first; ++N) {

		FLASH->CR = FLASH_CR_PSIZE_1 | (N << 3)
			| FLASH_CR_SER | FLASH_CR_STRT;

		__DSB();

		while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

			/* Kick IWDG.
			 * */
			IWDG->KR = 0xAAAA;

			__NOP();
		}
	}

	FLASH->CR = 0;

	self_IFCAN_node_ACK(IFCAN_ACK_FLASH_DATA_ACCEPT);

	/* Get the flash upload ADDRESS.
	 * */
	flash = (u32_t *) &ld_begin_vectors;

	queue_IN.wp = 0;
	queue_IN.rp = 0;

	accepted = 0UL;
	paused = 0;

	FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;

	do {
		while ((CAN1->RF0R & CAN_RF0R_FMP0_Msk) != 0) {

			CAN_msg_t	*msg;
			u32_t		xLO, xHI;

			msg = &queue_IN.MSG[queue_IN.wp];

			msg->ID = (u16_t) (CAN1->sFIFOMailBox[0].RIR >> 21);
			msg->len = (u16_t) (CAN1->sFIFOMailBox[0].RDTR & 0xFUL);

			if (		msg->ID == IFCAN_ID_FLASH_DATA
					&& msg->len == 8) {

				queue_IN.wp = (queue_IN.wp < self_INQ_SIZE - 1) ? queue_IN.wp + 1 : 0;

				xLO = CAN1->sFIFOMailBox[0].RDLR;
				xHI = CAN1->sFIFOMailBox[0].RDHR;

				msg->payload[0] = (u8_t) (xLO & 0xFFUL);
				msg->payload[1] = (u8_t) ((xLO >> 8) & 0xFFUL);
				msg->payload[2] = (u8_t) ((xLO >> 16) & 0xFFUL);
				msg->payload[3] = (u8_t) ((xLO >> 24) & 0xFFUL);
				msg->payload[4] = (u8_t) (xHI & 0xFFUL);
				msg->payload[5] = (u8_t) ((xHI >> 8) & 0xFFUL);
				msg->payload[6] = (u8_t) ((xHI >> 16) & 0xFFUL);
				msg->payload[7] = (u8_t) ((xHI >> 24) & 0xFFUL);
			}

			CAN1->RF0R |= CAN_RF0R_RFOM0;

			__DSB();
			__ISB();
		}

		if (queue_IN.rp != queue_IN.wp) {

			CAN_msg_t	*msg;

			msg = &queue_IN.MSG[queue_IN.rp];
			queue_IN.rp = (queue_IN.rp < self_INQ_SIZE - 1) ? queue_IN.rp + 1 : 0;

			/* Program flash memory.
			 * */
			self_flash_prog(flash + 0, * (u32_t *) &msg->payload[0]);
			self_flash_prog(flash + 1, * (u32_t *) &msg->payload[4]);

			flash += 2UL;
			accepted += 8UL;
		}

		if (paused == 0) {

			/* The number of available places in the queue.
			 * */
			length = queue_IN.rp - queue_IN.wp + self_INQ_SIZE - 1;
			length += (length > self_INQ_SIZE - 1) ? - self_INQ_SIZE : 0;

			if (length < 5) {

				paused = 1;

				self_IFCAN_node_ACK(IFCAN_ACK_FLASH_DATA_PAUSE);
			}
		}
		else {
			/* The number of messages in the queue.
			 * */
			length = queue_IN.wp - queue_IN.rp;
			length += (length < 0) ? self_INQ_SIZE : 0;

			if (length < 5) {

				paused = 0;

				self_IFCAN_node_ACK(IFCAN_ACK_FLASH_DATA_ACCEPT);
			}
		}

		if (accepted >= INIT_sizeof) {

			/* Program CRC32.
			 * */
			self_flash_prog(flash, INIT_crc32);

			while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

				/* Kick IWDG.
				 * */
				IWDG->KR = 0xAAAA;

				__NOP();
			}

			/* All is done.
			 * */
			break;
		}

		/* Kick IWDG.
		 * */
		IWDG->KR = 0xAAAA;

		__NOP();
	}
	while (1);

	FLASH->CR = 0;
	FLASH->CR |= FLASH_CR_LOCK;

	__DSB();
	__ISB();

	if (self_crc32a((const void *) &ld_begin_vectors, INIT_sizeof) == INIT_crc32) {

		self_IFCAN_node_ACK(IFCAN_ACK_FLASH_SELFUPDATE_DONE);
	}
	else {
		self_IFCAN_node_ACK(IFCAN_ACK_FLASH_CRC32_INVALID);
	}

	/* Wait until all messages are transfered.
	 * */
	self_futile_on_IWDG(10);

	/* Request a system RESET.
	 * */
	SCB->AIRCR = ((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)
			| (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk)
			| SCB_AIRCR_SYSRESETREQ_Msk);

	__DSB();

	while (1) { __NOP(); }
}

#endif /* HW_HAVE_NETWORK_CAN */

void *FLASH_erase(void *flash)
{
	int		N, sector_N = 0;

	for (N = 0; N < FLASH_config.s_total; ++N) {

		if (		(u32_t) flash >= FLASH_map[N]
				&& (u32_t) flash < FLASH_map[N + 1]) {

			flash = (void *) FLASH_map[N];
			sector_N = N + FLASH_config.s_first;

			break;
		}
	}

	if (sector_N != 0) {

		FLASH_unlock();
		FLASH_wait_BSY();

		/* Call the func from RAM because flash will busy.
		 * */
		FLASH_erase_on_IWDG(sector_N);

		FLASH_lock();

#if defined(STM32F4)

		/* Reset D-Cache.
		 * */
		FLASH->ACR &= ~FLASH_ACR_DCEN;
		FLASH->ACR |= FLASH_ACR_DCRST;
		FLASH->ACR &= ~FLASH_ACR_DCRST;
		FLASH->ACR |= FLASH_ACR_DCEN;

#elif defined(STM32F7)

		/* Invalidate D-Cache on the erased sector.
		 * */
		SCB_InvalidateDCacheByAddr((void *) FLASH_map[N], FLASH_map[N + 1] - FLASH_map[N]);

#endif /* STM32Fx */
	}

	return flash;
}

void FLASH_prog(void *flash, u32_t value)
{
	u32_t			*long_flash = flash;

	if (		(u32_t) long_flash >= fw.ld_end
			&& (u32_t) long_flash < FLASH_map[FLASH_config.s_total]) {

		FLASH_unlock();
		FLASH_wait_BSY();

		FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;

		/* Program flash memory.
		 * */
		*long_flash = value;

		__DSB();

#ifdef STM32F7
		/* D-Cache Clean and Invalidate.
		 * */
		SCB->DCCIMVAC = (u32_t) long_flash;

		__DSB();
		__ISB();

#endif /* STM32F7 */

		FLASH_wait_BSY();

		FLASH->CR = 0;

		FLASH_lock();
	}
}

void FLASH_selfupdate_CAN(u32_t INIT_sizeof, u32_t INIT_crc32)
{
#ifdef HW_HAVE_NETWORK_CAN

	FLASH_unlock();
	FLASH_wait_BSY();

	/* Call the func from RAM because flash will be erased.
	 * */
	FLASH_selfupdate_on_IWDG(INIT_sizeof, INIT_crc32);

#endif /* HW_HAVE_NETWORK_CAN */
}

