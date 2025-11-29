#include <stddef.h>

#include "hal.h"
#include "libc.h"

#include "cmsis/stm32xx.h"

#define HAL_ENABLED		0xDAA92EE2U
#define HAL_LOG_INC(np)		(((np) < sizeof(log.text) - 1U) ? (np) + 1 : 0)

uint32_t			clock_cpu_hz;

HAL_t				hal;
LOG_t				log		LD_NOINIT;

typedef struct {

	uint32_t	bootload_jump;
	uint32_t	crystal_fault;
}
priv_HAL_t;

static volatile priv_HAL_t	noinit_HAL	LD_NOINIT;

LD_IRQ void irq_NMI()
{
	log_TRACE("IRQ NMI" EOL);

	if (RCC->CIR & RCC_CIR_CSSF) {

		noinit_HAL.crystal_fault = HAL_ENABLED;

		RCC->CIR |= RCC_CIR_CSSC;

		log_TRACE("HSE clock fault" EOL);
	}

	hal_system_reset();
}

LD_IRQ void irq_HardFault(void)
{
	uint32_t		*sp, lr;

	__asm volatile
	(
		 "	tst lr, #4		\n"
		 "	ite eq			\n"
		 "	mrseq %0, msp		\n"
		 "	mrsne %0, psp		\n"
		 "	mov %1, lr		\n"

		 : "=r" (sp), "=r" (lr) :: "memory"
	);

	log_TRACE("IRQ HardFault" EOL);

	log_TRACE(" SP   %8x" EOL, (uint32_t) sp);
	log_TRACE(" EX   %8x" EOL, (uint32_t) lr);

	log_TRACE(" R0   %8x" EOL, sp[0]);
	log_TRACE(" R1   %8x" EOL, sp[1]);
	log_TRACE(" R2   %8x" EOL, sp[2]);
	log_TRACE(" R3   %8x" EOL, sp[3]);
	log_TRACE(" R12  %8x" EOL, sp[4]);
	log_TRACE(" LR   %8x" EOL, sp[5]);
	log_TRACE(" PC   %8x" EOL, sp[6]);
	log_TRACE(" PSR  %8x" EOL, sp[7]);

	log_TRACE(" HFSR %8x" EOL, SCB->HFSR);
	log_TRACE(" CFSR %8x" EOL, SCB->CFSR);
	log_TRACE(" MFAR %8x" EOL, SCB->MMFAR);
	log_TRACE(" BFAR %8x" EOL, SCB->BFAR);

	hal_system_reset();
}

LD_IRQ void irq_MemoryFault()
{
	log_TRACE("IRQ MemoryFault" EOL);

	hal_system_reset();
}

LD_IRQ void irq_BusFault()
{
	log_TRACE("IRQ BusFault" EOL);

	hal_system_reset();
}

LD_IRQ void irq_UsageFault()
{
	log_TRACE("IRQ UsageFault" EOL);

	hal_system_reset();
}

LD_IRQ void irq_Default()
{
	log_TRACE("IRQ Default" EOL);

	hal_system_reset();
}

static void
mcu_identify()
{
	uint32_t		ID;

	ID = DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID_Msk;

	if (ID == 0x413U) {

		hal.MCU_ID = MCU_ID_STM32F405;

		if (* (volatile const uint32_t *) 0x400238E8U == 0x88000000U) {

			hal.MCU_ID = MCU_ID_GD32F405;
		}
	}
	else if (ID == 0x452U) {

		hal.MCU_ID = MCU_ID_STM32F722;
	}
	else {
		hal.MCU_ID = MCU_ID_UNKNOWN;

		log_TRACE("Unknown MCU ID %4x" EOL, ID);
	}
}

static void
core_startup()
{
	uint32_t	CLOCK, PLLQ, PLLP, PLLN, PLLM;

	SCB->VTOR = (uint32_t) &ld_text_begin;

	NVIC_SetPriorityGrouping(0U);

	/* Reset RCC.
	 * */
	MODIFY_REG(RCC->CR, RCC_CR_PLLI2SON | RCC_CR_PLLON | RCC_CR_CSSON
			| RCC_CR_HSEBYP | RCC_CR_HSEON, RCC_CR_HSION);

	RCC->PLLCFGR = 0;
	RCC->CFGR = 0;
	RCC->CIR = 0;

#ifdef STM32F7
	RCC->DCKCFGR1 = 0;
	RCC->DCKCFGR2 = 0;
#endif /* STM32F7 */

	if (		HW_CLOCK_CRYSTAL_HZ != 0U
			&& noinit_HAL.crystal_fault != HAL_ENABLED) {

		int		N = 0;

		/* Enable HSE.
		 * */
		RCC->CR |= RCC_CR_HSEON;

		/* Wait until HSE is ready.
		 * */
		do {
			if ((RCC->CR & RCC_CR_HSERDY) == RCC_CR_HSERDY)
				break;

			__NOP();

			if (N > 70000) {

				log_TRACE("HSE not ready" EOL);

				noinit_HAL.crystal_fault = HAL_ENABLED;
				break;
			}

			N++;
		}
		while (1);
	}

	/* Enable power interface clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	/* Select voltage regulator scale 1 mode.
	 * */
#if defined(STM32F4)
	if (hal.MCU_ID == MCU_ID_STM32F405) {

		PWR->CR |= PWR_CR_VOS;
	}
	else if (hal.MCU_ID == MCU_ID_GD32F405) {

		PWR->CR |= (1U << 15) | (1U << 14);	/* LDOVS */
	}
#elif defined(STM32F7)
	PWR->CR1 |= PWR_CR1_VOS_1 | PWR_CR1_VOS_0;
#endif /* STM32Fx */

	/* Set AHB/APB1/APB2 prescalers.
	 * */
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	if (		HW_CLOCK_CRYSTAL_HZ != 0U
			&& noinit_HAL.crystal_fault != HAL_ENABLED) {

		CLOCK = HW_CLOCK_CRYSTAL_HZ;

		/* Clock from HSE.
		 * */
		RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

		/* Enable CSS.
		 * */
		RCC->CR |= RCC_CR_CSSON;
	}
	else {
		CLOCK = 16000000U;

		/* Clock from HSI.
		 * */
		RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
	}

#if defined(STM32F4)
	clock_cpu_hz = 168000000U;
#elif defined(STM32F7)
	clock_cpu_hz = 216000000U;
#endif /* STM32Fx */

	PLLP = 2;

	PLLM = (CLOCK + 1999999U) / 2000000U;
	CLOCK /= PLLM;

	PLLN = (PLLP * clock_cpu_hz) / CLOCK;
	CLOCK *= PLLN;

	PLLQ = (CLOCK + 47999999U) / 48000000U;

	RCC->PLLCFGR |= (PLLQ << RCC_PLLCFGR_PLLQ_Pos)
		| ((PLLP / 2U - 1U) << RCC_PLLCFGR_PLLP_Pos)
		| (PLLN << RCC_PLLCFGR_PLLN_Pos)
		| (PLLM << RCC_PLLCFGR_PLLM_Pos);

	/* Get actual clock frequency.
	 * */
	clock_cpu_hz = CLOCK / PLLP;

	/* Enable PLL and wait until it is ready.
	 * */
	RCC->CR |= RCC_CR_PLLON;

	while ((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY) { __NOP(); }

	/* Configure Flash.
	 * */
#if defined(STM32F4)
	FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN
		| FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
#elif defined(STM32F7)
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
#endif /* STM32Fx */

	/* Enable high frequency mode on GD32F405.
	 * */
#ifdef STM32F4
	if (hal.MCU_ID == MCU_ID_GD32F405) {

		PWR->CR |= (1U << 16);		/* HDEN */

		while ((PWR->CSR & (1U << 16)) == 0U) { __NOP(); }

		PWR->CR |= (1U << 17);		/* HDS */

		while ((PWR->CSR & (1U << 17)) == 0U) { __NOP(); }
	}
#endif /* STM32F4 */

	/* Select PLL clock and wait until it is used.
	 * */
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { __NOP(); }

	/* Enable caching on Cortex-M7.
	 * */
#ifdef STM32F7
	SCB_EnableICache();
	SCB_EnableDCache();
#endif /* STM32F7 */
}

static void
periph_startup()
{
	/* Enable LSI.
	 * */
	RCC->CSR |= RCC_CSR_LSION;

	/* Enable GPIO clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

	/* Enable DMA clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	/* Check for reset reason.
	 * */
#if defined(STM32F4)
	if (RCC->CSR & RCC_CSR_WDGRSTF) {
#elif defined(STM32F7)
	if (RCC->CSR & RCC_CSR_IWDGRSTF) {
#endif /* STM32Fx */

		log_TRACE("RESET on WD" EOL);
	}

	RCC->CSR |= RCC_CSR_RMVF;
}

static void
flash_verify()
{
	uint32_t		crc32;

	if (* (const uint32_t *) fw.ld_crc32 == 0xFFFFFFFFU) {

		crc32 = crc32u((const void *) fw.ld_begin, fw.ld_crc32 - fw.ld_begin);

		/* Update flash CRC32.
		 * */
		FLASH_prog_u32((uint32_t *) fw.ld_crc32, crc32);
	}

	crc32 = crc32u((const void *) fw.ld_begin, fw.ld_crc32 - fw.ld_begin);

	if (* (const uint32_t *) fw.ld_crc32 != crc32) {

		log_TRACE("Flash CRC32 does not match" EOL);
	}
}

void hal_bootload()
{
	const uint32_t		*sysmem;

	if (noinit_HAL.bootload_jump == HAL_ENABLED) {

		noinit_HAL.bootload_jump = 0U;

#ifdef STM32F7
		SCB_CleanDCache();
#endif /* STM32F7 */

#if defined(STM32F4)
		sysmem = (const uint32_t *) 0x1FFF0000U;
#elif defined(STM32F7)
		sysmem = (const uint32_t *) 0x1FF00000U;
#endif /* STM32Fx */

		SCB->VTOR = (uint32_t) sysmem;

		__DSB();
		__ISB();

		/* Load MSP.
		 * */
		__set_MSP(sysmem[0]);

		/* Jump to the bootloader.
		 * */
		((void (*) (void)) (sysmem[1])) ();
	}

	/* Enable FPU early.
	 * */
	SCB->CPACR |= (0xFU << 20);

	__DSB();
	__ISB();
}

void hal_startup()
{
	mcu_identify();
	core_startup();
	periph_startup();
	flash_verify();
}

int hal_lock_irq()
{
	int		irq;

	irq = __get_BASEPRI();
	__set_BASEPRI(1 << (8 - __NVIC_PRIO_BITS));

	__DSB();
	__ISB();

	return irq;
}

void hal_unlock_irq(int irq)
{
	__set_BASEPRI(irq);
}

void hal_system_reset()
{
#ifdef STM32F7
	SCB_CleanDCache();
#endif /* STM32F7 */

	NVIC_SystemReset();
}

void hal_bootload_reset()
{
	noinit_HAL.bootload_jump = HAL_ENABLED;

	hal_system_reset();
}

void hal_cpu_sleep()
{
	__DSB();
	__WFI();
}

void hal_memory_fence()
{
	__DMB();
}

int log_status()
{
	return (	log.boot_FLAG == HAL_ENABLED
			&& log.text_wp != log.text_rp) ? HAL_FAULT : HAL_OK;
}

void log_bootup()
{
	if (log.boot_FLAG != HAL_ENABLED) {

		log.boot_FLAG = HAL_ENABLED;
		log.boot_COUNT = 0U;

		log.text_wp = 0;
		log.text_rp = 0;
	}
	else {
		log.boot_COUNT += 1U;
	}
}

void log_putc(int c)
{
	if (unlikely(log.boot_FLAG != HAL_ENABLED)) {

		log.boot_FLAG = HAL_ENABLED;
		log.boot_COUNT = 0U;

		log.text_wp = 0;
		log.text_rp = 0;
	}

	log.text[log.text_wp] = (char) c;

	log.text_wp = HAL_LOG_INC(log.text_wp);
	log.text_rp = (log.text_rp == log.text_wp)
		? HAL_LOG_INC(log.text_rp) : log.text_rp;
}

void log_flush()
{
	int		rp, wp;

	if (log.boot_FLAG == HAL_ENABLED) {

		rp = log.text_rp;
		wp = log.text_wp;

		while (rp != wp) {

			putc(log.text[rp]);

			rp = HAL_LOG_INC(rp);
		}

		puts(EOL);
	}
}

void log_clean()
{
	if (log.boot_FLAG == HAL_ENABLED) {

		log.text_wp = 0;
		log.text_rp = 0;
	}
}

void DBGMCU_mode_stop()
{
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
}

