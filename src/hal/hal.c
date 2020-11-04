#include <stddef.h>

#include "cmsis/stm32f4xx.h"
#include "hal.h"
#include "libc.h"

#define CLOCK_CRYSTAL_HZ		12000000UL
#define CLOCK_CPU_TARGET_HZ		168000000UL

#ifdef _HW_REV2

#undef CLOCK_CRYSTAL_HZ
#define CLOCK_CRYSTAL_HZ		8000000UL

#endif /* _HW_REV2 */

#ifdef _HW_KLEN

#undef CLOCK_CRYSTAL_HZ
#define CLOCK_CRYSTAL_HZ		25000000UL

#endif /* _HW_KLEN */

unsigned long			clock_cpu_hz;

HAL_t				hal;
LOG_t				log		LD_NOINIT;
volatile int			bootload_jump	LD_NOINIT;

void irq_NMI()
{
	log_TRACE("IRQ: NMI" EOL);

	hal_system_reset();
}

void irq_HardFault()
{
	log_TRACE("IRQ: HardFault" EOL);

	hal_system_reset();
}

void irq_MemoryFault()
{
	log_TRACE("IRQ: MemoryFault" EOL);

	hal_system_reset();
}

void irq_BusFault()
{
	log_TRACE("IRQ: BusFault" EOL);

	hal_system_reset();
}

void irq_UsageFault()
{
	log_TRACE("IRQ: UsageFault" EOL);

	hal_system_reset();
}

void irq_Default()
{
	log_TRACE("IRQ: Default" EOL);

	hal_system_reset();
}

static void
base_startup()
{
	/* Enable FPU.
	 * */
	SCB->CPACR |= (3UL << 20) | (3UL << 22);

	/* Vector table offset.
	 * */
	SCB->VTOR = (u32_t) &ld_begin_vectors;

	/* Configure priority grouping.
	 * */
	NVIC_SetPriorityGrouping(0UL);
}

static void
clock_startup()
{
	u32_t		CLOCK, PLLQ, PLLP, PLLN, PLLM;
	int		HSE, N = 0;

	/* Enable HSI.
	 * */
	RCC->CR |= RCC_CR_HSION;

	/* Reset RCC.
	 * */
	RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_HSEON);
	RCC->CFGR = 0;
	RCC->CIR = 0;

	/* Enable HSE.
	 * */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait till HSE is ready.
	 * */
	do {
		HSE = RCC->CR & RCC_CR_HSERDY;
		N++;

		__NOP();
	}
	while (HSE == 0 && N < 40000UL);

	/* Enable power interface clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	/* Regulator voltage scale 1 mode.
	 * */
	PWR->CR |= PWR_CR_VOS;

	/* Set AHB/APB1/APB2 prescalers.
	 * */
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	if (HSE != 0) {

		CLOCK = CLOCK_CRYSTAL_HZ;

		/* From HSE.
		 * */
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE;
	}
	else {
		CLOCK = 16000000UL;

		/* From HSI.
		 * */
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI;

		log_TRACE("HSE failed" EOL);
	}

	PLLP = 2;

	PLLM = (CLOCK + 1999999UL) / 2000000UL;
	CLOCK /= PLLM;

	PLLN = (PLLP * CLOCK_CPU_TARGET_HZ) / CLOCK;
	CLOCK *= PLLN;

	PLLQ = (CLOCK + 47999999UL) / 48000000UL;

	RCC->PLLCFGR |= (PLLQ << 24)
		| ((PLLP / 2UL - 1UL) << 16)
		| (PLLN << 6) | (PLLM << 0);

	/* Define clock frequency.
	 * */
	clock_cpu_hz = CLOCK / PLLP;

	/* Enable PLL.
	 * */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till the main PLL is ready.
	 * */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) {

		__NOP();
	}

	/* Configure Flash.
	 * */
	FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

	/* Select PLL.
	 * */
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till PLL is used.
	 * */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {

		__NOP();
	}
}

static void
periph_startup()
{
	/* Enable Programmable voltage detector.
	 * */
	PWR->CR |= PWR_CR_PLS_LEV7 | PWR_CR_PVDE;

	/* Enable LSI.
	 * */
	RCC->CSR |= RCC_CSR_LSION;

	/* Enable GPIO clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
		| RCC_AHB1ENR_GPIOCEN;

	/* Check for reset reason.
	 * */
	if (RCC->CSR & RCC_CSR_WDGRSTF) {

		log_TRACE("RESET: WD" EOL);
	}

	RCC->CSR |= RCC_CSR_RMVF;
}

void hal_bootload()
{
	if (bootload_jump == INIT_SIGNATURE) {

		bootload_jump = 0;

		SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;

		__DSB();
		__ISB();

		/* Load MSP.
		 * */
		__set_MSP(* (u32_t *) 0x1FFF0000UL);

		/* Jump to bootloader.
		 * */
		((void (*) (void)) (* (u32_t *) 0x1FFF0004UL)) ();
	}
}

void hal_startup()
{
	base_startup();
	clock_startup();
	periph_startup();
}

void hal_delay_ns(int ns)
{
	u32_t			xVAL, xLOAD;
	int			elapsed, tickval;

	xVAL = SysTick->VAL;
	xLOAD = SysTick->LOAD + 1UL;

	tickval = ns * (clock_cpu_hz / 1000000UL) / 1000UL;

	do {
		elapsed = (int) (xVAL - SysTick->VAL);
		elapsed += (elapsed < 0) ? xLOAD : 0;

		if (elapsed >= tickval)
			break;

		__NOP();
	}
	while (1);
}

int hal_lock_irq()
{
	int		irq;

	irq = __get_BASEPRI();
	__set_BASEPRI(1 << (8 - __NVIC_PRIO_BITS));

	return irq;
}

void hal_unlock_irq(int irq)
{
	__set_BASEPRI(irq);
}

void hal_system_debug()
{
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
	DBGMCU->APB2FZ |= DBGMCU_APB1_FZ_DBG_TIM1_STOP;
}

void hal_system_reset()
{
	NVIC_SystemReset();
}

void hal_bootload_jump()
{
	bootload_jump = INIT_SIGNATURE;

	NVIC_SystemReset();
}

void hal_sleep()
{
	__WFI();
}

void hal_fence()
{
	__DMB();
}

int log_bootup()
{
	int		rc = 0;

	if (log.boot_SIGNATURE != INIT_SIGNATURE) {

		log.boot_SIGNATURE = INIT_SIGNATURE;
		log.boot_COUNT = 0;

		/* Initialize LOG at first usage.
		 * */
		memset(log.textbuf, 0, sizeof(log.textbuf));

		log.tail = 0;
	}
	else {
		log.boot_COUNT += 1;

		/* Make sure LOG is null-terminated.
		 * */
		log.textbuf[sizeof(log.textbuf) - 1] = 0;

		rc = (log.textbuf[0] != 0) ? 1 : 0;
	}

	return rc;
}

void log_putc(int c)
{
	const int	tailmax = sizeof(log.textbuf) - 1;

	log.tail = (log.tail >= 0 && log.tail < tailmax) ? log.tail : 0;
	log.textbuf[log.tail] = (char) c;

	++log.tail;
}

