#include "cmsis/stm32f4xx.h"
#include "hal.h"

#include "libc.h"

#define CLOCK_CRYSTAL_HZ		12000000UL
#define CLOCK_CPU_TARGET_HZ		168000000UL

extern long			ld_begin_vectors;
unsigned long			clock_cpu_hz;

HAL_t				hal;
LOG_t				log		LD_NOINIT;
volatile int			bootload_jump	LD_NOINIT;

void irq_NMI()
{
	log_TRACE("IRQ: NMI\r\n");

	hal_system_reset();
}

void irq_HardFault()
{
	log_TRACE("IRQ: HardFault\r\n");

	hal_system_reset();
}

void irq_MemoryFault()
{
	log_TRACE("IRQ: MemoryFault\r\n");

	hal_system_reset();
}

void irq_BusFault()
{
	log_TRACE("IRQ: BusFault\r\n");

	hal_system_reset();
}

void irq_UsageFault()
{
	log_TRACE("IRQ: UsageFault\r\n");

	hal_system_reset();
}

void irq_Default()
{
	log_TRACE("IRQ: Default\r\n");

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
	SCB->VTOR = (unsigned long) &ld_begin_vectors;

	/* Configure priority grouping.
	 * */
	NVIC_SetPriorityGrouping(0UL);
}

static void
clock_startup()
{
	unsigned long	CLOCK, PLLQ, PLLP, PLLN, PLLM;
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
	while (HSE == 0 && N < 20000UL);

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

		/* From HSE.
		 * */
		CLOCK = CLOCK_CRYSTAL_HZ;
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE;

		/* Define HSE frequency.
		 * */
		hal.HSE_crystal_clock = CLOCK_CRYSTAL_HZ;
	}
	else {
		/* From HSI.
		 * */
		CLOCK = 16000000UL;
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI;

		/* Define that HSE is disabled.
		 * */
		hal.HSE_crystal_clock = 0;

		log_TRACE("HSE failed\r\n");
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
	while ((RCC->CR & RCC_CR_PLLRDY) == 0)
		__NOP();

	/* Configure Flash.
	 * */
	FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

	/* Select PLL.
	 * */
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till PLL is used.
	 * */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
		__NOP();
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

		log_TRACE("RESET: WD\r\n");
	}

	RCC->CSR |= RCC_CSR_RMVF;
}

void hal_startup()
{
	if (bootload_jump != INIT_SIGNATURE) {

		base_startup();
		clock_startup();
		periph_startup();
	}
	else {
		bootload_jump = 0;

		SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;

		__DSB();
		__ISB();

		__set_MSP(* (unsigned long *) 0x1FFF0000UL);

		((void (*) (void)) (* (unsigned long *) 0x1FFF0004UL)) ();

		NVIC_SystemReset();
	}
}

void hal_delay_us(int us)
{
	unsigned long		begVAL, xLOAD;
	int			elapsed, delay;

	begVAL = SysTick->VAL;
	xLOAD = SysTick->LOAD + 1UL;

	delay = us * (clock_cpu_hz / 1000000UL);

	do {
		elapsed = (int) (begVAL - SysTick->VAL);
		elapsed += (elapsed < 0) ? xLOAD : 0;

		if (elapsed >= delay)
			break;

		__NOP();
	}
	while (1);
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

void log_putc(int c)
{
	if (log.finit != INIT_SIGNATURE) {

		/* Initialize log at first usage.
		 * */
		memset(log.text, 0, sizeof(log.text));

		log.finit = INIT_SIGNATURE;
		log.n = 0;
	}

	if (log.n < 0 || log.n > (sizeof(log.text) - 2UL)) {

		log.n = 0;
	}

	log.text[log.n] = (char) c;
	++log.n;
}

int log_validate()
{
	int		rc = 0;

	if (log.finit == INIT_SIGNATURE) {

		/* Make sure log is null-terminated.
		 * */
		log.text[sizeof(log.text) - 1UL] = 0;

		rc = 1;
	}

	return rc;
}

