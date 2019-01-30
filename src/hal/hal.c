#include "cmsis/stm32f4xx.h"
#include "hal.h"

#define CLOCK_CRYSTAL_HZ		12000000UL
#define CLOCK_CPU_TARGET_HZ		168000000UL

extern long			ld_begin_vectors;
unsigned long			clock_cpu_hz;

HAL_t				hal;

extern void lowTRACE(const char *fmt, ...);

void irqNMI()
{
	lowTRACE("IRQ: NMI \r\n");
}

void irqHardFault()
{
	lowTRACE("IRQ: HardFault \r\n");
}

void irqMemoryFault()
{
	lowTRACE("IRQ: MemoryFault \r\n");
}

void irqBusFault()
{
	lowTRACE("IRQ: BusFault \r\n");
}

void irqUsageFault()
{
	lowTRACE("IRQ: UsageFault \r\n");
}

void irqDefault()
{
	lowTRACE("IRQ: Default \r\n");
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
	NVIC_SetPriorityGrouping(3UL);
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
	RCC->CFGR = 0;
	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_PLLON);
	RCC->CIR = 0;

	/* Enable HSE.
	 * */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait till HSE is ready.
	 * */
	do {
		HSE = RCC->CR & RCC_CR_HSERDY;
		N++;
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
	while (!(RCC->CR & RCC_CR_PLLRDY))
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

	/* Enable GPIO clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
		| RCC_AHB1ENR_GPIOCEN;
}

void hal_startup()
{
	base_startup();
	clock_startup();
	periph_startup();
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

void hal_sleep()
{
	__WFI();
}

void hal_fence()
{
	__DMB();
}

