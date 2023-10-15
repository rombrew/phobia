#include <stddef.h>

#include "hal.h"
#include "libc.h"

#include "cmsis/stm32xx.h"

#define HAL_BOOT_SIGNATURE	0x3C34EB12U

uint32_t			clock_cpu_hz;

HAL_t				hal;
LOG_t				log		LD_NOINIT;
volatile uint32_t		bootload_jump	LD_NOINIT;

void irq_NMI()
{
	log_TRACE("IRQ: NMI" EOL);

	if (RCC->CIR & RCC_CIR_CSSF) {

		RCC->CIR |= RCC_CIR_CSSC;

		log_TRACE("HSE fault" EOL);
	}

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
	SCB->CPACR |= (15U << 20);

	/* Vector table offset.
	 * */
	SCB->VTOR = (uint32_t) &ld_begin_text;

	/* Configure priority grouping.
	 * */
	NVIC_SetPriorityGrouping(0U);

#ifdef STM32F7
	SCB_EnableICache();
	SCB_EnableDCache();
#endif /* STM32F7 */
}

static void
clock_startup()
{
	uint32_t	CLOCK, PLLQ, PLLP, PLLN, PLLM;
	int		HSE, N = 0;

	/* Enable HSI.
	 * */
	RCC->CR |= RCC_CR_HSION;

	/* Reset RCC.
	 * */
	RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_HSEON);
	RCC->CFGR = 0;
	RCC->CIR = 0;

#ifdef STM32F7
	RCC->DCKCFGR1 = 0;
	RCC->DCKCFGR2 = 0;
#endif  /* STM32F7 */

	/* Enable HSE.
	 * */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait till HSE is ready.
	 * */
	do {
		HSE = RCC->CR & RCC_CR_HSERDY;

		__NOP();
		__NOP();

		if (N > 70000U)
			break;

		N++;
	}
	while (HSE == 0);

	/* Enable power interface clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	/* Regulator voltage scale 1 mode.
	 * */
#if defined(STM32F4)
	PWR->CR |= PWR_CR_VOS;
#elif defined(STM32F7)
	PWR->CR1 |= PWR_CR1_VOS_1 | PWR_CR1_VOS_0;
#endif /* STM32Fx */

	/* Set AHB/APB1/APB2 prescalers.
	 * */
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	if (HSE != 0) {

		CLOCK = HW_CLOCK_CRYSTAL_HZ;

		/* Clock from HSE.
		 * */
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE;

		/* Enable CSS.
		 * */
		RCC->CR |= RCC_CR_CSSON;
	}
	else {
		CLOCK = 16000000U;

		/* Clock from HSI.
		 * */
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI;

		log_TRACE("HSE not ready" EOL);
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

	/* Update clock frequency.
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
#if defined(STM32F4)
	FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
#elif defined(STM32F7)
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
#endif /* STM32Fx */

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
	/* Enable LSI.
	 * */
	RCC->CSR |= RCC_CSR_LSION;

	/* Enable GPIO clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

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
	uint32_t	flash_sizeof, *flash_crc32, crc32;

	flash_sizeof = fw.ld_end - fw.ld_begin;
	flash_crc32 = (uint32_t *) fw.ld_end;

	if (*flash_crc32 == 0xFFFFFFFFU) {

		crc32 = crc32b((const void *) fw.ld_begin, flash_sizeof);

		/* Update flash CRC32.
		 * */
		FLASH_prog(flash_crc32, crc32);
	}

	if (crc32b((const void *) fw.ld_begin, flash_sizeof) != *flash_crc32) {

		log_TRACE("Flash CRC32 invalid" EOL);
	}
}

void hal_bootload()
{
	uint32_t	*sysmem;

	if (bootload_jump == HAL_BOOT_SIGNATURE) {

		bootload_jump = 0U;

#if defined(STM32F4)
		sysmem = (uint32_t *) 0x1FFF0000U;
#elif defined(STM32F7)
		sysmem = (uint32_t *) 0x1FF00000U;
#endif /* STM32Fx */

		/* Load MSP.
		 * */
		__set_MSP(sysmem[0]);

		/* Jump to the bootloader.
		 * */
		((void (*) (void)) (sysmem[1])) ();
	}
}

void hal_startup()
{
	base_startup();
	clock_startup();
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
	bootload_jump = HAL_BOOT_SIGNATURE;

#ifdef STM32F7
	SCB_CleanDCache();
#endif /* STM32F7 */

	NVIC_SystemReset();
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
	return (log.textbuf[0] != 0) ? 1 : 0;
}

void log_bootup()
{
	if (log.boot_SIGNATURE != HAL_BOOT_SIGNATURE) {

		log.boot_SIGNATURE = HAL_BOOT_SIGNATURE;
		log.boot_COUNT = 0U;

		memset(log.textbuf, 0, sizeof(log.textbuf));

		log.len = 0;
	}
	else {
		log.boot_COUNT += 1U;

		log.textbuf[sizeof(log.textbuf) - 1] = 0;
	}
}

void log_putc(int c)
{
	if (log.boot_SIGNATURE != HAL_BOOT_SIGNATURE) {

		log.boot_SIGNATURE = HAL_BOOT_SIGNATURE;
		log.boot_COUNT = 0U;

		memset(log.textbuf, 0, sizeof(log.textbuf));

		log.len = 0;
	}
	else if (	   log.len < 0
			|| log.len >= sizeof(log.textbuf) - 1U) {

		log.len = 0;
	}

	log.textbuf[log.len++] = (char) c;
}

void DBGMCU_mode_stop()
{
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
}

