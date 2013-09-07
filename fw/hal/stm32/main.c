#include "lib/types.h"
#include "f4xx.h"

static void
halt()
{
}

int hal_reset_reason()
{
	return RCC->CSR >> 25;
}

void irq_svcall()
{
}

void irq_systick()
{
}

static void
clock_init()
{
	int		hserdy = 0, hsecounter = 0;

	/* Enable HSE.
	 * */
	RCC->CR |= BIT(16);

	/* Wait till HSE is ready.
	 * */
	do {
		hserdy = RCC->CR & BIT(17);
		hsecounter++;
	}
	while (!hserdy && hsecounter < 4000);

	/* If HSE is ready.
	 * */
	if (hserdy) {

		/* Enable power interface clock.
		 * */
		RCC->APB1ENR |= BIT(28);

		/* Regulator voltage scale 1 mode.
		 * */
		PWR->CR |= BIT(14);

		/* Set AHB/APB1/APB2 prescalers.
		 * */
		RCC->CFGR &= ~(BITF(15UL, 4) | BITF(7UL, 10) | BITF(7UL, 13));
		RCC->CFGR |= BITF(0UL, 4) | BITF(5UL, 10) | BITF(4UL, 13);

		/* Enable PLL.
		 * */
		RCC->PLLCFGR = BITF(7UL, 24) | BIT(22) | BITF(0UL, 16)
			| BITF(168UL, 6) | BITF(4UL, 0);
		RCC->CR |= BIT(24);

		/* Wait till the main PLL is ready.
		 * */
		while (!(RCC->CR & BIT(25))) ;

		/* Configure Flash.
		 * */
		FLASH->ACR = BIT(10) | BIT(9) | BIT(8) | BITF(5UL, 0);

		/* Select PLL.
		 * */
		RCC->CFGR &= ~BITF(3UL, 0);
		RCC->CFGR |= BITF(2UL, 0);

		/* Wait till PLL is used.
		 * */
		while (RCC->CFGR & BITF(3UL, 2) != BITF(2UL, 2)) ;
	}
	else {
		/* HSE fails to start up.
		 * */

		halt();
	}
}

static void
power_init()
{
}

static void
cortex_init()
{
	/* Enable FPU.
	 * */
	SCB->CPACR |= BITF(3UL, 20) | BITF(3UL, 22);
}

static void
board_init()
{
}

int main()
{
	clock_init();
	power_init();
	cortex_init();
	board_init();
}

