#include "cmsis/stm32f4xx.h"
#include "hal.h"

void WD_startup()
{
	IWDG->KR = 0x5555;

	/* Configure timeout value (0.5 ms).
	 * */
	IWDG->PR = 0;
	IWDG->RLR = 4;

	IWDG->KR = 0xAAAA;
	IWDG->KR = 0xCCCC;
}

void WD_kick()
{
	IWDG->KR = 0xAAAA;
}

