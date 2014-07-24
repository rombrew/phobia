#ifndef _H_HAL_
#define _H_HAL_

#include "pwm.h"
#include "uart.h"

#define FREQ_AHB_HZ		(168000000UL)
#define FREQ_APB1_HZ		(FREQ_AHB_HZ / 4UL)
#define FREQ_APB2_HZ		(FREQ_AHB_HZ / 2UL)

enum {
	LED_GREEN		= 0x0001,
	LED_ORANGE		= 0x0002,
	LED_RED			= 0x0004,
	LED_BLUE		= 0x0008
};

void halStart();
int halResetReason();
void halWFI();
void halLED(int F);

extern void halMain();
extern void halTick();

#endif /* _H_HAL_ */

