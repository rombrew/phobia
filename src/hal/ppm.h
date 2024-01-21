#ifndef _H_PPM_
#define _H_PPM_

#define GPIO_TIM4_CH1			XGPIO_DEF4('B', 6, 0, 2)

#define GPIO_PPM			GPIO_TIM4_CH1

void PPM_startup();
void PPM_configure();

float PPM_get_PULSE();
float PPM_get_PERIOD();

#endif /* _H_PPM_ */

