#ifndef _H_PPM_
#define _H_PPM_

#define GPIO_TIM4_CH1			XGPIO_DEF4('B', 6, 0, 2)
#define GPIO_TIM4_CH2			XGPIO_DEF4('B', 7, 0, 2)

#define GPIO_PPM			GPIO_TIM4_CH1
#define GPIO_DIR			GPIO_TIM4_CH2

void PPM_startup();
void PPM_configure();

float PPM_get_PERIOD();
float PPM_get_PULSE();
int PPM_get_STEP_DIR();
int PPM_get_backup_EP();

#endif /* _H_PPM_ */

