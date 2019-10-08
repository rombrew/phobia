#ifndef _H_PPM_
#define _H_PPM_

#define GPIO_TIM4_CH1			XGPIO_DEF4('B', 6, 0, 2)
#define GPIO_TIM4_CH2			XGPIO_DEF4('B', 7, 0, 2)

#define GPIO_I2C_PPM			GPIO_TIM4_CH1
#define GPIO_I2C_DIR			GPIO_TIM4_CH2

void PPM_startup();
void PPM_configure();

float PPM_get_PERIOD();
float PPM_get_PULSE();

#endif /* _H_PPM_ */
