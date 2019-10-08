#ifndef _H_TIM_
#define _H_TIM_

#define GPIO_TIM3_CH1			XGPIO_DEF4('C', 6, 0, 2)
#define GPIO_TIM3_CH2			XGPIO_DEF4('C', 7, 0, 2)
#define GPIO_TIM3_CH3			XGPIO_DEF4('C', 8, 0, 2)

#define GPIO_HALL_A			GPIO_TIM3_CH1
#define GPIO_HALL_B			GPIO_TIM3_CH2
#define GPIO_HALL_C			GPIO_TIM3_CH3

void TIM_startup();
void TIM_configure();

int TIM_get_EP();

#endif /* _H_TIM_ */
