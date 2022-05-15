#ifndef _H_PWM_
#define _H_PWM_

#define GPIO_TIM1_CH1N			XGPIO_DEF4('B', 13, 0, 1)
#define GPIO_TIM1_CH2N			XGPIO_DEF4('B', 14, 0, 1)
#define GPIO_TIM1_CH3N			XGPIO_DEF4('B', 15, 0, 1)
#define GPIO_TIM1_CH1			XGPIO_DEF4('A', 8, 0, 1)
#define GPIO_TIM1_CH2			XGPIO_DEF4('A', 9, 0, 1)
#define GPIO_TIM1_CH3			XGPIO_DEF4('A', 10, 0, 1)

void PWM_startup();
void PWM_configure();

void PWM_set_DC(int A, int B, int C);
void PWM_set_Z(int Z);
void PWM_halt_Z();

#endif /* _H_PWM_ */

