#ifndef _H_GPIO_
#define _H_GPIO_

#define XGPIO_DEF4(PORT,N,CH,FUNC)	(((PORT) - 'A') << 4 | ((N) & 0xF) \
					| ((CH) & 0xF) << 8 | ((FUNC) & 0xF) << 12)

#define XGPIO_DEF3(PORT,N,CH)		XGPIO_DEF4(PORT, N, CH, 0)
#define XGPIO_DEF2(PORT,N)		XGPIO_DEF4(PORT, N, 0, 0)

#define XGPIO_GET_PORT(XGPIO)		(((XGPIO) >> 4) & 0xF)
#define XGPIO_GET_N(XGPIO)		((XGPIO) & 0xF)
#define XGPIO_GET_CH(XGPIO)		(((XGPIO) >> 8) & 0xF)
#define XGPIO_GET_FUNC(XGPIO)		(((XGPIO) >> 12) & 0xF)

void GPIO_set_mode_INPUT(int xGPIO);
void GPIO_set_mode_OUTPUT(int xGPIO);
void GPIO_set_mode_ANALOG(int xGPIO);
void GPIO_set_mode_FUNCTION(int xGPIO);
void GPIO_set_mode_PUSH_PULL(int xGPIO);
void GPIO_set_mode_OPEN_DRAIN(int xGPIO);
void GPIO_set_mode_SPEED_LOW(int xGPIO);
void GPIO_set_mode_SPEED_HIGH(int xGPIO);
void GPIO_set_mode_SPEED_FAST(int xGPIO);
void GPIO_set_mode_PULL_NONE(int xGPIO);
void GPIO_set_mode_PULL_UP(int xGPIO);
void GPIO_set_mode_PULL_DOWN(int xGPIO);

void GPIO_set_HIGH(int xGPIO);
void GPIO_set_LOW(int xGPIO);
int GPIO_get_VALUE(int xGPIO);

int GPIO_get_HALL();

#endif /* _H_GPIO_ */

