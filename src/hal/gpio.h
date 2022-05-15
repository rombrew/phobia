#ifndef _H_GPIO_
#define _H_GPIO_

#define XGPIO_DEF4(PORT, N, CH, FUNC)	(((PORT) - 'A') << 4 | ((N) & 0xFU) << 0	\
					| ((CH) & 0x1FU) << 7 | ((FUNC) & 0xFU) << 12)

#define XGPIO_DEF3(PORT, N, CH)		XGPIO_DEF4(PORT, N, CH, 0)
#define XGPIO_DEF2(PORT, N)		XGPIO_DEF4(PORT, N, 0, 0)

#define XGPIO_GET_PORT(XGPIO)		(((XGPIO) >> 4) & 0x7U)
#define XGPIO_GET_N(XGPIO)		(((XGPIO) >> 0) & 0xFU)
#define XGPIO_GET_CH(XGPIO)		(((XGPIO) >> 7) & 0x1FU)
#define XGPIO_GET_FUNC(XGPIO)		(((XGPIO) >> 12) & 0xFU)

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

#endif /* _H_GPIO_ */

