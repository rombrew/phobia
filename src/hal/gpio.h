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

/*
#define GPIO_CAN_RX			XGPIO_DEF4('B', 8, 0, 0)
#define GPIO_CAN_TX			XGPIO_DEF4('B', 9, 0, 0)

#define GPIO_I2C_SCL			XGPIO_DEF4('B', 6, 0, 4)
#define GPIO_I2C_SDA			XGPIO_DEF4('B', 7, 0, 4)

#define GPIO_TIM4_CH1			XGPIO_DEF4('B', 6, 0, 2)
#define GPIO_TIM4_CH2			XGPIO_DEF4('B', 7, 0, 2)

#define GPIO_SPI_NSS			XGPIO_DEF4('A', 4, 4, 5)
#define GPIO_SPI_SCK			XGPIO_DEF4('A', 5, 5, 5)
#define GPIO_SPI_MISO			XGPIO_DEF4('A', 6, 6, 5)
#define GPIO_SPI_MOSI			XGPIO_DEF4('A', 7, 7, 5)

#define GPIO_DAC1			GPIO_SPI_NSS
#define GPIO_DAC2			GPIO_SPI_SCK

#define GPIO_TIM3_CH1			XGPIO_DEF4('C', 6, 0, 2)
#define GPIO_TIM3_CH2			XGPIO_DEF4('C', 7, 0, 2)
#define GPIO_TIM3_CH3			XGPIO_DEF4('C', 8, 0, 2)

*/

void GPIO_set_mode_INPUT(int xGPIO);
void GPIO_set_mode_OUTPUT(int xGPIO);
void GPIO_set_mode_ANALOG(int xGPIO);
void GPIO_set_mode_FUNCTION(int xGPIO);
void GPIO_set_mode_PUSH_PULL(int xGPIO);
void GPIO_set_mode_OPEN_DRAIN(int xGPIO);
void GPIO_set_mode_SPEED_LOW(int xGPIO);
void GPIO_set_mode_SPEED_HIGH(int xGPIO);
void GPIO_set_mode_PULL_NONE(int xGPIO);
void GPIO_set_mode_PULL_UP(int xGPIO);
void GPIO_set_mode_PULL_DOWN(int xGPIO);

void GPIO_set_HIGH(int xGPIO);
void GPIO_set_LOW(int xGPIO);
int GPIO_get_VALUE(int xGPIO);

int GPIO_get_HALL();

#endif /* _H_GPIO_ */

