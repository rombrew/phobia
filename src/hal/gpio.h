/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

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
#define GPIO_ADC_CURRENT_A		XGPIO_DEF3('A', 2, 2)
#define GPIO_ADC_CURRENT_B		XGPIO_DEF3('C', 0, 10)
#define GPIO_ADC_VOLTAGE_U		XGPIO_DEF3('A', 1, 1)
#define GPIO_ADC_VOLTAGE_A		XGPIO_DEF3('A', 3, 3)
#define GPIO_ADC_VOLTAGE_B		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_VOLTAGE_C		XGPIO_DEF3('C', 1, 11)
#define GPIO_ADC_PCB_NTC		XGPIO_DEF3('C', 3, 13)
#define GPIO_ADC_EXT_NTC		XGPIO_DEF3('A', 0, 0)
*/
/* rev3.
 * */
#define GPIO_ADC_CURRENT_A		XGPIO_DEF3('C', 3, 13)
#define GPIO_ADC_CURRENT_B		XGPIO_DEF3('A', 1, 1)
#define GPIO_ADC_VOLTAGE_U		XGPIO_DEF3('A', 2, 2)
#define GPIO_ADC_VOLTAGE_A		XGPIO_DEF3('A', 0, 0)
#define GPIO_ADC_VOLTAGE_B		XGPIO_DEF3('A', 0, 0)
#define GPIO_ADC_VOLTAGE_C		XGPIO_DEF3('A', 0, 0)
#define GPIO_ADC_PCB_NTC		XGPIO_DEF3('C', 2, 12)
#define GPIO_ADC_EXT_NTC		XGPIO_DEF3('C', 1, 11)

#define GPIO_ADC_INTERNAL_TEMP		XGPIO_DEF3('J', 0, 0)

#define GPIO_CAN_RX			XGPIO_DEF4('B', 8, 0, 0)
#define GPIO_CAN_TX			XGPIO_DEF4('B', 9, 0, 0)

#define GPIO_TIM1_CH1N			XGPIO_DEF4('B', 13, 0, 1)
#define GPIO_TIM1_CH2N			XGPIO_DEF4('B', 14, 0, 1)
#define GPIO_TIM1_CH3N			XGPIO_DEF4('B', 15, 0, 1)
#define GPIO_TIM1_CH1			XGPIO_DEF4('A', 8, 0, 1)
#define GPIO_TIM1_CH2			XGPIO_DEF4('A', 9, 0, 1)
#define GPIO_TIM1_CH3			XGPIO_DEF4('A', 10, 0, 1)

#define GPIO_USART_TX			XGPIO_DEF4('C', 10, 0, 7)
#define GPIO_USART_RX			XGPIO_DEF4('C', 11, 0, 7)

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

#define GPIO_HALL_A			GPIO_TIM3_CH1
#define GPIO_HALL_B			GPIO_TIM3_CH2
#define GPIO_HALL_C			GPIO_TIM3_CH3

#define GPIO_BOOST_CONVERTER		XGPIO_DEF2('B', 2)
//#define GPIO_LED			XGPIO_DEF2('C', 12)
#define GPIO_LED			XGPIO_DEF2('B', 5)

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

#endif /* _H_GPIO_ */

