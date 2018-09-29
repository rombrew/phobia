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

#ifndef _H_HAL_
#define _H_HAL_

#include "adc.h"
#include "can.h"
#include "flash.h"
#include "gpio.h"
#include "ppm.h"
#include "pwm.h"
#include "usart.h"

#define GPIO_HALL_A			XGPIO_DEF2('C', 6)
#define GPIO_HALL_B			XGPIO_DEF2('C', 7)
#define GPIO_HALL_C			XGPIO_DEF2('C', 8)

#define GPIO_BOOST_12V			XGPIO_DEF2('B', 2)
#define GPIO_LED			XGPIO_DEF2('C', 12)

#define CLOCK_APB1_HZ			(clock_cpu_hz / 4UL)
#define CLOCK_APB2_HZ			(clock_cpu_hz / 2UL)

#define __section_ccmram		__attribute__ (( section(".ccmram") ))
#define __section_ramfunc		__attribute__ (( section(".ramfunc") ))

enum {
	LEG_A				= 1,
	LEG_B				= 2,
	LEG_C				= 4
};

enum {
	PPM_DISABLED			= 0,
	PPM_PULSE_WIDTH,
	PPM_STEP_DIR,
	PPM_ENCODER
};

typedef struct {

	int		HSE_crystal_clock;
	int		USART_baud_rate;

	float		PWM_frequency;
	int		PWM_resolution;
	float		PWM_deadtime;
	int		PWM_deadtime_tik;

	float		ADC_reference_voltage;
	float		ADC_current_shunt_resistance;
	float		ADC_amplifier_gain;
	float		ADC_voltage_divider_gain;

	float		ADC_current_A;
	float		ADC_current_B;
	float		ADC_voltage_U;
	float		ADC_voltage_A;
	float		ADC_voltage_B;
	float		ADC_voltage_C;

	struct {

		float		GA;
		float		GU;
		float		GS;
		float		TEMP[2];
	}
	ADC_const;

	unsigned long	CAN_msg_ID;
	int		CAN_msg_len;
	unsigned char	CAN_msg_payload[8];

	int		PPM_mode;
	int		PPM_timebase;
	int		PPM_signal_caught;
}
HAL_t;

extern unsigned long		clock_cpu_hz;
extern HAL_t			hal;

void hal_startup();

void hal_system_reset();
void hal_sleep();
void hal_fence();

extern void hal_main();

#endif /* _H_HAL_ */

