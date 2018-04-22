/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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
#include "flash.h"
#include "gpio.h"
#include "pwm.h"
#include "usart.h"

#define HAL_APB1_HZ		(clock_cpu_hz / 4UL)
#define HAL_APB2_HZ		(clock_cpu_hz / 2UL)

#define __CCM__			__attribute__ ((section (".ccm")))

enum {
	LEG_A			= 1,
	LEG_B			= 2,
	LEG_C			= 4
};

typedef struct {

	int		USART_baud_rate;

	int		PWM_freq_hz;
	int		PWM_resolution;
	int		PWM_dead_time_ns;

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
}
HAL_t;

extern unsigned long	clock_cpu_hz;
extern HAL_t		hal;

void hal_startup();

void hal_system_reset();
void hal_sleep();
void hal_fence();

extern void hal_main();

#endif /* _H_HAL_ */

