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

#define HAL_APB1_HZ		(hal.clock_cpu_hz / 4)
#define HAL_APB2_HZ		(hal.clock_cpu_hz / 2)

#define __CCM__			__attribute__ ((section (".ccm")))

enum {
	LEG_A			= 1,
	LEG_B			= 2,
	LEG_C			= 4
};

typedef struct {

	int		clock_cpu_hz;
	int		usart_baud_rate;

	int		pwm_freq_hz;
	int		pwm_resolution;
	int		pwm_dead_time_ns;

	float		adc_current_A;
	float		adc_current_B;
	float		adc_voltage_U;
	float		adc_voltage_A;
	float		adc_voltage_B;
	float		adc_voltage_C;
	int		adc_thermal_PCB_NTC;
	int		adc_thermal_EXT_NTC;
	int		adc_thermal_TEMP;

	struct {

		float		GA;
		float		GV;
		float		NTC[4];
		float		TEMP[2];
	}
	adc_const;
}
hal_t;

extern hal_t		hal;

void hal_startup();

void hal_halt();
void hal_reset();
void hal_sleep();
void hal_fence();

void hal_boost_converter(int x);
void hal_set_LED(int x);
int hal_get_HALL();

extern void hal_main();

#endif /* _H_HAL_ */

