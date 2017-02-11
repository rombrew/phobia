/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2016 Roman Belov <romblv@gmail.com>

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

#ifndef _H_HAL_TASK_
#define _H_HAL_TASK_

#include "lib.h"
#include "pm_control.h"

typedef struct {

	/* IO interfaces.
	 * */
	io_ops_t		io_usart;
	io_ops_t		io_can;

	/* CPU load counters.
	 * */
	int			load_count_flag;
	int			load_count_value;
	int			load_count_limit;

	/* Thermal information.
	 * */
	int			thermal_xAVG[3];
	int			thermal_sample_N;
	float			thermal_NTC;
	float			thermal_TEMP;
	float			thermal_REF;

	/* ADC event handler.
	 * */
	void			(* pEX) ();

	/* To obtain an average values.
	 * */
	float			*av_IN[4];
	float			av_VAL[4];
	int			av_variable_N;
	int			av_sample_N;
	int			av_sample_MAX;
	float			av_default_time;

	/* J estimation.
	 * */
	int			ap_J_fsm_state;
	float			ap_J_measure_T;
	float			ap_J_vars[4];
}
task_data_t;

extern task_data_t		ts;
extern pmc_t			pm;

void ts_av_EH();
float ts_av_float_1(float *param, float time);
float ts_av_float_arg_1(float *param, const char *s);

#endif /* _H_HAL_TASK_ */
