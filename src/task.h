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

#ifndef _H_TASK_
#define _H_TASK_

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
	float			thermal_NTC;
	float			thermal_TEMP;

	/* IRQ callback.
	 * */
	void			(* p_irq_callback) ();

	/* To obtain an average values.
	 * */
	float			*av_IN[4];
	float			av_VAL[4];
	int			av_variable_N;
	int			av_sample_N;
	int			av_sample_MAX;
	float			av_default_time;
}
task_data_t;

extern task_data_t		ts;
extern pmc_t			pm;

void ts_av_handler();
float ts_av_float_1(float *param, float time);
int ts_av_float_4(float *param_0, float *param_1, float *param_2,
		float *param_3, float *result, float time);
float ts_av_float_arg_1(float *param, const char *s);

#endif /* _H_TASK_ */

