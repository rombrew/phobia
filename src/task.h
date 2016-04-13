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

#ifndef _H_TASK_
#define _H_TASK_

#include "pmc.h"
#include "sh.h"

typedef struct {

	/* Seconds from Power Up.
	 * */
	int			uSEC;
	int			uDS;
	int			uTIM;

	/* CPU usage.
	 * */
	int			usage_S;
	int			usage_T;

	/* Task IOMUX.
	 * */
	int			mux_TEMP[2];

	/* ADC Event Handler.
	 * */
	void			(* pEX) ();

	/* Average Values.
	 * */
	float			*av_IN[8];
	float			av_VAL[8];
	int			av_variable_N;
	int			av_sample_N;
	int			av_sample_MAX;
	float			av_default_time;
}
taskDATA_t;

extern taskDATA_t		td;
extern pmc_t			pm;

void taskYIELD();

void evAV_8();
float task_av_float_1(float *param, float time);
float task_av_float_arg_1(float *param, const char *s);

SH_DEF(hal_uptime);
SH_DEF(hal_cpu_usage);
SH_DEF(hal_av_default_time);
SH_DEF(hal_reboot);
SH_DEF(hal_keycodes);
SH_DEF(hal_pwm_freq_hz);
SH_DEF(hal_pwm_dead_time_ns);
SH_DEF(hal_pwm_DC);
SH_DEF(hal_pwm_Z);

#endif /* _H_TASK_ */

