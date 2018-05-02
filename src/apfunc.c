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

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"
#include "pm/pm.h"
#include "pm/pm_m.h"

#include "lib.h"
#include "main.h"
#include "shell.h"
#include "telinfo.h"

#define AP_ERROR_BARRIER()	\
	{ if (pm.err_no != PM_OK) { break; } }

#define AP_PRINT_ERROR()	\
	{ if (pm.err_no != PM_OK) { printf("ERR %i: %s" EOL, \
		pm.err_no, pm_strerror(pm.err_no)); } }

#define AP_WAIT_FOR(expr)	\
	{ do { if (pm.err_no != PM_OK || (expr)) break; vTaskDelay(1); } while (1); }

#define AP_WAIT_FOR_IDLE()	AP_WAIT_FOR(pm.fsm_state == PM_STATE_IDLE)

SH_DEF(ap_pm_default)
{
	if (pm.lu_region != PM_LU_DISABLED)
		return;

	pm_default(&pm);
}

SH_DEF(ap_pm_tune_current_loop)
{
	if (pm.lu_region != PM_LU_DISABLED)
		return;

	pm_tune_current_loop(&pm);
}

SH_DEF(ap_pm_strerror)
{
	printf("%i: %s" EOL, pm.err_no, pm_strerror(pm.err_no));
}

SH_DEF(ap_probe_base)
{
	if (pm.lu_region != PM_LU_DISABLED)
		return;

	do {
		pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		/*pm_fsm_req(&pm, PM_STATE_POWER_STAGE_TEST);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();*/

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_R);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_R);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		printf("R %4e (Ohm)" EOL, &pm.const_R);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_L);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_L);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		printf("Ld %4e (H)" EOL, &pm.const_Ld);
		printf("Lq %4e (H)" EOL, &pm.const_Lq);

		pm_tune_current_loop(&pm);
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(ap_probe_spinup)
{
	if (pm.lu_region != PM_LU_DISABLED)
		return;

	do {
		pm_fsm_req(&pm, PM_STATE_LU_INITIATE);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pm.s_set_point = 500.f;

		/* FIXME: Wait for transient is done.
		 * */

		vTaskDelay(1000);
		AP_ERROR_BARRIER();

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_E);

		vTaskDelay(500);
		AP_ERROR_BARRIER();
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(test_PWM_set_DC)
{
	int		xDC;

	if (pm.lu_region != PM_LU_DISABLED)
		return;

	if (stoi(&xDC, s) != NULL) {

		xDC = (xDC < 0) ? 0 : (xDC > hal.PWM_resolution)
			? hal.PWM_resolution : xDC;

		PWM_set_DC(xDC, xDC, xDC);
	}
}

SH_DEF(test_PWM_set_Z)
{
	int		xZ;

	if (pm.lu_region != PM_LU_DISABLED)
		return;

	if (stoi(&xZ, s) != NULL) {

		PWM_set_Z(xZ);
	}
}

SH_DEF(test_current_ramp)
{
	float		iSP;
	int		xHold = 5;

	if (pm.lu_region == PM_LU_DISABLED)
		return;

	telinfo_enable(&ti, hal.PWM_freq_hz);

	do {
		iSP = pm.i_set_point_Q;
		vTaskDelay(xHold);

		pm.i_set_point_Q = pm.i_maximal;
		vTaskDelay(xHold);

		pm.i_set_point_Q = iSP;
		vTaskDelay(xHold);
	}
	while (0);

	telinfo_disable(&ti);
}

SH_DEF(test_speed_ramp)
{
	float			wSP;
	int			xHold = 300;

	if (pm.lu_region == PM_LU_DISABLED)
		return;

	telinfo_enable(&ti, 1000);

	do {
		wSP = pm.s_set_point;
		vTaskDelay(xHold);

		pm.s_set_point = pm.probe_speed_ramp;
		vTaskDelay(xHold);

		pm.s_set_point = wSP;
		vTaskDelay(xHold);
	}
	while (0);

	telinfo_disable(&ti);
}

