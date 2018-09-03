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

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "teli.h"

#define PM_WAIT_FOR_IDLE()	if (pm_wait_for_IDLE() != 0) break

static void
pm_print_fail_reason()
{
	if (pm.fail_reason != PM_OK) {

		printf("ERR %i: %s" EOL, pm.fail_reason,
				pm_strerror(pm.fail_reason));
	}
}

int pm_wait_for_IDLE()
{
	int			rc = 0;

	do {
		if (pm.fsm_state == PM_STATE_IDLE)
			break;

		if (pm.fail_reason != PM_OK) {

			rc = pm.fail_reason;
			break;
		}

		vTaskDelay(1);
	}
	while (1);

	return rc;
}

SH_DEF(pm_config_default_req)
{
	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	pm_config_default(&pm);
}

SH_DEF(pm_config_tune_current_loop_req)
{
	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	pm_config_tune_current_loop(&pm);
}

SH_DEF(pm_fail_reason)
{
	pm_print_fail_reason();
}

SH_DEF(pm_probe_base)
{
	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	do {
		pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);
		PM_WAIT_FOR_IDLE();

		reg_print_fmt(&regfile[ID_PM_ADJUST_IA_0], 1);
		reg_print_fmt(&regfile[ID_PM_ADJUST_IB_0], 1);

		/*pm_fsm_req(&pm, PM_STATE_POWER_STAGE_TEST);
		PM_WAIT_FOR_IDLE();*/

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_R);
		PM_WAIT_FOR_IDLE();

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_R);
		PM_WAIT_FOR_IDLE();

		reg_print_fmt(&regfile[ID_PM_CONST_R], 1);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_L);
		PM_WAIT_FOR_IDLE();

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_L);
		PM_WAIT_FOR_IDLE();

		reg_print_fmt(&regfile[ID_PM_CONST_LD], 1);
		reg_print_fmt(&regfile[ID_PM_CONST_LQ], 1);

		pm_config_tune_current_loop(&pm);
	}
	while (0);

	pm_print_fail_reason();
}

SH_DEF(pm_probe_spinup)
{
	int			xWait;

	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	do {
		pm_fsm_req(&pm, PM_STATE_LU_INITIATE);
		PM_WAIT_FOR_IDLE();

		pm.s_setpoint = pm.probe_speed_low;

		xWait = (int) (pm.s_setpoint / pm.forced_accel * 1000.f);
		xWait += 100;

		vTaskDelay(xWait);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_E);
		PM_WAIT_FOR_IDLE();

		reg_print_fmt(&regfile[ID_PM_CONST_E_KV], 1);
	}
	while (0);

	pm_print_fail_reason();
}

SH_DEF(pm_fsm_req_lu_initiate)
{
	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	do {
		pm_fsm_req(&pm, PM_STATE_LU_INITIATE);
		PM_WAIT_FOR_IDLE();
	}
	while (0);

	pm_print_fail_reason();
}

SH_DEF(pm_fsm_req_lu_shutdown)
{
	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	do {
		pm_fsm_req(&pm, PM_STATE_LU_SHUTDOWN);
		PM_WAIT_FOR_IDLE();
	}
	while (0);

	pm_print_fail_reason();
}

SH_DEF(pm_test_PWM_set_DC)
{
	int			xDC;

	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	if (stoi(&xDC, s) != NULL) {

		xDC = (xDC < 0) ? 0 : (xDC > hal.PWM_resolution)
			? hal.PWM_resolution : xDC;

		PWM_set_DC(xDC, xDC, xDC);
	}
}

SH_DEF(pm_test_PWM_set_Z)
{
	int			xZ;

	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	if (stoi(&xZ, s) != NULL) {

		PWM_set_Z(xZ);
	}
}

SH_DEF(pm_test_current_ramp)
{
	float			iSP;
	int			xHold = 3;

	if (pm.lu_mode == PM_LU_DISABLED)
		return;

	teli_enable(&ti, 0);

	do {
		iSP = pm.i_setpoint_Q;
		vTaskDelay(xHold);

		pm.i_setpoint_Q = pm.i_maximal;
		vTaskDelay(xHold);

		pm.i_setpoint_Q = iSP;
		vTaskDelay(xHold);
	}
	while (0);
}

SH_DEF(pm_test_speed_ramp)
{
	float			wSP;
	int			xHold = 300;

	if (pm.lu_mode == PM_LU_DISABLED)
		return;

	teli_enable(&ti, 1000);

	do {
		wSP = pm.s_setpoint;
		vTaskDelay(xHold);

		pm.s_setpoint = pm.probe_speed_ramp;
		vTaskDelay(xHold);

		pm.s_setpoint = wSP;
		vTaskDelay(xHold);
	}
	while (0);
}

