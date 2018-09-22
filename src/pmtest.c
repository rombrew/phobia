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

static void
pm_print_self_BM()
{
	printf("BM = %2x %2x %2x %2x %2x %2x" EOL, pm.self_BM[0],
			pm.self_BM[1], pm.self_BM[2], pm.self_BM[3],
			pm.self_BM[4], pm.self_BM[5], pm.self_BM[6]);
}

SH_DEF(pm_self_test)
{
	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	do {
		pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);
		pm_wait_for_IDLE();

		reg_print_fmt(&regfile[ID_PM_ADJUST_IA_0], 1);
		reg_print_fmt(&regfile[ID_PM_ADJUST_IB_0], 1);

		if (pm.fail_reason != PM_OK)
			break;

		pm_fsm_req(&pm, PM_STATE_POWER_STAGE_SELF_TEST);
		pm_wait_for_IDLE();

		pm_print_self_BM();

		if (pm.fail_reason != PM_OK && pm.fail_reason != PM_ERROR_NO_MOTOR_CONNECTED)
			break;
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
	TickType_t		xHold = (TickType_t) 3;

	if (pm.lu_mode == PM_LU_DISABLED)
		return;

	teli_startup(&ti, 0, TEL_MODE_SINGLE_GRAB);

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
	TickType_t		xHold = (TickType_t) 300;

	if (pm.lu_mode == PM_LU_DISABLED)
		return;

	teli_startup(&ti, 1000, TEL_MODE_SINGLE_GRAB);

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

