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

void pm_print_fail_reason()
{
	if (pm.fail_reason != PM_OK) {

		printf("%s" EOL, pm_strerror(pm.fail_reason));
	}
}

int pm_wait_for_IDLE()
{
	do {
		vTaskDelay((TickType_t) 5);

		if (pm.fsm_state == PM_STATE_IDLE)
			break;
	}
	while (1);

	return pm.fail_reason;
}

void pm_ppm_control_tune()
{
	float			radps, rpm;

	if (ap.ppm_reg_ID == ID_PM_S_SETPOINT_RPM) {

		if (pm.const_E != 0.f) {

			radps = (2.f / 3.f) * pm.const_lpf_U / pm.const_E;
			rpm = 5.513289f / (radps * pm.const_Zp);
			ap.ppm_control_range[1] = rpm;
		}
	}
}

SH_DEF(pm_config_default_1)
{
	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	pm_config_default(&pm);
}

SH_DEF(pm_config_tune_current_loop_1)
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
		reg_format(&regfile[ID_PM_CONST_LPF_U]);

		pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_ADJUST_IA_0]);
		reg_format(&regfile[ID_PM_ADJUST_IB_0]);

		if (pm.fail_reason != PM_OK)
			break;

		/*pm_fsm_req(&pm, PM_STATE_POWER_STAGE_SELF_TEST);

		if (pm_wait_for_IDLE() != PM_OK)
			break;*/

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_R);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_R);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_R]);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_L);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_L);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_LD]);
		reg_format(&regfile[ID_PM_CONST_LQ]);

		pm_config_tune_current_loop(&pm);
	}
	while (0);

	pm_print_fail_reason();
}

SH_DEF(pm_probe_spinup)
{
	TickType_t		xWait;

	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	do {
		pm_fsm_req(&pm, PM_STATE_LU_INITIATE);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		pm.s_setpoint = pm.probe_speed_low;

		xWait = (TickType_t) (pm.s_setpoint / pm.forced_accel * 1000.f);
		xWait += (TickType_t) 100;

		vTaskDelay(xWait);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_E);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);
	}
	while (0);

	pm_print_fail_reason();
}

SH_DEF(pm_fsm_req_lu_initiate)
{
	do {
		pm_fsm_req(&pm, PM_STATE_LU_INITIATE);

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	pm_print_fail_reason();
}

SH_DEF(pm_fsm_req_lu_shutdown)
{
	do {
		pm_fsm_req(&pm, PM_STATE_LU_SHUTDOWN);

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	pm_print_fail_reason();
}

