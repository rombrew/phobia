#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "tel.h"

/* This file contains the LINEAR drive service functions.
 * */

int pm_wait_for_SETTLE()
{
	TickType_t		xTick = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 100);
		xTick += (TickType_t) 100;

		if (pm.fail_reason != PM_OK)
			break;

		if (m_fabsf(pm.s_setpoint) < pm.lu_MPPE)
			break;

		if (xTick > (TickType_t) 5000) {

			pm.fail_reason = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	vTaskDelay((TickType_t) 100);

	return pm.fail_reason;
}

SH_DEF(ld_probe_const_J)
{
	if (		pm.lu_mode == PM_LU_DISABLED
			|| pm.lu_mode == PM_LU_DETACHED
			|| pm.lu_mode == PM_LU_FORCED
			|| pm.config_SERVO == PM_DISABLED
			|| pm.const_D < M_EPS_F) {

		printf("Unable when SERVO mode is not functioning" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_FM, ap.ld_probe_m[0]);
		reg_format(&regfile[ID_PM_X_SETPOINT_FM]);

		if (pm_wait_for_SETTLE() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 100);

		reg_SET_F(ID_PM_X_SETPOINT_FM, ap.ld_probe_m[1]);
		reg_format(&regfile[ID_PM_X_SETPOINT_FM]);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_J]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(ld_FT_uniform)
{
	TickType_t		xWake, xTim0;
	float			xSP, xDP;
	int			DIRF;

	if (		pm.lu_mode == PM_LU_DISABLED
			|| pm.lu_mode == PM_LU_DETACHED
			|| pm.lu_mode == PM_LU_FORCED
			|| pm.config_SERVO == PM_DISABLED
			|| pm.const_D < M_EPS_F) {

		printf("Unable when SERVO mode is not functioning" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_FM, ap.ld_probe_m[0]);
		reg_format(&regfile[ID_PM_X_SETPOINT_FM]);

		if (pm_wait_for_SETTLE() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);

	xWake = xTaskGetTickCount();
	xTim0 = xWake;

	xSP = ap.ld_probe_m[0];
	xDP = ap.ld_probe_mps / 1000.f;
	DIRF = 1;

	tel_startup(&ti, ap.FT_grab_hz, TEL_MODE_SINGLE_GRAB);

	do {
		/* 1000 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 1);

		xSP += xDP * DIRF;

		reg_SET_F(ID_PM_X_SETPOINT_FM, xSP);

		if (DIRF == 1) {

			if (xSP >= ap.ld_probe_m[1])
				DIRF = - 1;
		}
		else {
			if (xSP <= ap.ld_probe_m[0])
				break;
		}

		if ((xWake - xTim0) > (TickType_t) 8000) {

			pm.fail_reason = PM_ERROR_TIMEOUT;
			break;
		}

		if (pm.fail_reason != PM_OK)
			break;
	}
	while (1);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

