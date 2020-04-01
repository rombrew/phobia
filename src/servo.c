#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "tel.h"

int pm_wait_for_SETTLE()
{
	TickType_t		xTick = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 100);
		xTick += (TickType_t) 100;

		if (pm.fail_reason != PM_OK)
			break;

		if (m_fabsf(pm.x_residual) < pm.x_near_tol)
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

SH_DEF(servo_probe_const_J)
{
	if (		pm.lu_mode == PM_LU_DISABLED
			|| pm.lu_mode == PM_LU_DETACHED
			|| pm.lu_mode == PM_LU_FORCED
			|| pm.config_SERVO == PM_DISABLED
			|| pm.const_ld_S < M_EPS_F) {

		printf("Unable when SERVO mode is not functioning" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_F_MM, ap.servo_span_mm[0]);

		if (pm_wait_for_SETTLE() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 50);

		reg_SET_F(ID_PM_X_SETPOINT_F_MM, ap.servo_span_mm[1]);

		vTaskDelay((TickType_t) 200);

		reg_SET_F(ID_PM_X_SETPOINT_F_MM, ap.servo_span_mm[0]);

		vTaskDelay((TickType_t) 200);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_JA_KG]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(mice)
{
	float		mice_x, mice_y;

	/* FIXME: Move to application */

	stof(&mice_x, s);
	stof(&mice_y, sh_next_arg(s));

	if (ap.servo_mice_role == 1) {

		mice_x += reg_GET_F(ID_PM_X_SETPOINT_F_MM);
		reg_SET_F(ID_PM_X_SETPOINT_F_MM, mice_x);
	}
	else if (ap.servo_mice_role == 2) {

		mice_y += reg_GET_F(ID_PM_X_SETPOINT_F_MM);
		reg_SET_F(ID_PM_X_SETPOINT_F_MM, mice_y);
	}
}

SH_DEF(servo_FT_uniform)
{
	TickType_t		xWake, xTim0;
	float			xSP, wSP, tDT;
	int			DIRF;

	if (		pm.lu_mode == PM_LU_DISABLED
			|| pm.lu_mode == PM_LU_DETACHED
			|| pm.lu_mode == PM_LU_FORCED
			|| pm.config_SERVO == PM_DISABLED
			|| pm.const_ld_S < M_EPS_F) {

		printf("Unable when SERVO mode is not functioning" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_F_MM, ap.servo_span_mm[0]);

		if (pm_wait_for_SETTLE() != PM_OK)
			break;
	}
	while (0);

	xWake = xTaskGetTickCount();
	xTim0 = xWake;

	xSP = ap.servo_span_mm[0];
	wSP = ap.servo_uniform_mmps;
	tDT = 1.f / (float) configTICK_RATE_HZ;
	DIRF = 1;

	tel_startup(&ti, ap.FT_grab_hz, TEL_MODE_SINGLE_GRAB);

	do {
		/* 1000 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 1);

		xSP += wSP * tDT * DIRF;

		reg_SET_F(ID_PM_X_SETPOINT_F_MM, xSP);
		reg_SET_F(ID_PM_X_SETPOINT_WS_MMPS, wSP * DIRF);

		if (DIRF == 1) {

			if (xSP >= ap.servo_span_mm[1])
				DIRF = - 1;
		}
		else {
			if (xSP <= ap.servo_span_mm[0])
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

	reg_SET_F(ID_PM_X_SETPOINT_WS_MMPS, 0.f);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

