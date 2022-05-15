#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "tlm.h"

int pm_wait_for_SETTLE()
{
	TickType_t		xTick = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 50);
		xTick += (TickType_t) 50;

		if (pm.fsm_errno != PM_OK)
			break;

		if (m_fabsf(pm.x_discrepancy) < (pm.x_tol_Z * 3.f))
			break;

		if (xTick > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	vTaskDelay((TickType_t) 100);

	return pm.fsm_errno;
}

SH_DEF(servo_probe_const_J)
{
	if (		pm.lu_MODE == PM_LU_DISABLED
			|| pm.lu_MODE == PM_LU_DETACHED
			|| pm.lu_MODE == PM_LU_FORCED
			|| pm.config_LU_DRIVE != PM_DRIVE_SERVO
			|| pm.const_ld_S < M_EPS_F) {

		printf("Enable SERVO mode before" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_LOCATION_MM, ap.servo_SPAN_mm[0]);

		if (pm_wait_for_SETTLE() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 100);

		reg_SET_F(ID_PM_X_SETPOINT_LOCATION_MM, ap.servo_SPAN_mm[1]);

		vTaskDelay((TickType_t) 300);

		reg_SET_F(ID_PM_X_SETPOINT_LOCATION_MM, ap.servo_SPAN_mm[0]);

		vTaskDelay((TickType_t) 300);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_JA_KG]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(servo_test_uniform)
{
	TickType_t		xWake, xTim0;
	float			xSP, wSP, tDT;
	int			DIRF;

	if (		pm.lu_MODE == PM_LU_DISABLED
			|| pm.lu_MODE == PM_LU_DETACHED
			|| pm.lu_MODE == PM_LU_FORCED
			|| pm.config_LU_DRIVE != PM_DRIVE_SERVO
			|| pm.const_ld_S < M_EPS_F) {

		printf("Enable SERVO mode before" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_LOCATION_MM, ap.servo_SPAN_mm[0]);

		if (pm_wait_for_SETTLE() != PM_OK)
			break;
	}
	while (0);

	xWake = xTaskGetTickCount();
	xTim0 = xWake;

	xSP = ap.servo_SPAN_mm[0];
	wSP = ap.servo_UNIFORM_mmps;
	tDT = 1.f / (float) configTICK_RATE_HZ;
	DIRF = 1;

	TLM_startup(&tlm, tlm.freq_grab_hz, TLM_MODE_SINGLE_GRAB);

	do {
		/* 1000 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 1);

		xSP += wSP * tDT * DIRF;

		reg_SET_F(ID_PM_X_SETPOINT_LOCATION_MM, xSP);
		reg_SET_F(ID_PM_X_SETPOINT_SPEED_MMPS, wSP * DIRF);

		if (DIRF == 1) {

			if (xSP >= ap.servo_SPAN_mm[1])
				DIRF = - 1;
		}
		else {
			if (xSP <= ap.servo_SPAN_mm[0])
				break;
		}

		if ((xWake - xTim0) > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		if (pm.fsm_errno != PM_OK)
			break;
	}
	while (1);

	reg_SET_F(ID_PM_X_SETPOINT_SPEED_MMPS, 0.f);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

