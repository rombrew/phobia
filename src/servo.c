#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "tlm.h"

int pm_wait_for_settle()
{
	TickType_t		xTick = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 50);
		xTick += (TickType_t) 50;

		if (pm.fsm_errno != PM_OK)
			break;

		if (m_fabsf(pm.x_discrepancy) < (pm.x_tolerance * 3.f))
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
			|| pm.config_LU_DRIVE != PM_DRIVE_SERVO) {

		printf("Enable SERVO mode before" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_LOCATION, pm.x_location_range[0]);

		if (pm_wait_for_settle() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 100);

		reg_SET_F(ID_PM_X_SETPOINT_LOCATION, pm.x_location_range[1]);

		vTaskDelay((TickType_t) 300);

		reg_SET_F(ID_PM_X_SETPOINT_LOCATION, pm.x_location_range[0]);

		vTaskDelay((TickType_t) 300);

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_JA_KG]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(servo_test_uniform)
{
	float			wSP = 1.f;

	if (		pm.lu_MODE == PM_LU_DISABLED
			|| pm.config_LU_DRIVE != PM_DRIVE_SERVO) {

		printf("Enable SERVO mode before" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_LOCATION, pm.x_location_range[0]);

		if (pm_wait_for_settle() != PM_OK)
			break;

		reg_SET_F(ID_PM_X_SETPOINT_SPEED, wSP);

		/* TODO */
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

