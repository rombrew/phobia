#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "shell.h"

/* This is mice control application.
 * */

SH_DEF(mice)
{
	float		mice_x, mice_y;

	if (		pm.lu_mode == PM_LU_DISABLED
			&& pm.fsm_state == PM_STATE_IDLE) {

		pm.fsm_req = PM_STATE_LU_STARTUP;
	}

	if (pm.lu_mode == PM_LU_SENSOR_ABI) {

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
}

