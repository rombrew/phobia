#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "tel.h"

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

SH_DEF(pm_default)
{
	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	pm_config_default(&pm);
}

SH_DEF(pm_tune_current_loop)
{
	if (pm.lu_mode != PM_LU_DISABLED)
		return;

	pm_config_tune_current_loop(&pm);
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

		pm_fsm_req(&pm, PM_STATE_SELF_TEST_POWER_STAGE);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

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

	reg_format(&regfile[ID_PM_FAIL_REASON]);
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

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

