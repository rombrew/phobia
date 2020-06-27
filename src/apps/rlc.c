#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "shell.h"

/* This is RLC meter application.
 * */

SH_DEF(rlc_probe_R)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (PM_CONFIG_NOP(&pm) != PM_NOP_ONE_PHASE) {

		printf("Enable ONE phase before" EOL);
		return;
	}

	do {
		reg_format(&regfile[ID_PM_CONST_FB_U]);

		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_AD_IA_0]);
		reg_format(&regfile[ID_PM_AD_IB_0]);

		if (pm.fail_reason != PM_OK)
			break;

		reg_format(&regfile[ID_PM_PROBE_CURRENT_HOLD]);

		pm.fsm_req = PM_STATE_PROBE_CONST_R;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_IM_R]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(rlc_probe_L)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (PM_CONFIG_NOP(&pm) != PM_NOP_ONE_PHASE) {

		printf("Enable ONE phase before" EOL);
		return;
	}

	do {
		reg_format(&regfile[ID_PM_CONST_FB_U]);

		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_AD_IA_0]);
		reg_format(&regfile[ID_PM_AD_IB_0]);

		if (pm.fail_reason != PM_OK)
			break;

		reg_format(&regfile[ID_PM_PROBE_CURRENT_WEAK]);
		reg_format(&regfile[ID_PM_PROBE_CURRENT_SINE]);
		reg_format(&regfile[ID_PM_PROBE_FREQ_SINE_HZ]);

		pm.fsm_req = PM_STATE_PROBE_CONST_L;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_IM_L1]);
		reg_format(&regfile[ID_PM_CONST_IM_R]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

