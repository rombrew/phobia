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

SH_DEF(pm_standard_voltage)
{
	float		STD;

	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (stof(&STD, s) != NULL) {

		/* The value could be validated here.
		 * */
	}
	else {
		printf("You must specify standard voltage" EOL);
		return;
	}

	do {
		reg_format(&regfile[ID_PM_CONST_LPF_U]);

		pm.probe_DFT[4] = STD;

		pm_fsm_req(&pm, PM_STATE_STANDARD_VOLTAGE);
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_ADJUST_US_0]);
		reg_format(&regfile[ID_PM_ADJUST_US_1]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_standard_current)
{
	float		STD;

	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (stof(&STD, s) != NULL) {

		/* The value could be validated here.
		 * */
	}
	else {
		printf("You must specify standard resistance" EOL);
		return;
	}

	do {
		reg_format(&regfile[ID_PM_CONST_LPF_U]);

		pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_ADJUST_IA_0]);
		reg_format(&regfile[ID_PM_ADJUST_IB_0]);

		if (pm.fail_reason != PM_OK)
			break;

		pm.probe_DFT[4] = STD;

		pm_fsm_req(&pm, PM_STATE_STANDARD_CURRENT);
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_ADJUST_IA_1]);
		reg_format(&regfile[ID_PM_ADJUST_IB_1]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_self_adjust)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		reg_format(&regfile[ID_PM_CONST_LPF_U]);

		pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_ADJUST_IA_0]);
		reg_format(&regfile[ID_PM_ADJUST_IB_0]);

		if (pm.fail_reason != PM_OK)
			break;

		if (PM_CONFIG_VM(&pm) == PM_ENABLED) {

			pm_fsm_req(&pm, PM_STATE_ADJUST_VOLTAGE);
			pm_wait_for_IDLE();

			reg_format(&regfile[ID_PM_ADJUST_UA_0]);
			reg_format(&regfile[ID_PM_ADJUST_UA_1]);
			reg_format(&regfile[ID_PM_ADJUST_UB_0]);
			reg_format(&regfile[ID_PM_ADJUST_UB_1]);
			reg_format(&regfile[ID_PM_ADJUST_UC_0]);
			reg_format(&regfile[ID_PM_ADJUST_UC_1]);

			reg_format(&regfile[ID_PM_VM_FIR_A_0]);
			reg_format(&regfile[ID_PM_VM_FIR_A_1]);
			reg_format(&regfile[ID_PM_VM_FIR_A_2]);
			reg_format(&regfile[ID_PM_VM_FIR_B_0]);
			reg_format(&regfile[ID_PM_VM_FIR_B_1]);
			reg_format(&regfile[ID_PM_VM_FIR_B_2]);
			reg_format(&regfile[ID_PM_VM_FIR_C_0]);
			reg_format(&regfile[ID_PM_VM_FIR_C_1]);
			reg_format(&regfile[ID_PM_VM_FIR_C_2]);

			if (pm.fail_reason != PM_OK)
				break;
		}

		pm_fsm_req(&pm, PM_STATE_ADJUST_CURRENT);
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_ADJUST_IA_1]);
		reg_format(&regfile[ID_PM_ADJUST_IB_1]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_probe_base)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		reg_format(&regfile[ID_PM_CONST_LPF_U]);

		pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_ADJUST_IA_0]);
		reg_format(&regfile[ID_PM_ADJUST_IB_0]);

		if (pm.fail_reason != PM_OK)
			break;

		if (PM_CONFIG_VM(&pm) == PM_ENABLED) {

			pm_fsm_req(&pm, PM_STATE_SELF_TEST_POWER_STAGE);

			if (pm_wait_for_IDLE() != PM_OK)
				break;
		}

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_R);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_R]);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_L);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_LD]);
		reg_format(&regfile[ID_PM_CONST_LQ]);
		reg_format(&regfile[ID_PM_PROBE_IMPEDANCE_R]);
		reg_format(&regfile[ID_PM_PROBE_ROTATION_DQ]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_probe_spinup)
{
	TickType_t		xWait;

	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		pm_fsm_req(&pm, PM_STATE_LU_STARTUP);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		xWait = (TickType_t) (pm.probe_speed_low / pm.forced_accel * 1000.f);
		xWait += (TickType_t) 100;

		pm.s_setpoint = pm.probe_speed_low;

		vTaskDelay(xWait);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_E);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);

		xWait = (TickType_t) (pm.probe_speed_ramp / pm.s_accel * 1000.f);
		xWait = (TickType_t) 100;

		pm.s_setpoint = pm.probe_speed_ramp;

		vTaskDelay(xWait);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_E);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
	pm_fsm_req(&pm, PM_STATE_LU_SHUTDOWN);
}

SH_DEF(pm_fsm_startup)
{
	do {
		pm_fsm_req(&pm, PM_STATE_LU_STARTUP);

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_fsm_shutdown)
{
	do {
		pm_fsm_req(&pm, PM_STATE_LU_SHUTDOWN);

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

