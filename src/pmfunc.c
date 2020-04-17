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
	TickType_t		xTick = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 10);
		xTick += (TickType_t) 10;

		if (pm.fsm_state == PM_STATE_IDLE)
			break;

		if (xTick > (TickType_t) 5000) {

			pm.fail_reason = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	return pm.fail_reason;
}

int pm_wait_for_SPINUP(float ref)
{
	TickType_t		xTick = (TickType_t) 0;
	float			wS = pm.lu_flux_lpf_wS;

	do {
		vTaskDelay((TickType_t) 100);
		xTick += (TickType_t) 100;

		if (pm.fail_reason != PM_ERROR_TIMEOUT
				&& pm.fail_reason != PM_OK)
			break;

		if (ref - pm.lu_wS < 1E-1f * ref)
			break;

		if (pm.lu_wS - wS < - M_EPS_F * pm.lu_wS
				&& xTick > (TickType_t) 1000)
			break;

		wS = pm.lu_flux_lpf_wS;

		if (xTick > (TickType_t) 5000) {

			pm.fail_reason = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	vTaskDelay((TickType_t) 100);

	return pm.fail_reason;
}

int pm_wait_for_MOTION(float ref)
{
	TickType_t		xTick = (TickType_t) 0;
	int			revol = pm.im_revol_total;

	do {
		vTaskDelay((TickType_t) 100);
		xTick += (TickType_t) 100;

		if (pm.fail_reason != PM_ERROR_TIMEOUT
				&& pm.fail_reason != PM_OK)
			break;

		if (pm.im_revol_total != revol) {

			if (m_fabsf(pm.lu_flux_lpf_wS) > ref)
				break;
		}

		if (xTick > (TickType_t) 10000) {

			pm.fail_reason = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	return pm.fail_reason;
}

SH_DEF(pm_probe_base)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
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

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;

			if (pm_wait_for_IDLE() != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_R;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_R]);

		pm.fsm_req = PM_STATE_PROBE_CONST_L;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_L]);
		reg_format(&regfile[ID_PM_CONST_IM_LD]);
		reg_format(&regfile[ID_PM_CONST_IM_LQ]);
		reg_format(&regfile[ID_PM_CONST_IM_ANGLE]);
		reg_format(&regfile[ID_PM_CONST_IM_R]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_probe_spinup)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (pm.config_FORCED == PM_DISABLED) {

		printf("Unable when forced control is disabled" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		pm.s_setpoint = pm.probe_speed_hold;

		if (pm.const_E < M_EPS_F) {

			if (pm_wait_for_SPINUP(pm.forced_maximal) != PM_OK)
				break;

			pm.fsm_req = PM_STATE_PROBE_CONST_E;

			if (pm_wait_for_IDLE() != PM_OK)
				break;

			reg_format(&regfile[ID_PM_CONST_E_KV]);
		}

		if (pm_wait_for_SPINUP(pm.s_setpoint) != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);
		reg_format(&regfile[ID_PM_LU_FLUX_LPF_WS_RPM]);

		pm.fsm_req = PM_STATE_PROBE_LU_MPPE;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_LU_MPPE_RPM]);

		if (pm_wait_for_SPINUP(pm.s_setpoint) != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 50);

		pm.s_setpoint = pm.probe_speed_spinup;

		vTaskDelay((TickType_t) 200);

		pm.s_setpoint = pm.probe_speed_hold;

		vTaskDelay((TickType_t) 200);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_JA_KGM2]);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);

	if (pm.lu_mode != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
		pm_wait_for_IDLE();
	}
}

SH_DEF(pm_probe_detached)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_LU_DETACHED;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		if (pm_wait_for_MOTION(pm.probe_speed_detached) != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);
		reg_format(&regfile[ID_PM_LU_FLUX_LPF_WS_RPM]);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);

	if (pm.lu_mode != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
		pm_wait_for_IDLE();
	}
}

SH_DEF(pm_probe_const_E)
{
	do {
		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_probe_lu_MPPE)
{
	do {
		pm.fsm_req = PM_STATE_PROBE_LU_MPPE;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_LU_MPPE_RPM]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_probe_const_J)
{
	do {
		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 50);

		pm.s_setpoint = pm.probe_speed_spinup;

		vTaskDelay((TickType_t) 200);

		pm.s_setpoint = pm.probe_speed_hold;

		vTaskDelay((TickType_t) 200);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_JA_KGM2]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_adjust_HALL)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		pm.s_setpoint = pm.probe_speed_hold;

		if (pm_wait_for_SPINUP(pm.s_setpoint) != PM_OK)
			break;

		pm.fsm_req = PM_STATE_ADJUST_HALL;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_HALL_ST_1_G]);
		reg_format(&regfile[ID_PM_HALL_ST_2_G]);
		reg_format(&regfile[ID_PM_HALL_ST_3_G]);
		reg_format(&regfile[ID_PM_HALL_ST_4_G]);
		reg_format(&regfile[ID_PM_HALL_ST_5_G]);
		reg_format(&regfile[ID_PM_HALL_ST_6_G]);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);

	if (pm.lu_mode != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
		pm_wait_for_IDLE();
	}
}

SH_DEF(pm_fsm_detached)
{
	pm.fsm_req = PM_STATE_LU_DETACHED;
	pm_wait_for_IDLE();

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_fsm_startup)
{
	pm.fsm_req = PM_STATE_LU_STARTUP;
	pm_wait_for_IDLE();

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_fsm_shutdown)
{
	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	pm_wait_for_IDLE();

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

