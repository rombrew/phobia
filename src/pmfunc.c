#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "tlm.h"

int pm_wait_for_IDLE()
{
	TickType_t		xTIME = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 10);

		if (pm.fsm_state == PM_STATE_IDLE)
			break;

		if (xTIME > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		xTIME += (TickType_t) 10;
	}
	while (1);

	return pm.fsm_errno;
}

int pm_wait_for_SPINUP()
{
	TickType_t		xTIME = (TickType_t) 0;
	const float		wS_tol = 10.f;

	do {
		vTaskDelay((TickType_t) 50);

		if (pm.fsm_errno != PM_OK)
			break;

		/* Check the target speed has reached.
		 * */
		if (m_fabsf(pm.lu_wS) + wS_tol > pm.s_setpoint_speed)
			break;

		if (		pm.lu_MODE == PM_LU_FORCED
				&& pm.vsi_lpf_DC > pm.forced_maximal_DC) {

			pm.s_setpoint_speed = pm.lu_wS;
			break;
		}

		if (xTIME > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		xTIME += (TickType_t) 50;
	}
	while (1);

	vTaskDelay((TickType_t) 100);

	return pm.fsm_errno;
}

int pm_wait_for_MOTION(float s_ref)
{
	TickType_t		xTIME = (TickType_t) 0;
	int			revol = pm.lu_total_revol;

	do {
		vTaskDelay((TickType_t) 50);

		if (pm.fsm_errno != PM_OK)
			break;

		if (pm.lu_total_revol != revol) {

			if (m_fabsf(pm.zone_lpf_wS) > s_ref)
				break;
		}

		if (xTIME > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		xTIME += (TickType_t) 50;
	}
	while (1);

	return pm.fsm_errno;
}

SH_DEF(pm_probe_base)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		reg_format(&regfile[ID_PM_CONST_FB_U]);

		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_AD_IA_0]);
		reg_format(&regfile[ID_PM_AD_IB_0]);
		reg_format(&regfile[ID_PM_AD_IC_0]);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;

			if (pm_wait_for_IDLE() != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_R;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		pm.const_R = pm.const_im_R;

		reg_format(&regfile[ID_PM_CONST_R]);

		pm.fsm_req = PM_STATE_PROBE_CONST_L;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_IM_L1]);
		reg_format(&regfile[ID_PM_CONST_IM_L2]);
		reg_format(&regfile[ID_PM_CONST_IM_B]);
		reg_format(&regfile[ID_PM_CONST_IM_R]);

		pm_tune(&pm, PM_TUNE_MAXIMAL_CURRENT);
		pm_tune(&pm, PM_TUNE_LOOP_CURRENT);

		reg_format(&regfile[ID_PM_I_MAXIMAL]);
		reg_format(&regfile[ID_PM_I_GAIN_P]);
		reg_format(&regfile[ID_PM_I_GAIN_I]);
		reg_format(&regfile[ID_PM_I_SLEW_RATE]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_probe_spinup)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (		pm.config_LU_FORCED != PM_ENABLED
			|| pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

		printf("Unable without SPEED loop and FORCED" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		if (pm.const_E < M_EPS_F) {

			reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

			if (pm_wait_for_SPINUP() != PM_OK)
				break;

			reg_format(&regfile[ID_PM_ZONE_LPF_WS]);

			pm.fsm_req = PM_STATE_PROBE_CONST_E;

			if (pm_wait_for_IDLE() != PM_OK)
				break;

			reg_format(&regfile[ID_PM_CONST_E_KV]);
		}

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

		if (pm_wait_for_SPINUP() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_ZONE_LPF_WS]);

		if (pm.flux_ZONE != PM_ZONE_HIGH) {

			pm.fsm_errno = PM_ERROR_NO_FLUX_CAUGHT;
			break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);

		pm_tune(&pm, PM_TUNE_ZONE_THRESHOLD);

		reg_format(&regfile[ID_PM_ZONE_MPPE]);
		reg_format(&regfile[ID_PM_ZONE_MURE]);
		reg_format(&regfile[ID_PM_ZONE_GAIN_TA]);
		reg_format(&regfile[ID_PM_ZONE_GAIN_GI]);

		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 100);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, 110.f);

		vTaskDelay((TickType_t) 300);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

		vTaskDelay((TickType_t) 300);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_JA_KGM2]);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		pm_tune(&pm, PM_TUNE_LOOP_FORCED);
		pm_tune(&pm, PM_TUNE_LOOP_SPEED);

		reg_format(&regfile[ID_PM_FORCED_ACCEL]);
		reg_format(&regfile[ID_PM_LU_GAIN_TQ]);
		reg_format(&regfile[ID_PM_S_GAIN_P]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	if (pm.lu_MODE != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}
}

SH_DEF(pm_probe_detached)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

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

		reg_format(&regfile[ID_PM_ZONE_LPF_WS]);
		reg_format(&regfile[ID_PM_CONST_E_KV]);

		pm_tune(&pm, PM_TUNE_ZONE_THRESHOLD);

		reg_format(&regfile[ID_PM_ZONE_MPPE]);
		reg_format(&regfile[ID_PM_ZONE_MURE]);
		reg_format(&regfile[ID_PM_ZONE_GAIN_TA]);
		reg_format(&regfile[ID_PM_ZONE_GAIN_GI]);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	if (pm.lu_MODE != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}
}

SH_DEF(pm_probe_const_E)
{
	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is NOT running" EOL);
		return;
	}

	if (pm.flux_ZONE != PM_ZONE_HIGH) {

		printf("Unable when FLUX is NOT in HIGH zone" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);

		pm_tune(&pm, PM_TUNE_ZONE_THRESHOLD);

		reg_format(&regfile[ID_PM_ZONE_MPPE]);
		reg_format(&regfile[ID_PM_ZONE_MURE]);
		reg_format(&regfile[ID_PM_ZONE_GAIN_TA]);
		reg_format(&regfile[ID_PM_ZONE_GAIN_GI]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_probe_const_J)
{
	float		wSP;

	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is NOT running" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

		printf("Unable when SPEED loop is DISABLED" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 100);

		wSP = reg_GET_F(ID_PM_S_SETPOINT_SPEED);
		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, 110.f);

		vTaskDelay((TickType_t) 300);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, wSP);

		vTaskDelay((TickType_t) 300);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_JA_KGM2]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_adjust_sensor_hall)
{
	int		ACTIVE = 0;

	do {
		if (pm.lu_MODE == PM_LU_DISABLED) {

			if (pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

				printf("Unable when SPEED loop is DISABLED" EOL);
				return;
			}

			pm.fsm_req = PM_STATE_LU_STARTUP;

			if (pm_wait_for_IDLE() != PM_OK)
				break;

			reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

			if (pm_wait_for_SPINUP() != PM_OK)
				break;

			ACTIVE = 1;
		}
		else {
			if (m_fabsf(pm.lu_wS) < pm.probe_speed_hold * .5f) {

				printf("Unable at too LOW speed" EOL);
				return;
			}
		}

		pm.fsm_req = PM_STATE_ADJUST_SENSOR_HALL;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_HALL_ST_1]);
		reg_format(&regfile[ID_PM_HALL_ST_2]);
		reg_format(&regfile[ID_PM_HALL_ST_3]);
		reg_format(&regfile[ID_PM_HALL_ST_4]);
		reg_format(&regfile[ID_PM_HALL_ST_5]);
		reg_format(&regfile[ID_PM_HALL_ST_6]);

		if (ACTIVE != 0) {

			pm.fsm_req = PM_STATE_LU_SHUTDOWN;

			if (pm_wait_for_IDLE() != PM_OK)
				break;
		}
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	if (ACTIVE != 0 && pm.lu_MODE != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}
}

SH_DEF(pm_adjust_sensor_abi)
{
	/* TODO */
}

SH_DEF(pm_adjust_sensor_sincos)
{
	/* TODO */
}

SH_DEF(pm_fsm_detached)
{
	pm.fsm_req = PM_STATE_LU_DETACHED;
	pm_wait_for_IDLE();

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_fsm_startup)
{
	pm.fsm_req = PM_STATE_LU_STARTUP;
	pm_wait_for_IDLE();

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_fsm_shutdown)
{
	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	pm_wait_for_IDLE();

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_default_all)
{
	pm_tune(&pm, PM_TUNE_ALL_DEFAULT);
}

SH_DEF(pm_default_probe)
{
	pm_tune(&pm, PM_TUNE_PROBE_DEFAULT);

	reg_format(&regfile[ID_PM_FORCED_ACCEL]);
	reg_format(&regfile[ID_PM_ZONE_MPPE]);
	reg_format(&regfile[ID_PM_ZONE_MURE]);
	reg_format(&regfile[ID_PM_CONST_E]);
	reg_format(&regfile[ID_PM_CONST_E_KV]);
	reg_format(&regfile[ID_PM_CONST_R]);
	reg_format(&regfile[ID_PM_CONST_JA]);
	reg_format(&regfile[ID_PM_CONST_JA_KGM2]);
	reg_format(&regfile[ID_PM_CONST_JA_KG]);
	reg_format(&regfile[ID_PM_CONST_IM_L1]);
	reg_format(&regfile[ID_PM_CONST_IM_L2]);
	reg_format(&regfile[ID_PM_I_MAXIMAL]);
	reg_format(&regfile[ID_PM_I_REVERSE]);
	reg_format(&regfile[ID_PM_I_SLEW_RATE]);
	reg_format(&regfile[ID_PM_I_GAIN_P]);
	reg_format(&regfile[ID_PM_I_GAIN_I]);
	reg_format(&regfile[ID_PM_LU_GAIN_TQ]);
	reg_format(&regfile[ID_PM_S_GAIN_P]);
}

