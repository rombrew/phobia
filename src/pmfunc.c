#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "tlm.h"

int pm_wait_for_idle()
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

int pm_wait_for_spinup(float ref)
{
	TickType_t		xTIME = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 50);

		if (pm.fsm_errno != PM_OK)
			break;

		/* Check the target speed has reached.
		 * */
		if (m_fabsf(pm.lu_wS) + 10.f > ref)
			break;

		if (		pm.lu_MODE == PM_LU_FORCED
				&& pm.vsi_lpf_DC > pm.forced_maximal_DC) {

			ref = pm.lu_wS;
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

int pm_wait_for_motion(float ref)
{
	TickType_t		xTIME = (TickType_t) 0;
	int			revob = pm.lu_total_revol;

	do {
		vTaskDelay((TickType_t) 50);

		if (pm.fsm_errno != PM_OK)
			break;

		if (pm.lu_total_revol != revob) {

			if (m_fabsf(pm.zone_lpf_wS) > ref)
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

	ap.probe_LOCK = PM_ENABLED;

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_for_idle();

		reg_format(&regfile[ID_PM_CONST_FB_U]);
		reg_format(&regfile[ID_PM_AD_IA0]);
		reg_format(&regfile[ID_PM_AD_IB0]);
		reg_format(&regfile[ID_PM_AD_IC0]);
		reg_format(&regfile[ID_PM_SELF_STDI]);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;

			if (pm_wait_for_idle() != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_R;

		if (pm_wait_for_idle() != PM_OK)
			break;

		pm.const_R = pm.const_im_R;

		reg_format(&regfile[ID_PM_CONST_R]);

		pm.fsm_req = PM_STATE_PROBE_CONST_L;

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_IM_L1]);
		reg_format(&regfile[ID_PM_CONST_IM_L2]);
		reg_format(&regfile[ID_PM_CONST_IM_B]);
		reg_format(&regfile[ID_PM_CONST_IM_R]);

		pm_auto(&pm, PM_AUTO_MAXIMAL_CURRENT);
		pm_auto(&pm, PM_AUTO_LOOP_CURRENT);

		reg_format(&regfile[ID_PM_I_MAXIMAL]);
		reg_format(&regfile[ID_PM_I_GAIN_P]);
		reg_format(&regfile[ID_PM_I_GAIN_I]);
		reg_format(&regfile[ID_PM_I_SLEW_RATE]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	ap.probe_LOCK = PM_DISABLED;
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

	ap.probe_LOCK = PM_ENABLED;

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (pm_wait_for_idle() != PM_OK)
			break;

		if (pm.const_E < M_EPS_F) {

			reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

			if (pm_wait_for_spinup(pm.probe_speed_hold) != PM_OK)
				break;

			reg_format(&regfile[ID_PM_ZONE_LPF_WS]);

			pm.fsm_req = PM_STATE_PROBE_CONST_E;

			if (pm_wait_for_idle() != PM_OK)
				break;

			reg_format(&regfile[ID_PM_CONST_E_KV]);

			pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);
			pm_auto(&pm, PM_AUTO_PROBE_SPEED_HOLD);
			pm_auto(&pm, PM_AUTO_FORCED_MAXIMAL);

			reg_format(&regfile[ID_PM_PROBE_SPEED_HOLD]);
			reg_format(&regfile[ID_PM_ZONE_THRESHOLD_NOISE]);
			reg_format(&regfile[ID_PM_ZONE_THRESHOLD_BASE]);
		}

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

		if (pm_wait_for_spinup(pm.probe_speed_hold) != PM_OK)
			break;

		reg_format(&regfile[ID_PM_ZONE_LPF_WS]);

		if (pm.flux_ZONE != PM_ZONE_HIGH) {

			pm.fsm_errno = PM_ERROR_NO_FLUX_CAUGHT;
			break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);

		pm.fsm_req = PM_STATE_PROBE_NOISE_THRESHOLD;

		if (pm_wait_for_idle() != PM_OK)
			break;

		pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);

		reg_format(&regfile[ID_PM_ZONE_THRESHOLD_NOISE]);
		reg_format(&regfile[ID_PM_ZONE_THRESHOLD_BASE]);

		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 100);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, 110.f);

		vTaskDelay((TickType_t) 300);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

		vTaskDelay((TickType_t) 300);

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_JA_KGM2]);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_idle() != PM_OK)
			break;

		pm_auto(&pm, PM_AUTO_FORCED_MAXIMAL);
		pm_auto(&pm, PM_AUTO_FORCED_ACCEL);
		pm_auto(&pm, PM_AUTO_LOOP_SPEED);

		reg_format(&regfile[ID_PM_FORCED_MAXIMAL]);
		reg_format(&regfile[ID_PM_FORCED_ACCEL]);
		reg_format(&regfile[ID_PM_LU_GAIN_MQ_LP]);
		reg_format(&regfile[ID_PM_S_GAIN_P]);
		reg_format(&regfile[ID_PM_S_GAIN_I]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	if (pm.lu_MODE != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}

	ap.probe_LOCK = PM_DISABLED;
}

SH_DEF(pm_probe_detached)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	ap.probe_LOCK = PM_ENABLED;

	do {
		pm.fsm_req = PM_STATE_LU_DETACHED;

		if (pm_wait_for_idle() != PM_OK)
			break;

		if (pm_wait_for_motion(pm.probe_speed_detached) != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_ZONE_LPF_WS]);
		reg_format(&regfile[ID_PM_CONST_E_KV]);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_idle() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	if (pm.lu_MODE != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}

	ap.probe_LOCK = PM_DISABLED;
}

SH_DEF(pm_probe_const_R)
{
	float		R[3];

	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	ap.probe_LOCK = PM_ENABLED;

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_for_idle();

		reg_format(&regfile[ID_PM_CONST_FB_U]);
		reg_format(&regfile[ID_PM_AD_IA0]);
		reg_format(&regfile[ID_PM_AD_IB0]);
		reg_format(&regfile[ID_PM_AD_IC0]);
		reg_format(&regfile[ID_PM_SELF_STDI]);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;

			if (pm_wait_for_idle() != PM_OK)
				break;
		}

		pm.probe_hold_angle = 0.f;

		pm.fsm_req = PM_STATE_PROBE_CONST_R;

		if (pm_wait_for_idle() != PM_OK)
			break;

		R[0] = pm.const_im_R;

		reg_format(&regfile[ID_PM_CONST_IM_R]);

		pm.probe_hold_angle = 120.f;

		pm.fsm_req = PM_STATE_PROBE_CONST_R;

		if (pm_wait_for_idle() != PM_OK)
			break;

		R[1] = pm.const_im_R;

		reg_format(&regfile[ID_PM_CONST_IM_R]);

		pm.probe_hold_angle = - 120.f;

		pm.fsm_req = PM_STATE_PROBE_CONST_R;

		if (pm_wait_for_idle() != PM_OK)
			break;

		R[2] = pm.const_im_R;

		reg_format(&regfile[ID_PM_CONST_IM_R]);

		pm.const_R = (R[0] + R[1] + R[2]) / 3.f;

		reg_format(&regfile[ID_PM_CONST_R]);
	}
	while (0);

	pm.probe_hold_angle = 0.f;

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	ap.probe_LOCK = PM_DISABLED;
}

SH_DEF(pm_probe_const_E)
{
	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	if (pm.flux_ZONE != PM_ZONE_HIGH) {

		printf("Unable when FLUX is NOT in HIGH zone" EOL);
		return;
	}

	ap.probe_LOCK = PM_ENABLED;

	do {
		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	ap.probe_LOCK = PM_DISABLED;
}

SH_DEF(pm_probe_const_J)
{
	float		wSP;

	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

		printf("Unable when SPEED loop is DISABLED" EOL);
		return;
	}

	ap.probe_LOCK = PM_ENABLED;

	do {
		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 100);

		wSP = reg_GET_F(ID_PM_S_SETPOINT_SPEED);
		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, 110.f);

		vTaskDelay((TickType_t) 300);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, wSP);

		vTaskDelay((TickType_t) 300);

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_JA_KGM2]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	ap.probe_LOCK = PM_DISABLED;
}

SH_DEF(pm_probe_noise_threshold)
{
	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	if (pm.flux_ZONE != PM_ZONE_HIGH) {

		printf("Unable when FLUX is NOT in HIGH zone" EOL);
		return;
	}

	ap.probe_LOCK = PM_ENABLED;

	do {
		pm.fsm_req = PM_STATE_PROBE_NOISE_THRESHOLD;

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_ZONE_THRESHOLD_NOISE]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	ap.probe_LOCK = PM_DISABLED;
}

SH_DEF(pm_adjust_sensor_hall)
{
	int		ACTIVE = 0;

	if (pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

		printf("Unable when SPEED loop is DISABLED" EOL);
		return;
	}

	ap.probe_LOCK = PM_ENABLED;

	do {
		if (pm.lu_MODE == PM_LU_DISABLED) {

			pm.fsm_req = PM_STATE_LU_STARTUP;

			if (pm_wait_for_idle() != PM_OK)
				break;

			reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

			if (pm_wait_for_spinup(pm.probe_speed_hold) != PM_OK)
				break;

			ACTIVE = 1;
		}

		if (m_fabsf(pm.zone_lpf_wS) < pm.probe_speed_detached) {

			printf("Unable at too LOW speed" EOL);
			return;
		}

		pm.fsm_req = PM_STATE_ADJUST_SENSOR_HALL;

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_HALL_ST1]);
		reg_format(&regfile[ID_PM_HALL_ST2]);
		reg_format(&regfile[ID_PM_HALL_ST3]);
		reg_format(&regfile[ID_PM_HALL_ST4]);
		reg_format(&regfile[ID_PM_HALL_ST5]);
		reg_format(&regfile[ID_PM_HALL_ST6]);

		if (ACTIVE != 0) {

			pm.fsm_req = PM_STATE_LU_SHUTDOWN;

			if (pm_wait_for_idle() != PM_OK)
				break;
		}
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	if (		pm.lu_MODE != PM_LU_DISABLED
			&& ACTIVE != 0) {

		pm.fsm_req = PM_STATE_HALT;
	}

	ap.probe_LOCK = PM_DISABLED;
}

SH_DEF(pm_adjust_sensor_abi)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

		printf("Unable when SPEED loop is DISABLED" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (pm_wait_for_idle() != PM_OK)
			break;

		/* TODO */

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_idle() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	if (pm.lu_MODE != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}
}

SH_DEF(pm_adjust_sensor_sincos)
{
	/* TODO */
}

SH_DEF(pm_fsm_detached)
{
	pm.fsm_req = PM_STATE_LU_DETACHED;
	pm_wait_for_idle();

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_fsm_startup)
{
	pm.fsm_req = PM_STATE_LU_STARTUP;
	pm_wait_for_idle();

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_fsm_shutdown)
{
	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	pm_wait_for_idle();

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_default_config)
{
	pm_auto(&pm, PM_AUTO_CONFIG_DEFAULT);
}

SH_DEF(pm_default_probe)
{
	pm_auto(&pm, PM_AUTO_PROBE_DEFAULT);
	pm_auto(&pm, PM_AUTO_MAXIMAL_CURRENT);
}

