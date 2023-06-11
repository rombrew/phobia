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

int pm_wait_for_motion()
{
	TickType_t		xTIME = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 50);

		if (pm.fsm_errno != PM_OK)
			break;

		if (		m_fabsf(pm.zone_lpf_wS) > pm.zone_speed_threshold
				&& pm.detach_TIM > PM_TSMS(&pm, pm.tm_transient_slow))
			break;

		if (xTIME > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		xTIME += (TickType_t) 50;
	}
	while (1);

	return pm.fsm_errno;
}

int pm_wait_for_spinup()
{
	TickType_t		xTIME = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 50);

		if (pm.fsm_errno != PM_OK)
			break;

		if (m_fabsf(pm.s_setpoint_speed - pm.lu_wS) < pm.probe_speed_tol)
			break;

		if (		pm.lu_MODE == PM_LU_FORCED
				&& pm.vsi_lpf_DC > pm.forced_maximal_DC)
			break;

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

int pm_wait_for_settle()
{
	TickType_t		xTick = (TickType_t) 0;

	do {
		float		xER;

		vTaskDelay((TickType_t) 50);

		if (pm.fsm_errno != PM_OK)
			break;

		xER = pm.x_setpoint_location - pm.lu_location;

		if (m_fabsf(xER) < pm.probe_location_tol)
			break;

		if (xTick > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		xTick += (TickType_t) 50;
	}
	while (1);

	vTaskDelay((TickType_t) 100);

	return pm.fsm_errno;
}

SH_DEF(pm_probe_impedance)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_for_idle();

		reg_format(&regfile[ID_PM_CONST_FB_U]);
		reg_format(&regfile[ID_PM_SCALE_IA0]);
		reg_format(&regfile[ID_PM_SCALE_IB0]);
		reg_format(&regfile[ID_PM_SCALE_IC0]);
		reg_format(&regfile[ID_PM_SELF_STDI]);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;

			if (pm_wait_for_idle() != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_RESISTANCE;

		if (pm_wait_for_idle() != PM_OK)
			break;

		pm.const_Rs = pm.const_im_R;

		reg_format(&regfile[ID_PM_CONST_RS]);

		pm.fsm_req = PM_STATE_PROBE_CONST_INDUCTANCE;

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

		if (pm_wait_for_idle() != PM_OK)
			break;

		if (pm.const_lambda < M_EPS_F) {

			reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

			if (pm_wait_for_spinup() != PM_OK)
				break;

			reg_format(&regfile[ID_PM_ZONE_LPF_WS]);

			pm.fsm_req = PM_STATE_PROBE_CONST_FLUX_LINKAGE;

			if (pm_wait_for_idle() != PM_OK)
				break;

			reg_format(&regfile[ID_PM_CONST_LAMBDA_KV]);

			pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);
			pm_auto(&pm, PM_AUTO_PROBE_SPEED_HOLD);
			pm_auto(&pm, PM_AUTO_FORCED_MAXIMAL);

			reg_format(&regfile[ID_PM_ZONE_SPEED_NOISE]);
			reg_format(&regfile[ID_PM_ZONE_SPEED_THRESHOLD]);
			reg_format(&regfile[ID_PM_PROBE_SPEED_HOLD]);
			reg_format(&regfile[ID_PM_FORCED_MAXIMAL]);
		}

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

		if (pm_wait_for_spinup() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_ZONE_LPF_WS]);

		if (pm.flux_ZONE != PM_ZONE_HIGH) {

			pm.fsm_errno = PM_ERROR_NO_FLUX_CAUGHT;
			break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_FLUX_LINKAGE;

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_LAMBDA_KV]);

		pm.fsm_req = PM_STATE_PROBE_NOISE_THRESHOLD;

		if (pm_wait_for_idle() != PM_OK)
			break;

		pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);

		reg_format(&regfile[ID_PM_ZONE_SPEED_NOISE]);
		reg_format(&regfile[ID_PM_ZONE_SPEED_THRESHOLD]);

		pm.fsm_req = PM_STATE_PROBE_CONST_INERTIA;

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

		if (pm_wait_for_motion() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_FLUX_LINKAGE;

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_ZONE_LPF_WS]);
		reg_format(&regfile[ID_PM_CONST_LAMBDA_KV]);

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

SH_DEF(pm_probe_const_resistance)
{
	float		R[3];

	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_for_idle();

		reg_format(&regfile[ID_PM_CONST_FB_U]);
		reg_format(&regfile[ID_PM_SCALE_IA0]);
		reg_format(&regfile[ID_PM_SCALE_IB0]);
		reg_format(&regfile[ID_PM_SCALE_IC0]);
		reg_format(&regfile[ID_PM_SELF_STDI]);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;

			if (pm_wait_for_idle() != PM_OK)
				break;
		}

		pm.probe_hold_angle = 0.f;

		pm.fsm_req = PM_STATE_PROBE_CONST_RESISTANCE;

		if (pm_wait_for_idle() != PM_OK)
			break;

		R[0] = pm.const_im_R;

		reg_format(&regfile[ID_PM_CONST_IM_R]);

		pm.probe_hold_angle = 120.f;

		pm.fsm_req = PM_STATE_PROBE_CONST_RESISTANCE;

		if (pm_wait_for_idle() != PM_OK)
			break;

		R[1] = pm.const_im_R;

		reg_format(&regfile[ID_PM_CONST_IM_R]);

		pm.probe_hold_angle = - 120.f;

		pm.fsm_req = PM_STATE_PROBE_CONST_RESISTANCE;

		if (pm_wait_for_idle() != PM_OK)
			break;

		R[2] = pm.const_im_R;

		reg_format(&regfile[ID_PM_CONST_IM_R]);

		pm.const_Rs = (R[0] + R[1] + R[2]) / 3.f;

		reg_format(&regfile[ID_PM_CONST_RS]);
	}
	while (0);

	pm.probe_hold_angle = 0.f;

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_probe_const_flux_linkage)
{
	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	if (pm.flux_ZONE != PM_ZONE_HIGH) {

		printf("Unable when FLUX is NOT in HIGH zone" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_PROBE_CONST_FLUX_LINKAGE;

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_LAMBDA_KV]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_probe_const_inertia)
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

	do {
		pm.fsm_req = PM_STATE_PROBE_CONST_INERTIA;

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

	do {
		pm.fsm_req = PM_STATE_PROBE_NOISE_THRESHOLD;

		if (pm_wait_for_idle() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_ZONE_SPEED_NOISE]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_adjust_sensor_hall)
{
	int		ACTIVE = 0;

	if (pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

		printf("Unable when SPEED loop is DISABLED" EOL);
		return;
	}

	do {
		if (pm.lu_MODE == PM_LU_DISABLED) {

			pm.fsm_req = PM_STATE_LU_STARTUP;

			if (pm_wait_for_idle() != PM_OK)
				break;

			reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

			if (pm_wait_for_spinup() != PM_OK)
				break;

			ACTIVE = 1;
		}

		if (m_fabsf(pm.zone_lpf_wS) < pm.zone_speed_threshold) {

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

SH_DEF(pm_location_probe_const_inertia)
{
	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_LOCATION) {

		printf("Unable when LOCATION loop is DISABLED" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_LOCATION, pm.x_location_range[0]);
		reg_SET_F(ID_PM_X_SETPOINT_SPEED, 0.f);

		if (pm_wait_for_settle() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_INERTIA;

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

SH_DEF(pm_location_adjust_range)
{
	float			wSP = 1.f;

	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_LOCATION) {

		printf("Unable when LOCATION loop is DISABLED" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_LOCATION, pm.x_location_range[0]);
		reg_SET_F(ID_PM_X_SETPOINT_SPEED, 0.f);

		if (pm_wait_for_settle() != PM_OK)
			break;

		reg_SET_F(ID_PM_X_SETPOINT_SPEED, wSP);

		/* TODO */
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
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

