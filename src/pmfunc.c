#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "tlm.h"

int pm_wait_IDLE()
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

int pm_wait_motion()
{
	TickType_t		xTIME = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 50);

		if (pm.fsm_errno != PM_OK)
			break;

		if (		m_fabsf(pm.zone_lpf_wS) > pm.zone_threshold
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

int pm_wait_spinup()
{
	TickType_t		xTIME = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 50);

		if (pm.fsm_errno != PM_OK)
			break;

		if (m_fabsf(pm.s_setpoint_speed - pm.lu_wS) < pm.probe_speed_tol)
			break;

		if (		pm.lu_MODE == PM_LU_FORCED
				&& pm.vsi_lpf_DC > pm.forced_stop_DC)
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

int pm_wait_settle()
{
	TickType_t		xTick = (TickType_t) 0;

	do {
		float		eRSU;

		vTaskDelay((TickType_t) 50);

		if (pm.fsm_errno != PM_OK)
			break;

		eRSU = pm.x_setpoint_location - pm.lu_location;

		if (m_fabsf(eRSU) < pm.probe_location_tol)
			break;

		if (xTick > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		xTick += (TickType_t) 50;
	}
	while (1);

	return pm.fsm_errno;
}

SH_DEF(pm_adjust_dtc_voltage)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_IDLE();

		reg_OUTP(ID_PM_CONST_FB_U);
		reg_OUTP(ID_PM_SCALE_IA0);
		reg_OUTP(ID_PM_SCALE_IB0);
		reg_OUTP(ID_PM_SCALE_IC0);
		reg_OUTP(ID_PM_SELF_STDI);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;

			if (pm_wait_IDLE() != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_ADJUST_DTC_VOLTAGE;
		pm_wait_IDLE();

		reg_OUTP(ID_PM_CONST_IM_RZ);
		reg_OUTP(ID_PM_DTC_DEADBAND);
		reg_OUTP(ID_PM_SELF_DTU);
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);
}

SH_DEF(pm_probe_impedance)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_IDLE();

		tlm_startup(&tlm, tlm.rate_watch, TLM_MODE_WATCH);

		reg_OUTP(ID_PM_CONST_FB_U);
		reg_OUTP(ID_PM_SCALE_IA0);
		reg_OUTP(ID_PM_SCALE_IB0);
		reg_OUTP(ID_PM_SCALE_IC0);
		reg_OUTP(ID_PM_SELF_STDI);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;

			if (pm_wait_IDLE() != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_RESISTANCE;

		if (pm_wait_IDLE() != PM_OK)
			break;

		pm.const_Rs = pm.const_im_Rz;

		reg_OUTP(ID_PM_CONST_RS);
		reg_OUTP(ID_PM_SELF_DTU);

		pm.fsm_req = PM_STATE_PROBE_CONST_INDUCTANCE;

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_OUTP(ID_PM_CONST_IM_LD);
		reg_OUTP(ID_PM_CONST_IM_LQ);
		reg_OUTP(ID_PM_CONST_IM_A);
		reg_OUTP(ID_PM_CONST_IM_RZ);

		pm_auto(&pm, PM_AUTO_MAXIMAL_CURRENT);
		pm_auto(&pm, PM_AUTO_LOOP_CURRENT);

		reg_OUTP(ID_PM_I_MAXIMAL);
		reg_OUTP(ID_PM_I_GAIN_P);
		reg_OUTP(ID_PM_I_GAIN_I);
		reg_OUTP(ID_PM_I_SLEW_RATE);
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);

	tlm_halt(&tlm);
}

SH_DEF(pm_probe_spinup)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

		printf("Unable when DRIVE is not SPEED" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (pm_wait_IDLE() != PM_OK)
			break;

		tlm_startup(&tlm, tlm.rate_watch, TLM_MODE_WATCH);

		if (		pm.flux_LINKAGE != PM_ENABLED
				&& pm.config_EXCITATION == PM_EXCITATION_CONST) {

			reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

			if (pm_wait_spinup() != PM_OK)
				break;

			vTaskDelay((TickType_t) 200);

			pm.fsm_req = PM_STATE_PROBE_CONST_FLUX_LINKAGE;

			if (pm_wait_IDLE() != PM_OK)
				break;

			reg_OUTP(ID_PM_CONST_LAMBDA_KV);
		}

		pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);
		pm_auto(&pm, PM_AUTO_PROBE_SPEED_HOLD);
		pm_auto(&pm, PM_AUTO_FORCED_MAXIMAL);

		reg_OUTP(ID_PM_PROBE_SPEED_HOLD_RPM);
		reg_OUTP(ID_PM_FORCED_MAXIMAL_RPM);
		reg_OUTP(ID_PM_ZONE_NOISE_U);
		reg_OUTP(ID_PM_ZONE_THRESHOLD_U);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

		if (pm_wait_spinup() != PM_OK)
			break;

		if (pm.flux_ZONE != PM_ZONE_HIGH) {

			pm.fsm_errno = PM_ERROR_NO_FLUX_CAUGHT;
			break;
		}

		if (pm.config_EXCITATION == PM_EXCITATION_CONST) {

			vTaskDelay((TickType_t) 200);

			pm.fsm_req = PM_STATE_PROBE_CONST_FLUX_LINKAGE;

			if (pm_wait_IDLE() != PM_OK)
				break;

			reg_OUTP(ID_PM_CONST_LAMBDA_KV);
		}

		vTaskDelay((TickType_t) 200);

		pm.fsm_req = PM_STATE_PROBE_NOISE_THRESHOLD;

		if (pm_wait_IDLE() != PM_OK)
			break;

		pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);
		pm_auto(&pm, PM_AUTO_PROBE_SPEED_HOLD);
		pm_auto(&pm, PM_AUTO_FORCED_MAXIMAL);

		reg_OUTP(ID_PM_PROBE_SPEED_HOLD_RPM);
		reg_OUTP(ID_PM_FORCED_MAXIMAL_RPM);
		reg_OUTP(ID_PM_ZONE_NOISE_U);
		reg_OUTP(ID_PM_ZONE_THRESHOLD_U);

		pm.fsm_req = PM_STATE_PROBE_CONST_INERTIA;

		vTaskDelay((TickType_t) 100);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, 110.f);

		vTaskDelay((TickType_t) 400);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

		vTaskDelay((TickType_t) 400);

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_OUTP(ID_PM_CONST_JA_KGM2);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_IDLE() != PM_OK)
			break;

		pm_auto(&pm, PM_AUTO_FORCED_ACCEL);
		pm_auto(&pm, PM_AUTO_LOOP_SPEED);

		reg_OUTP(ID_PM_FORCED_ACCEL_RPM);
		reg_OUTP(ID_PM_LU_GAIN_MQ_LP);
		reg_OUTP(ID_PM_S_GAIN_P);
		reg_OUTP(ID_PM_S_GAIN_D);
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);

	if (pm.lu_MODE != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}

	tlm_halt(&tlm);
}

SH_DEF(pm_probe_detached)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (pm.config_EXCITATION != PM_EXCITATION_CONST) {

		printf("Unable when EXCITATION is not CONST" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_LU_DETACHED;

		if (pm_wait_motion() != PM_OK)
			break;

		tlm_startup(&tlm, tlm.rate_watch, TLM_MODE_WATCH);

		pm.fsm_req = PM_STATE_PROBE_CONST_FLUX_LINKAGE;

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_OUTP(ID_PM_CONST_LAMBDA_KV);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);

	if (pm.lu_MODE != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}

	tlm_halt(&tlm);
}

SH_DEF(pm_probe_const_flux_linkage)
{
	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	if (pm.config_EXCITATION != PM_EXCITATION_CONST) {

		printf("Unable when EXCITATION is not CONST" EOL);
		return;
	}

	if (pm.flux_ZONE != PM_ZONE_HIGH) {

		printf("Unable when FLUX is NOT in HIGH zone" EOL);
		return;
	}

	do {
		tlm_startup(&tlm, tlm.rate_watch, TLM_MODE_WATCH);

		pm.fsm_req = PM_STATE_PROBE_CONST_FLUX_LINKAGE;

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_OUTP(ID_PM_CONST_LAMBDA_KV);
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);

	tlm_halt(&tlm);
}

SH_DEF(pm_probe_const_inertia)
{
	float		wSP;

	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

		printf("Unable when DRIVE is not SPEED" EOL);
		return;
	}

	do {
		tlm_startup(&tlm, tlm.rate_watch, TLM_MODE_WATCH);

		pm.fsm_req = PM_STATE_PROBE_CONST_INERTIA;

		vTaskDelay((TickType_t) 100);

		wSP = reg_GET_F(ID_PM_S_SETPOINT_SPEED);
		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, 110.f);

		vTaskDelay((TickType_t) 400);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, wSP);

		vTaskDelay((TickType_t) 400);

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_OUTP(ID_PM_CONST_JA_KGM2);
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);

	tlm_halt(&tlm);
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
		tlm_startup(&tlm, tlm.rate_watch, TLM_MODE_WATCH);

		pm.fsm_req = PM_STATE_PROBE_NOISE_THRESHOLD;

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_OUTP(ID_PM_ZONE_NOISE);
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);

	tlm_halt(&tlm);
}

SH_DEF(pm_adjust_sensor_hall)
{
	int		backup_LU_SENSOR;

	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

		printf("Unable when DRIVE is not SPEED" EOL);
		return;
	}

	backup_LU_SENSOR = pm.config_LU_SENSOR;
	pm.config_LU_SENSOR = PM_SENSOR_NONE;

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

		if (pm_wait_spinup() != PM_OK)
			break;

		tlm_startup(&tlm, tlm.rate_watch, TLM_MODE_WATCH);

		pm.fsm_req = PM_STATE_ADJUST_SENSOR_HALL;

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_OUTP(ID_PM_HALL_ST1);
		reg_OUTP(ID_PM_HALL_ST2);
		reg_OUTP(ID_PM_HALL_ST3);
		reg_OUTP(ID_PM_HALL_ST4);
		reg_OUTP(ID_PM_HALL_ST5);
		reg_OUTP(ID_PM_HALL_ST6);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);

	if (pm.lu_MODE != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}

	pm.config_LU_SENSOR = backup_LU_SENSOR;

	tlm_halt(&tlm);
}

SH_DEF(pm_adjust_sensor_eabi)
{
	int		backup_LU_SENSOR;

	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_SPEED) {

		printf("Unable when DRIVE is not SPEED" EOL);
		return;
	}

	backup_LU_SENSOR = pm.config_LU_SENSOR;
	pm.config_LU_SENSOR = PM_SENSOR_NONE;

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.zone_threshold);

		if (pm_wait_spinup() != PM_OK)
			break;

		tlm_startup(&tlm, tlm.rate_watch, TLM_MODE_WATCH);

		pm.fsm_req = PM_STATE_ADJUST_SENSOR_EABI;

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_OUTP(ID_PM_EABI_CONST_EP);
		reg_OUTP(ID_PM_EABI_CONST_ZS);
		reg_OUTP(ID_PM_EABI_F0);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);

	if (pm.lu_MODE != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}

	pm.config_LU_SENSOR = backup_LU_SENSOR;

	tlm_halt(&tlm);
}

SH_DEF(pm_adjust_sensor_sincos)
{
	/* TODO */
}

SH_DEF(ld_probe_const_inertia)
{
	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_LOCATION) {

		printf("Unable when DRIVE is not LOCATION" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_LOCATION, pm.x_minimal);
		reg_SET_F(ID_PM_X_SETPOINT_SPEED, 0.f);

		if (pm_wait_settle() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_INERTIA;

		vTaskDelay((TickType_t) 100);

		reg_SET_F(ID_PM_X_SETPOINT_LOCATION, pm.x_maximal);

		vTaskDelay((TickType_t) 400);

		reg_SET_F(ID_PM_X_SETPOINT_LOCATION, pm.x_minimal);

		vTaskDelay((TickType_t) 400);

		if (pm_wait_IDLE() != PM_OK)
			break;

		reg_OUTP(ID_PM_CONST_JA_KG);
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);
}

SH_DEF(ld_adjust_limit)
{
	float			wSP = 1.f;

	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	if (pm.config_LU_DRIVE != PM_DRIVE_LOCATION) {

		printf("Unable when DRIVE is not LOCATION" EOL);
		return;
	}

	do {
		reg_SET_F(ID_PM_X_SETPOINT_LOCATION, pm.x_minimal);
		reg_SET_F(ID_PM_X_SETPOINT_SPEED, 0.f);

		if (pm_wait_settle() != PM_OK)
			break;

		reg_SET_F(ID_PM_X_SETPOINT_SPEED, wSP);

		/* TODO */
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);
}

SH_DEF(pm_fsm_detached)
{
	pm.fsm_req = PM_STATE_LU_DETACHED;
	pm_wait_IDLE();

	reg_OUTP(ID_PM_FSM_ERRNO);
}

SH_DEF(pm_fsm_startup)
{
	pm.fsm_req = PM_STATE_LU_STARTUP;
	pm_wait_IDLE();

	reg_OUTP(ID_PM_FSM_ERRNO);
}

SH_DEF(pm_fsm_shutdown)
{
	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	pm_wait_IDLE();

	reg_OUTP(ID_PM_FSM_ERRNO);
}

SH_DEF(pm_default_config)
{
	pm_auto(&pm, PM_AUTO_CONFIG_DEFAULT);
}

SH_DEF(pm_default_machine)
{
	pm_auto(&pm, PM_AUTO_MACHINE_DEFAULT);
	pm_auto(&pm, PM_AUTO_MAXIMAL_CURRENT);
}

SH_DEF(pm_default_scale)
{
	pm_auto(&pm, PM_AUTO_SCALE_DEFAULT);
}

