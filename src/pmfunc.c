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

int pm_wait_for_SPINUP()
{
	TickType_t		xTIME = (TickType_t) 0;
	int			lack_N = 0;

	do {
		vTaskDelay((TickType_t) 50);
		xTIME += (TickType_t) 50;

		if (pm.fail_reason != PM_OK)
			break;

		if (pm.s_setpoint_speed - pm.lu_wS < M_EPS_F)
			break;

		if (pm.lu_mode == PM_LU_FORCED) {

			lack_N = (pm.vsi_DC > pm.probe_speed_maximal_pc / 100.f)
				? lack_N + 1 : 0;

			if (lack_N >= 3) {

				/* We are unable to keep such a high speed at
				 * this DC link volate. Stop.
				 * */
				pm.s_setpoint_speed = pm.lu_wS;
				break;
			}
		}

		if (xTIME > (TickType_t) 5000) {

			pm.fail_reason = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	vTaskDelay((TickType_t) 100);

	return pm.fail_reason;
}

int pm_wait_for_MOTION(float s_ref)
{
	TickType_t		xTIME = (TickType_t) 0;
	int			revol = pm.im_revol_total;

	do {
		vTaskDelay((TickType_t) 50);
		xTIME += (TickType_t) 50;

		if (pm.fail_reason != PM_OK)
			break;

		if (pm.im_revol_total != revol) {

			if (m_fabsf(pm.flux_lpf_wS) > s_ref)
				break;
		}

		if (xTIME > (TickType_t) 10000) {

			pm.fail_reason = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	return pm.fail_reason;
}

static void
pm_reg_SET_SETPOINT_SPEED()
{
	reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

	if (reg_GET_F(ID_PM_S_SETPOINT_SPEED_PC) > pm.probe_speed_maximal_pc) {

		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, pm.probe_speed_maximal_pc);
	}
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

		pm.const_R = pm.const_im_R;

		reg_format(&regfile[ID_PM_CONST_R]);

		pm.fsm_req = PM_STATE_PROBE_CONST_L;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_L]);
		reg_format(&regfile[ID_PM_CONST_IM_L1]);
		reg_format(&regfile[ID_PM_CONST_IM_L2]);
		reg_format(&regfile[ID_PM_CONST_IM_B]);
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

	if (		pm.config_FORCED == PM_DISABLED
			|| pm.config_DRIVE != PM_DRIVE_SPEED) {

		printf("Enable SPEED control mode before" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		if (pm.const_E < M_EPS_F) {

			pm_reg_SET_SETPOINT_SPEED();

			if (pm_wait_for_SPINUP() != PM_OK)
				break;

			pm.fsm_req = PM_STATE_PROBE_CONST_E;

			if (pm_wait_for_IDLE() != PM_OK)
				break;

			reg_format(&regfile[ID_PM_CONST_E_KV]);
		}

		pm_reg_SET_SETPOINT_SPEED();

		if (pm_wait_for_SPINUP() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);
		reg_format(&regfile[ID_PM_FLUX_LPF_WS_RPM]);

		pm.fsm_req = PM_STATE_PROBE_FLUX_MPPE;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_FLUX_MPPE_RPM]);
		reg_format(&regfile[ID_PM_FLUX_GAIN_TAKE_U]);
		reg_format(&regfile[ID_PM_FLUX_GAIN_GIVE_U]);

		if (pm_wait_for_SPINUP() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 100);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, 110.f);

		vTaskDelay((TickType_t) 300);

		pm_reg_SET_SETPOINT_SPEED();

		vTaskDelay((TickType_t) 300);

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
		reg_format(&regfile[ID_PM_FLUX_LPF_WS_RPM]);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);

	if (pm.lu_mode != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
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
		pm.fsm_req = PM_STATE_PROBE_FLUX_MPPE;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_FLUX_MPPE_RPM]);
		reg_format(&regfile[ID_PM_FLUX_GAIN_TAKE_U]);
		reg_format(&regfile[ID_PM_FLUX_GAIN_GIVE_U]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_probe_const_J)
{
	do {
		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		vTaskDelay((TickType_t) 100);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, 110.f);

		vTaskDelay((TickType_t) 300);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, 0.f);

		vTaskDelay((TickType_t) 300);

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_JA_KGM2]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_adjust_hall)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (pm.config_DRIVE != PM_DRIVE_SPEED) {

		printf("Enable SPEED control mode before" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		pm_reg_SET_SETPOINT_SPEED();

		if (pm_wait_for_SPINUP() != PM_OK)
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

