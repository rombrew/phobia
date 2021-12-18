#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "min.h"
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

		if (xTick > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	return pm.fsm_errno;
}

int pm_wait_for_SPINUP()
{
	const float		wS_tol = 10.f;
	TickType_t		xTIME = (TickType_t) 0;

	do {
		vTaskDelay((TickType_t) 50);
		xTIME += (TickType_t) 50;

		if (pm.fsm_errno != PM_OK)
			break;

		/* Check the target speed has reached.
		 * */
		if (m_fabsf(pm.lu_wS) + wS_tol > pm.s_setpoint_speed)
			break;

		if (		pm.lu_mode == PM_LU_FORCED
				&& pm.vsi_lpf_DC > pm.probe_speed_maximal_pc / 100.f) {

			/* We are unable to keep such a high speed at
			 * this DC link voltage. Stop.
			 * */
			pm.s_setpoint_speed = pm.lu_wS;
			break;
		}

		if (xTIME > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	vTaskDelay((TickType_t) 100);

	return pm.fsm_errno;
}

int pm_wait_for_MOTION(float s_ref)
{
	TickType_t		xTIME = (TickType_t) 0;
	int			revol = pm.im_total_revol;

	do {
		vTaskDelay((TickType_t) 50);
		xTIME += (TickType_t) 50;

		if (pm.fsm_errno != PM_OK)
			break;

		if (pm.im_total_revol != revol) {

			if (m_fabsf(pm.flux_lpf_wS) > s_ref)
				break;
		}

		if (xTIME > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	return pm.fsm_errno;
}

static float
pm_proc_ripple_STD(void *link, const float *KF)
{
	pm.flux_imbalance_KF[0] = KF[0];
	pm.flux_imbalance_KF[1] = KF[1];
	pm.flux_imbalance_KF[2] = KF[2];
	pm.flux_imbalance_KF[3] = KF[3];
	pm.flux_imbalance_KF[4] = KF[4];
	pm.flux_imbalance_KF[5] = KF[5];
	pm.flux_imbalance_KF[6] = KF[6];

	hal_fence();

	pm.flux_imbalance_ONFLAG = PM_ENABLED;

	do {
		vTaskDelay((TickType_t) 5);
	}
	while (pm.flux_imbalance_ONFLAG == PM_ENABLED);

	return pm.flux_imbalance_ripple_STD;
}

static void
pm_proc_step_format(void *link)
{
	min_t		*m = (min_t *) link;
	float		x_tol_pc = m->x_tol * 100.f;

	if (pm.fsm_errno != PM_OK) {

		/* Terminate the SEARCH if an error occurs.
		 * */
		m->n_final_step = 0;
	}

	printf("[%2i] %4f %4f" EOL, m->n_step, &x_tol_pc, &m->fval[m->n_best]);
}

static void
pm_reg_SET_SETPOINT_SPEED()
{
	reg_SET_F(ID_PM_S_SETPOINT_SPEED, pm.probe_speed_hold);

	/* We check if speed percentage is higher than maximal allowed.
	 * */
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

		pm_tune_loop_current(&pm);

		reg_format(&regfile[ID_PM_I_GAIN_P]);
		reg_format(&regfile[ID_PM_I_GAIN_I]);
		reg_format(&regfile[ID_PM_I_SLEW_RATE]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_probe_spinup)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (		pm.config_LU_FORCED != PM_ENABLED
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

			reg_format(&regfile[ID_PM_FLUX_LPF_WS]);

			pm.fsm_req = PM_STATE_PROBE_CONST_E;

			if (pm_wait_for_IDLE() != PM_OK)
				break;

			reg_format(&regfile[ID_PM_CONST_E_KV]);
		}

		pm_reg_SET_SETPOINT_SPEED();

		if (pm_wait_for_SPINUP() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_FLUX_LPF_WS]);

		if (pm.flux_mode != PM_FLUX_HIGH) {

			pm.fsm_errno = PM_ERROR_NO_FLUX_CAUGHT;
			break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_CONST_E_KV]);

		pm_tune_MPPE(&pm);

		reg_format(&regfile[ID_PM_FLUX_MPPE]);
		reg_format(&regfile[ID_PM_FLUX_MURE]);
		reg_format(&regfile[ID_PM_FLUX_GAIN_TAKE]);
		reg_format(&regfile[ID_PM_FLUX_GAIN_GIVE]);

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

		pm_tune_forced(&pm);
		pm_tune_loop_speed(&pm);

		reg_format(&regfile[ID_PM_FORCED_MAXIMAL]);
		reg_format(&regfile[ID_PM_FORCED_ACCEL]);

		reg_format(&regfile[ID_PM_LU_GAIN_TF]);
		reg_format(&regfile[ID_PM_S_GAIN_P]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

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

		reg_format(&regfile[ID_PM_FLUX_LPF_WS]);
		reg_format(&regfile[ID_PM_CONST_E_KV]);

		pm_tune_MPPE(&pm);

		reg_format(&regfile[ID_PM_FLUX_MPPE]);
		reg_format(&regfile[ID_PM_FLUX_MURE]);
		reg_format(&regfile[ID_PM_FLUX_GAIN_TAKE]);
		reg_format(&regfile[ID_PM_FLUX_GAIN_GIVE]);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

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

		pm_tune_MPPE(&pm);

		reg_format(&regfile[ID_PM_FLUX_MPPE]);
		reg_format(&regfile[ID_PM_FLUX_MURE]);
		reg_format(&regfile[ID_PM_FLUX_GAIN_TAKE]);
		reg_format(&regfile[ID_PM_FLUX_GAIN_GIVE]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
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

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_adjust_imbalance)
{
	min_t			*m;

	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (pm.config_FLUX_IMBALANCE != PM_ENABLED) {

		printf("Enable IMBALANCE function before" EOL);
		return;
	}

	if (pm.config_DRIVE != PM_DRIVE_SPEED) {

		printf("Enable SPEED control mode before" EOL);
		return;
	}

	m = pvPortMalloc(sizeof(min_t));

	if (m != NULL) {

		m->n_size_of_x = 7;
		m->link = (void *) m;
		m->x0_range = 0.02f;
		m->x_maximal = 0.5f;
		m->pfun = &pm_proc_ripple_STD;
		m->pstep = &pm_proc_step_format;
		m->n_final_step = 180;
		m->x_final_tol = 5E-4f;

		do {
			pm.fsm_req = PM_STATE_LU_STARTUP;

			if (pm_wait_for_IDLE() != PM_OK)
				break;

			pm_reg_SET_SETPOINT_SPEED();

			if (pm_wait_for_SPINUP() != PM_OK)
				break;

			minsearch(m);

			if (pm.fsm_errno != PM_OK) {

				pm.flux_imbalance_KF[0] = 0.f;
				pm.flux_imbalance_KF[1] = 0.f;
				pm.flux_imbalance_KF[2] = 0.f;
				pm.flux_imbalance_KF[3] = 0.f;
				pm.flux_imbalance_KF[4] = 0.f;
				pm.flux_imbalance_KF[5] = 0.f;
				pm.flux_imbalance_KF[6] = 0.f;
				break;
			}

			minsolution(m, pm.flux_imbalance_KF);

			reg_format(&regfile[ID_PM_FLUX_IMBALANCE_KF_0]);
			reg_format(&regfile[ID_PM_FLUX_IMBALANCE_KF_1]);
			reg_format(&regfile[ID_PM_FLUX_IMBALANCE_KF_2]);
			reg_format(&regfile[ID_PM_FLUX_IMBALANCE_KF_3]);
			reg_format(&regfile[ID_PM_FLUX_IMBALANCE_KF_4]);
			reg_format(&regfile[ID_PM_FLUX_IMBALANCE_KF_5]);
			reg_format(&regfile[ID_PM_FLUX_IMBALANCE_KF_6]);

			pm.fsm_req = PM_STATE_LU_SHUTDOWN;

			if (pm_wait_for_IDLE() != PM_OK)
				break;
		}
		while (0);

		vPortFree(m);

		reg_format(&regfile[ID_PM_FSM_ERRNO]);

		if (pm.lu_mode != PM_LU_DISABLED) {

			pm.fsm_req = PM_STATE_HALT;
		}
	}
}

SH_DEF(pm_adjust_sensor_hall)
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

		pm.fsm_req = PM_STATE_ADJUST_SENSOR_HALL;

		if (pm_wait_for_IDLE() != PM_OK)
			break;

		reg_format(&regfile[ID_PM_HALL_ST_1]);
		reg_format(&regfile[ID_PM_HALL_ST_2]);
		reg_format(&regfile[ID_PM_HALL_ST_3]);
		reg_format(&regfile[ID_PM_HALL_ST_4]);
		reg_format(&regfile[ID_PM_HALL_ST_5]);
		reg_format(&regfile[ID_PM_HALL_ST_6]);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (pm_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	if (pm.lu_mode != PM_LU_DISABLED) {

		pm.fsm_req = PM_STATE_HALT;
	}
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

