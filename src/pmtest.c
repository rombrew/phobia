#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "tlm.h"

SH_DEF(pm_self_test)
{
	int			N, xDC;

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

			pm.fsm_req = PM_STATE_SELF_TEST_BOOTSTRAP;
			pm_wait_for_IDLE();

			reg_format(&regfile[ID_PM_SELF_BST]);
			reg_format(&regfile[ID_PM_FSM_ERRNO]);

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;
			pm_wait_for_IDLE();

			reg_format(&regfile[ID_PM_SELF_BM]);
			reg_format(&regfile[ID_PM_FSM_ERRNO]);
		}

		xDC = pm.dc_resolution - pm.ts_clearance;

		for (N = 0; N < 2; ++N) {

			switch (N) {

				case 0:
					pm.proc_set_DC(0, 0, 0);
					pm.proc_set_Z(pm.k_ZNUL);
					break;

				case 1:
					pm.proc_set_DC(xDC, xDC, xDC);
					pm.proc_set_Z(pm.k_ZNUL);
					break;
			}

			pm.fsm_req = PM_STATE_SELF_TEST_CLEARANCE;
			pm_wait_for_IDLE();

			reg_format(&regfile[ID_PM_SELF_RMSI]);

			if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

				reg_format(&regfile[ID_PM_SELF_RMSU]);
			}
		}
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_self_adjust)
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

			pm.fsm_req = PM_STATE_ADJUST_VOLTAGE;
			pm_wait_for_IDLE();

			reg_format(&regfile[ID_PM_AD_UA_0]);
			reg_format(&regfile[ID_PM_AD_UA_1]);
			reg_format(&regfile[ID_PM_AD_UB_0]);
			reg_format(&regfile[ID_PM_AD_UB_1]);
			reg_format(&regfile[ID_PM_AD_UC_0]);
			reg_format(&regfile[ID_PM_AD_UC_1]);

			reg_format(&regfile[ID_PM_TVM_FIR_A_TAU]);
			reg_format(&regfile[ID_PM_TVM_FIR_B_TAU]);
			reg_format(&regfile[ID_PM_TVM_FIR_C_TAU]);

			reg_format(&regfile[ID_PM_SELF_RMSU]);

			if (pm.fsm_errno != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_ADJUST_CURRENT;
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_AD_IA_1]);
		reg_format(&regfile[ID_PM_AD_IB_1]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);
}

SH_DEF(pm_test_current_ramp)
{
	TickType_t		xTS1;
	float			iSP;

	if (pm.lu_mode == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	xTS1 = (TickType_t) (100UL * TLM_DATA_MAX / tlm.freq_grab_hz);

	TLM_startup(&tlm, tlm.freq_grab_hz, TLM_MODE_SINGLE_GRAB);

	do {
		iSP = reg_GET_F(ID_PM_I_SETPOINT_TORQUE);
		vTaskDelay(1UL * xTS1);

		reg_SET_F(ID_PM_I_SETPOINT_TORQUE, pm.i_maximal);
		vTaskDelay(5UL * xTS1);

		reg_SET_F(ID_PM_I_SETPOINT_TORQUE, iSP);
		vTaskDelay(4UL * xTS1);
	}
	while (0);
}

SH_DEF(pm_test_speed_ramp)
{
	TickType_t		xTS1;
	float			wSP;

	if (pm.lu_mode == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	xTS1 = (TickType_t) (100UL * TLM_DATA_MAX / tlm.freq_grab_hz);

	TLM_startup(&tlm, tlm.freq_grab_hz, TLM_MODE_SINGLE_GRAB);

	do {
		wSP = reg_GET_F(ID_PM_S_SETPOINT_SPEED);
		vTaskDelay(1UL * xTS1);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, 110.f);
		vTaskDelay(5UL * xTS1);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, wSP);
		vTaskDelay(4UL * xTS1);
	}
	while (0);
}

SH_DEF(pm_test_thrust_curve)
{
	/* TODO */
}

SH_DEF(pm_test_TVM_ramp)
{
	TickType_t		xWake, xTim0;
	int			xDC, xMIN, xMAX;

	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (		PM_CONFIG_TVM(&pm) == PM_DISABLED
			|| pm.tvm_ALLOWED == PM_DISABLED) {

		printf("Enable TVM before" EOL);
		return;
	}

	xMIN = pm.ts_minimal;
	xMAX = (int) (pm.dc_resolution * pm.tvm_range_DC);

	xDC = xMIN;

	PWM_set_DC(xDC, xDC, xDC);
	PWM_set_Z(0);

	pm.vsi_UF = 0;
	pm.vsi_AZ = 1;
	pm.vsi_BZ = 1;
	pm.vsi_CZ = 1;

	xWake = xTaskGetTickCount();
	xTim0 = xWake;

	TLM_startup(&tlm, tlm.freq_grab_hz, TLM_MODE_SINGLE_GRAB);

	do {
		/* 1000 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 1);

		xDC = (xDC < xMAX) ? xDC + 1 : xMIN;

		PWM_set_DC(xDC, xDC, xDC);

		/* Reference voltage.
		 * */
		pm.vsi_X = xDC * pm.const_fb_U * pm.ts_inverted;

		if (tlm.mode == TLM_MODE_DISABLED)
			break;

		if ((xWake - xTim0) > (TickType_t) 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		if (pm.fsm_errno != PM_OK)
			break;
	}
	while (1);

	pm.fsm_req = PM_STATE_HALT;
}

SH_DEF(hal_DBGMCU_mode_stop)
{
	DBGMCU_mode_stop();
}

SH_DEF(hal_PWM_set_DC)
{
	int			xDC;

	if (stoi(&xDC, s) != NULL) {

		xDC = (xDC < 0) ? 0 : (xDC > hal.PWM_resolution)
			? hal.PWM_resolution : xDC;

		PWM_set_DC(xDC, xDC, xDC);
	}
}

SH_DEF(hal_PWM_set_Z)
{
	int			xZ;

	if (stoi(&xZ, s) != NULL) {

		PWM_set_Z(xZ);
	}
}

SH_DEF(hal_RNG_rseed)
{
	rseed = RNG_urand();

	printf("%8x " EOL, rseed);
}

SH_DEF(hal_GPIO_set_high_BOOST_12V)
{
	GPIO_set_HIGH(GPIO_BOOST_12V);
}

SH_DEF(hal_GPIO_set_low_BOOST_12V)
{
	GPIO_set_LOW(GPIO_BOOST_12V);
}

SH_DEF(hal_GPIO_set_high_FAN)
{
	GPIO_set_HIGH(GPIO_FAN);
}

SH_DEF(hal_GPIO_set_low_FAN)
{
	GPIO_set_LOW(GPIO_FAN);
}

