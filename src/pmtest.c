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

		if (pm.fail_reason != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_BOOTSTRAP;
			pm_wait_for_IDLE();

			reg_format(&regfile[ID_PM_SELF_BST]);
			reg_format(&regfile[ID_PM_FAIL_REASON]);

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;
			pm_wait_for_IDLE();

			reg_format(&regfile[ID_PM_SELF_BM]);
			reg_format(&regfile[ID_PM_FAIL_REASON]);
		}

		xDC = pm.dc_resolution - pm.ts_clearance;

		for (N = 0; N < 2; ++N) {

			switch (N) {

				case 0:
					pm.proc_set_DC(0, 0, 0);
					pm.proc_set_Z(0);
					break;

				case 1:
					pm.proc_set_DC(xDC, xDC, xDC);
					pm.proc_set_Z(0);
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

	reg_format(&regfile[ID_PM_FAIL_REASON]);
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

		if (pm.fail_reason != PM_OK)
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

			if (pm.fail_reason != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_ADJUST_CURRENT;
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_AD_IA_1]);
		reg_format(&regfile[ID_PM_AD_IB_1]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FAIL_REASON]);
}

SH_DEF(pm_FT_current_ramp)
{
	TickType_t		xTS1;
	float			iSP;

	if (pm.lu_mode == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	xTS1 = (TickType_t) (100UL * TLM_DATA_MAX / ap.FT_grab_hz);

	TLM_startup(&tlm, ap.FT_grab_hz, TLM_MODE_SINGLE_GRAB);

	do {
		iSP = reg_GET_F(ID_PM_I_SETPOINT_TORQUE);
		vTaskDelay(xTS1);

		reg_SET_F(ID_PM_I_SETPOINT_TORQUE, pm.i_maximal);
		vTaskDelay(5UL * xTS1);

		reg_SET_F(ID_PM_I_SETPOINT_TORQUE, iSP);
		vTaskDelay(4UL * xTS1);
	}
	while (0);
}

SH_DEF(pm_FT_speed_ramp)
{
	TickType_t		xTS1;
	float			wSP;

	if (pm.lu_mode == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	xTS1 = (TickType_t) (100UL * TLM_DATA_MAX / ap.FT_grab_hz);

	TLM_startup(&tlm, ap.FT_grab_hz, TLM_MODE_SINGLE_GRAB);

	do {
		wSP = reg_GET_F(ID_PM_S_SETPOINT_SPEED);
		vTaskDelay(xTS1);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED_PC, pm.probe_speed_spinup_pc);
		vTaskDelay(5UL * xTS1);

		reg_SET_F(ID_PM_S_SETPOINT_SPEED, wSP);
		vTaskDelay(4UL * xTS1);
	}
	while (0);
}

SH_DEF(pm_FT_thrust_curve)
{
	/* TODO */
}

SH_DEF(pm_FT_tvm_ramp)
{
	TickType_t		xWake, xTim0;
	int			xDC, xMIN, xMAX;

	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (		PM_CONFIG_TVM(&pm) == PM_DISABLED
			|| pm.tvm_ENABLED == PM_DISABLED) {

		printf("Enable TVM before" EOL);
		return;
	}

	xMIN = pm.ts_minimal;
	xMAX = (int) (pm.dc_resolution * pm.tvm_range_DC);

	xDC = xMIN;

	PWM_set_DC(xDC, xDC, xDC);
	PWM_set_Z(0);

	pm.vsi_UF = 0;
	pm.vsi_AZ = 0;
	pm.vsi_BZ = 0;
	pm.vsi_CZ = 0;

	xWake = xTaskGetTickCount();
	xTim0 = xWake;

	TLM_startup(&tlm, ap.FT_grab_hz, TLM_MODE_SINGLE_GRAB);

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

		if ((xWake - xTim0) > (TickType_t) 8000) {

			pm.fail_reason = PM_ERROR_TIMEOUT;
			break;
		}

		if (pm.fail_reason != PM_OK)
			break;
	}
	while (1);

	pm.fsm_req = PM_STATE_HALT;
	pm_wait_for_IDLE();
}

SH_DEF(hal_PPM_get_PERIOD)
{
	float		period, freq;

	period = PPM_get_PERIOD();
	freq = 1000000.f / period;

	printf("%3f (us) %1f (Hz)" EOL, &period, &freq);
}

SH_DEF(hal_PPM_get_PULSE)
{
	float		pulse;

	pulse = PPM_get_PULSE();

	printf("%3f (us)" EOL, &pulse);
}

SH_DEF(hal_ADC_get_analog_ANG)
{
	float		analog;

	analog = ADC_get_analog_ANG();

	printf("%3f (V)" EOL, &analog);
}

SH_DEF(hal_ADC_get_analog_BRK)
{
	float		analog;

	analog = ADC_get_analog_BRK();

	printf("%3f (V)" EOL, &analog);
}

SH_DEF(hal_GPIO_get_HALL)
{
	int		HS;

	HS = GPIO_get_HALL();

	printf("%i" EOL, HS);
}

SH_DEF(hal_TIM_get_EP)
{
	int		EP;

	EP = TIM_get_EP();

	printf("%i" EOL, EP);
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

