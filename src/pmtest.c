#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "tel.h"

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

SH_DEF(pm_FT_current_ramp)
{
	float			iSP;

	if (pm.lu_mode == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	tel_startup(&ti, 0, TEL_MODE_SINGLE_GRAB);

	do {
		iSP = pm.i_setpoint_Q;
		vTaskDelay((TickType_t) 1);

		pm.i_setpoint_Q = pm.i_maximal;
		vTaskDelay((TickType_t) 5);

		pm.i_setpoint_Q = iSP;
		vTaskDelay((TickType_t) 4);
	}
	while (0);
}

SH_DEF(pm_FT_speed_ramp)
{
	float			wSP;

	if (pm.lu_mode == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	tel_startup(&ti, 1000, TEL_MODE_SINGLE_GRAB);

	do {
		wSP = pm.s_setpoint;
		vTaskDelay((TickType_t) 100);

		pm.s_setpoint = pm.probe_speed_hold;
		vTaskDelay((TickType_t) 500);

		pm.s_setpoint = wSP;
		vTaskDelay((TickType_t) 400);
	}
	while (0);
}

SH_DEF(pm_FT_thrust_curve)
{
	/* TODO */
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

SH_DEF(hal_ADC_get_ANALOG)
{
	float		analog;

	analog = ADC_get_ANALOG();

	printf("%3f (V)" EOL, &analog);
}

SH_DEF(hal_ADC_get_BRAKE)
{
	float		analog;

	analog = ADC_get_BRAKE();

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

