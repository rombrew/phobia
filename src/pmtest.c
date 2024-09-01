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

	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_IDLE();

		tlm_startup(&tlm, tlm.rate_grab, TLM_MODE_WATCH);

		reg_OUTP(ID_PM_CONST_FB_U);
		reg_OUTP(ID_PM_SCALE_IA0);
		reg_OUTP(ID_PM_SCALE_IB0);
		reg_OUTP(ID_PM_SCALE_IC0);
		reg_OUTP(ID_PM_SELF_STDI);

		if (pm.fsm_errno != PM_OK) {

			reg_OUTP(ID_PM_FSM_ERRNO);
			break;
		}

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_BOOTSTRAP;
			pm_wait_IDLE();

			reg_OUTP(ID_PM_SELF_BST);
			reg_OUTP(ID_PM_FSM_ERRNO);

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;
			pm_wait_IDLE();

			reg_OUTP(ID_PM_SELF_IST);
			reg_OUTP(ID_PM_FSM_ERRNO);
		}

		xDC = pm.dc_resolution - pm.ts_clearance;

		for (N = 0; N < 2; ++N) {

			switch (N) {

				case 0:
					pm.proc_set_DC(0, 0, 0);
					pm.proc_set_Z(PM_Z_NONE);
					break;

				case 1:
					pm.proc_set_DC(xDC, xDC, xDC);
					pm.proc_set_Z(PM_Z_NONE);
					break;
			}

			pm.fsm_req = PM_STATE_SELF_TEST_CLEARANCE;
			pm_wait_IDLE();

			reg_OUTP(ID_PM_SELF_RMSI);

			if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

				reg_OUTP(ID_PM_SELF_RMSU);
				reg_OUTP(ID_PM_SELF_RMST);
			}

			reg_OUTP(ID_PM_FSM_ERRNO);
		}
	}
	while (0);

	tlm_halt(&tlm);
}

SH_DEF(pm_self_adjust)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_IDLE();

		tlm_startup(&tlm, tlm.rate_grab, TLM_MODE_WATCH);

		reg_OUTP(ID_PM_CONST_FB_U);
		reg_OUTP(ID_PM_SCALE_IA0);
		reg_OUTP(ID_PM_SCALE_IB0);
		reg_OUTP(ID_PM_SCALE_IC0);
		reg_OUTP(ID_PM_SELF_STDI);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_ADJUST_ON_PCB_VOLTAGE;
			pm_wait_IDLE();

			reg_OUTP(ID_PM_SCALE_UA0);
			reg_OUTP(ID_PM_SCALE_UA1);
			reg_OUTP(ID_PM_SCALE_UB0);
			reg_OUTP(ID_PM_SCALE_UB1);
			reg_OUTP(ID_PM_SCALE_UC0);
			reg_OUTP(ID_PM_SCALE_UC1);

			reg_OUTP(ID_PM_SELF_RMSU);
			reg_OUTP(ID_PM_SELF_RMST);

			if (pm.fsm_errno != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_ADJUST_ON_PCB_CURRENT;
		pm_wait_IDLE();

		reg_OUTP(ID_PM_SCALE_IA1);
		reg_OUTP(ID_PM_SCALE_IB1);
		reg_OUTP(ID_PM_SCALE_IC1);
		reg_OUTP(ID_PM_SELF_RMSI);

		if (pm.fsm_errno != PM_OK)
			break;

		pm.fsm_req = PM_STATE_ADJUST_DTC_VOLTAGE;
		pm_wait_IDLE();

		reg_OUTP(ID_PM_CONST_IM_RZ);
		reg_OUTP(ID_PM_DTC_DEADBAND);
		reg_OUTP(ID_PM_SELF_DTU);

		if (pm.fsm_errno != PM_OK)
			break;
	}
	while (0);

	reg_OUTP(ID_PM_FSM_ERRNO);

	tlm_halt(&tlm);
}

SH_DEF(pm_analysis_impedance)
{
	float		usual_freq, walk_freq, stop_freq;

	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	pm.fsm_req = PM_STATE_ZERO_DRIFT;
	pm_wait_IDLE();

	reg_OUTP(ID_PM_CONST_FB_U);
	reg_OUTP(ID_PM_SCALE_IA0);
	reg_OUTP(ID_PM_SCALE_IB0);
	reg_OUTP(ID_PM_SCALE_IC0);
	reg_OUTP(ID_PM_SELF_STDI);

	if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

		pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;
		pm_wait_IDLE();

		reg_OUTP(ID_PM_SELF_IST);
	}

	reg_OUTP(ID_PM_FSM_ERRNO);

	if (pm.fsm_errno != PM_OK) {

		printf("Unable to continue if there are errors" EOL);
		return;
	}

	/*
	tlm.reg_ID[0] = ID_PM_PROBE_FREQ_SINE;
	tlm.reg_ID[1] = ID_PM_CONST_IM_LD;
	tlm.reg_ID[2] = ID_PM_CONST_IM_LQ;
	tlm.reg_ID[3] = ID_PM_CONST_IM_A;
	tlm.reg_ID[4] = ID_PM_CONST_IM_RZ;
	*/

	tlm_startup(&tlm, tlm.rate_live, TLM_MODE_WATCH);

	usual_freq = pm.probe_freq_sine;
	pm.probe_freq_sine = pm.m_freq / 6.f;

	stop_freq = 400.f;
	walk_freq = (float) (int) ((pm.probe_freq_sine - stop_freq) / 90.f);

	printf("Fq@Hz     Ld@H   Lq@H   Rz@Ohm" EOL);

	do {
		if (pm.fsm_errno != PM_OK)
			break;

		pm.fsm_req = PM_STATE_PROBE_CONST_INDUCTANCE;
		pm_wait_IDLE();

		printf("%4g    %4g %4g %4g" EOL, &pm.probe_freq_sine,
				&pm.const_im_Ld, &pm.const_im_Lq, &pm.const_im_Rz);

		pm.probe_freq_sine += - walk_freq;

		if (pm.probe_freq_sine < stop_freq)
			break;
	}
	while (1);

	pm.probe_freq_sine = usual_freq;

	reg_OUTP(ID_PM_FSM_ERRNO);

	tlm_halt(&tlm);
}

SH_DEF(hal_ADC_scan)
{
	int			xCH, xGPIO;

	const int gpios_stm32f405_lqfp64[16] = {

		XGPIO_DEF3('A', 0, 0),
		XGPIO_DEF3('A', 1, 1),
		XGPIO_DEF3('A', 2, 2),
		XGPIO_DEF3('A', 3, 3),
		XGPIO_DEF3('A', 4, 4),
		XGPIO_DEF3('A', 5, 5),
		XGPIO_DEF3('A', 6, 6),
		XGPIO_DEF3('A', 7, 7),
		XGPIO_DEF3('B', 0, 8),
		XGPIO_DEF3('B', 1, 9),
		XGPIO_DEF3('C', 0, 10),
		XGPIO_DEF3('C', 1, 11),
		XGPIO_DEF3('C', 2, 12),
		XGPIO_DEF3('C', 3, 13),
		XGPIO_DEF3('C', 4, 14),
		XGPIO_DEF3('C', 5, 15)
	};

	if (		stoi(&xCH, s) != NULL
			&& xCH >= 0 && xCH < 16) {

		float		um;

		xGPIO = gpios_stm32f405_lqfp64[xCH];

		GPIO_set_mode_ANALOG(xGPIO);

		um = ADC_get_sample(xGPIO);

		printf("P%c%i %4f (V)" EOL, 'A' + XGPIO_GET_PORT(xGPIO),
				XGPIO_GET_N(xGPIO), &um);
	}
}

SH_DEF(hal_PWM_set_DC)
{
	int			xDC, xMAX;

	if (stoi(&xDC, s) != NULL) {

		xMAX = hal.PWM_resolution;
		xDC = (xDC < 0) ? 0 : (xDC > xMAX) ? xMAX : xDC;

		PWM_set_Z(0);
		PWM_set_DC(xDC, xDC, xDC);
	}
}

#ifdef HW_HAVE_FAN_CONTROL
SH_DEF(hal_FAN_control)
{
	int			control;

	if (stoi(&control, s) != NULL) {

		if (control != 0) {

			GPIO_set_HIGH(GPIO_FAN_EN);
		}
		else {
			GPIO_set_LOW(GPIO_FAN_EN);
		}
	}
}
#endif /* HW_HAVE_FAN_CONTROL */

SH_DEF(hal_DBGMCU_mode_stop)
{
	DBGMCU_mode_stop();
}

