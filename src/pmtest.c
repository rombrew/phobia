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
		pm_wait_for_idle();

		tlm_startup(&tlm, tlm.grabfreq, TLM_MODE_WATCH);

		reg_format(&regfile[ID_PM_CONST_FB_U]);
		reg_format(&regfile[ID_PM_SCALE_IA0]);
		reg_format(&regfile[ID_PM_SCALE_IB0]);
		reg_format(&regfile[ID_PM_SCALE_IC0]);
		reg_format(&regfile[ID_PM_SELF_STDI]);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_BOOTSTRAP;
			pm_wait_for_idle();

			reg_format(&regfile[ID_PM_SELF_BST]);
			reg_format(&regfile[ID_PM_FSM_ERRNO]);

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;
			pm_wait_for_idle();

			reg_format(&regfile[ID_PM_SELF_IST]);
			reg_format(&regfile[ID_PM_FSM_ERRNO]);
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
			pm_wait_for_idle();

			reg_format(&regfile[ID_PM_SELF_RMSI]);
			reg_format(&regfile[ID_PM_FSM_ERRNO]);

			if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

				reg_format(&regfile[ID_PM_SELF_RMSU]);
			}
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
		pm_wait_for_idle();

		tlm_startup(&tlm, tlm.grabfreq, TLM_MODE_WATCH);

		reg_format(&regfile[ID_PM_CONST_FB_U]);
		reg_format(&regfile[ID_PM_SCALE_IA0]);
		reg_format(&regfile[ID_PM_SCALE_IB0]);
		reg_format(&regfile[ID_PM_SCALE_IC0]);
		reg_format(&regfile[ID_PM_SELF_STDI]);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_ADJUST_VOLTAGE;
			pm_wait_for_idle();

			reg_format(&regfile[ID_PM_SCALE_UA0]);
			reg_format(&regfile[ID_PM_SCALE_UA1]);
			reg_format(&regfile[ID_PM_SCALE_UB0]);
			reg_format(&regfile[ID_PM_SCALE_UB1]);
			reg_format(&regfile[ID_PM_SCALE_UC0]);
			reg_format(&regfile[ID_PM_SCALE_UC1]);

			reg_format(&regfile[ID_PM_TVM_USEABLE]);
			reg_format(&regfile[ID_PM_TVM_FIR_A_TAU]);
			reg_format(&regfile[ID_PM_TVM_FIR_B_TAU]);
			reg_format(&regfile[ID_PM_TVM_FIR_C_TAU]);

			reg_format(&regfile[ID_PM_SELF_RMSU]);

			if (pm.fsm_errno != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_ADJUST_CURRENT;
		pm_wait_for_idle();

		reg_format(&regfile[ID_PM_SCALE_IA1]);
		reg_format(&regfile[ID_PM_SCALE_IB1]);
		reg_format(&regfile[ID_PM_SCALE_IC1]);
		reg_format(&regfile[ID_PM_SELF_RMSI]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	tlm_halt(&tlm);
}

SH_DEF(pm_self_tvm_ramp)
{
	TickType_t		xWake, xTim0;
	int			xDC, xMIN, xMAX;

	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	if (		PM_CONFIG_TVM(&pm) != PM_ENABLED
			|| pm.tvm_USEABLE != PM_ENABLED) {

		printf("Unable with TVM disabled" EOL);
		return;
	}

	xMIN = pm.ts_minimal;
	xMAX = (int) (pm.dc_resolution * pm.tvm_clean_zone);

	xDC = xMIN;

	PWM_set_DC(0, 0, 0);
	PWM_set_Z(0);

	pm_clearance(&pm, 0, 0, 0);
	pm_clearance(&pm, 0, 0, 0);

	xWake = xTaskGetTickCount();
	xTim0 = xWake;

	tlm_startup(&tlm, tlm.grabfreq, TLM_MODE_GRAB);

	do {
		/* 1000 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 1);

		xDC = (xDC < xMAX) ? xDC + 1 : xMIN;

		PWM_set_DC(xDC, xDC, xDC);

		pm_clearance(&pm, xDC, xDC, xDC);

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

	PWM_set_DC(0, 0, 0);
	PWM_set_Z(PM_Z_ABC);
}

SH_DEF(hal_ADC_scan_CH)
{
	int			xCH, xGPIO;
	float			fvoltage;

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

		xGPIO = gpios_stm32f405_lqfp64[xCH];

		GPIO_set_mode_ANALOG(xGPIO);

		fvoltage = ADC_get_sample(xGPIO) * hal.ADC_reference_voltage;

		printf("P%c%i %4f (V)" EOL, 'A' + XGPIO_GET_PORT(xGPIO),
				XGPIO_GET_N(xGPIO), &fvoltage);
	}
}

SH_DEF(hal_PWM_set_DC)
{
	int			xDC, xMAX;

	if (stoi(&xDC, s) != NULL) {

		xMAX = hal.PWM_resolution;
		xDC = (xDC < 0) ? 0 : (xDC > xMAX) ? xMAX : xDC;

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

SH_DEF(hal_SPI_startup)
{
	float		freq;
	int		bus_ID, mode;

	if (stoi(&bus_ID, s) != NULL) {

		freq = 1000000.f;
		mode = 0;

		stof(&freq, s = sh_next_arg(s));
		stoi(&mode, s = sh_next_arg(s));

		SPI_startup(bus_ID, (int) freq, mode);
	}
}

SH_DEF(hal_SPI_halt)
{
	int		bus_ID;

	if (stoi(&bus_ID, s) != NULL) {

		SPI_halt(bus_ID);
	}
}

SH_DEF(hal_SPI_transfer)
{
	int		bus_ID, txbuf, rxbuf;

	if (stoi(&bus_ID, s) != NULL) {

		while (htoi(&txbuf, s = sh_next_arg(s)) != NULL) {

			rxbuf = SPI_transfer(bus_ID, txbuf, 500);

			printf("%4x ", rxbuf);
		}

		puts(EOL);
	}
}

SH_DEF(hal_DBGMCU_mode_stop)
{
	DBGMCU_mode_stop();
}

