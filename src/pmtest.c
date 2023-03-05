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

	ap.probe_LOCK = PM_ENABLED;

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_CONST_FB_U]);
		reg_format(&regfile[ID_PM_AD_IA0]);
		reg_format(&regfile[ID_PM_AD_IB0]);
		reg_format(&regfile[ID_PM_AD_IC0]);

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

			if (		pm.fsm_errno == PM_ERROR_NO_MOTOR_CONNECTED
					|| pm.fsm_errno == PM_OK) ;
			else break;
		}

		xDC = pm.dc_resolution - pm.ts_clearance;

		for (N = 0; N < 2; ++N) {

			switch (N) {

				case 0:
					pm.proc_set_DC(0, 0, 0);
					pm.proc_set_Z(PM_Z_NUL);
					break;

				case 1:
					pm.proc_set_DC(xDC, xDC, xDC);
					pm.proc_set_Z(PM_Z_NUL);
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

	ap.probe_LOCK = PM_DISABLED;
}

SH_DEF(pm_self_adjust)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return;
	}

	ap.probe_LOCK = PM_ENABLED;

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_CONST_FB_U]);
		reg_format(&regfile[ID_PM_AD_IA0]);
		reg_format(&regfile[ID_PM_AD_IB0]);
		reg_format(&regfile[ID_PM_AD_IC0]);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_ADJUST_VOLTAGE;
			pm_wait_for_IDLE();

			reg_format(&regfile[ID_PM_AD_UA0]);
			reg_format(&regfile[ID_PM_AD_UA1]);
			reg_format(&regfile[ID_PM_AD_UB0]);
			reg_format(&regfile[ID_PM_AD_UB1]);
			reg_format(&regfile[ID_PM_AD_UC0]);
			reg_format(&regfile[ID_PM_AD_UC1]);

			reg_format(&regfile[ID_PM_TVM_USEABLE]);
			reg_format(&regfile[ID_PM_TVM_FIR_A_TAU]);
			reg_format(&regfile[ID_PM_TVM_FIR_B_TAU]);
			reg_format(&regfile[ID_PM_TVM_FIR_C_TAU]);

			reg_format(&regfile[ID_PM_SELF_RMSU]);

			if (pm.fsm_errno != PM_OK)
				break;
		}

		pm.fsm_req = PM_STATE_ADJUST_CURRENT;
		pm_wait_for_IDLE();

		reg_format(&regfile[ID_PM_AD_IA1]);
		reg_format(&regfile[ID_PM_AD_IB1]);
		reg_format(&regfile[ID_PM_AD_IC1]);
	}
	while (0);

	reg_format(&regfile[ID_PM_FSM_ERRNO]);

	ap.probe_LOCK = PM_DISABLED;
}

SH_DEF(pm_test_current_ramp)
{
	TickType_t		xTS1;
	float			iSP;

	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	xTS1 = (TickType_t) (100UL * TLM_DATA_MAX / tlm.freq_grab_hz);

	tlm_startup(&tlm, tlm.freq_grab_hz, TLM_MODE_SINGLE_GRAB);

	do {
		iSP = reg_GET_F(ID_PM_I_SETPOINT_CURRENT);
		vTaskDelay(1UL * xTS1);

		reg_SET_F(ID_PM_I_SETPOINT_CURRENT, pm.i_maximal);
		vTaskDelay(5UL * xTS1);

		reg_SET_F(ID_PM_I_SETPOINT_CURRENT, iSP);
		vTaskDelay(4UL * xTS1);
	}
	while (0);
}

SH_DEF(pm_test_speed_ramp)
{
	TickType_t		xTS1;
	float			wSP;

	if (pm.lu_MODE == PM_LU_DISABLED) {

		printf("Unable when PM is stopped" EOL);
		return;
	}

	xTS1 = (TickType_t) (100UL * TLM_DATA_MAX / tlm.freq_grab_hz);

	tlm_startup(&tlm, tlm.freq_grab_hz, TLM_MODE_SINGLE_GRAB);

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
	xMAX = (int) (pm.dc_resolution * pm.tvm_range_pc);

	xDC = xMIN;

	PWM_set_DC(xDC, xDC, xDC);
	PWM_set_Z(0);

	pm.vsi_UF = 0;
	pm.vsi_AZ = 1;
	pm.vsi_BZ = 1;
	pm.vsi_CZ = 1;

	xWake = xTaskGetTickCount();
	xTim0 = xWake;

	tlm_startup(&tlm, tlm.freq_grab_hz, TLM_MODE_SINGLE_GRAB);

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

SH_DEF(hal_ADC_scan_CH)
{
	int			xCH, xGPIO;
	float			fU;

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
		fU = ADC_get_VALUE(xGPIO);

		printf("P%c%i %4f" EOL, 'A' + XGPIO_GET_PORT(xGPIO),
				XGPIO_GET_N(xGPIO), &fU);
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
	int		hz, mode;

	if (stof(&freq, s) != NULL) {

		hz = (int) freq;
		mode = SPI_MODE_LOW_RISING;

		stoi(&mode, sh_next_arg(s));

		SPI_startup(SPI_ID_EXT, hz, mode);
	}
}

SH_DEF(hal_SPI_halt)
{
	SPI_halt(SPI_ID_EXT);
}

SH_DEF(hal_SPI_transfer)
{
	int		txbuf, rxbuf;

	while (htoi(&txbuf, s) != NULL) {

		rxbuf = SPI_transfer(SPI_ID_EXT, txbuf);

		printf("%4x ", rxbuf);

		s = sh_next_arg(s);
	}

	puts(EOL);
}

SH_DEF(hal_GPIO_set_high_FAN_EN)
{
#ifdef HW_HAVE_FAN_CONTROL
	GPIO_set_HIGH(GPIO_FAN_EN);
#endif /* HW_HAVE_FAN_CONTROL */
}

SH_DEF(hal_GPIO_set_low_FAN_EN)
{
#ifdef HW_HAVE_FAN_CONTROL
	GPIO_set_LOW(GPIO_FAN_EN);
#endif /* HW_HAVE_FAN_CONTROL */
}

