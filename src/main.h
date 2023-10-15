#ifndef _H_MAIN_
#define _H_MAIN_

#include "phobia/libm.h"
#include "phobia/pm.h"

#include "ntc.h"
#include "tlm.h"

typedef struct {

	/* PPM interface knob.
	 * */
	float			ppm_in_pulse;
	float			ppm_in_freq;
	float			ppm_reg_DATA;
	int			ppm_reg_ID;
	int			ppm_STARTUP;
	int			ppm_ACTIVE;
	int			ppm_DISARM;
	float			ppm_range_pulse[3];
	float			ppm_range_control[3];

	/* STEP/DIR interface knob.
	 * */
	int			step_in_EP;
	float			step_reg_DATA;
	int			step_reg_ID;
	int			step_STARTUP;
	int			step_ACTIVE;
	int			step_prev_EP;
	int			step_accu_EP;
	float			step_const_ld_EP;

	/* Analog interface knob.
	 * */
#ifdef HW_HAVE_ANALOG_KNOB
	float			knob_in_ANG;
	float			knob_in_BRK;
	float			knob_reg_DATA;
	int			knob_reg_ID;
	int			knob_ENABLED;
#ifdef HW_HAVE_BRAKE_KNOB
	int			knob_BRAKE;
#endif /* HW_HAVE_BRAKE_KNOB */
	int			knob_STARTUP;
	int			knob_ACTIVE;
	int			knob_DISARM;
	float			knob_range_ANG[3];
#ifdef HW_HAVE_BRAKE_KNOB
	float			knob_range_BRK[2];
#endif /* HW_HAVE_BRAKE_KNOB */
	float			knob_range_LST[2];
	float			knob_control_ANG[3];
	float			knob_control_BRK;
#endif /* HW_HAVE_ANALOG_KNOB */

	/* IDLE function.
	 * */
	float			idle_timeout;
	int			idle_RESET;
	int			idle_INVOKE;
	int			idle_revol_cached;

	/* Disarm function.
	 * */
	float			disarm_timeout;
	int			disarm_RESET;
	int			disarm_INVOKE;

	/* Autostart function.
	 * */
	float			auto_reg_DATA;
	int			auto_reg_ID;
	int			auto_ENABLED;

	/* NTC constants.
	 * */
	ntc_t			ntc_PCB;
	ntc_t			ntc_EXT;

	/* Thermal info.
	 * */
	float			temp_PCB;
	float			temp_EXT;
	float			temp_MCU;

	/* Overheat protection.
	 * */
	float			heat_PCB_temp_halt;
	float			heat_PCB_temp_derate;
	float			heat_PCB_temp_FAN;
	float			heat_EXT_temp_derate;
	float			heat_derated_PCB;
	float			heat_derated_EXT;
	float			heat_temp_recovery;

	/* App enable knobs.
	 * */
	int			task_BUTTON;
	int			task_AS5047;
	int			task_HX711;
	int			task_MPU6050;

	/* ADC load cell (e.g. HX711).
	 * */
	int			load_HX711;

	/* SPI magnetic encoder (e.g. AS5047).
	 * */
	int 			(* proc_get_EP) ();
}
app_main_t;

extern app_main_t		ap;
extern pmc_t			pm;
extern tlm_t			tlm;

extern int flash_block_regs_load();
extern int pm_wait_for_idle();

void app_halt();

#endif /* _H_MAIN_ */

