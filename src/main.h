#ifndef _H_MAIN_
#define _H_MAIN_

#include "phobia/pm.h"
#include "phobia/libm.h"

#include "libc.h"
#include "ntc.h"
#include "tlm.h"

typedef struct {

	/* IRQ entrance counter.
	 * */
	int			lc_irq_CNT;

	/* CPU load counters.
	 * */
	int			lc_FLAG;
	int			lc_TICK;
	int			lc_IDLE;

	/* INPUT lock during probe.
	 * */
	int			probe_LOCK;

	/* PPM interface (PWM).
	 * */
	int			ppm_reg_ID;
	int			ppm_STARTUP;
	int			ppm_ACTIVE;
	int			ppm_DISARM;
	float			ppm_in_range[3];
	float			ppm_control_range[3];

	/* STEP/DIR interface.
	 * */
	int			step_reg_ID;
	int			step_STARTUP;
	int			step_ACTIVE;
	int			step_baseEP;
	int			step_accuEP;
	float			step_const_ld_EP;

	/* Knob analog interface.
	 * */
	int			knob_ENABLED;
	int			knob_reg_ID;
	int			knob_STARTUP;
	int			knob_ACTIVE;
	float			knob_in_ANG[3];
	float			knob_in_BRK[2];
	float			knob_in_lost[2];
	float			knob_control_ANG[3];
	float			knob_control_BRK;

	/* IDLE control.
	 * */
	float			idle_TIME;
	int			idle_RESET;
	int			idle_INVOKE;
	int			idle_revol_cached;

	/* DISARM control.
	 * */
	int			disarm_RESET;
	int			disarm_INVOKE;

	/* NTC constants.
	 * */
	ntc_t			ntc_PCB;
	ntc_t			ntc_EXT;

	/* Thermal info.
	 * */
	float			temp_PCB;
	float			temp_EXT;
	float			temp_MCU;

	/* Thermal protection.
	 * */
	float			tpro_PCB_temp_halt;
	float			tpro_PCB_temp_derate;
	float			tpro_PCB_temp_FAN;
	float			tpro_EXT_temp_derate;
	float			tpro_derated_PCB;
	float			tpro_derated_EXT;
	float			tpro_temp_recovery;

	/* App knobs.
	 * */
	int			task_AS5047;
	int			task_HX711;
	int			task_MPU6050;
	int			task_PUSHTWO;

	/* ADC load cell (e.g. HX711).
	 * */
	float			adc_load_kg;
	float			adc_load_scale[2];

	/* Digital position sensor (e.g. AS5047).
	 * */
	int			pulse_EP;
}
app_main_t;

extern app_main_t		ap;
extern pmc_t			pm;
extern tlm_t			tlm;

extern int flash_block_regs_load();
extern int pm_wait_for_idle();

void app_halt();

float ADC_get_knob_ANG();
float ADC_get_knob_BRK();

#endif /* _H_MAIN_ */

