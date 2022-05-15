#ifndef _H_MAIN_
#define _H_MAIN_

#include "phobia/pm.h"
#include "phobia/libm.h"

#include "libc.h"
#include "ntc.h"
#include "tlm.h"

typedef struct {

	/* CPU load (internals).
	 * */
	int			lc_flag;
	int			lc_tick;
	int			lc_idle;

	/* PPM interface (PWM).
	 * */
	int			ppm_reg_ID;
	int			ppm_STARTUP;
	int			ppm_ACTIVE;
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
	float			idle_TIME_s;
	int			idle_RESET;
	int			idle_INVOKE;
	int			idle_revol_cached;

	/* NTC constants.
	 * */
	ntc_t			ntc_PCB;
	ntc_t			ntc_EXT;

	/* Thermal info.
	 * */
	float			temp_PCB;
	float			temp_EXT;
	float			temp_INT;

	/* Thermal protection.
	 * */
	float			tpro_PCB_on_halt;
	float			tpro_PCB_on_FAN;
	float			tpro_derated_PCB;
	float			tpro_EXT_on_halt;
	float			tpro_derated_EXT;
	float			tpro_recovery;

	/* Servo drive.
	 * */
	float			servo_SPAN_mm[2];
	float			servo_UNIFORM_mmps;

	/* HX711 (load cell).
	 * */
	float			hx711_kg;
	float			hx711_scale[2];
}
app_main_t;

extern app_main_t		ap;
extern pmc_t			pm;
extern TLM_t			tlm;

extern int flash_block_regs_load();
extern int pm_wait_for_IDLE();

void app_halt();

float ADC_get_knob_ANG();
float ADC_get_knob_BRK();

#endif /* _H_MAIN_ */

