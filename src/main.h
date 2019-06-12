#ifndef _H_MAIN_
#define _H_MAIN_

#include "phobia/pm.h"
#include "phobia/libm.h"

#include "libc.h"
#include "ntc.h"
#include "tel.h"

typedef struct {

	/* Serial IO interfaces.
	 * */
	io_ops_t		io_USART;
	io_ops_t		io_CAN;

	/* CAN interface.
	 * */
	int			can_node_ID;

	/* PPM interface.
	 * */
	int			ppm_reg_ID;
	int			ppm_locked;
	float			ppm_pulse_cached;
	float			ppm_pulse_range[3];
	float			ppm_control_range[3];
	float			ppm_startup_range[2];

	/* Analog interface.
	 * */
	int			analog_reg_ID;
	int			analog_locked;
	float			analog_voltage_ratio;
	float			analog_timeout;
	float			analog_no_lost_range[2];
	float			analog_voltage_range[3];
	float			analog_control_range[3];
	float			analog_startup_range[2];

	/* CPU load.
	 * */
	int			lc_flag;
	int			lc_tick;
	int			lc_idle;

	/* NTC constants.
	 * */
	ntc_t			ntc_PCB;
	ntc_t			ntc_EXT;

	/* Thermal info.
	 * */
	float			temp_PCB;
	float			temp_EXT;
	float			temp_INT;

	/* Heating control.
	 * */
	float			heat_PCB;
	float			heat_PCB_derated;
	float			heat_EXT;
	float			heat_EXT_derated;
	float			heat_PCB_FAN;
	float			heat_gap;

	/* Battery voltage.
	 * */
	float			batt_voltage_low;
	float			batt_gap;
	float			batt_derated;

	/* Load cell.
	 * */
	float			pull_g;
	float			pull_ad[2];
}
application_t;

extern application_t		ap;
extern pmc_t			pm;
extern tel_t			ti;

extern int flash_block_load();
extern int pm_wait_for_IDLE();

float ADC_get_ANALOG();

void lowTRACE(const char *fmt, ...);

#endif /* _H_MAIN_ */

