/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _H_MAIN_
#define _H_MAIN_

#include "phobia/pm.h"

#include "libc.h"
#include "ntc.h"
#include "teli.h"

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
	float			ppm_pulse_cached;
	float			ppm_pulse_range[2];
	float			ppm_control_range[2];

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

	/* Overheat.
	 * */
	float			temp_PCB_overheat;
	float			temp_superheat;
	float			temp_current_PCB_derated;

	/* Battery voltage.
	 * */
	float			batt_voltage_low;
	float			batt_voltage_high;

	/* Load cell.
	 * */
	float			load_thrust_gram;
	float			load_transform[2];
}
application_t;

extern application_t		ap;
extern pmc_t			pm;
extern teli_t			ti;

extern int flash_block_load();
extern int pm_wait_for_IDLE();
extern void pm_print_fail_reason();

void lowTRACE(const char *fmt, ...);

#endif /* _H_MAIN_ */

