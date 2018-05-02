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

#include "pm/pm.h"

#include "lib.h"
#include "ntc.h"
#include "telinfo.h"

typedef struct {

	/* Serial IO interfaces.
	 * */
	io_ops_t		io_USART;
	io_ops_t		io_CAN;

	/* CPU load.
	 * */
	int			lc_flag;
	int			lc_tick;
	int			lc_idle;

	/* NTC constants.
	 * */
	ntc_t			ntc_PCB;
	ntc_t			ntc_EXT;

	/* Thermal.
	 * */
	float			t_PCB;
	float			t_EXT;
	float			t_TEMP;
}
application_t;

extern application_t		ap;
extern pmc_t			pm;
extern telinfo_t		ti;

extern int flash_block_load();

#endif /* _H_MAIN_ */

