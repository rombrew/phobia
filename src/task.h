/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

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

#ifndef _H_TASK_
#define _H_TASK_

#include "pmc.h"

typedef struct {

	/* Seconds from Power Up.
	 * */
	int			uSEC;
	int			uDS;

	/* IRQ load ticks.
	 * */
	int			Tirq;

	/* IRQ handler.
	 * */
	void			(* pIRQ) ();

	/* Average variables.
	 * */
	float			*avg_IN[8];
	float			avg_SUM[8];
	int			avg_K, avg_N, avg_MAX;
	float			avg_default_time;
}
taskDATA_t;

extern taskDATA_t		td;
extern pmc_t			pm;

extern void taskIOMUX();

#endif /* _H_TASK_ */

