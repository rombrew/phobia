/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2013 Roman Belov <romblv@gmail.com>

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

#ifndef _H_PLANT_
#define _H_PLANT_

#define PLANT_STATE_SIZE	6

typedef struct {

	double		tsim;
	double		tdel;
	int		pwmf;

	/* Input variables.
	 * */
	double		u[3];
	int		i[3];

	/* State variabes.
	 * */
	double		x[PLANT_STATE_SIZE];

	/* Constants.
	 * */
	double		const_R;
	double		const_L;
	double		const_E;
	double		const_U;
	double		const_Z;
	double		const_J;
	double		const_M[4];

	/* Output variables.
	 * */
	int		z[2];
}
plant_t;

extern plant_t		plant;

void plant_enable();
void plant_update();

#endif /* _H_PLANT_ */

