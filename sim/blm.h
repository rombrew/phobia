/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2014 Roman Belov <romblv@gmail.com>

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

#ifndef _H_BLM_
#define _H_BLM_

typedef struct {

	double		Tsim;
	double		dT, sT;
	int		PWMR;

	/* Duty Cycle (Input).
	 * */
	int		uA;
	int		uB;
	int		uC;

	/* State of the FETs.
	 * */
	int		sF[3];

	/* State variabes.
	 * */
	double		X[7];

	/* Constants.
	 * */
	double		R;
	double		Q;
	double		L;
	double		E;
	double		U;
	int		Zp;
	double		J;
	double		M[4];

	/* Output variables.
	 * */
	int		iA;
	int		iB;
	int		uS;
}
blm_t;

void blmEnable(blm_t *m);
void blmUpdate(blm_t *m);

#endif /* _H_BLDC_ */

