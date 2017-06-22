/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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

#ifndef _H_BL_MODEL_
#define _H_BL_MODEL_

typedef struct {

	double		Tsim;
	double		dT, sT;
	int		PWM_resolution;

	/* Duty Cycle (Input).
	 * */
	int		uA;
	int		uB;
	int		uC;

	/* State of VSI.
	 * */
	int		sF[3];

	/* Motor variabes.
	 * */
	double		X[5];

	/* Constants.
	 * */
	double		R;
	double		Q;
	double		Ld;
	double		Lq;
	double		E;
	double		U;
	int		Zp;
	double		J;
	double		M[4];

	/* ADC result (Output).
	 * */
	float		sensor_A;
	float		sensor_B;
	float		supply_U;
}
blm_t;

void blm_Enable(blm_t *m);
void blm_Update(blm_t *m);

#endif /* _H_BL_MODEL_ */

