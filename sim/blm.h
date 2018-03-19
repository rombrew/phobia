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

#ifndef _H_BLM_
#define _H_BLM_

typedef struct {

	double		Tsim;
	double		dT, sT;
	int		PWM_R;

	/* Duty Cycle (Input).
	 * */
	int		PWM_xA;
	int		PWM_xB;
	int		PWM_xC;

	/* State of VSI.
	 * */
	int		VSI[3];

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
	float		ADC_iA;
	float		ADC_iB;
	float		ADC_uS;

	float		ADC_uA;
	float		ADC_uB;
	float		ADC_uC;
}
blm_t;

void blm_Enable(blm_t *m);
void blm_Update(blm_t *m);

#endif /* _H_BLM_ */

