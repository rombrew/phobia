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

#ifndef _H_KALF_
#define _H_KALF_

typedef struct {

	/* Conversion gain.
	 * */
	float		gI;

	/* Zero drift.
	 * */
	float		aD;
	float		cD;

	/* Plant constants.
	 * */
	float		R;
	float		iL;
	float		E;
	float		U;
	float		Z;
	float		iJ;
}
kalf_const_t;

typedef struct {

	float		tdel;

	/* Control signal.
	 * */
	float		u[2];

	/* State and covariance.
	 * */
	float		x[6];
	float		pp[15];

	/* Kalman gain.
	 * */
	float		kk[10];

	/* Noise variance.
	 * */
	float		qq[6];
	float		rr[2];

	/* Constants of the plant.
	 * */
	kalf_const_t	c;
}
kalf_t;

extern kalf_t	kf;

void kalf_enable(float tdel);
void kalf_update(int i[2]);

#endif /* _H_KALF_ */

