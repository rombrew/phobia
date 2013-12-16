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

#ifndef _H_BLC_
#define _H_BLC_

typedef struct {

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
	int		Z;
	float		iJ;
	float		M1;
	float		M2;
}
blc_const_t;

typedef struct {

	/* Zero drift filter.
	 * */
	float		zp;
	float		zq;
	float		zr;

	/* Plant filter.
	 * */
	float		pp[15];
	float		qq[5];
	float		rr[3];
}
blc_kali_t;

typedef struct {

	/* Current control loop.
	 * */
	float		sp;
	float		k[4];
	float		x[2];
}
blc_ccl_t;

typedef struct {

	float		stub;
}
blc_scl_t;

typedef struct {

	/* Delta.
	 * */
	float		tdel;

	/* FSM variables.
	 * */
	int		mode;

	/* Control signal.
	 * */
	float		u[2];

	/* State and covariance.
	 * */
	float		x[5];
	float		pp[25];

	/* Number of full turns.
	 * */
	int		noft;

	/* DQ frame.
	 * */
	float		dq[2];

	/* Observer innovation.
	 * */
	float		e[5];

	/* Kalman gain.
	 * */
	float		kk[10];

	/* Noise variance.
	 * */
	float		qq[5];
	float		rr[2];

	/* Constants of the plant.
	 * */
	blc_const_t	c;

	/* Plant identifier.
	 * */
	blc_kali_t	i;

	/* Current control loop.
	 * */
	blc_ccl_t	ccl;

	/* Speed control loop.
	 * */
	blc_scl_t	scl;
}
blc_t;

enum {
	BLC_MODE_IDLE		= 0,
	BLC_MODE_DRIFT,
	BLC_MODE_ALIGN,
	BLC_MODE_RUN,
};

extern blc_t		bl;

void blc_enable(float tdel);
void blc_update(const float z[2]);
void bridge_dc(const float dc[3]);

#endif /* _H_BLC_ */

