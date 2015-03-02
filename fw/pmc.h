/*
   Phobia DC Motor Controller for RC and robotics.
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

#ifndef _H_PMC_
#define _H_PMC_

enum {
	PMC_REQ_NULL				= 0,
	PMC_REQ_SPINUP,
	PMC_REQ_BREAK,
	PMC_REQ_SINE,
	PMC_REQ_CALIBRATE
};

enum {
	PMC_MODE_EKF_5X_BASE			= 0x0100,
	PMC_MODE_EKF_8X_J			= 0x0200,
	PMC_MODE_SPEED_CONTROL_LOOP		= 0x0002,
	PMC_MODE_EFFICIENT_MODULATION		= 0x0004,
	PMC_MODE_HIGH_FREQUENCY_INJECTION	= 0x0008,
};

enum {
	PMC_STATE_IDLE				= 0,
	PMC_STATE_DRIFT,
	PMC_STATE_SINE,
	PMC_STATE_CALIBRATE,
	PMC_STATE_SPINUP,
	PMC_STATE_BREAK,
	PMC_STATE_END
};

typedef struct {

	/* Frequency (Hz).
	 * */
	float		hzF;
	float		dT;

	/* PWM resolution.
	 * */
	int		pwmR;

	/* FSM variables.
	 * */
	int		mReq;
	int		mBit;
	int		mS1;
	int		mS2;

	/* Timer variables.
	 * */
	int		timVal;
	int		timEnd;

	/* Configuration.
	 * */
	float		Tdrift, Tend;
	float		pwmEPS;
	int		pwmMP;

	/* Conversion constants.
	 * */
	float		cA0, cA1;
	float		cB0, cB1;
	float		cU0, cU1;

	/* Control voltage.
	 * */
	float		uX, uY;

	/* Base EKF.
	 * */
	float		kX[5];
	float		kP[21];
	float		kQ[6];
	float		kR;

	/* Temporal.
	 * */
	float		kT[5];

	/* Zero Drift.
	 * */
	float		Ad;
	float		Bd;
	float		Qd;

	/* Electrical constants.
	 * */
	float		U;
	float		E;
	float		R;
	float		Ld;
	float		Lq;

	/* Mechanical constants.
	 * */
	int		Zp;
	float		M;
	float		IJ;

	/* Current control loop.
	 * */
	float		iSPD;
	float		iSPQ;
	float		iXD;
	float		iXQ;
	float		iKP;
	float		iKI;

	/* Speed control loop.
	 * */
	float		wSP;
	float		wXX;
	float		wKP;
	float		wKI;

	/* Limit values.
	 * */
	float		iMAX;
	float		wMAX;
	float		wMIN;

	/* Control interface.
	 * */
	void 		(* pDC) (int, int, int);
	void 		(* pZ) (int);
}
pmc_t;

void pmcEnable(pmc_t *pm);
void pmcFeedBack(pmc_t *pm, int xA, int xB, int xU);

#endif /* _H_PMC_ */

