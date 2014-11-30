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

#ifndef _H_PMC_
#define _H_PMC_

enum {
	PMC_MODE_OBSERVER		= 0x0002,
	PMC_MODE_VOLTAGE_ESTIMATE	= 0x0008,
	PMC_MODE_CURRENT_LOOP		= 0x0010,
	PMC_MODE_SPEED_LOOP		= 0x0020,
	PMC_MODE_CALIBRATE		= 0x0100,
	PMC_MODE_ESTIMATE_RL		= 0x0200,
};

enum {
	PMC_STATE_IDLE			= 0,
	PMC_STATE_DRIFT,
	PMC_STATE_CALIBRATE,
	PMC_STATE_ALIGN,
	PMC_STATE_ESTIMATE_RL,
	PMC_STATE_SPINUP,
	PMC_STATE_BREAK,
	PMC_STATE_ESTIMATE_Q,
	PMC_STATE_ESTIMATE_J,
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
	int		fMOF;
	int		fST1;
	int		fST2;

	/* Timer variables.
	 * */
	int		timVal;
	int		timEnd;

	/* Startup configuration.
	 * */
	float		sT0, sT1, sT2;
	float		sT3, sT4, sT5;
	float		sISP, sFq, sAq;

	/* Conversion constants.
	 * */
	float		cA0, cA1;
	float		cB0, cB1;
	float		cU0, cU1;

	/* Control voltage.
	 * */
	float		uX, uY;

	/* Kalman filter.
	 * */
	float		kX[8];
	float		kP[64];
	float		kQ[8];
	float		kR;
	float		kK[16];

	/* DQ frame.
	 * */
	float		pX, pY;

	/* Electrical constants.
	 * */
	float		U;
	float		R;
	float		IL;
	float		E;

	/* Mechanical constants.
	 * */
	int		Zp;
	float		IJ;

	/* Voltage observer tunable.
	 * */
	float		gainU;

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

	/* Control interface.
	 * */
	void 		(* pDC) (int, int, int);
	void 		(* pZ) (int);

	/* Temporal storage.
	 * */
	struct {

		float		re;
		float		im;
	}
	tA, tB, tC, tD;

	float		eD, eQ;
}
pmc_t;

void pmcEnable(pmc_t *pm);
void pmcFeedBack(pmc_t *pm, int xA, int xB, int xU);

#endif /* _H_PMC_ */

