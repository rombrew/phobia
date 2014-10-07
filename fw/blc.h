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

#ifndef _H_BLC_
#define _H_BLC_

enum {
	BLC_MODE_FLUX_OBSERVER		= 0x0002,
	BLC_MODE_FLUX_LOCK		= 0x0004,
	BLC_MODE_VOLTAGE_ESTIMATE	= 0x0008,
	BLC_MODE_CURRENT_LOOP		= 0x0010,
	BLC_MODE_SPEED_LOOP		= 0x0020,
	BLC_MODE_CALIBRATE		= 0x0100,
	BLC_MODE_ESTIMATE_RL		= 0x0200,
};

enum {
	BLC_STATE_IDLE			= 0,
	BLC_STATE_DRIFT,
	BLC_STATE_CALIBRATE,
	BLC_STATE_ALIGN,
	BLC_STATE_ESTIMATE_R,
	BLC_STATE_ESTIMATE_L,
	BLC_STATE_SPINUP,
	BLC_STATE_BREAK,
	BLC_STATE_ESTIMATE_Q,
	BLC_STATE_ESTIMATE_J,
	BLC_STATE_END
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
	float		sT6;
	int		sN1;
	float		sISP;
	float		sAq;

	/* Conversion constants.
	 * */
	float		cA0, cA1;
	float		cB0, cB1;
	float		cU0, cU1;

	/* Control voltage (mV).
	 * */
	float		uX, uY;

	/* Flux estimation.
	 * */
	float		fluxX;
	float		fluxY;

	/* Flux observer tunables.
	 * */
	float		gainF;
	float		gainK;

	/* Speed estimation.
	 * */
	float		wR;

	/* Speed observer tunables.
	 * */
	float		gainW;

	/* DQ frame.
	 * */
	short int	dqX, dqY;

	/* Plant constants.
	 * */
	float		R;
	float		L;
	float		E;
	float		U;

	/*
	 * */
	float		gainU;

	/* Current control loop.
	 * */
	float		iSPD;		/* INPUT (mA) */
	float		iSPQ;
	float		iXD;
	float		iXQ;
	float		iKP;
	float		iKI;

	/* Speed control loop.
	 * */
	float		wSP;		/* INPUT */

	/* Control interface.
	 * */
	void 		(* pDC) (int, int, int);
	void 		(* pZ) (int);

	/* Temporal storage.
	 * */
	float		tempA;
	float		tempB;
	float		tempC;
}
blc_t;

void blcEnable(blc_t *bl);
void blcFeedBack(blc_t *bl, int xA, int xB, int xU);

#endif /* _H_BLC_ */

