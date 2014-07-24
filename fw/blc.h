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
	BLC_MODE_		= 0x0002,
	BLC_MODE_2		= 0x0002,
	BLC_MODE_FLAG_ILOOP	= 0x0002,
	BLC_MODE_FLAG_WLOOP	= 0x0002,
};

enum {
	BLC_STATE_IDLE		= 0,
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

enum {
	BLC_REQUEST_CALIBRATE	= 0x0002,
	BLC_REQUEST_ESTIMATE_R	= 0x0004,
	BLC_REQUEST_ESTIMATE_L	= 0x0008,
	BLC_REQUEST_SPINUP	= 0x0010,
	BLC_REQUEST_BREAK	= 0x0020,
};

typedef struct {

	/* Frequency (Hz).
	 * */
	int		hzF;

	/* PWM resolution.
	 * */
	short int	pwmR;

	/* FSM variables.
	 * */
	short int	fMOF;
	short int	fST1;
	short int	fST2;
	short int	fREQ;

	/* Timer variables.
	 * */
	int		timVal;
	int		timEnd;

	/* Stage durations (millisecond).
	 * */
	short int	sT0, sT1, sT2;
	short int	sT3;

	/* Conversion constants.
	 * */
	short int	cA1, cB1;
	short int	cA0, cB0;
	short int	cU0, cU1;

	/* Control signal.
	 * */
	int		uX;
	int		uY;

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
	int		dqX, dqY;

	/*
	 * */
	float		gainZD;

	/* Plant constants.
	 * */
	float		R;
	float		L;
	float		K;
	int		U;

	/*
	 * */
	float		gainU;

	/* Current control loop.
	 * */
	int		iSPD;		/* INPUT (mA) */
	int		iSPQ;
	int		iXD;
	int		iXQ;
	short int	iKP;
	short int	iKI;

	/* Speed control loop.
	 * */
	float		wSP;		/* INPUT */

	/* Control interface.
	 * */
	void 		(* pDC) (int, int, int);
	void 		(* pZ) (int);

	/* Temporal storage.
	 * */
	int		tempA;
	int		tempB;
	int		tempU;
}
blc_t;

void blcEnable(blc_t *bl);
void blcFeedBack(blc_t *bl, int iA, int iB, int uS);

int blcReq(blc_t *bl, int reQ);

#endif /* _H_BLC_ */

