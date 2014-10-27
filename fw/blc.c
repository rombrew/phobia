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

#include "blc.h"
#include "math.h"

inline float
clamp(float x, float h, float l)
{
	return (x > h) ? h : (x < l) ? l : x;
}

void blcEnable(blc_t *bl)
{
	bl->dT = 1.f / bl->hzF;

	/* Default config.
	 * */
	bl->sT0 = .1f;		/* Zero Drift */
	bl->sT1 = .4f;		/* Ramp Up */
	bl->sT2 = .7f;		/* Hold */
	bl->sT3 = .5f;		/* Estimate RL */
	bl->sT4 = .5f;		/* Spin Up */
	bl->sT5 = .0f;		/*  */
	bl->sISP = 1.f;		/* Current SetPoint */
	bl->sFq = 20.f;		/* Frequency (Hz)*/
	bl->sAq = 75.f;		/* Acceleration */

	bl->cA0 = 0.f;
	bl->cA1 = .01464844f;
	bl->cB0 = 0.f;
	bl->cB1 = .01464844f;
	bl->cU0 = 0.f;
	bl->cU1 = .00725098f;

	bl->iKP = 2e-2f;
	bl->iKI = 5e-3f;

	bl->gainF = .01f;
	bl->gainK = .001f;
	bl->gainR = .0f;
	bl->gainU = .011f;
}

static void
fFB(blc_t *bl, float iX, float iY)
{
	float		pX, pY, eF;

	bl->fX += (bl->uX - iX * bl->R) * bl->dT;
	bl->fY += (bl->uY - iY * bl->R) * bl->dT;

	pX = bl->fX - iX * bl->L;
	pY = bl->fY - iY * bl->L;

	eF = 1.f - bl->fK * bl->fK * (pX * pX + pY * pY);

	bl->fX += pX * eF * bl->gainF;
	bl->fY += pY * eF * bl->gainF;
	bl->fK += bl->fK * eF * bl->gainK;

	pX = (bl->fX - iX * bl->L) * bl->fK;
	pY = (bl->fY - iY * bl->L) * bl->fK;

	eF = (3.f - pX * pX - pY * pY) / 2.f;
	pX *= eF;
	pY *= eF;

	bl->pX = pX;
	bl->pY = pY;
}

static void
iFB(blc_t *bl, float iX, float iY)
{
	float		eD, eQ, uD, uQ, Q;
	float		uA, uB, uC, uX, uY;
	int		xA, xB, xC;

	eD = bl->iSPD - (bl->dqX * iX + bl->dqY * iY);
	eQ = bl->iSPQ - (bl->dqX * iY - bl->dqY * iX);

	bl->iXD = clamp(bl->iXD + bl->iKI * eD, 1.f, -1.f);
	uD = bl->iXD + bl->iKP * eD;

	bl->iXQ = clamp(bl->iXQ + bl->iKI * eQ, 1.f, -1.f);
	uQ = bl->iXQ + bl->iKP * eQ;

	uX = bl->dqX * uD - bl->dqY * uQ;
	uY = bl->dqY * uD + bl->dqX * uQ;

	uA = uX;
	uB = -.5f * uX + .8660254f * uY;
	uC = -.5f * uX - .8660254f * uY;

	uA = clamp(.5f * uA + .5f, 1.f, 0.f);
	uB = clamp(.5f * uB + .5f, 1.f, 0.f);
	uC = clamp(.5f * uC + .5f, 1.f, 0.f);

	xA = (int) (bl->pwmR * uA + .5f);
	xB = (int) (bl->pwmR * uB + .5f);
	xC = (int) (bl->pwmR * uC + .5f);

	bl->pDC(xA, xB, xC);

	Q = .33333333f * (xA + xB + xC);
	uA = (xA - Q) * bl->U / bl->pwmR;
	uB = (xB - Q) * bl->U / bl->pwmR;

	uX = uA;
	uY = .57735027f * uA + 1.1547005f * uB;

	bl->uX = uX;
	bl->uY = uY;
}

static void
wFB(blc_t *bl)
{
}

static void
bFSM(blc_t *bl, float iA, float iB, float uS, float iX, float iY)
{
	float		L, P;

	switch (bl->fST1) {

		case BLC_STATE_IDLE:
			break;

		case BLC_STATE_DRIFT:

			if (bl->fST2 == 0) {

				bl->tA.re = 0.f;
				bl->tB.re = 0.f;
				bl->tC.re = 0.f;

				bl->timVal = 0;
				bl->timEnd = 64;

				bl->fST2++;
			}
			else {
				bl->tA.re += -iA;
				bl->tB.re += -iB;
				bl->tC.re += uS - bl->U;

				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					/* Zero Drift */

					bl->cA0 += bl->tA.re / bl->timEnd;
					bl->cB0 += bl->tB.re / bl->timEnd;
					bl->U += bl->tC.re / bl->timEnd;

					if (bl->fST2 == 1) {

						bl->tA.re = 0.f;
						bl->tB.re = 0.f;
						bl->tC.re = 0.f;

						bl->timVal = 0;
						bl->timEnd = bl->hzF * bl->sT0;

						bl->fST2++;
					}
					else {
						bl->fMOF |= BLC_MODE_VOLTAGE_ESTIMATE;

						if (bl->fMOF & BLC_MODE_CALIBRATE)
							bl->fST1 = BLC_STATE_CALIBRATE;
						else
							bl->fST1 = BLC_STATE_ALIGN;

						bl->fST2 = 0;
					}
				}
			}
			break;

		case BLC_STATE_CALIBRATE:
			break;

		case BLC_STATE_ALIGN:

			if (bl->fST2 == 0) {

				bl->fMOF |= BLC_MODE_CURRENT_LOOP;

				bl->iSPD = 0.f;
				bl->iSPQ = 0.f;

				bl->iXD = 0.f;
				bl->iXQ = 0.f;

				bl->dqX = 1.f;
				bl->dqY = 0.f;

				bl->timVal = 0;
				bl->timEnd = bl->hzF * bl->sT1;

				bl->fST2++;
			}
			else if (bl->fST2 == 1) {

				/* Ramp Up */

				bl->iSPD = bl->sISP * bl->timVal / bl->timEnd;
				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					bl->timVal = 0;
					bl->timEnd = bl->hzF * bl->sT2;

					bl->fST2++;
				}
			}
			else if (bl->fST2 == 2) {

				/* Hold */

				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					if (bl->fMOF & BLC_MODE_ESTIMATE_RL)
						bl->fST1 = BLC_STATE_ESTIMATE_RL;
					else
						bl->fST1 = BLC_STATE_SPINUP;

					bl->fST2 = 0;
				}
			}
			break;

		case BLC_STATE_ESTIMATE_RL:

			if (bl->fST2 == 0) {

				bl->tA.re = 1.f;
				bl->tA.im = 0.f;
				bl->tB.re = 0.f;
				bl->tB.im = 0.f;
				bl->tC.re = 0.f;
				bl->tC.im = 0.f;
				bl->tD.re = 0.f;
				bl->tD.im = 6.2831853f * bl->sFq * bl->dT;

				bl->timVal = 0;
				bl->timEnd = bl->hzF * bl->sT3;

				bl->fST2++;
			}
			else if (bl->fST2 == 1) {

				bl->tA.re += -bl->tA.im * bl->tD.im;
				bl->tA.im += bl->tA.re * bl->tD.im;

				L = (3.f - bl->tA.re * bl->tA.re - bl->tA.im * bl->tA.im) * .5f;
				bl->tA.re *= L;
				bl->tA.im *= L;

				L = .5f * (iX + bl->iX);
				bl->tB.re += L * bl->tA.re;
				bl->tB.im += L * bl->tA.im;

				L = bl->uX;
				bl->tC.re += L * bl->tA.re;
				bl->tC.im += L * bl->tA.im;

				bl->iSPD = bl->sISP * (bl->tA.re + 1.f) * .5f;

				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					bl->fST2++;
				}
			}
			else {
				bl->tA.re = bl->tC.re * bl->tB.re + bl->tC.im * bl->tB.im;
				bl->tA.im = bl->tC.re * bl->tB.im - bl->tC.im * bl->tB.re;

				L = bl->tB.re * bl->tB.re + bl->tB.im * bl->tB.im;
				bl->tA.re /= L;
				bl->tA.im /= L;

				bl->R = bl->tA.re;
				bl->L = bl->tA.im / (bl->sFq * 6.2831853f);

				bl->fST1 = BLC_STATE_END;
				bl->fST2 = 0;
			}
			break;

		case BLC_STATE_SPINUP:

			if (bl->fST2 == 0) {

				bl->fMOF |= BLC_MODE_FLUX_OBSERVER;

				bl->fK = 1.f / bl->E;
				bl->fX = iX * bl->L + bl->E;
				bl->fY = iY * bl->L;

				bl->tA.re = 0.f;

				bl->timVal = 0;
				bl->timEnd = bl->hzF * bl->sT4;

				bl->fST2++;
			}
			else if (bl->fST2 == 1) {

				P = bl->sAq * bl->dT;
				L = (bl->tA.re + .5f * P) * bl->dT;
				bl->tA.re += P;

				bl->dqX += -bl->dqY * L;
				bl->dqY += bl->dqX * L;

				L = (3.f - bl->dqX * bl->dqX - bl->dqY * bl->dqY) * .5f;

				bl->dqX *= L;
				bl->dqY *= L;

				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					bl->iSPD = 0.f;
					bl->iSPQ = bl->sISP;

					bl->fMOF |= BLC_MODE_FLUX_LOCK;

					bl->fST1 = BLC_STATE_IDLE;
					bl->fST2 = 0;
				}
			}
			break;

		case BLC_STATE_END:
			break;
	}
}

void blcFeedBack(blc_t *bl, int xA, int xB, int xU)
{
	float		iA, iB, uS;
	float		iX, iY;

	/* Conversion to Ampere and Volt.
	 * */
	iA = bl->cA1 * (xA - 2048) + bl->cA0;
	iB = bl->cB1 * (xB - 2048) + bl->cB0;
	uS = bl->cU1 * xU + bl->cU0;

	/* Transform from ABC to XY axes.
	 * */
	iX = iA;
	iY = .57735027f * iA + 1.1547005f * iB;

	/* Call FSM.
	 * */
	bFSM(bl, iA, iB, uS, iX, iY);

	/* Flux observer.
	 * */
	if (bl->fMOF & BLC_MODE_FLUX_OBSERVER) {

		fFB(bl, iX, iY);

		if (bl->fMOF & BLC_MODE_FLUX_LOCK) {

			bl->dqX = bl->pX;
			bl->dqY = bl->pY;
		}
	}

	/* Source voltage estimate.
	 * */
	if (bl->fMOF & BLC_MODE_VOLTAGE_ESTIMATE) {

		bl->U += (uS - bl->U) * bl->gainU;
	}

	/* Current control loop.
	 * */
	if (bl->fMOF & BLC_MODE_CURRENT_LOOP) {

		iFB(bl, iX, iY);

		/* Speed control loop.
		 * */
		if (bl->fMOF & BLC_MODE_SPEED_LOOP) {
		}
	}
	else
		bl->pDC(0, 0, 0);

	/* Measured current.
	 * */
	bl->iX = iX;
	bl->iY = iY;
}

