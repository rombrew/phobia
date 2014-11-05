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

#include "pmc.h"

inline float
clamp(float x, float h, float l)
{
	return (x > h) ? h : (x < l) ? l : x;
}

void pmcEnable(pmc_t *pm)
{
	pm->dT = 1.f / pm->hzF;

	/* Default config.
	 * */
	pm->sT0 = .1f;		/* Zero Drift */
	pm->sT1 = .4f;		/* Ramp Up */
	pm->sT2 = .7f;		/* Hold */
	pm->sT3 = .5f;		/* Estimate RL */
	pm->sT4 = .5f;		/* Spin Up */
	pm->sT5 = .0f;		/*  */
	pm->sISP = 1.f;		/* Current SetPoint */
	pm->sFq = 20.f;		/* Frequency (Hz)*/
	pm->sAq = 75.f;		/* Acceleration */

	pm->cA0 = 0.f;
	pm->cA1 = .01464844f;
	pm->cB0 = 0.f;
	pm->cB1 = .01464844f;
	pm->cU0 = 0.f;
	pm->cU1 = .00725098f;

	pm->iKP = 2e-2f;
	pm->iKI = 5e-3f;
}

static void
dEq(pmc_t *pm, float D[], float X[])
{
	float		uD, uQ;

	uD = X[2] * pm->uX + X[3] * pm->uY;
	uQ = X[2] * pm->uY - X[3] * pm->uX;

	D[0] = pm->IL * (uD - X[0] * pm->R) + X[4] * X[1];
	D[1] = pm->IL * (uQ - X[1] * pm->R - X[4] * pm->E) - X[4] * X[0];
	D[2] = -X[3] * X[4];
	D[3] = X[2] * X[4];
	D[4] = pm->Zp * pm->IJ * (1.5f * pm->E * pm->Zp * X[1] - X[5]);
}

static void
sFC(pmc_t *pm)
{
	float		*X = pm->X;
	float		D1[5], D2[5], X2[6];
	float		dT, L;

	dEq(pm, D1, pm->X);
	dT = pm->dT;

	X2[0] = X[0] + D1[0] * dT;
	X2[1] = X[1] + D1[1] * dT;
	X2[2] = X[2] + D1[2] * dT;
	X2[3] = X[3] + D1[3] * dT;
	X2[4] = X[4] + D1[4] * dT;
	X2[5] = X[5];

	L = (3.f - X2[2] * X2[2] - X2[3] * X2[3]) * .5f;
	X2[2] *= L;
	X2[3] *= L;

	dEq(pm, D2, X2);
	dT *= .5f;

	X[0] += (D1[0] + D2[0]) * dT;
	X[1] += (D1[1] + D2[1]) * dT;
	X[2] += (D1[2] + D2[2]) * dT;
	X[3] += (D1[3] + D2[3]) * dT;
	X[4] += (D1[4] + D2[4]) * dT;

	L = (3.f - X[2] * X[2] - X[3] * X[3]) * .5f;
	X[2] *= L;
	X[3] *= L;
}

static void
sFB(pmc_t *pm, float iX, float iY)
{
	float		*X = pm->X;
	float		iD, iQ, eD, eQ;
	float		L, R;

	iD = X[2] * iX + X[3] * iY;
	iQ = X[2] * iY - X[3] * iX;

	eD = iD - X[0];
	eQ = iQ - X[1];

	X[0] += .2f * eD;
	X[1] += .2f * eQ;
	R = X[1] * eD - X[0] * eQ;
	R = clamp(R * .01f, .5f, -.5f);
	X[2] += -X[3] * R;
	X[3] += X[2] * R;
	X[4] += .1f * R;
	X[5] += -.002f * R;

	L = (3.f - X[2] * X[2] - X[3] * X[3]) * .5f;
	X[2] *= L;
	X[3] *= L;

	sFC(pm);
}

static void
iFB(pmc_t *pm, float iX, float iY)
{
	float		iD, iQ, eD, eQ, uD, uQ, Q;
	float		uA, uB, uC, uX, uY;
	int		xA, xB, xC;

	iD = pm->pX * iX + pm->pY * iY;
	iQ = pm->pX * iY - pm->pY * iX;

	eD = pm->iSPD - iD;
	eQ = pm->iSPQ - iQ;

	pm->iXD = clamp(pm->iXD + pm->iKI * eD, 1.f, -1.f);
	uD = pm->iXD + pm->iKP * eD;

	pm->iXQ = clamp(pm->iXQ + pm->iKI * eQ, 1.f, -1.f);
	uQ = pm->iXQ + pm->iKP * eQ;

	uX = pm->pX * uD - pm->pY * uQ;
	uY = pm->pY * uD + pm->pX * uQ;

	uA = uX;
	uB = -.5f * uX + .8660254f * uY;
	uC = -.5f * uX - .8660254f * uY;

	uA = clamp(.5f * uA + .5f, 1.f, 0.f);
	uB = clamp(.5f * uB + .5f, 1.f, 0.f);
	uC = clamp(.5f * uC + .5f, 1.f, 0.f);

	xA = (int) (pm->pwmR * uA + .5f);
	xB = (int) (pm->pwmR * uB + .5f);
	xC = (int) (pm->pwmR * uC + .5f);

	pm->pDC(xA, xB, xC);

	Q = .33333333f * (xA + xB + xC);
	uA = (xA - Q) * pm->U / pm->pwmR;
	uB = (xB - Q) * pm->U / pm->pwmR;

	uX = uA;
	uY = .57735027f * uA + 1.1547005f * uB;

	pm->uX = uX;
	pm->uY = uY;
}

static void
wFB(pmc_t *pm)
{
}

static void
bFSM(pmc_t *pm, float iA, float iB, float uS, float iX, float iY)
{
	float		L, P;

	switch (pm->fST1) {

		case PMC_STATE_IDLE:
			break;

		case PMC_STATE_DRIFT:

			if (pm->fST2 == 0) {

				pm->tA.re = 0.f;
				pm->tB.re = 0.f;
				pm->tC.re = 0.f;

				pm->timVal = 0;
				pm->timEnd = 64;

				pm->fST2++;
			}
			else {
				pm->tA.re += -iA;
				pm->tB.re += -iB;
				pm->tC.re += uS - pm->U;

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					/* Zero Drift */

					pm->cA0 += pm->tA.re / pm->timEnd;
					pm->cB0 += pm->tB.re / pm->timEnd;
					pm->U += pm->tC.re / pm->timEnd;

					if (pm->fST2 == 1) {

						pm->tA.re = 0.f;
						pm->tB.re = 0.f;
						pm->tC.re = 0.f;

						pm->timVal = 0;
						pm->timEnd = pm->hzF * pm->sT0;

						pm->fST2++;
					}
					else {
						pm->fMOF |= PMC_MODE_VOLTAGE_ESTIMATE;

						if (pm->fMOF & PMC_MODE_CALIBRATE)
							pm->fST1 = PMC_STATE_CALIBRATE;
						else
							pm->fST1 = PMC_STATE_ALIGN;

						pm->fST2 = 0;
					}
				}
			}
			break;

		case PMC_STATE_CALIBRATE:
			break;

		case PMC_STATE_ALIGN:

			if (pm->fST2 == 0) {

				pm->fMOF |= PMC_MODE_CURRENT_LOOP;

				pm->iSPD = 0.f;
				pm->iSPQ = 0.f;

				pm->iXD = 0.f;
				pm->iXQ = 0.f;

				pm->pX = 1.f;
				pm->pY = 0.f;

				pm->timVal = 0;
				pm->timEnd = pm->hzF * pm->sT1;

				pm->fST2++;
			}
			else if (pm->fST2 == 1) {

				/* Ramp Up */

				pm->iSPD = pm->sISP * pm->timVal / pm->timEnd;
				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					pm->timVal = 0;
					pm->timEnd = pm->hzF * pm->sT2;

					pm->fST2++;
				}
			}
			else if (pm->fST2 == 2) {

				/* Hold */

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					if (pm->fMOF & PMC_MODE_ESTIMATE_RL)
						pm->fST1 = PMC_STATE_ESTIMATE_RL;
					else
						pm->fST1 = PMC_STATE_SPINUP;

					pm->fST2 = 0;
				}
			}
			break;

		case PMC_STATE_ESTIMATE_RL:

			if (pm->fST2 == 0) {

				pm->tA.re = 1.f;
				pm->tA.im = 0.f;
				pm->tB.re = 0.f;
				pm->tB.im = 0.f;
				pm->tC.re = 0.f;
				pm->tC.im = 0.f;
				pm->tD.re = iX;
				pm->tD.im = 6.2831853f * pm->sFq * pm->dT;

				pm->timVal = 0;
				pm->timEnd = pm->hzF * pm->sT3;

				pm->fST2++;
			}
			else if (pm->fST2 == 1) {

				pm->tA.re += -pm->tA.im * pm->tD.im;
				pm->tA.im += pm->tA.re * pm->tD.im;

				L = (3.f - pm->tA.re * pm->tA.re - pm->tA.im * pm->tA.im) * .5f;
				pm->tA.re *= L;
				pm->tA.im *= L;

				L = .5f * (pm->tD.re + iX);
				pm->tB.re += L * pm->tA.re;
				pm->tB.im += L * pm->tA.im;

				L = pm->uX;
				pm->tC.re += L * pm->tA.re;
				pm->tC.im += L * pm->tA.im;

				pm->iSPD = pm->sISP * (pm->tA.re + 1.f) * .5f;
				pm->tD.re = iX;

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					pm->tA.re = pm->tC.re * pm->tB.re + pm->tC.im * pm->tB.im;
					pm->tA.im = pm->tC.re * pm->tB.im - pm->tC.im * pm->tB.re;

					L = pm->tB.re * pm->tB.re + pm->tB.im * pm->tB.im;
					pm->tA.re /= L;
					pm->tA.im /= L;

					pm->R = pm->tA.re;
					pm->IL = 6.2831853f * pm->sFq / pm->tA.im;

					pm->fST1 = PMC_STATE_END;
					pm->fST2 = 0;
				}
			}
			break;

		case PMC_STATE_SPINUP:

			if (pm->fST2 == 0) {

				pm->tA.re = 0.f;

				pm->timVal = 0;
				pm->timEnd = pm->hzF * pm->sT4;

				pm->fST2++;
			}
			else if (pm->fST2 == 1) {

				P = pm->sAq * pm->dT;
				L = (pm->tA.re + .5f * P) * pm->dT;
				pm->tA.re += P;

				pm->pX += -pm->pY * L;
				pm->pY += pm->pX * L;

				L = (3.f - pm->pX * pm->pX - pm->pY * pm->pY) * .5f;
				pm->pX *= L;
				pm->pY *= L;

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					pm->fMOF |= PMC_MODE_OBSERVER;

					pm->iSPD = 0.f;
					pm->iSPQ = pm->sISP;

					pm->X[0] = iX;
					pm->X[1] = iY;
					pm->X[2] = pm->pX;
					pm->X[3] = pm->pY;
					pm->X[4] = pm->tA.re;
					pm->X[5] = 0.f;

					pm->fST1 = PMC_STATE_IDLE;
					pm->fST2 = 0;
				}
			}
			break;

		case PMC_STATE_END:
			break;
	}
}

void pmcFeedBack(pmc_t *pm, int xA, int xB, int xU)
{
	float		iA, iB, uS;
	float		iX, iY;

	/* Conversion to Ampere and Volt.
	 * */
	iA = pm->cA1 * (xA - 2048) + pm->cA0;
	iB = pm->cB1 * (xB - 2048) + pm->cB0;
	uS = pm->cU1 * xU + pm->cU0;

	/* Transform from ABC to XY axes.
	 * */
	iX = iA;
	iY = .57735027f * iA + 1.1547005f * iB;

	/* Call FSM.
	 * */
	bFSM(pm, iA, iB, uS, iX, iY);

	/* State observer.
	 * */
	if (pm->fMOF & PMC_MODE_OBSERVER) {

		sFB(pm, iX, iY);

		pm->pX = pm->X[2];
		pm->pY = pm->X[3];
	}

	/* Source voltage estimate.
	 * */
	if (pm->fMOF & PMC_MODE_VOLTAGE_ESTIMATE) {

		pm->U += (uS - pm->U) * pm->gainU;
	}

	/* Current control loop.
	 * */
	if (pm->fMOF & PMC_MODE_CURRENT_LOOP) {

		iFB(pm, iX, iY);

		/* Speed control loop.
		 * */
		if (pm->fMOF & PMC_MODE_SPEED_LOOP) {
		}
	}
	else
		pm->pDC(0, 0, 0);
}

