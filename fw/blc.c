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
	bl->sT3 = .5f;		/* Estimate R */
	bl->sT4 = 80e-3f;	/* Estimate L (Ramp) */
	bl->sT5 = 20e-3f;	/* ~ (Hold) */
	bl->sN1 = 40;		/* ~ (Number) */
	bl->sISP = 1.f;		/* Current SetPoint */
	bl->sAq = 1e-3f;	/* Acceleration */

	bl->cA0 = 0.f;
	bl->cA1 = .01464844f;
	bl->cB0 = 0.f;
	bl->cB1 = .01464844f;
	bl->cU0 = 0.f;
	bl->cU1 = .00976563f;

	bl->iKP = 2.e-3f;
	bl->iKI = 2.e-3f;
}

static void
iFB(blc_t *bl, float iX, float iY)
{
	float		eD, eQ, uD, uQ;
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

	uA = xA * bl->U / bl->pwmR + .5f;
	uB = xB * bl->U / bl->pwmR + .5f;

	uX = uA;
	uY = .57735027f * uA + 1.1547005f * uB;

	bl->uX = uX;
	bl->uY = uY;
}

void blcFeedBack(blc_t *bl, int xA, int xB, int xU)
{
	float		iA, iB, uS;
	float		iX, iY, dW, L;

	/* Conversion to Ampere and Volt.
	 * */
	iA = bl->cA1 * (xA - 2048) + bl->cA0;
	iB = bl->cB1 * (xB - 2048) + bl->cA0;
	uS = bl->cU1 * xU + bl->cU0;

	/* Transform from ABC to XY axes.
	 * */
	iX = iA;
	iY = .57735027f * iA + 1.1547005f * iB;

	/* Flux observer.
	 * */
	if (bl->fMOF & BLC_MODE_FLUX_OBSERVER) {

		if (bl->fMOF & BLC_MODE_FLUX_LOCK) {
		}
	}

	/* Source voltage estimate.
	 * */
	if (bl->fMOF & BLC_MODE_VOLTAGE_ESTIMATE) {

		bl->U = (uS - bl->U) * .017f;
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

	/* FSM.
	 * */
	switch (bl->fST1) {

		case BLC_STATE_IDLE:
			break;

		case BLC_STATE_DRIFT:

			if (bl->fST2 == 0) {

				bl->tempA = 0.f;
				bl->tempB = 0.f;
				bl->tempC = 0.f;

				bl->timVal = 0;
				bl->timEnd = 64;

				bl->fST2++;
			}
			else {
				bl->tempA += iA;
				bl->tempB += iB;
				bl->tempC += uS - bl->U;

				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					/* Zero Drift */

					bl->cA0 -= bl->tempA / bl->timEnd;
					bl->cB0 -= bl->tempB / bl->timEnd;
					bl->U += bl->tempC / bl->timEnd;

					if (bl->fST2 == 1) {

						bl->tempA = 0.f;
						bl->tempB = 0.f;
						bl->tempC = 0.f;

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
						bl->fST1 = BLC_STATE_ESTIMATE_R;
					else
						bl->fST1 = BLC_STATE_SPINUP;

					bl->fST2 = 0;
				}
			}
			break;

		case BLC_STATE_ESTIMATE_R:

			if (bl->fST2 == 0) {

				bl->timVal = 0;
				bl->timEnd = 64;

				bl->tempA = 0.f;
				bl->tempB = 0.f;

				bl->fST2++;
			}
			else if (bl->fST2 == 1) {

				bl->tempA += bl->uX;
				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					bl->tempA /= bl->timEnd;

					bl->timVal = 0;
					bl->timEnd = bl->hzF * bl->sT3;

					bl->fST2++;
				}
			}
			else if (bl->fST2 == 2) {

				bl->tempB += bl->uX - bl->tempA;
				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					/* Resistance */

					bl->R = (bl->tempB / bl->timEnd + bl->tempA) / bl->iSPD;

					bl->fST1 = BLC_STATE_ESTIMATE_L;
					bl->fST2 = 0;
				}
			}
			break;

		case BLC_STATE_ESTIMATE_L:

			if (bl->fST2 == 0) {

				bl->timVal = 0;
				bl->timEnd = bl->hzF * bl->sT4;

				bl->tempA = 0.f;
				bl->tempB = 0.f;
				bl->tempC = 0.f;

				bl->fST2++;
			}
			else if (bl->fST2 < bl->sN1) {

				if (bl->fST2 & 1) {

					bl->iSPD = bl->sISP * ((bl->fST2 & 2) ? bl->timVal
							: bl->timEnd - bl->timVal) / bl->timEnd;
				}

				bl->tempA += bl->uX - bl->iSPD * bl->R;
				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					if (bl->fST2 & 1) {

						bl->timVal = 0;
						bl->timEnd = bl->hzF * bl->sT5;
					}
					else {
						bl->timVal = 0;
						bl->timEnd = bl->hzF * bl->sT4;

						bl->tempB += (bl->fST2 & 2) ? -bl->tempA : bl->tempA;
						bl->tempC += 1.f;
						bl->tempA = 0.f;
					}

					bl->fST2++;
				}
			}
			else {
				/* Inductance */

				bl->L = bl->tempB / (bl->tempC * bl->hzF * bl->sISP);

				bl->fST1 = BLC_STATE_END;
				bl->fST2 = 0;
			}
			break;

		case BLC_STATE_SPINUP:

			if (bl->fST2 == 0) {

				bl->timVal = 0;
				bl->timEnd = bl->hzF * bl->sT6;

				bl->tempA = 0.f;

				bl->fST2++;
			}
			else if (bl->fST2 == 1) {

				dW = bl->sAq * bl->dT;
				L = (bl->tempA + .5f * dW) * bl->dT;
				bl->tempA += dW;

				bl->dqX += bl->dqX * L;
				bl->dqY += -bl->dqY * L;

				L = (3.f - bl->dqX * bl->dqX - bl->dqY * bl->dqY) * .5f;

				bl->dqX *= L;
				bl->dqY *= L;

				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
				}
			}
			break;
	}
}

