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

void blcEnable(blc_t *bl)
{
	bl->iKP = (500 << 5) + 13;
	bl->iKI = (500 << 5) + 13;
}

static void
bFSM(blc_t *bl, int iA, int iB, int iX, int iY, int uS)
{
	switch (bl->fST1) {

		case BLC_STATE_IDLE:

			if (bl->fREQ & (BLC_REQUEST_CALIBRATE | BLC_REQUEST_SPINUP)) {

				bl->fST1 = BLC_STATE_DRIFT;
				bl->fST2 = 0;
			}
			break;

		case BLC_STATE_DRIFT:

			if (bl->fST2 == 0) {

				bl->fMOF = 0;

				bl->tempA = 0;
				bl->tempB = 0;
				bl->tempU = 0;

				bl->timVal = 0;
				bl->timEnd = 64;

				bl->fST2++;
			}
			else {
				/* Zero Drift */

				bl->tempA += iA;
				bl->tempB += iB;
				bl->tempU += uS - bl->U;

				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					bl->cA0 -= bl->tempA / bl->timEnd;
					bl->cB0 -= bl->tempB / bl->timEnd;
					bl->U += bl->tempU / bl->timEnd;

					if (bl->fST2 == 1) {

						bl->tempA = 0;
						bl->tempB = 0;
						bl->tempU = 0;

						bl->timVal = 0;
						bl->timEnd = bl->hzF * bl->sT0 / 1000;

						bl->fST2++;
					}
					else {
						if (bl->fREQ & BLC_REQUEST_CALIBRATE)
							bl->fST1 = BLC_STATE_CALIBRATE;
						else
							bl->fST1 = BLC_STATE_ALIGN;

						bl->fST2 = 0;
					}
				}
			}
			break;

		case BLC_STATE_ALIGN:

			if (bl->fST2 == 0) {

				bl->fMOF = BLC_MODE_CURRENT_LOOP;

				bl->iSPD = 0;
				bl->iSPQ = 0;

				bl->dqX = 16384;
				bl->dqY = 0;

				bl->timVal = 0;
				bl->timEnd = bl->hzF * bl->sT1 / 1000;

				bl->fST2++;
			}
			else if (bl->fST2 == 1) {

				/* Ramp Up Stage */

				bl->iSPD = 1000 * bl->timVal / bl->timEnd;
				bl->iSPQ = 0;

				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					bl->timVal = 0;
					bl->timEnd = bl->hzF * bl->sT2 / 1000;

					bl->fST2++;
				}
			}
			else if (bl->fST2 == 2) {

				/* Hold Stage */

				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					bl->fST1 = BLC_STATE_ESTIMATE_R;
					bl->fST2 = 0;
				}
			}
			break;

		case BLC_STATE_ESTIMATE_R:
			break;
	}
}

static void
iFB(blc_t *bl, int iX, int iY)
{
	int		eD, eQ, uD, uQ, R;
	int		uA, uB, uC, uX, uY;

	eD = (bl->dqX * iX + bl->dqY * iY) >> 14;
	eQ = (bl->dqX * iY - bl->dqY * iX) >> 14;

	eD = bl->iSPD - eD;
	eQ = bl->iSPQ - eQ;

	bl->iXD += (bl->iKI >> 5) * eD;
	uD = (bl->iKP >> 5) * eD >> (bl->iKP & 0x1F);
	bl->iXD = ssat(bl->iXD, 14 + (bl->iKI & 0x1F));
	uD += bl->iXD >> (bl->iKI & 0x1F);

	bl->iXQ += (bl->iKI >> 5) * eQ;
	uQ = (bl->iKP >> 5) * eQ >> (bl->iKP & 0x1F);
	bl->iXQ = ssat(bl->iXQ, 14 + (bl->iKI & 0x1F));
	uQ += bl->iXQ >> (bl->iKI & 0x1F);

	uD = __SSAT(uD, 14);
	uQ = __SSAT(uQ, 14);

	uX = (bl->dqX * uD - bl->dqY * uQ) >> 14;
	uY = (bl->dqY * uD + bl->dqX * uQ) >> 14;

	uA = uX;
	uB = (-32768 * uX + 56756 * uY) >> 16;
	uC = (-32768 * uX - 56756 * uY) >> 16;

	uA = __USAT(uA + 16384, 15);
	uB = __USAT(uB + 16384, 15);
	uC = __USAT(uC + 16384, 15);

	R = bl->pwmR;

	uA = (R * uA + 16384) >> 15;
	uB = (R * uB + 16384) >> 15;
	uC = (R * uC + 16384) >> 15;

	bl->pDC(uA, uB, uC);

	uC = R >> 1;
	uA = bl->U * (uA - uC) / R;
	uB = bl->U * (uB - uC) / R;

	uX = uA;
	uY = (9459 * uA + 18919 * uB) >> 14;

	bl->uX = uX;
	bl->uY = uY;
}

void blcFeedBack(blc_t *bl, int iA, int iB, int uS)
{
	int		iX, iY;

	/* Conversion to mA and mV.
	 * */
	iA = (60000 + bl->cA1) * (iA - 2048) >> 12;
	iB = (60000 + bl->cB1) * (iB - 2048) >> 12;
	uS = (30000 + bl->cU1) * uS >> 12;

	/* Zero Drift cancellation.
	 * */
	iA += bl->cA0;
	iB += bl->cB0;
	uS += bl->cU0;

	/* Transform from ABC to XY axes.
	 * */
	iX = iA;
	iY = (9459 * iA + 18919 * iB) >> 14;

	if (bl->fMOF & BLC_MODE_FLUX_OBSERVER) {

		if (bl->fMOF & BLC_MODE_FLUX_LOCK) {
		}
	}

	if (bl->fMOF & BLC_MODE_CURRENT_LOOP) {

		iFB(bl, iX, iY);
	}
	else {
		bl->pDC(0, 0, 0);
	}

	if (bl->fMOF & BLC_MODE_VOLTAGE_ESTIMATE) {
	}

	if (bl->fMOF & BLC_MODE_SPEED_LOOP) {
	}

	bFSM(bl, iA, iB, iX, iY, uS);
}

