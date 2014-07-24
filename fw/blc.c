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

inline int
SSAT(int x, int m)
{
	m = (1 << m) - 1;

	return (x < 0) ? x | ~m : x & m;
}

inline int
USAT(int x, int m)
{
	m = (1 << m) - 1;

	return (x < 0) ? 0 : x & m;
}

static void
iFB(struct blc *bl, int iX, int iY)
{
	int		eD, eQ, uD, uQ, uX, uY;
	int		uA, uB, uC;

	eD = (bl->dqX * iX + bl->dqY * iY) >> 14;
	eQ = (bl->dqX * iY - bl->dqY * iX) >> 14;

	eD = bl->iSPD - eD;
	eQ = bl->iSPQ - eQ;

	bl->iXD += (bl->iKI >> 5) * eD;
	uD = (bl->iKP >> 5) * eD >> (bl->iKP & 0x1F);
	bl->iXD = SSAT(bl->iXD, 14 + (bl->iKI & 0x1F));
	uD += bl->iXD >> (bl->iKI & 0x1F);

	bl->iXQ += 4200 * eQ;
	uQ = 5200 * eQ >> 16;
	bl->iXQ = SSAT(bl->iXQ, 30);
	uQ += bl->iXQ >> 16;

	uD = SSAT(uD, 14);
	uQ = SSAT(uQ, 14);

	uX = (bl->dqX * uD - bl->dqY * uQ) >> 14;
	uY = (bl->dqY * uD + bl->dqX * uQ) >> 14;

	uA = uX;
	uB = (-32768 * uX + 56756 * uY) >> 16;
	uC = (-32768 * uX - 56756 * uY) >> 16;

	uA = USAT(uA + 16384, 15);
	uB = USAT(uB + 16384, 15);
	uC = USAT(uC + 16384, 15);

	uA = (bl->pwmR * uA + 16384) >> 15;
	uB = (bl->pwmR * uB + 16384) >> 15;
	uC = (bl->pwmR * uC + 16384) >> 15;

	bl->pDC(uA, uB, uC);

	uC = bl->pwmR >> 1;
	uA = bl->U * (uA - uC) / bl->pwmR;
	uB = bl->U * (uB - uC) / bl->pwmR;

	uX = uA;
	uY = (9459 * uA + 18919 * uB) >> 14;

	bl->uX = uX;
	bl->uY = uY;
}

void blcEnable(struct blc *bl)
{
}

static void
bFSM()
{
	switch (bl->fST1) {

		case BLC_ST1_IDLE:
			break;

		case BLC_ST1_DRIFT:

			if (bl->fST2 == 0) {

				bl->fMode = 0;

				bl->tempA = 0;
				bl->tempB = 0;
				bl->tempU = 0;

				bl->timVal = 0;
				bl->timEnd = 64;

				bl->fST2++;
			}
			else {
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
						if (bl->fsmReq & BLC_REQ_CALIBRATE)
							bl->fsmMode = BLC_MODE_CALIBRATE;
						else
							bl->fsmMode = BLC_MODE_ALIGN;

						bl->fST2 = 0;
					}
				}
			}
			break;
	}

	if (bl->fsmMode == BLC_MODE_RUNING) {


	}
	else if (bl->fsmMode == BLC_MODE_SPINUP) {
	}
	else if (bl->fsmMode == BLC_MODE_BREAK) {
	}
	else if (bl->fsmMode == BLC_MODE_ESTIMATE_R) {
	}
	else if (bl->fsmMode == BLC_MODE_ESTIMATE_L) {
	}
	else if (bl->fsmMode == BLC_MODE_ALIGN) {

		if (bl->fsmStage == 0) {

			bl->dqX = 16384;
			bl->dqY = 0;

			bl->timVal = 0;
			bl->timEnd = bl->hzF * bl->sT1 / 1000;

			bl->fsmStage++;
		}
		else if (bl->fsmStage == 1) {

			/* Ramp Up Stage */

			bl->iSP = 1000 * bl->timVal / bl->timEnd;

			iFB(bl, iX, iY, bl->iSP, 0);

			bl->timVal++;

			if (bl->timVal < bl->timEnd) ;
			else {
				bl->timVal = 0;
				bl->timEnd = bl->hzF * bl->sT2 / 1000;

				bl->fsmStage++;
			}
		}
		else if (bl->fsmStage == 2) {

			/* Hold Stage */

			iFB(bl, iX, iY, bl->iSP, 0);

			bl->timVal++;

			if (bl->timVal < bl->timEnd) ;
			else {
				bl->fsmMode = BLC_MODE_ESTIMATE_R;
				bl->fsmStage = 0;
			}
		}
	}
	else if (bl->fsmMode == BLC_MODE_DRIFT) {

		bl->uDC(0, 0, 0);


	}
	else if (bl->fsmMode == BLC_MODE_END) {

		iFB(bl, iX, iY, 0, 0);

		if (bl->fsmStage == 0) {

			bl->timVal = 0;
			bl->timEnd = bl->hzF * bl->sT0 / 1000;

			bl->fsmStage++;
		}
		else if (bl->fsmStage == 1) {

			bl->timVal++;

			if (bl->timVal < bl->timEnd) ;
			else {

				bl->fsmMode = BLC_MODE_IDLE;
				bl->fsmReq = 0;
			}
		}
	}
	else if (bl->fsmMode == BLC_MODE_IDLE) {

		bl->uDC(0, 0, 0);

		if (bl->fsmReq & (BLC_REQ_CALIBRATE | BLC_REQ_SPINUP)) {

			bl->fsmMode = BLC_MODE_DRIFT;
			bl->fsmStage = 0;
		}
	}
}

void blcFeedBack(struct blc *bl, int iA, int iB, int uS)
{
	int		iX, iY;

	/* Conversion to mA and mV.
	 * */
	iA = (60000 + bl->cA1) * (iA - 2048) >> 12;
	iB = (60000 + bl->cB1) * (iB - 2048) >> 12;
	uS = (40000 + bl->cU1) * uS >> 12;

	/* Zero Drift cancellation.
	 * */
	iA += bl->cA0;
	iB += bl->cB0;
	uS += bl->cU0;

	/* Transform from ABC to XY axes.
	 * */
	iX = iA;
	iY = (9459 * iA + 18919 * iB) >> 14;

	if (bl->fStat & 0) {
	}

	if (bl->fStat & 0) {
	}

	if (bl->fStat & BLC_STAT_ILOOP) {

		iFB(bl, iX, iY);
	}
	else {
		bl->pDC(0, 0, 0);
	}

	if (bl->fStat & BLC_STAT_WLOOP) {
	}

	bFSM(bl, iX, iY, uS);
}

