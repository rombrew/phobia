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
	bl->sT0 = 100;		/* Zero Drift */
	bl->sT1 = 400;		/* Ramp Up */
	bl->sT2 = 700;
	bl->sT3 = 500;
	bl->sT4 = 80;
	bl->sT5 = 20;

	bl->sISP = 1000;
	bl->sLN = 40;

	bl->cA1 = 0;
	bl->cB1 = 0;
	bl->cA0 = 0;
	bl->cB0 = 0;
	bl->cU1 = 0;
	bl->cU0 = 0;

	bl->iKP = (500 << 5) + 13;
	bl->iKI = (500 << 5) + 13;

	
}

static void
bFSM(blc_t *bl, int iA, int iB, int uS)
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
				bl->tempC = 0;

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

						bl->tempA = 0;
						bl->tempB = 0;
						bl->tempC = 0;

						bl->timVal = 0;
						bl->timEnd = bl->hzF * bl->sT0 / 1000;

						bl->fST2++;
					}
					else {
						bl->fMOF |= BLC_MODE_VOLTAGE_ESTIMATE;

						printf("\n\tZab = %i %i\n", bl->cA0, bl->cB0);

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

				bl->fMOF |= BLC_MODE_CURRENT_LOOP;

				bl->iSPD = 0;
				bl->iSPQ = 0;

				bl->dqX = 1UL << 14;
				bl->dqY = 0;

				bl->timVal = 0;
				bl->timEnd = bl->hzF * bl->sT1 / 1000;

				bl->fST2++;
			}
			else if (bl->fST2 == 1) {

				/* Ramp Up Stage */

				bl->iSPD = bl->sISP * bl->timVal / bl->timEnd;
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

			if (bl->fST2 == 0) {

				bl->timVal = 0;
				bl->timEnd = 64;

				bl->tempA = 0;
				bl->tempB = 0;

				bl->fST2++;
			}
			else if (bl->fST2 == 1) {

				bl->tempA += bl->uX;
				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					bl->tempA /= bl->timEnd;

					bl->timVal = 0;
					bl->timEnd = bl->hzF * bl->sT3 / 1000;

					bl->fST2++;
				}
			}
			else if (bl->fST2 == 2) {

				bl->tempB += bl->uX - bl->tempA;
				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					/* Resistance */

					bl->R = xsdivi((bl->tempB << 6) / bl->timEnd
							+ (bl->tempA << 6), bl->iSPD << 6);

					printf("\n\tR = %lf mOhm\n", 1e+3 * (double) (bl->R >> 5)
						/ (double) (1UL << (bl->R & 0x1F)));

					bl->fST1 = BLC_STATE_ESTIMATE_L;
					bl->fST2 = 0;
				}
			}
			break;

		case BLC_STATE_ESTIMATE_L:

			if (bl->fST2 == 0) {

				bl->timVal = 0;
				bl->timEnd = bl->hzF * bl->sT4 / 1000;

				bl->tempA = 0;
				bl->tempB = 0;
				bl->tempC = 0;

				bl->fST2++;
			}
			else if (bl->fST2 < 40) {

				if (bl->fST2 & 1) {

					bl->iSPD = (bl->fST2 & 2) ? bl->sISP * bl->timVal
						/ bl->timEnd : bl->sISP
						* (bl->timEnd - bl->timVal) / bl->timEnd;
				}

				bl->tempA += bl->uX - (bl->iSPD * (bl->R >> 5) >> (bl->R & 0x1F));
				bl->timVal++;

				if (bl->timVal < bl->timEnd) ;
				else {
					if (bl->fST2 & 1) {

						bl->timVal = 0;
						bl->timEnd = bl->hzF * bl->sT5 / 1000;
					}
					else {
						bl->timVal = 0;
						bl->timEnd = bl->hzF * bl->sT4 / 1000;

						bl->tempB += (bl->fST2 & 2) ? -bl->tempA : bl->tempA;
						bl->tempC += 1;
						bl->tempA = 0;
					}

					bl->fST2++;
				}
			}
			else {
				/* Inductance */

				bl->L = xsdivi(bl->tempB, bl->tempC * bl->hzF * bl->sISP);

				printf("\n\tL = %lf uH\n", 1e+6 * (double) (bl->L >> 5)
						/ (double) (1UL << (bl->L & 0x1F)));

				bl->fST1 = BLC_STATE_SPINUP;
				bl->fST2 = 0;
			}
			break;

		case BLC_STATE_SPINUP:

			if (bl->fST2 == 0) {

				bl->timVal = 0;
				bl->timEnd = 64;

				bl->tempA = 0;
				bl->tempB = 0;

				bl->fST2++;
			}
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

	uA = __USAT(uA + (1UL << 14), 15);
	uB = __USAT(uB + (1UL << 14), 15);
	uC = __USAT(uC + (1UL << 14), 15);

	R = bl->pwmR;

	uA = (R * uA + (1UL << 14)) >> 15;
	uB = (R * uB + (1UL << 14)) >> 15;
	uC = (R * uC + (1UL << 14)) >> 15;

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
	else
		bl->pDC(0, 0, 0);

	if (bl->fMOF & BLC_MODE_VOLTAGE_ESTIMATE) {
	}

	if (bl->fMOF & BLC_MODE_SPEED_LOOP) {
	}

	bFSM(bl, iA, iB, uS);
}

