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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include "blm.h"
#include "pmc.h"
#include "lib.h"

#define TEL_FILE	"/tmp/TEL"

static blm_t		m;
static pmc_t		pm;

static void
blmDC(int uA, int uB, int uC)
{
	m.uA = uA;
	m.uB = uB;
	m.uC = uC;
}

static void
blmZ(int Z) { }

static void
simScript(double T)
{
	if (T > 0.) {

		pm.wSP = 2000.f;
	}

	if (T > 1.) {

		pm.wSP = 9000.f;
	}

	if (T > 4.) {

		pm.wSP = 3000.f;
	}
}

static void
simABtoDQ(double A, double B, double R, double *D, double *Q)
{
	double		X, Y, rS, rC;

	X = A;
	Y = .577350269189626 * A + 1.15470053837925 * B;

	rS = sin(R);
	rC = cos(R);

	*D = rC * X + rS * Y;
	*Q = rC * Y - rS * X;
}

static void
simTel(float *pTel)
{
	double		A, B, C, D, Q;

	simABtoDQ(m.X[0], m.X[1], m.X[3], &D, &Q);

	/* Model.
	 * */
	pTel[1] = m.Tsim;
	pTel[2] = D;
	pTel[3] = Q;
	pTel[4] = m.X[2];
	pTel[5] = m.X[3];
	pTel[6] = m.X[4];

	/* Duty cycle.
	 * */
	pTel[7] = m.uA / (double) m.PWMR;
	pTel[8] = m.uB / (double) m.PWMR;
	pTel[9] = m.uC / (double) m.PWMR;

	/* Estimated current.
	 * */
	pTel[10] = pm.kX[0];
	pTel[11] = pm.kX[1];

	D = cos(m.X[3]);
	Q = sin(m.X[3]);
	A = D * pm.kX[2] + Q * pm.kX[3];
	B = D * pm.kX[3] - Q * pm.kX[2];
	C = atan2(B, A);

	/* Estimated position.
	 * */
	pTel[12] = atan2(pm.kX[3], pm.kX[2]);
	pTel[13] = C;

	/* Speed.
	 * */
	pTel[14] = pm.kX[4];

	/* Plant constants.
	 * */
	pTel[15] = pm.Qd;
	pTel[16] = pm.R;
	pTel[17] = pm.Ld;
	pTel[18] = pm.Lq;
	pTel[19] = pm.E;
	pTel[20] = pm.Zp;
	pTel[21] = pm.M;
	pTel[22] = pm.eD;
	pTel[23] = pm.eQ;
}

static void
simF(double Tend, int Verb)
{
	const int	szTel = 40;
	float		Tel[szTel], *pTel;
	double		Tin;
	FILE		*fdTel;

	fdTel = fopen(TEL_FILE, "wb");

	if (fdTel == NULL) {

		fprintf(stderr, "fopen: %s", strerror(errno));
		exit(errno);
	}

	pTel = Tel - 1;
	Tin = m.Tsim;

	while (m.Tsim < Tend) {

		/* Script update.
		 * */
		simScript(m.Tsim);

		/* Plant model update.
		 * */
		blmUpdate(&m);

		/* PMC update.
		 * */
		pmcFeedBack(&pm, m.xA, m.xB, m.xU);

		/* Collect telemetry.
		 * */
		simTel(pTel);

		/* Dump telemetry array.
		 * */
		fwrite(Tel, sizeof(float), szTel, fdTel);

		/* Progress indication.
		 * */
		if (Verb && Tin < m.Tsim) {

			Tin += .1;

			printf("\rTsim = %2.1lf", m.Tsim);
			fflush(stdout);
		}
	}

	fclose(fdTel);
}

int main(int argc, char *argv[])
{
	double		Tend = 5.;

	libStart();

	blmEnable(&m);

	pm.hzF = (float) (1. / m.dT);
	pm.pwmR = m.PWMR;

	pm.pDC = &blmDC;
	pm.pZ = &blmZ;

	pm.R = 0.f;
	pm.Ld = m.L * (1. + .1);
	pm.Lq = m.L * (1. + .1);
	pm.E = m.E * (1. + .1);

	pm.Zp = 11;
	pm.M = 0.f;
	pm.IJ = 1.f / m.J * 10.f;

	pmcEnable(&pm);

	pm.mReq = PMC_REQ_SINE;
	pm.sineF = 100.f;
	pm.iSPD = 1.f;
	pm.iSPQ = 0.f;
	pm.hSP = .5f;

	simF(Tend, 0);

	libStop();

	return 0;
}

