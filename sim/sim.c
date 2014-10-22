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
#include "blc.h"
#include "lib.h"

#define TEL_FILE	"/tmp/TEL"

static blm_t	m;
static blc_t	bl;

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

	A = m.uA / (double) m.PWMR;
	B = m.uB / (double) m.PWMR;
	C = m.uC / (double) m.PWMR;
	Q = (A + B + C) / 3.;

	simABtoDQ(A - Q, B - Q, m.X[3], &D, &Q);

	/* Duty cycle.
	 * */
	pTel[7] = D;
	pTel[8] = Q;

	/* Estimated variables.
	 * */
	pTel[10] = 0;
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

		/* Collect telemetry.
		 * */
		simTel(pTel);

		/* Dump telemetry array.
		 * */
		fwrite(Tel, sizeof(float), szTel, fdTel);

		/* BLC update.
		 * */
		blcFeedBack(&bl, m.xA, m.xB, m.xU);

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

static void
simReport(FILE *fout)
{
	fprintf(fout, "\n-----------------------------------------\n");
	fprintf(fout, "U = %.3E \tVolt \t%.2f%%\n", bl.U,
			100.f * (bl.U - m.U) / m.U);
	fprintf(fout, "R = %.3E \tOhm \t%.2f%%\n", bl.R,
			100.f * (bl.R - m.R) / m.R);
	fprintf(fout, "L = %.3E \tHenry \t%.2f%%\n", bl.L,
			100.f * (bl.L - m.L) / m.L);
	fprintf(fout, "\n-----------------------------------------\n");
}

int main(int argc, char *argv[])
{
	double		Tend = 5.;

	libStart();

	blmEnable(&m);

	bl.hzF = (float) (1. / m.dT);
	bl.pwmR = m.PWMR;

	bl.pDC = &blmDC;
	bl.pZ = &blmZ;

	blcEnable(&bl);

	bl.fMOF = BLC_MODE_ESTIMATE_RL;
	bl.fST1 = BLC_STATE_DRIFT;

	simF(Tend, 0);

	simReport(stdout);

	libStop();

	return 0;
}

