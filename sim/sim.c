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
	Y = 0.577350269189626 * A + 1.15470053837925 * B;

	rS = sin(R);
	rC = cos(R);

	*D = rC * X + rS * Y;
	*Q = rC * Y - rS * X;
}

static void
simF(double Tend)
{
	const int	szTel = 40;
	float		Tel[szTel], *pTel;
	FILE		*fdTel;

	double		A, B, C, D, Q;
	int		tl, ts = 0;

	fdTel = fopen(TEL_FILE, "wb");

	if (fdTel == NULL) {

		fprintf(stderr, "fopen: %s", strerror(errno));
		exit(errno);
	}

	pTel = Tel - 1;

	while (m.Tsim < Tend) {

		/* Script update.
		 * */
		simScript(m.Tsim);

		/* Plant model update.
		 * */
		blmUpdate(&m);

		/* Base plant telemetry.
		 * */
		simABtoDQ(m.X[0], m.X[1], m.X[3], &D, &Q);
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

		pTel[7] = D;
		pTel[8] = Q;

		/* Estimated variables.
		 * */
		pTel[10] = 0;

		/* Dump telemetry array.
		 * */
		fwrite(Tel, sizeof(float), szTel, fdTel);

		/* BLC update.
		 * */
		blcFeedBack(&bl, m.iA, m.iB, m.uS);

		/* Progress indication.
		 * */
		tl = ts;
		ts = (int) (m.Tsim * 1e+1);

		if (tl < ts) {

			printf("\rTsim = %2.1lf", m.Tsim);
			fflush(stdout);
		}
	}

	fclose(fdTel);
	puts("\n");
}

#include "math.h"
int main(int argc, char *argv[])
{
	double		Tend = 5.;

	blmEnable(&m);

	bl.hzF = (int) (1. / m.dT);
	bl.pwmR = m.PWMR;

	bl.pDC = &blmDC;
	bl.pZ = &blmZ;

	bl.fREQ = BLC_REQUEST_SPINUP;

	blcEnable(&bl);

	libEnable();
	simF(Tend);

	return 0;
}

