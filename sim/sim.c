/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

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
sim_AB_DQ(double A, double B, double R, double *D, double *Q)
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
sim_Tel(float *pTel)
{
	double		A, B, C, D, Q;

	if (m.mDQ) {

		D = m.X[0];
		Q = m.X[1];
	}
	else
		sim_AB_DQ(m.X[0], m.X[1], m.X[3], &D, &Q);

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
	pTel[10] = pm.lu_X[0];
	pTel[11] = pm.lu_X[1];

	D = cos(m.X[3]);
	Q = sin(m.X[3]);
	A = D * pm.lu_X[2] + Q * pm.lu_X[3];
	B = D * pm.lu_X[3] - Q * pm.lu_X[2];
	C = atan2(B, A);

	/* Estimated position.
	 * */
	pTel[12] = atan2(pm.lu_X[3], pm.lu_X[2]);
	pTel[13] = C;

	/* Estimated speed.
	 * */
	pTel[14] = pm.lu_X[4];

	/* VSI voltage.
	 * */
	pTel[15] = pm.vsi_X;
	pTel[16] = pm.vsi_Y;

	/* Measurement residual.
	 * */
	pTel[17] = pm.lu_residual_D;
	pTel[18] = pm.lu_residual_Q;

	/* Zero Drift.
	 * */
	pTel[19] = pm.drift_A;
	pTel[20] = pm.drift_B;
	pTel[21] = pm.drift_Q;

	/* Plant constants.
	 * */
	pTel[22] = pm.const_U;
	pTel[23] = pm.const_E;
	pTel[24] = pm.const_R;
	pTel[25] = pm.const_Ld;
	pTel[26] = pm.const_Lq;
	pTel[27] = pm.const_Zp;
	pTel[28] = pm.n_power_watt;
	pTel[29] = 0.f;

	/* Set Point.
	 * */
	pTel[30] = pm.i_set_point_D;
	pTel[31] = pm.i_set_point_Q;
	pTel[32] = 0.f;

	/* Variance.
	 * */
	pTel[33] = pm.wave_temp[0];
}

static void
sim_F(FILE *fdTel, double dT, int Verb)
{
	const int	szTel = 40;
	float		Tel[szTel], *pTel;
	double		Tin, Tend;

	pTel = Tel - 1;
	Tin = m.Tsim;
	Tend = Tin + dT;

	while (m.Tsim < Tend) {

		/* Plant model update.
		 * */
		blm_Update(&m);

		/* PMC update.
		 * */
		pmc_feedback(&pm, m.xA, m.xB);
		pmc_voltage(&pm, m.xU);

		/* Collect telemetry.
		 * */
		sim_Tel(pTel);

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
}

static void
sim_Script(FILE *fdTel)
{
	float		IMP[6];

	pm.freq_hz = (float) (1. / m.dT);
	pm.dT = 1.f / pm.freq_hz;
	pm.pwm_resolution = m.PWMR;

	pm.pDC = &blmDC;
	pm.pZ = &blmZ;

	pmc_default(&pm);

	pm.const_U = m.U;
	pm.const_R = m.R * (1. + .0);
	pm.const_Ld = m.Ld * (1. - .0);
	pm.const_Lq = m.Lq * (1. + .0);
	pm.const_E = m.E * (1. - .0);

	pm.const_Ld_inversed = 1.f / pm.const_Ld;
	pm.const_Lq_inversed = 1.f / pm.const_Lq;

	pm.const_Zp = m.Zp;

	pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
	sim_F(fdTel, .5, 0);

	pmc_request(&pm, PMC_STATE_WAVE_HOLD);
	sim_F(fdTel, 1., 0);

	pm.const_R += pm.wave_temp[2];

	printf("R\t%.4e\t(%.2f%%)\n", pm.const_R, 100. * (pm.const_R - m.R) / m.R);

	pmc_request(&pm, PMC_STATE_WAVE_SINE);
	sim_F(fdTel, .7, 0);

	pmc_impedance(pm.wave_DFT, pm.wave_freq_sine_hz, IMP);

	printf("IMP\t%.4e %.4e %.4e %.4e %.4e %.4e\n",
		IMP[0], IMP[1], IMP[2], IMP[3], IMP[4], IMP[5]);

	printf("Ld\t%.4e\t(%.2f%%)\n", pm.const_Ld, 100. * (pm.const_Ld - m.Ld) / m.Ld);
	printf("Lq\t%.4e\t(%.2f%%)\n", pm.const_Lq, 100. * (pm.const_Lq - m.Lq) / m.Lq);

	pmc_request(&pm, PMC_STATE_START);
	sim_F(fdTel, .5, 0);

	m.X[3] += .5;
	sim_F(fdTel, .5, 0);

	pm.p_set_point_w = 0.f;
	sim_F(fdTel, 1., 0);

	pm.p_set_point_w = 8000.f;
	sim_F(fdTel, 1., 0);

	m.M[2] = 1e-5;
	sim_F(fdTel, 1., 0);
}

int main(int argc, char *argv[])
{
	FILE		*fdTel;

	libStart();
	blm_Enable(&m);

	fdTel = fopen(TEL_FILE, "wb");

	if (fdTel == NULL) {

		fprintf(stderr, "fopen: %s", strerror(errno));
		exit(errno);
	}

	sim_Script(fdTel);

	fclose(fdTel);
	libStop();

	return 0;
}

