/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2016 Roman Belov <romblv@gmail.com>

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
sim_Tel(float *pTel)
{
	double		A, B, C, D, Q;

	/* Model.
	 * */
	pTel[1] = m.Tsim;
	pTel[2] = m.X[0];
	pTel[3] = m.X[1];
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

	/* Other.
	 * */
	pTel[22] = pm.hf_flux_polarity;
	pTel[23] = pm.thermal_R;
	pTel[24] = pm.thermal_E;
	pTel[25] = pm.p_alt_X4;
	pTel[26] = 0.f;
	pTel[27] = pm.const_Zp;
	pTel[28] = pm.n_power_watt;
	pTel[29] = pm.n_temperature_c;
	pTel[30] = 0.f;
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
	pm.const_Lq = m.Lq * (1. - .0);
	pm.const_E = m.E * (1. - .0);

	pm.const_Ld_inversed = 1.f / pm.const_Ld;
	pm.const_Lq_inversed = 1.f / pm.const_Lq;

	pm.const_Zp = m.Zp;

	pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
	sim_F(fdTel, .5, 0);

	/*pmc_request(&pm, PMC_STATE_WAVE_HOLD);
	sim_F(fdTel, 2., 0);

	pm.const_R += pm.wave_temp[2];

	printf("R\t%.4e\t(%.2f%%)\n", pm.const_R, 100. * (pm.const_R - m.R) / m.R);

	pmc_request(&pm, PMC_STATE_WAVE_SINE);
	sim_F(fdTel, 2., 0);

	pmc_impedance(pm.wave_DFT, pm.wave_freq_sine_hz, IMP);

	printf("IMP\t%.4e %.4e %.1f %.4e %.4e %.1f\n",
		IMP[0], IMP[1], IMP[2], IMP[3], IMP[4], IMP[5]);*/

	//pm.m_bitmask |= PMC_BIT_SERVO_CONTROL_LOOP;
	//pm.m_bitmask |= PMC_BIT_HIGH_FREQUENCY_INJECTION;

	pmc_request(&pm, PMC_STATE_START);
	sim_F(fdTel, .1, 0);

	sim_F(fdTel, 1., 0);

	m.X[3] += .5;
	sim_F(fdTel, 1., 0);

	//pm.p_set_point_w = 2000.f * (2.f * M_PI / 60.f * m.Zp);
	pm.i_set_point_Q = 4.f;
	sim_F(fdTel, 1., 0);

	//pm.p_set_point_w = -60.f * (2.f * M_PI / 60.f * m.Zp);
	pm.i_set_point_Q = 40.f;
	sim_F(fdTel, 3., 0);
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

