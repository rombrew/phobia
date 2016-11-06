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
	pTel[0] = m.Tsim;
	pTel[1] = m.X[0];
	pTel[2] = m.X[1];
	pTel[3] = m.X[2] * 30. / M_PI / m.Zp;
	pTel[4] = m.X[3] * 180. / M_PI;
	pTel[5] = m.X[4];

	/* Duty cycle.
	 * */
	pTel[6] = m.uA * 100. / (double) m.PWM_resolution;
	pTel[7] = m.uB * 100. / (double) m.PWM_resolution;
	pTel[8] = m.uC * 100. / (double) m.PWM_resolution;

	/* Estimated current.
	 * */
	pTel[9] = pm.lu_X[0];
	pTel[10] = pm.lu_X[1];

	D = cos(m.X[3]);
	Q = sin(m.X[3]);
	A = D * pm.lu_X[2] + Q * pm.lu_X[3];
	B = D * pm.lu_X[3] - Q * pm.lu_X[2];
	C = atan2(B, A);

	/* Estimated position.
	 * */
	pTel[11] = atan2(pm.lu_X[3], pm.lu_X[2]) * 180. / M_PI;
	pTel[12] = C * 180. / M_PI;

	/* Estimated speed.
	 * */
	pTel[13] = pm.lu_X[4] * 30. / M_PI / m.Zp;

	/* .
	 * */
	pTel[14] = 0.f;

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

	/* Informational.
	 * */
	pTel[22] = pm.n_power_watt;
	pTel[23] = pm.n_temperature_c;

	/* Other.
	 * */
	pTel[24] = pm.s_nonl_X4 * 30. / M_PI / m.Zp;
}

static void
sim_F(FILE *fdTel, double dT, int Verb)
{
	const int	szTel = 40;
	float		Tel[szTel];
	double		Tin, Tend;

	Tin = m.Tsim;
	Tend = Tin + dT;

	while (m.Tsim < Tend) {

		/* Plant model update.
		 * */
		blm_Update(&m);

		/* PMC update.
		 * */
		pmc_feedback(&pm, m.sensor_A, m.sensor_B);
		pmc_voltage(&pm, m.supply_U);

		/* Collect telemetry.
		 * */
		sim_Tel(Tel);

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
	pm.freq_hz = (float) (1. / m.dT);
	pm.dT = 1.f / pm.freq_hz;
	pm.pwm_resolution = m.PWM_resolution;
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

	//pm.m_bitmask |= PMC_BIT_HIGH_FREQUENCY_INJECTION;
	//pm.m_bitmask |= PMC_BIT_FORCED_CONTROL;

	pm.m_bitmask |= PMC_BIT_SPEED_CONTROL_LOOP
		| PMC_BIT_POSITION_CONTROL_LOOP;

	pmc_request(&pm, PMC_STATE_START);
	sim_F(fdTel, .1, 0);

	/*sim_F(fdTel, 1., 0);

	m.X[3] += .5;
	sim_F(fdTel, 1., 0);*/

	/*pm.s_set_point = 1000.f * (2.f * M_PI / 60.f * m.Zp);
	sim_F(fdTel, .2, 0);

	pm.s_set_point = 2000.f * (2.f * M_PI / 60.f * m.Zp);
	sim_F(fdTel, .2, 0);

	pm.s_set_point = 3000.f * (2.f * M_PI / 60.f * m.Zp);
	sim_F(fdTel, .2, 0);

	pm.s_set_point = 6000.f * (2.f * M_PI / 60.f * m.Zp);
	sim_F(fdTel, .2, 0);*/

	pm.s_set_point = 0.f;
	pm.p_set_point_revol = 3;
	sim_F(fdTel, 1., 0);

	pm.p_set_point_revol = -10;
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

