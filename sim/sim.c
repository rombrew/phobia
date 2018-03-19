/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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
#include "pm.h"
#include "lib.h"

#define TEL_FILE	"/tmp/TEL"

static blm_t		m;
static pmc_t		pm;

static void
blmDC(int uA, int uB, int uC)
{
	m.PWM_xA = uA;
	m.PWM_xB = uB;
	m.PWM_xC = uC;
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
	pTel[6] = (double) m.PWM_xA * 100. / (double) m.PWM_R;
	pTel[7] = (double) m.PWM_xB * 100. / (double) m.PWM_R;
	pTel[8] = (double) m.PWM_xC * 100. / (double) m.PWM_R;

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

	/* Zero Drift Q.
	 * */
	pTel[14] = pm.lu_drift_Q;

	/* VSI voltage (XY).
	 * */
	pTel[15] = pm.vsi_X;
	pTel[16] = pm.vsi_Y;

	/* VSI voltage (DQ).
	 * */
	pTel[17] = pm.vsi_lpf_D;
	pTel[18] = pm.vsi_lpf_Q;

	/* Measurement residual.
	 * */
	pTel[19] = pm.lu_residual_D;
	pTel[20] = pm.lu_residual_Q;
	pTel[21] = sqrt(pm.lu_residual_lpf);
	
	/* Informational.
	 * */
	pTel[25] = pm.n_power_lpf;
}

static void
sim_F(FILE *fdTel, double dT, int Verb)
{
	const int	szTel = 40;
	float		Tel[szTel];
	double		Tin, Tend;

	pmfb_t		fb;

	Tin = m.Tsim;
	Tend = Tin + dT;

	while (m.Tsim < Tend) {

		/* Plant model update.
		 * */
		blm_Update(&m);

		fb.iA = m.ADC_iA;
		fb.iB = m.ADC_iB;
		fb.uS = m.ADC_uS;

		/* PM update.
		 * */
		pm_feedback(&pm, &fb);

		if (pm.error != PM_OK) {

			printf("ERROR: %s\n", pm_strerror(pm.error));
			exit(1);
		}

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
	pm.pwm_R = m.PWM_R;
	pm.pDC = &blmDC;
	pm.pZ = &blmZ;

	pm_default(&pm);

	pm.const_lpf_U = m.U;
	pm.const_R = m.R * (1. - .0);
	pm.const_Ld = m.Ld * (1. + .0);
	pm.const_Lq = m.Lq * (1. + .0);
	pm.const_E = m.E * (1. - .0);
	pm.const_Zp = m.Zp;

	pm.b_FORCED = 0;
	pm.b_HFI = 0;
	pm.b_LOOP = 1;

	pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);
	sim_F(fdTel, .5, 0);

	pm_fsm_req(&pm, PM_STATE_LU_INITIATE);
	sim_F(fdTel, .1, 0);

	pm.s_set_point = 500.f;
	sim_F(fdTel, .5, 0);
}

int main(int argc, char *argv[])
{
	FILE		*fdTel;

	libStart();
	blm_Enable(&m);

	fdTel = fopen(TEL_FILE, "wb");

	if (fdTel == NULL) {

		fprintf(stderr, "fopen: %s", strerror(errno));
		exit(2);
	}

	sim_Script(fdTel);

	fclose(fdTel);
	libStop();

	return 0;
}

