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
blmDC(int A, int B, int C)
{
	m.PWM_A = A;
	m.PWM_B = B;
	m.PWM_C = C;
}

static void
blmZ(int Z) { /* Not implemented */ }

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
	pTel[6] = (double) m.PWM_A * 100. / (double) m.PWM_R;
	pTel[7] = (double) m.PWM_B * 100. / (double) m.PWM_R;
	pTel[8] = (double) m.PWM_C * 100. / (double) m.PWM_R;

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
	pTel[14] = pm.flux_drift_Q;

	/* VSI voltage (XY).
	 * */
	pTel[15] = pm.vsi_X;
	pTel[16] = pm.vsi_Y;

	/* VSI voltage (DQ).
	 * */
	pTel[17] = pm.vsi_lpf_D;
	pTel[18] = pm.vsi_lpf_Q;

	/* Measurement residue.
	 * */
	pTel[19] = pm.flux_residue_D;
	pTel[20] = pm.flux_residue_Q;
	pTel[21] = pm.flux_residue_lpf;

	/* Power (Watt).
	 * */
	pTel[22] = m.iP;
	pTel[23] = pm.vsi_lpf_watt;

	pTel[24] = pm.vsi_clamp_to_GND;
	pTel[25] = pm.vsi_zone_A;
	pTel[26] = pm.vsi_zone_B;
	pTel[27] = pm.vsi_zone_C;
	pTel[28] = pm.tvse_residue_X;
	pTel[29] = pm.tvse_residue_Y;
	pTel[30] = pm.lu_mode;
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

		fb.current_A = m.ADC_IA;
		fb.current_B = m.ADC_IB;
		fb.voltage_U = m.ADC_US;
		fb.voltage_A = m.ADC_UA;
		fb.voltage_B = m.ADC_UB;
		fb.voltage_C = m.ADC_UC;

		/* PM update.
		 * */
		pm_feedback(&pm, &fb);

		if (pm.fail_reason != PM_OK) {

			printf("%s\n", pm_strerror(pm.fail_reason));
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
	pm.dc_resolution = m.PWM_R;
	pm.proc_set_DC = &blmDC;
	pm.proc_set_Z = &blmZ;

	pm.dc_minimal = 50;
	pm.dc_clearance = 350;
	pm.fb_current_clamp = 50.f;

	pm_config_default(&pm);

	pm.const_lpf_U = m.U;
	pm.const_R = m.R;
	pm.const_Ld = m.Ld;
	pm.const_Lq = m.Lq;
	pm.const_E = m.E;
	pm.const_Zp = m.Zp;

	pm.config_HFI = 0;
	pm.config_LOOP = 1;

	pm_config_tune_current_loop(&pm);

	pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);
	sim_F(fdTel, .4, 0);

	if (1) {

		pm_fsm_req(&pm, PM_STATE_ADJUST_VOLTAGE);
		sim_F(fdTel, 1., 0);

		printf("%4f %4f (V)\n", pm.adjust_UA[1], pm.adjust_UA[0]);
		printf("%4f %4f (V)\n", pm.adjust_UB[1], pm.adjust_UB[0]);
		printf("%4f %4f (V)\n", pm.adjust_UC[1], pm.adjust_UC[0]);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_R);
		sim_F(fdTel, 1., 0);

		printf("%4e (Ohm)\n", pm.const_R);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_L);
		sim_F(fdTel, 1., 0);

		printf("%4e (H)\n", pm.const_Ld);
		printf("%4e (H)\n", pm.const_Lq);
	}

	pm_fsm_req(&pm, PM_STATE_LU_INITIATE);
	sim_F(fdTel, .5, 0);

	pm.s_setpoint = 9000.f;
	sim_F(fdTel, .5, 0);

	pm.vsi_clamp_to_GND = 1;

	sim_F(fdTel, .5, 0);

	pm.s_setpoint = -2000.f;
	sim_F(fdTel, .5, 0);

	/*pm.s_setpoint = -30000.f;
	sim_F(fdTel, .5, 0);

	sim_F(fdTel, .2, 0);

	//m.R *= (1. + 40E-2);

	sim_F(fdTel, .2, 0);*/
}

int main(int argc, char *argv[])
{
	FILE		*fdTel;

	lib_start();
	blm_Enable(&m);

	fdTel = fopen(TEL_FILE, "wb");

	if (fdTel == NULL) {

		fprintf(stderr, "fopen: %s", strerror(errno));
		exit(2);
	}

	sim_Script(fdTel);

	fclose(fdTel);
	lib_stop();

	return 0;
}

