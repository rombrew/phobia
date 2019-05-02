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
	pTel[6] = m.X[6];

	/* Duty cycle.
	 * */
	pTel[7] = (double) m.PWM_A * 100. / (double) m.PWM_R;
	pTel[8] = (double) m.PWM_B * 100. / (double) m.PWM_R;
	pTel[9] = (double) m.PWM_C * 100. / (double) m.PWM_R;

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
	pTel[12] = atan2(pm.lu_X[3], pm.lu_X[2]) * 180. / M_PI;
	pTel[13] = C * 180. / M_PI;

	/* Estimated speed.
	 * */
	pTel[14] = pm.lu_X[4] * 30. / M_PI / m.Zp;

	/* Zero Drift Q.
	 * */
	pTel[15] = pm.flux_drift_Q;

	/* VSI voltage (XY).
	 * */
	pTel[16] = pm.vsi_X;
	pTel[17] = pm.vsi_Y;

	/* VSI voltage (DQ).
	 * */
	pTel[18] = pm.vsi_lpf_D;
	pTel[19] = pm.vsi_lpf_Q;

	/* VSI zone flags.
	 * */
	pTel[20] = pm.vsi_current_ZONE;
	pTel[21] = pm.vsi_voltage_ZONE;

	/* VM voltages (ABC).
	 * */
	pTel[22] = pm.vm_A;
	pTel[23] = pm.vm_B;
	pTel[24] = pm.vm_C;

	/* VM residue (XY).
	 * */
	pTel[25] = pm.vm_residue_X;
	pTel[26] = pm.vm_residue_Y;

	/* FLUX residue (DQ).
	 * */
	pTel[27] = pm.flux_residue_D;
	pTel[28] = pm.flux_residue_Q;
	pTel[29] = pm.flux_residue_lpf;

	/* Power (Watt).
	 * */
	pTel[30] = m.iP;
	pTel[31] = pm.vsi_lpf_watt;

	/* DC voltage measured.
	 * */
	pTel[32] = pm.const_lpf_U;

	/* LU mode.
	 * */
	pTel[33] = pm.lu_mode;

	pTel[34] = pm.s_track * 30. / M_PI / m.Zp;
}

static void
sim_F(FILE *fdTel, double dT, int Wait)
{
	const int	szTel = 40;
	float		Tel[szTel];
	double		Tend;

	pmfb_t		fb;

	Tend = m.Tsim + dT;

	while (m.Tsim < Tend || (Wait && pm.fsm_state != PM_STATE_IDLE)) {

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

		/* Collect telemetry.
		 * */
		sim_Tel(Tel);

		/* Dump telemetry array.
		 * */
		fwrite(Tel, sizeof(float), szTel, fdTel);

		if (pm.fail_reason != PM_OK) {

			printf("%s\n", pm_strerror(pm.fail_reason));
			exit(1);
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

	pm.fb_current_clamp = 50.f;

	pm_default(&pm);

	pm.const_R = m.R;
	pm.const_Ld = m.Ld;
	pm.const_Lq = m.Lq;
	pm.const_E = m.E;
	pm.const_Zp = m.Zp;

	pm.config_VM = PM_ENABLED;
	pm.config_HFI = PM_ENABLED;
	pm.config_LOOP = PM_LOOP_DRIVE_SPEED;

	pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);
	sim_F(fdTel, 0., 1);

	if (1) {

		printf("IAB %.4f %.4f (A)\n", pm.adjust_IA[0], pm.adjust_IB[0]);

		pm_fsm_req(&pm, PM_STATE_ADJUST_VOLTAGE);
		sim_F(fdTel, 0., 1);

		printf("UA %.4f %.4f (V)\n", pm.adjust_UA[1], pm.adjust_UA[0]);
		printf("UB %.4f %.4f (V)\n", pm.adjust_UB[1], pm.adjust_UB[0]);
		printf("UC %.4f %.4f (V)\n", pm.adjust_UC[1], pm.adjust_UC[0]);

		printf("FIR_A: %.4e %.4e %.4e\n", pm.vm_FIR_A[0], pm.vm_FIR_A[1], pm.vm_FIR_A[2]);
		printf("FIR_B: %.4e %.4e %.4e\n", pm.vm_FIR_B[0], pm.vm_FIR_B[1], pm.vm_FIR_B[2]);
		printf("FIR_C: %.4e %.4e %.4e\n", pm.vm_FIR_C[0], pm.vm_FIR_C[1], pm.vm_FIR_C[2]);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_R);
		sim_F(fdTel, 0., 1);

		printf("R %.4e (Ohm)\n", pm.const_R);

		pm_fsm_req(&pm, PM_STATE_PROBE_CONST_L);
		sim_F(fdTel, 0., 1);

		printf("LD %.4e (H)\n", pm.const_Ld);
		printf("LQ %.4e (H)\n", pm.const_Lq);
		printf("imp_R %.4e (Ohm)\n", pm.probe_impedance_R);
		printf("DQ %.2f (g)\n", pm.probe_rotation_DQ);
	}

	pm_fsm_req(&pm, PM_STATE_LU_INITIATE);
	sim_F(fdTel, 0., 1);

	pm.s_setpoint = 50.f;
	sim_F(fdTel, 1., 0);

	pm.s_setpoint = 700.f;
	sim_F(fdTel, 1., 0);

	pm.s_setpoint = 7000.f;
	sim_F(fdTel, 1., 0);
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

