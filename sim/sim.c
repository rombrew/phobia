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
	pTel[10] = pm.lu_iD;
	pTel[11] = pm.lu_iQ;

	D = cos(m.X[3]);
	Q = sin(m.X[3]);
	A = D * pm.lu_F[0] + Q * pm.lu_F[1];
	B = D * pm.lu_F[1] - Q * pm.lu_F[0];
	C = atan2(B, A);

	/* FLUX position.
	 * */
	pTel[12] = atan2(pm.lu_F[1], pm.lu_F[0]) * 180. / M_PI;
	pTel[13] = C * 180. / M_PI;

	/* FLUX speed.
	 * */
	pTel[14] = pm.lu_wS * 30. / M_PI / m.Zp;

	/* FLUX E.
	 * */
	pTel[15] = pm.flux_E;

	/* VSI voltage (XY).
	 * */
	pTel[16] = pm.vsi_X;
	pTel[17] = pm.vsi_Y;

	/* WATT voltage (DQ).
	 * */
	pTel[18] = pm.watt_lpf_D;
	pTel[19] = pm.watt_lpf_Q;

	/* VSI zone flags.
	 * */
	pTel[20] = pm.vsi_IF;
	pTel[21] = pm.vsi_UF;

	/* VM voltages (ABC).
	 * */
	pTel[22] = pm.tvm_A;
	pTel[23] = pm.tvm_B;
	pTel[24] = pm.tvm_C;

	/* VM voltages (XY).
	 * */
	pTel[25] = pm.tvm_DX;
	pTel[26] = pm.tvm_DY;

	/* FLUX residue (DQ).
	 * */
	pTel[27] = pm.flux[pm.flux_H].lpf_E;
	pTel[28] = 0.f;
	pTel[29] = 0.f;

	/* WATT power.
	 * */
	pTel[30] = m.iP;
	pTel[31] = pm.watt_lpf_wP;

	/* DC link voltage measured.
	 * */
	pTel[32] = pm.const_lpf_U;

	/* LU mode.
	 * */
	pTel[33] = pm.lu_mode;

	/* SPEED tracking point.
	 * */
	pTel[34] = pm.s_track * 30. / M_PI / m.Zp;
	pTel[35] = pm.flux_H;
	pTel[36] = pm.s_setpoint * 30. / M_PI / m.Zp;
	pTel[37] = pm.hfi_polarity;
	pTel[38] = m.short_F;
}

static void
sim_F(FILE *fdTel, double dT)
{
	const int	szTel = 40;
	float		Tel[szTel];
	double		Tend;

	pmfb_t		fb;

	Tend = m.Tsim + ((dT < m.dT) ? m.dT : dT);

	while (m.Tsim < Tend || (dT < m.dT && pm.fsm_state != PM_STATE_IDLE)) {

		/* Plant model update.
		 * */
		blm_Update(&m);

		fb.halt_OCP = 0;
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

	pm_default(&pm);

	pm.const_Zp = m.Zp;

	pm.fsm_req = PM_STATE_ZERO_DRIFT;
	sim_F(fdTel, 0.);

	printf("AB %.4f %.4f (A)\n", pm.ad_IA[0], pm.ad_IB[0]);

	if (1) {

		pm.fsm_req = PM_STATE_ADJUST_VOLTAGE;
		sim_F(fdTel, 0.);

		printf("UA %.4f %.4f (V)\n", pm.ad_UA[1], pm.ad_UA[0]);
		printf("UB %.4f %.4f (V)\n", pm.ad_UB[1], pm.ad_UB[0]);
		printf("UC %.4f %.4f (V)\n", pm.ad_UC[1], pm.ad_UC[0]);

		printf("FIR_A: %.4e %.4e %.4e\n", pm.tvm_FIR_A[0], pm.tvm_FIR_A[1], pm.tvm_FIR_A[2]);
		printf("FIR_B: %.4e %.4e %.4e\n", pm.tvm_FIR_B[0], pm.tvm_FIR_B[1], pm.tvm_FIR_B[2]);
		printf("FIR_C: %.4e %.4e %.4e\n", pm.tvm_FIR_C[0], pm.tvm_FIR_C[1], pm.tvm_FIR_C[2]);

		pm.fsm_req = PM_STATE_PROBE_CONST_R;
		sim_F(fdTel, 0.);

		printf("R %.4e (Ohm)\n", pm.const_R);

		pm.fsm_req = PM_STATE_PROBE_CONST_L;
		sim_F(fdTel, 0.);

		printf("L %.4e (H)\n", pm.const_L);
		printf("im_LD %.4e (H)\n", pm.const_im_LD);
		printf("im_LQ %.4e (H)\n", pm.const_im_LQ);
		printf("im_B %.2f (g)\n", pm.const_im_B);
		printf("im_R %.4e (Ohm)\n", pm.const_im_R);

		pm.fsm_req = PM_STATE_LU_STARTUP;
		sim_F(fdTel, 0.);

		pm.s_setpoint = pm.probe_speed_low;
		sim_F(fdTel, 1.);

		pm.fsm_req = PM_STATE_PROBE_CONST_E;
		sim_F(fdTel, 0.);

		printf("Kv %.1f (rpm/v)\n", 5.513289f / (pm.const_E * pm.const_Zp));

		pm.s_setpoint = pm.probe_speed_ramp;
		sim_F(fdTel, 1.);

		pm.fsm_req = PM_STATE_PROBE_CONST_E;
		sim_F(fdTel, 0.);

		printf("Kv %.1f (rpm/v)\n", 5.513289f / (pm.const_E * pm.const_Zp));
	}

	pm.config_HFI = 1;

	pm.s_setpoint = 10.f;
	sim_F(fdTel, 1.);

	m.M[2] = 9e-1;

	pm.s_setpoint = 10.f;
	sim_F(fdTel, 2.);

	pm.s_setpoint = 200.f;
	sim_F(fdTel, 2.);
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

