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
blmZ(int Z)
{
	if (Z == 7) {

		m.HI_Z = 1;
	}
	else {
		m.HI_Z = 0;
	}
}

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

	/* TVM voltages (ABC).
	 * */
	pTel[22] = pm.tvm_A;
	pTel[23] = pm.tvm_B;
	pTel[24] = pm.tvm_C;

	/* TVM voltages (XY).
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
	pTel[38] = pm.vsi_EU;
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

		fb.current_A = m.ADC_IA;
		fb.current_B = m.ADC_IB;
		fb.voltage_U = m.ADC_US;
		fb.voltage_A = m.ADC_UA;
		fb.voltage_B = m.ADC_UB;
		fb.voltage_C = m.ADC_UC;
		fb.pulse_HS = m.pulse_HS;
		fb.pulse_EP = m.pulse_EP;

		/* PM update.
		 * */
		pm_feedback(&pm, &fb);

		if (fdTel != NULL) {

			/* Collect telemetry.
			 * */
			sim_Tel(Tel);

			/* Dump telemetry array.
			 * */
			fwrite(Tel, sizeof(float), szTel, fdTel);
		}

		if (pm.fail_reason != PM_OK) {

			printf("** pm.fail_reason: %s\n", pm_strerror(pm.fail_reason));
			return ;
		}
	}
}

#define t_prologue()		printf("\n# %s\n", __FUNCTION__);
#define t_xprintf(s)		fprintf(stderr, "** assert(%s) in %s:%i\n", s, __FILE__, __LINE__)
#define t_assert(x)		if ((x) == 0) { t_xprintf(#x); return 0; }
#define t_assert_ref(x,ref)	t_assert(fabs((x) - (ref)) / (ref) < 0.1)

static int
sim_test_BASE(FILE *fdTel)
{
	double		tau_A, tau_B, tau_C;

	t_prologue();

	pm.freq_hz = (float) (1. / m.dT);
	pm.dT = 1.f / pm.freq_hz;
	pm.dc_resolution = m.PWM_R;
	pm.proc_set_DC = &blmDC;
	pm.proc_set_Z = &blmZ;

	pm_default(&pm);

	pm.const_Zp = m.Zp;

	pm.fsm_req = PM_STATE_ZERO_DRIFT;
	sim_F(fdTel, 0.);

	printf("Z[AB] %.4f %.4f (A)\n", pm.ad_IA[0], pm.ad_IB[0]);

	t_assert(pm.fail_reason == PM_OK);

	pm.fsm_req = PM_STATE_ADJUST_VOLTAGE;
	sim_F(fdTel, 0.);

	printf("UA %.4f %.4f (V)\n", pm.ad_UA[1], pm.ad_UA[0]);
	printf("UB %.4f %.4f (V)\n", pm.ad_UB[1], pm.ad_UB[0]);
	printf("UC %.4f %.4f (V)\n", pm.ad_UC[1], pm.ad_UC[0]);

	t_assert(pm.fail_reason == PM_OK);

	tau_A = pm.dT / log(pm.tvm_FIR_A[0] / - pm.tvm_FIR_A[1]);
	tau_B = pm.dT / log(pm.tvm_FIR_B[0] / - pm.tvm_FIR_B[1]);
	tau_C = pm.dT / log(pm.tvm_FIR_C[0] / - pm.tvm_FIR_C[1]);

	printf("FIR[A] %.4E %.4E %.4E [%.4E] (s)\n", pm.tvm_FIR_A[0],
			pm.tvm_FIR_A[1], pm.tvm_FIR_A[2], tau_A);

	printf("FIR[B] %.4E %.4E %.4E [%.4E] (s)\n", pm.tvm_FIR_B[0],
			pm.tvm_FIR_B[1], pm.tvm_FIR_B[2], tau_B);

	printf("FIR[C] %.4E %.4E %.4E [%.4E] (s)\n", pm.tvm_FIR_C[0],
			pm.tvm_FIR_C[1], pm.tvm_FIR_C[2], tau_C);

	t_assert_ref(tau_A, m.tau_U);
	t_assert_ref(tau_B, m.tau_U);
	t_assert_ref(tau_C, m.tau_U);

	pm.fsm_req = PM_STATE_PROBE_CONST_R;
	sim_F(fdTel, 0.);

	printf("R %.4E (Ohm)\n", pm.const_R);

	t_assert(pm.fail_reason == PM_OK);
	t_assert_ref(pm.const_R, m.R);

	pm.fsm_req = PM_STATE_PROBE_CONST_L;
	sim_F(fdTel, 0.);

	t_assert(pm.fail_reason == PM_OK);

	pm.fsm_req = PM_STATE_PROBE_CONST_L;
	sim_F(fdTel, 0.);

	printf("L %.4E (H)\n", pm.const_L);
	printf("im_LD %.4E (H)\n", pm.const_im_LD);
	printf("im_LQ %.4E (H)\n", pm.const_im_LQ);
	printf("im_B %.2f (g)\n", pm.const_im_B);
	printf("im_R %.4E (Ohm)\n", pm.const_im_R);

	t_assert(pm.fail_reason == PM_OK);
	t_assert_ref(pm.const_im_LD, m.Ld);
	t_assert_ref(pm.const_im_LQ, m.Lq);

	pm.fsm_req = PM_STATE_LU_STARTUP;
	sim_F(fdTel, 0.);

	t_assert(pm.fail_reason == PM_OK);

	pm.s_setpoint = pm.probe_speed_hold;
	sim_F(fdTel, 1.);

	pm.fsm_req = PM_STATE_PROBE_CONST_E;
	sim_F(fdTel, 0.);

	printf("Kv %.2f (rpm/v)\n", 5.513289f / (pm.const_E * pm.const_Zp));

	t_assert(pm.fail_reason == PM_OK);
	t_assert_ref(pm.const_E, m.E);

	sim_F(fdTel, 1.);

	pm.fsm_req = PM_STATE_PROBE_CONST_E;
	sim_F(fdTel, 0.);

	printf("Kv %.2f (rpm/v)\n", 5.513289f / (pm.const_E * pm.const_Zp));

	t_assert(pm.fail_reason == PM_OK);
	t_assert_ref(pm.const_E, m.E);

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	sim_F(fdTel, 0.);

	t_assert(pm.fail_reason == PM_OK);

	return 1;
}

static int
sim_test_SPEED(FILE *fdTel)
{
	t_prologue();

	pm.config_DRIVE = PM_DRIVE_SPEED;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	sim_F(fdTel, 0.);

	t_assert(pm.fail_reason == PM_OK);

	pm.s_setpoint = 0.f;
	sim_F(fdTel, 1.);

	t_assert(pm.fail_reason == PM_OK);

	pm.s_setpoint = .2f * m.U / m.E;
	sim_F(fdTel, 2.);

	printf("wSP %.2f (rpm)\n", pm.s_setpoint * 30. / M_PI / m.Zp);
	printf("lu_wS %.2f (rpm)\n", pm.lu_wS * 30. / M_PI / m.Zp);

	t_assert(pm.fail_reason == PM_OK);
	t_assert_ref(pm.lu_wS, pm.s_setpoint);

	pm.s_setpoint = 0.f;
	sim_F(fdTel, 1.);

	t_assert(pm.fail_reason == PM_OK);

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	sim_F(fdTel, 0.);

	t_assert(pm.fail_reason == PM_OK);

	return 1;
}

static int
sim_test_HFI(FILE *fdTel)
{
	/* TODO */

	return 1;
}

static int
sim_test_HALL(FILE *fdTel)
{
	double		rot_H;
	int		N;

	t_prologue();

	pm.fsm_req = PM_STATE_LU_STARTUP;
	sim_F(fdTel, 0.);

	t_assert(pm.fail_reason == PM_OK);

	pm.s_setpoint = pm.probe_speed_hold;
	sim_F(fdTel, 1.);

	t_assert(pm.fail_reason == PM_OK);

	pm.fsm_req = PM_STATE_ADJUST_HALL;
	sim_F(fdTel, 0.);

	t_assert(pm.fail_reason == PM_OK);

	for (N = 1; N < 7; ++N) {

		rot_H = atan2(pm.hall_ST[N].Y, pm.hall_ST[N].X) * 180. / M_PI;

		printf("hall_AT[%i] %.1f\n", N, rot_H);
	}

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	sim_F(fdTel, 0.);

	t_assert(pm.fail_reason == PM_OK);

	return 1;
}

static int
sim_test_WEAK(FILE *fdTel)
{
	/* TODO */

	return 1;
}

static int
sim_TEST(FILE *fdTel)
{
	/* E-scooter hub motor (250W).
         * */
	m.R = 2.4E-1;
	m.Ld = 5.2E-4;
	m.Lq = 6.5E-4;
	m.U = 48.;
	m.Rs = 0.7;
	m.Zp = 15;
        m.E = 60. / 2. / M_PI / sqrt(3.) / (15.7 * m.Zp);
	m.J = 5E-3;
	m.M[0] = 0E-3;
	m.M[1] = 5E-2;
	m.M[2] = 5E-6;

	if (sim_test_BASE(NULL) == 0)
		return 0;

	if (sim_test_SPEED(NULL) == 0)
		return 0;

	if (sim_test_HFI(NULL) == 0)
		return 0;

	if (sim_test_HALL(NULL) == 0)
		return 0;

	if (sim_test_WEAK(NULL) == 0)
		return 0;

	/* Turnigy RotoMax 1.20.
         * */
	m.R = 22E-3;
	m.Ld = 11E-6;
	m.Lq = 17E-6;
	m.U = 32.;
	m.Rs = 0.2;
	m.Zp = 7;
        m.E = 60. / 2. / M_PI / sqrt(3.) / (280. * m.Zp);
	m.J = 5E-4;
	m.M[0] = 0E-3;
	m.M[1] = 2E-3;
	m.M[2] = 5E-6;

	if (sim_test_BASE(NULL) == 0)
		return 0;

	if (sim_test_SPEED(NULL) == 0)
		return 0;

	return 1;
}

static int
sim_RUN(FILE *fdTel)
{
	sim_test_BASE(NULL);

	pm.forced_hold_D = 0.f;

	pm.fsm_req = PM_STATE_LU_DETACHED;
	sim_F(fdTel, 0.);

	sim_F(fdTel, 1.);

	m.M[0] = 1.f;

	sim_F(fdTel, 1.);

	pm.fsm_req = PM_STATE_PROBE_CONST_E;
	sim_F(fdTel, 0.);

	printf("Kv %.2f (rpm/v)\n", 5.513289f / (pm.const_E * pm.const_Zp));

	sim_F(fdTel, 1.);

	return 1;
}

int main(int argc, char *argv[])
{
	FILE		*fdTel;

	lib_start();
	blm_Enable(&m);

	fdTel = fopen(TEL_FILE, "wb");

	if (fdTel == NULL) {

		fprintf(stderr, "fopen: %s", strerror(errno));
		return 1;
	}

	if (argc == 2) {

		sim_TEST(fdTel);
	}
	else {
		sim_RUN(fdTel);
	}

	fclose(fdTel);
	lib_stop();

	return 0;
}

