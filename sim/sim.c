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
	double		A, B, E, D, Q;

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

	/* Sensor pulse.
	 * */
	pTel[10] = m.pulse_HS;
	pTel[11] = m.pulse_EP;

	/* VSI.
	 * */
	pTel[12] = pm.vsi_DC;
	pTel[13] = pm.vsi_X;
	pTel[14] = pm.vsi_Y;
	pTel[15] = pm.vsi_AF;
	pTel[16] = pm.vsi_UF;

	/* TVM.
	 * */
	pTel[17] = pm.tvm_A;
	pTel[18] = pm.tvm_B;
	pTel[19] = pm.tvm_C;
	pTel[20] = pm.tvm_DX;
	pTel[21] = pm.tvm_DY;

	/* Estimated current.
	 * */
	pTel[22] = pm.lu_iD;
	pTel[23] = pm.lu_iQ;

	D = cos(m.X[3]);
	Q = sin(m.X[3]);
	A = D * pm.lu_F[0] + Q * pm.lu_F[1];
	B = D * pm.lu_F[1] - Q * pm.lu_F[0];
	E = atan2(B, A);

	pTel[24] = E * 180. / M_PI;

	/* Estimated position.
	 * */
	pTel[25] = atan2(pm.lu_F[1], pm.lu_F[0]) * 180. / M_PI;

	/* Estimated speed.
	 * */
	pTel[26] = pm.lu_wS * 30. / M_PI / m.Zp;
	pTel[27] = pm.lu_mode;

	pTel[28] = pm.lu_flux_lpf_wS * 30. / M_PI / m.Zp;
	pTel[29] = pm.lu_flux_zone;
	pTel[30] = pm.im_revol_1;

	pTel[31] = atan2(pm.forced_F[1], pm.forced_F[0]) * 180. / M_PI;
	pTel[32] = pm.forced_wS * 30. / M_PI / m.Zp;
	pTel[33] = pm.detach_TIM;

	pTel[34] = atan2(pm.flux_F[1], pm.flux_F[0]) * 180. / M_PI;
	pTel[35] = pm.flux_wS * 30. / M_PI / m.Zp;
	pTel[36] = pm.flux_E;
	pTel[37] = 0.;
	pTel[38] = 0.;

	pTel[39] = atan2(pm.hfi_F[1], pm.hfi_F[0]) * 180. / M_PI;
	pTel[40] = pm.hfi_wS * 30. / M_PI / m.Zp;
	pTel[41] = pm.hfi_const_POLAR;

	pTel[42] = atan2(pm.hall_F[1], pm.hall_F[0]) * 180. / M_PI;
	pTel[43] = pm.hall_wS * 30. / M_PI / m.Zp;
	pTel[44] = 0.f;

	pTel[45] = atan2(pm.abi_F[1], pm.abi_F[0]) * 180. / M_PI;
	pTel[46] = pm.abi_wS * 30. / M_PI / m.Zp;
	pTel[47] = 0.f;

	pTel[48] = pm.watt_lpf_D;
	pTel[49] = pm.watt_lpf_Q;
	pTel[50] = m.iP;
	pTel[51] = pm.watt_lpf_wP;
	pTel[52] = pm.const_fb_U;

	pTel[53] = pm.i_setpoint_D;
	pTel[54] = pm.i_setpoint_Q;
	pTel[55] = pm.weak_D;

	pTel[56] = pm.s_setpoint * 30. / M_PI / m.Zp;
	pTel[57] = pm.s_track * 30. / M_PI / m.Zp;
	pTel[58] = pm.s_integral;
}

static void
sim_F(FILE *fdTel, double dT)
{
	const int	szTel = 80;
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
	double		ZpE;

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

	printf("FIR[A] %.4E %.4E %.4E [%.4f] (us)\n", pm.tvm_FIR_A[0],
			pm.tvm_FIR_A[1], pm.tvm_FIR_A[2], tau_A * 1000000.);

	printf("FIR[B] %.4E %.4E %.4E [%.4f] (us)\n", pm.tvm_FIR_B[0],
			pm.tvm_FIR_B[1], pm.tvm_FIR_B[2], tau_B * 1000000.);

	printf("FIR[C] %.4E %.4E %.4E [%.4f] (us)\n", pm.tvm_FIR_C[0],
			pm.tvm_FIR_C[1], pm.tvm_FIR_C[2], tau_C * 1000000.);

	t_assert_ref(tau_A, m.tau_U);
	t_assert_ref(tau_B, m.tau_U);
	t_assert_ref(tau_C, m.tau_U);

	pm.fsm_req = PM_STATE_PROBE_CONST_R;
	sim_F(fdTel, 0.);

	pm.const_R = pm.const_im_R;

	printf("R %.4E (Ohm)\n", pm.const_R);

	t_assert(pm.fail_reason == PM_OK);
	t_assert_ref(pm.const_R, m.R);

	pm.fsm_req = PM_STATE_PROBE_CONST_L;
	sim_F(fdTel, 0.);

	printf("L %.4E (H)\n", pm.const_L);
	printf("im_L1 %.4E (H)\n", pm.const_im_L1);
	printf("im_L2 %.4E (H)\n", pm.const_im_L2);
	printf("im_B %.2f (g)\n", pm.const_im_B);
	printf("im_R %.4E (Ohm)\n", pm.const_im_R);

	t_assert(pm.fail_reason == PM_OK);
	t_assert_ref(pm.const_im_L1, m.Ld);
	t_assert_ref(pm.const_im_L2, m.Lq);

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

	pm.fsm_req = PM_STATE_PROBE_LU_MPPE;
	sim_F(fdTel, 0.);

	printf("MPPE %.2f (rpm)\n", pm.lu_MPPE * 30. / M_PI / m.Zp);

	sim_F(fdTel, 1.);

	pm.fsm_req = PM_STATE_PROBE_CONST_J;
	sim_F(fdTel, .05);

	pm.s_setpoint = pm.probe_speed_spinup;
	sim_F(fdTel, .2);

	pm.s_setpoint = pm.probe_speed_hold;
	sim_F(fdTel, 0.);

	ZpE = 1.5f * pm.const_Zp * pm.const_Zp * pm.const_E;

	printf("J %.4E (kgm2) \n", pm.const_Ja * ZpE);

	t_assert(pm.fail_reason == PM_OK);
	t_assert_ref(pm.const_Ja * ZpE, m.J);

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
	sim_F(fdTel, 3.);

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

		printf("hall_ST[%i] %.1f\n", N, rot_H);
	}

	pm.s_setpoint = 0.f;
	sim_F(fdTel, 1.);

	t_assert(pm.fail_reason == PM_OK);

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
	m.J = 6.2E-3;

	if (sim_test_BASE(fdTel) == 0)
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
	m.R = 14E-3;
	m.Ld = 10E-6;
	m.Lq = 15E-6;
	m.U = 22.;
	m.Rs = 0.1;
	m.Zp = 14;
        m.E = 60. / 2. / M_PI / sqrt(3.) / (270. * m.Zp);
	m.J = 2.7E-4;

	if (sim_test_BASE(NULL) == 0)
		return 0;

	if (sim_test_SPEED(NULL) == 0)
		return 0;

	return 1;
}

static int
sim_RUN(FILE *fdTel)
{
	m.R = 2.4E-1;
	m.Ld = 5.2E-4;
	m.Lq = 6.5E-4;
	m.U = 48.;
	m.Rs = 0.7;
	m.Zp = 15;
        m.E = 60. / 2. / M_PI / sqrt(3.) / (15.7 * m.Zp);
	m.J = 6.2E-3;

	sim_test_BASE(fdTel);
	sim_test_HALL(fdTel);

	sim_F(fdTel, 1.);

	pm.config_DRIVE = PM_DRIVE_SPEED;
	pm.config_SENSOR = PM_SENSOR_HALL;
	pm.config_ESTIMATE = PM_ESTIMATE_DISABLED;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	sim_F(fdTel, 0.);

	pm.s_setpoint = 500.f;
	sim_F(fdTel, 1.);

	pm.s_setpoint = 10.f;
	sim_F(fdTel, 1.);

	pm.s_setpoint = -50.f;
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

