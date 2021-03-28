#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>

#include "blm.h"
#include "lfg.h"
#include "pm.h"
#include "tsfunc.h"

#define TLM_FILE	"/tmp/pm-TLM"
#define AGP_FILE	"/tmp/pm-auto.gp"

blm_t			m;
pmc_t			pm;

FILE			*fdTlm;
FILE			*fdGP;

static void
fmt_page_GP(int N, const char *figure, const char *label)
{
	fprintf(fdGP, "page \"%s\"\n", figure);
	if (label != NULL) { fprintf(fdGP, "label 1 \"(%s)\"\n", label); }
	fprintf(fdGP, "figure 0 %i \"%s\"\n\n", N, figure);
}

static void
sim_Tlm(float *pTlm)
{
	double		A, B, E, D, Q;
	int		gp_N;

	const double	kRPM = 30. / M_PI / m.Zp;
	const double	kDEG = 180. / M_PI;

#define sym_GP(x,s,l)	{ pTlm[gp_N] = (x); if (fdGP != NULL) { fmt_page_GP(gp_N, s, l); } gp_N++; }
#define fmt_GP(x,l)	sym_GP(x, #x, l)

	/* Model.
	 * */
	pTlm[0] = m.Tsim;
	pTlm[1] = m.X[0];
	pTlm[2] = m.X[1];
	pTlm[3] = m.X[2] * kRPM;
	pTlm[4] = m.X[3] * kDEG;
	pTlm[5] = m.X[4];
	pTlm[6] = m.X[6];

	/* Duty cycle.
	 * */
	pTlm[7] = (double) m.PWM_A * 100. / (double) m.PWM_R;
	pTlm[8] = (double) m.PWM_B * 100. / (double) m.PWM_R;
	pTlm[9] = (double) m.PWM_C * 100. / (double) m.PWM_R;

	/* VSI.
	 * */
	pTlm[10] = pm.vsi_X;
	pTlm[11] = pm.vsi_Y;

	/* Estimated current.
	 * */
	pTlm[12] = pm.lu_iD;
	pTlm[13] = pm.lu_iQ;

	D = cos(m.X[3]);
	Q = sin(m.X[3]);
	A = D * pm.lu_F[0] + Q * pm.lu_F[1];
	B = D * pm.lu_F[1] - Q * pm.lu_F[0];
	E = atan2(B, A);

	if (m.sync_F != 0 && fabs(E) > 1.5f) {

		/* Throw an error if position estimate error is too large.
		 * */
		pm.fsm_errno = PM_ERROR_LOSS_OF_SYNC;
	}

	pTlm[14] = E * kDEG;

	/* Estimated position.
	 * */
	pTlm[15] = atan2(pm.lu_F[1], pm.lu_F[0]) * kDEG;

	/* Estimated speed.
	 * */
	pTlm[16] = pm.lu_wS * kRPM;

	/* Power consumption.
	 * */
	pTlm[17] = m.iP;
	pTlm[18] = pm.watt_lpf_wP;

	/* Supply voltage.
	 * */
	pTlm[19] = pm.const_fb_U;

	/* NOTE: Individual parameters are managed with automatic generation of
	 * GP configuration. So you only need to add one line of code for each
	 * parameter below.
	 * */
	gp_N = 30;

	fmt_GP(pm.fb_HS, NULL);
	fmt_GP(pm.fb_EP, NULL);

	fmt_GP(pm.vsi_DC, NULL);
	fmt_GP(pm.vsi_lpf_DC, NULL);
	fmt_GP(pm.vsi_AF, NULL);
	fmt_GP(pm.vsi_BF, NULL);
	fmt_GP(pm.vsi_SF, NULL);
	fmt_GP(pm.vsi_UF, NULL);

	fmt_GP(pm.tvm_A, NULL);
	fmt_GP(pm.tvm_B, NULL);
	fmt_GP(pm.tvm_C, NULL);

	fmt_GP(pm.lu_mode, NULL);
	fmt_GP(pm.lu_lpf_torque, "A");

	sym_GP(atan2(pm.forced_F[1], pm.forced_F[0]) * kDEG, "pm.forced_F", "°");
	fmt_GP(pm.forced_wS * kRPM, "rpm");
	fmt_GP(pm.forced_TIM, NULL);

	fmt_GP(pm.detach_X, "V");
	fmt_GP(pm.detach_Y, "V");
	sym_GP(atan2(pm.detach_V[1], pm.detach_V[0]) * kDEG, "pm.detach_V", "°");
	fmt_GP(pm.detach_TIM, NULL);
	fmt_GP(pm.detach_SKIP, NULL);

	fmt_GP(pm.flux_X, "Wb");
	fmt_GP(pm.flux_Y, "Wb");
	fmt_GP(pm.flux_E, "Wb");
	sym_GP(atan2(pm.flux_F[1], pm.flux_F[0]) * kDEG, "pm.flux_F", "°");
	fmt_GP(pm.flux_wS * kRPM, "rpm");
	fmt_GP(pm.flux_mode, NULL);
	fmt_GP(pm.flux_lpf_wS * kRPM, "rpm");

	sym_GP(atan2(pm.hfi_F[1], pm.hfi_F[0]) * kDEG, "pm.hfi_F", "°");
	fmt_GP(pm.hfi_wS * kRPM, "rpm");

	sym_GP(atan2(pm.hall_F[1], pm.hall_F[0]) * kDEG, "pm.hall_F", "°");
	fmt_GP(pm.hall_wS * kRPM, "rpm");

	sym_GP(atan2(pm.abi_F[1], pm.abi_F[0]) * kDEG, "pm.abi_F", "°");
	fmt_GP(pm.abi_wS * kRPM, "rpm");

	fmt_GP(pm.watt_lpf_D, "V");
	fmt_GP(pm.watt_lpf_Q, "V");

	fmt_GP(pm.i_derated_WEAK, "A");
	fmt_GP(pm.i_setpoint_torque, "A");
	fmt_GP(pm.i_track_D, "A");
	fmt_GP(pm.i_track_Q, "A");

	fmt_GP(pm.weak_D, "A");

	fmt_GP(pm.s_setpoint_speed * kRPM, "rpm");
	fmt_GP(pm.s_track * kRPM, "rpm");
	fmt_GP(pm.s_iSP, "A");

	fmt_GP(pm.im_total_revol, NULL);

	if (fdGP != NULL) { fclose(fdGP); fdGP = NULL; }
}

void sim_TlmDrop()
{
	if (fdTlm == NULL) {

		fdTlm = fopen(TLM_FILE, "wb");

		if (fdTlm == NULL) {

			fprintf(stderr, "fopen: %s", strerror(errno));
			exit(1);
		}

		fdGP = fopen(AGP_FILE, "w");

		if (fdGP == NULL) {

			fprintf(stderr, "fopen: %s", strerror(errno));
			exit(1);
		}
	}
	else {
		fdTlm = freopen(NULL, "wb", fdTlm);
	}
}

void sim_Run(double dT)
{
	const int	szTlm = 100;
	float		Tlm[szTlm];
	double		Tend;

	pmfb_t		fb;

	Tend = m.Tsim + dT;

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
		fb.pulse_HS = m.pulse_HS;
		fb.pulse_EP = m.pulse_EP;

		/* PM update.
		 * */
		pm_feedback(&pm, &fb);

		if (fdTlm != NULL) {

			/* Collect telemetry.
			 * */
			sim_Tlm(Tlm);

			/* Dump telemetry array.
			 * */
			fwrite(Tlm, sizeof(float), szTlm, fdTlm);
		}

		if (pm.fsm_errno != PM_OK) {

			fprintf(stderr, "pm.fsm_errno: %s\n", pm_strerror(pm.fsm_errno));

			fclose(fdTlm);
			exit(1);
		}
	}
}

void sim_START()
{
	blm_Enable(&m);
	blm_Stop(&m);
	sim_TlmDrop();

	/*m.R = 2.4E-1;
	m.Ld = 5.2E-4;
	m.Lq = 6.5E-4;
	m.U = 48.;
	m.Rs = 0.5;
	m.Zp = 15;
        m.E = 60. / 2. / M_PI / sqrt(3.) / (15.7 * m.Zp);
	m.J = 6.2E-3;*/

	m.R = 14E-3;
	m.Ld = 10E-6;
	m.Lq = 15E-6;
	m.U = 22.;
	m.Rs = 0.1;
	m.Zp = 14;
        m.E = 60. / 2. / M_PI / sqrt(3.) / (270. * m.Zp);
	m.J = 2.7E-4;

	ts_BASE();
	blm_Stop(&m);
	sim_TlmDrop();

	pm.config_DRIVE = PM_DRIVE_CURRENT;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_for_IDLE();

	pm.config_LIMITED = PM_ENABLED;

	pm.s_maximal = 300.f * M_PI / 30. * m.Zp;
	pm.s_accel = 1000.f;

	m.M[2] = 5E-5;

	pm.i_setpoint_torque = 20.f;
	sim_Run(1.);

	pm.i_setpoint_torque = -10.f;
	sim_Run(1.);

	pm.i_setpoint_torque = 20.f;
	sim_Run(.1);

	m.M[2] = 5E-4;

	sim_Run(2.);
}

int main(int argc, char *argv[])
{
	lfg_start((int) time(NULL));

	if (argc == 2 && strcmp(argv[1], "test") == 0) {

		ts_START();
	}
	else {
		sim_START();
	}

	fclose(fdTlm);

	return 0;
}

