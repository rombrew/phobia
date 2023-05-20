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
#define PWM_FILE	"/tmp/pm-PWM"
#define AGP_FILE	"/tmp/pm-auto.gp"

#define TLM_SIZE	100

blm_t			m;
pmc_t			pm;

typedef struct {

	int		hatch;

	float		y[TLM_SIZE];

	FILE		*fd_tlm;
	FILE		*fd_pwm;
	FILE		*fd_gp;
}
tlm_t;

static tlm_t		tlm;

static void
tlm_page_GP(int nGP, const char *figure, const char *label)
{
	fprintf(tlm.fd_gp, "page \"%s\"\n", figure);
	if (label != NULL) { fprintf(tlm.fd_gp, "label 1 \"(%s)\"\n", label); }
	fprintf(tlm.fd_gp, "figure 0 %i \"%s\"\n\n", nGP, figure);
}

static void
tlm_plot_grab()
{
	const double	kRPM = 30. / M_PI / m.Zp;
	const double	kDEG = 180. / M_PI;

	double		A, B, C, D, Q, rel;
	int		nGP;

#define sym_GP(x, s, l)		{ tlm.y[nGP] = (float) (x); if (tlm.fd_gp != NULL) \
				{ tlm_page_GP(nGP, s, (const char *) l); } nGP++; }
#define fmt_GP(x, l)		sym_GP(x, #x, l)
#define fmk_GP(x, k, l)		sym_GP((x) * (k), #x, l)

	/* Machine State Variables.
	 * */
	tlm.y[0] = m.time;
	tlm.y[1] = m.state[0];
	tlm.y[2] = m.state[1];
	tlm.y[3] = m.state[2] * kRPM;
	tlm.y[4] = m.state[3] * kDEG;
	tlm.y[5] = m.state[4];
	tlm.y[6] = m.state[6];

	/* Duty Cycle.
	 * */
	tlm.y[7] = (double) m.pwm_A * 100. / (double) m.pwm_resolution;
	tlm.y[8] = (double) m.pwm_B * 100. / (double) m.pwm_resolution;
	tlm.y[9] = (double) m.pwm_C * 100. / (double) m.pwm_resolution;

	/* VSI Voltage.
	 * */
	tlm.y[10] = pm.vsi_X;
	tlm.y[11] = pm.vsi_Y;

	/* Estimated Current.
	 * */
	tlm.y[12] = pm.lu_iD;
	tlm.y[13] = pm.lu_iQ;

	D = cos(m.state[3]);
	Q = sin(m.state[3]);
	A = D * pm.lu_F[0] + Q * pm.lu_F[1];
	B = D * pm.lu_F[1] - Q * pm.lu_F[0];
	rel = atan2(B, A);

	if (m.unsync_flag != 0 && fabs(rel) > 1.2) {

		/* Throw an ERROR if position estimate deviation is too large.
		 * */
		pm.fsm_errno = PM_ERROR_SYNC_FAULT;
	}

	tlm.y[14] = rel * kDEG;

	/* Estimated Position.
	 * */
	tlm.y[15] = atan2(pm.lu_F[1], pm.lu_F[0]) * kDEG;

	/* Estimated Speed.
	 * */
	tlm.y[16] = pm.lu_wS * kRPM;

	/* Power Consumption.
	 * */
	tlm.y[17] = m.drain_wP;
	tlm.y[18] = pm.watt_drain_wP;

	/* DC link Voltage.
	 * */
	tlm.y[19] = pm.const_fb_U;

	blm_DQ_ABC(m.state[3], m.state[0], m.state[1], &A, &B, &C);

	/* Absolute Current.
	 * */
	tlm.y[20] = fabsf(A);
	tlm.y[21] = fabsf(B);
	tlm.y[22] = fabsf(C);

	/* NOTE: Private parameters are managed with automatic generation of GP
	 * configuration. So you only need to add a one line of code for each
	 * parameter here.
	 * */
	nGP = 30;

	fmt_GP(pm.fb_uA, 0);
	fmt_GP(pm.fb_uB, 0);
	fmt_GP(pm.fb_uC, 0);

	fmt_GP(pm.fb_HS, 0);
	fmt_GP(pm.fb_EP, 0);
	fmt_GP(pm.fb_SIN, 0);
	fmt_GP(pm.fb_COS, 0);

	fmt_GP(pm.vsi_DC, 0);
	fmt_GP(pm.vsi_lpf_DC, 0);
	fmt_GP(pm.vsi_X, "V");
	fmt_GP(pm.vsi_Y, "V");
	fmt_GP(pm.vsi_AF, 0);
	fmt_GP(pm.vsi_BF, 0);
	fmt_GP(pm.vsi_CF, 0);
	fmt_GP(pm.vsi_IF, 0);
	fmt_GP(pm.vsi_SF, 0);
	fmt_GP(pm.vsi_UF, 0);

	fmt_GP(pm.tvm_A, "V");
	fmt_GP(pm.tvm_B, "V");
	fmt_GP(pm.tvm_C, "V");

	fmt_GP(pm.lu_MODE, 0);
	fmk_GP(pm.lu_mq_load, pm.const_Zp, "Nm");

	fmt_GP(pm.base_TIM, 0);
	fmt_GP(pm.hold_TIM, 0);

	sym_GP(atan2(pm.forced_F[1], pm.forced_F[0]) * kDEG, "pm.forced_F", "°");
	fmk_GP(pm.forced_wS, kRPM, "rpm");

	fmt_GP(pm.flux_ZONE, 0);
	fmt_GP(pm.flux_X[0], "Wb");
	fmt_GP(pm.flux_X[1], "Wb");
	fmt_GP(pm.flux_lambda, "Wb");
	sym_GP(atan2(pm.flux_F[1], pm.flux_F[0]) * kDEG, "pm.flux_F", "°");
	fmk_GP(pm.flux_wS, kRPM, "rpm");

	fmt_GP(pm.kalman_bias_Q, "V");
	fmk_GP(pm.kalman_lpf_wS, kRPM, "rpm");

	fmk_GP(pm.zone_lpf_wS, kRPM, "rpm");

	fmt_GP(pm.hfi_wave[0], 0);
	fmt_GP(pm.hfi_wave[1], 0);
	fmt_GP(pm.hfi_pole, 0);

	sym_GP(atan2(pm.hall_F[1], pm.hall_F[0]) * kDEG, "pm.hall_F", "°");
	fmk_GP(pm.hall_wS, kRPM, "rpm");

	sym_GP(atan2(pm.eabi_F[1], pm.eabi_F[0]) * kDEG, "pm.eabi_F", "°");
	fmk_GP(pm.eabi_wS, kRPM, "rpm");

	fmt_GP(pm.watt_lpf_D, "V");
	fmt_GP(pm.watt_lpf_Q, "V");

	fmt_GP(pm.i_setpoint_current, "A");
	fmt_GP(pm.i_track_D, "A");
	fmt_GP(pm.i_track_Q, "A");

	fmt_GP(pm.weak_D, "A");

	fmk_GP(pm.s_setpoint_speed, kRPM, "rpm");
	fmk_GP(pm.s_track, kRPM, "rpm");

	if (tlm.fd_gp != NULL) { fclose(tlm.fd_gp); tlm.fd_gp = NULL; }

	fwrite(tlm.y, sizeof(float), TLM_SIZE, tlm.fd_tlm);
}

static void
tlm_proc_step(double dT)
{
	double		iA, iB, iC;

	tlm.y[0] += dT / 1.E-6;

	/* VSI Output.
	 * */
	tlm.y[1] = (m.xdtu[0] == 0) ? (float) m.xfet[0] : (float) tlm.hatch;
	tlm.y[2] = (m.xdtu[1] == 0) ? (float) m.xfet[1] : (float) tlm.hatch;
	tlm.y[3] = (m.xdtu[2] == 0) ? (float) m.xfet[2] : (float) tlm.hatch;

	/* Deadtime Uncertainty.
	 * */
	tlm.y[4] = (float) m.xdtu[0];
	tlm.y[5] = (float) m.xdtu[1];
	tlm.y[6] = (float) m.xdtu[2];

	blm_DQ_ABC(m.state[3], m.state[0], m.state[1], &iA, &iB, &iC);

	/* Machine Current.
	 * */
	tlm.y[7] = iA;
	tlm.y[8] = iB;
	tlm.y[9] = iC;

	/* Machine DC link Voltage.
	 * */
	tlm.y[10] = m.state[6];

	/* Machine ADC.
	 * */
	tlm.y[11] = m.state[7];
	tlm.y[12] = m.state[8];
	tlm.y[13] = m.state[9];
	tlm.y[14] = m.state[10];
	tlm.y[15] = m.state[11];
	tlm.y[16] = m.state[12];
	tlm.y[17] = m.state[13];
	tlm.y[18] = m.state[14];

	fwrite(tlm.y, sizeof(float), 40, tlm.fd_pwm);

	tlm.hatch = (tlm.hatch == 0) ? 1 : 0;
}

static void
tlm_PWM_grab()
{
	tlm.fd_pwm = fopen(PWM_FILE, "wb");

	if (tlm.fd_pwm == NULL) {

		fprintf(stderr, "fopen: %s", strerror(errno));
		exit(-1);
	}

	tlm.y[0] = 0.f;

	m.sol_dT = 10.E-9;
	m.proc_step = &tlm_proc_step;

	/* Collect telemetry in one PWM cycle.
	 * */
	blm_update(&m);

	fclose(tlm.fd_pwm);

	m.sol_dT = 5.E-6;
	m.proc_step = NULL;
}

void tlm_restart()
{
	if (tlm.fd_tlm == NULL) {

		tlm.fd_tlm = fopen(TLM_FILE, "wb");

		if (tlm.fd_tlm == NULL) {

			fprintf(stderr, "fopen: %s", strerror(errno));
			exit(-1);
		}

		tlm.fd_gp = fopen(AGP_FILE, "w");

		if (tlm.fd_gp == NULL) {

			fprintf(stderr, "fopen: %s", strerror(errno));
			exit(-1);
		}
	}
	else {
		tlm.fd_tlm = freopen(NULL, "wb", tlm.fd_tlm);
	}
}

void sim_runtime(double dT)
{
	pmfb_t		fb;
	double		stop;

	stop = m.time + dT;

	while (m.time < stop) {

		/* Plant model update.
		 * */
		blm_update(&m);

		fb.current_A = m.analog_iA;
		fb.current_B = m.analog_iB;
		fb.current_C = m.analog_iC;
		fb.voltage_U = m.analog_uS;
		fb.voltage_A = m.analog_uA;
		fb.voltage_B = m.analog_uB;
		fb.voltage_C = m.analog_uC;

		fb.analog_SIN = m.analog_SIN;
		fb.analog_COS = m.analog_COS;

		fb.pulse_HS = m.pulse_HS;
		fb.pulse_EP = m.pulse_EP;

		/* PM update.
		 * */
		pm_feedback(&pm, &fb);

		if (tlm.fd_tlm != NULL) {

			/* Collect telemetry.
			 * */
			tlm_plot_grab();
		}

		if (pm.fsm_errno != PM_OK) {

			fprintf(stderr, "fsm_errno: %s\n", pm_strerror(pm.fsm_errno));

			if (tlm.fd_tlm != NULL) {

				fclose(tlm.fd_tlm);
			}

			exit(-1);
		}
	}
}

void bench_script()
{
	blm_enable(&m);
	blm_restart(&m);
	tlm_restart();

	m.Rs = 14.E-3;
	m.Ld = 22.E-6;
	m.Lq = 35.E-6;
	m.Udc = 22.;
	m.Rdc = 0.1;
	m.Zp = 14;
	m.lambda = blm_Kv_lambda(&m, 270.);
	m.Jm = 3.E-4;

	ts_script_base();

	blm_restart(&m);
	tlm_restart();

	pm.config_LU_ESTIMATE = PM_FLUX_KALMAN;
	pm.config_HFI_WAVE = PM_HFI_SINE;
	pm.config_LU_DRIVE = PM_DRIVE_CURRENT;

	pm.s_maximal = 2000.f;
	pm.s_reverse = 2000.f;
	pm.s_accel = 700.f;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_for_idle();

	sim_runtime(.1);

	pm.i_setpoint_current = 10.f;
	sim_runtime(1.);

	pm.i_setpoint_current = 0.f;
	sim_runtime(1.);

	pm.i_setpoint_current = 10.f;
	sim_runtime(1.);

	pm.i_setpoint_current = 0.f;
	sim_runtime(1.);

	pm.i_setpoint_current = 10.f;
	sim_runtime(1.);

	pm.i_setpoint_current = 0.f;
	sim_runtime(1.);

	tlm_PWM_grab();
}

int main(int argc, char *argv[])
{
	if (argc < 2) {

		exit(-1);
	}

	lfg_start((int) time(NULL));

	if (strcmp(argv[1], "verify") == 0) {

		ts_script_verify();
	}
	else if (strcmp(argv[1], "bench") == 0) {

		bench_script();
	}

	if (tlm.fd_tlm != NULL) {

		fclose(tlm.fd_tlm);
	}

	return 0;
}

