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
#define MLD_FILE	"/tmp/pm-mldata"
#define AGP_FILE	"/tmp/pm-auto.gp"

#define TLM_SIZE	100

blm_t			m;
pmc_t			pm;

typedef struct {

	int		hatch;

	float		y[TLM_SIZE];

	FILE		*fd_tlm;
	FILE		*fd_pwm;
	FILE		*fd_mld;
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
		pm.fsm_errno = PM_ERROR_NO_SYNC_FAULT;
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
	fmt_GP(pm.vsi_UF, 0);

	fmt_GP(pm.dcu_DX, "V");
	fmt_GP(pm.dcu_DY, "V");

	fmt_GP(pm.lu_MODE, 0);
	fmk_GP(pm.lu_mq_produce, pm.const_Zp, "Nm");
	fmk_GP(pm.lu_mq_load, pm.const_Zp, "Nm");

	fmt_GP(pm.base_TIM, 0);
	fmt_GP(pm.hold_TIM, 0);

	sym_GP(atan2(pm.forced_F[1], pm.forced_F[0]) * kDEG, "pm.forced_F", "deg");
	fmk_GP(pm.forced_wS, kRPM, "rpm");

	fmt_GP(pm.forced_track_D, "A");

	fmt_GP(pm.detach_TIM, 0);

	fmt_GP(pm.flux_LINKAGE, 0);
	fmt_GP(pm.flux_ZONE, 0);

	fmt_GP(pm.flux_X[0], "Wb");
	fmt_GP(pm.flux_X[1], "Wb");
	fmt_GP(pm.flux_lambda, "Wb");
	sym_GP(atan2(pm.flux_F[1], pm.flux_F[0]) * kDEG, "pm.flux_F", "deg");
	fmk_GP(pm.flux_wS, kRPM, "rpm");

	fmt_GP(pm.kalman_rsu_D, "A");
	fmt_GP(pm.kalman_rsu_Q, "A");
	fmt_GP(pm.kalman_bias_Q, "V");
	fmk_GP(pm.kalman_lpf_wS, kRPM, "rpm");

	fmk_GP(pm.zone_lpf_wS, kRPM, "rpm");

	fmt_GP(pm.hfi_wave[0], 0);
	fmt_GP(pm.hfi_wave[1], 0);

	sym_GP(atan2(pm.hall_F[1], pm.hall_F[0]) * kDEG, "pm.hall_F", "deg");
	fmk_GP(pm.hall_wS, kRPM, "rpm");

	fmt_GP(pm.eabi_ADJUST, 0);

	sym_GP(atan2(pm.eabi_F[1], pm.eabi_F[0]) * kDEG, "pm.eabi_F", "deg");
	fmk_GP(pm.eabi_wS, kRPM, "rpm");

	fmt_GP(pm.watt_DC_MAX, 0);
	fmt_GP(pm.watt_DC_MIN, 0);

	fmt_GP(pm.watt_lpf_D, "V");
	fmt_GP(pm.watt_lpf_Q, "V");

	fmt_GP(pm.i_setpoint_current, "A");
	fmk_GP(pm.i_setpoint_torque, pm.const_Zp, "Nm");
	fmt_GP(pm.i_track_D, "A");
	fmt_GP(pm.i_track_Q, "A");
	fmt_GP(pm.i_integral_D, "V");
	fmt_GP(pm.i_integral_Q, "V");

	fmt_GP(pm.mtpa_setpoint_Q, "A");
	fmt_GP(pm.mtpa_load_Q, "A");
	fmt_GP(pm.mtpa_track_D, "A");
	fmt_GP(pm.weak_track_D, "A");

	fmk_GP(pm.s_setpoint_speed, kRPM, "rpm");
	fmk_GP(pm.s_track, kRPM, "rpm");
	fmt_GP(pm.s_integral, "A");

	if (tlm.fd_gp != NULL) { fclose(tlm.fd_gp); tlm.fd_gp = NULL; }

	fwrite(tlm.y, sizeof(float), TLM_SIZE, tlm.fd_tlm);

	if (tlm.fd_mld != NULL) {

		D = cos(m.state[3]);
		Q = sin(m.state[3]);

		C = (m.pwm_A + m.pwm_B + m.pwm_C) / 3.;
		A = (m.pwm_A - C) * m.state[6] / (double) m.pwm_resolution;
		B = (m.pwm_B - C) * m.state[6] / (double) m.pwm_resolution;

		B = 0.577350269189626 * A + 1.15470053837925 * B;

		tlm.y[0] = D * m.state[0] + Q * m.state[1];	/* iX */
		tlm.y[1] = D * m.state[1] - Q * m.state[0];	/* iY */
		tlm.y[2] = A;					/* uX */
		tlm.y[3] = B;					/* uY */
		tlm.y[4] = D;					/* cos(\th) */
		tlm.y[5] = Q;					/* sin(\th) */
		tlm.y[6] = m.state[2];				/* \omega */
		tlm.y[7] = m.state[4];				/* Tc */
		tlm.y[8] = 0;
		tlm.y[9] = 0;

		fwrite(tlm.y, sizeof(float), 10, tlm.fd_mld);
	}
}

static void
tlm_proc_step(double dT)
{
	double		iA, iB, iC;

	tlm.y[0] += dT / 1.e-6;

	/* VSI Output.
	 * */
	tlm.y[1] = (m.xdtu[0] == 0) ? (float) m.xfet[0] : (float) tlm.hatch;
	tlm.y[2] = (m.xdtu[1] == 0) ? (float) m.xfet[1] : (float) tlm.hatch;
	tlm.y[3] = (m.xdtu[2] == 0) ? (float) m.xfet[2] : (float) tlm.hatch;

	/* Dead-Time Uncertainty.
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

	/* Analog feedback.
	 * */
	tlm.y[19] = m.hold_iA;
	tlm.y[20] = m.hold_iB;
	tlm.y[21] = m.hold_iC;
	tlm.y[22] = m.analog_uA;
	tlm.y[23] = m.analog_uB;
	tlm.y[24] = m.analog_uC;
	tlm.y[25] = m.analog_uS;

	fwrite(tlm.y, sizeof(float), 40, tlm.fd_pwm);

	tlm.hatch = (tlm.hatch == 0) ? 1 : 0;
}

static void
tlm_PWM_grab()
{
	double		usual_dT;

	tlm.fd_pwm = fopen(PWM_FILE, "wb");

	if (tlm.fd_pwm == NULL) {

		fprintf(stderr, "fopen: %s\n", strerror(errno));
		abort();
	}

	tlm.y[0] = 0.f;

	usual_dT = m.sol_dT;
	m.sol_dT = 10.e-9;
	m.proc_step = &tlm_proc_step;

	/* Collect telemetry in three PWM cycle.
	 * */
	blm_update(&m);
	blm_update(&m);
	blm_update(&m);

	fclose(tlm.fd_pwm);

	m.sol_dT = usual_dT;
	m.proc_step = NULL;
}

void tlm_restart()
{
	if (tlm.fd_tlm == NULL) {

		tlm.fd_tlm = fopen(TLM_FILE, "wb");

		if (tlm.fd_tlm == NULL) {

			fprintf(stderr, "fopen: %s\n", strerror(errno));
			abort();
		}

		tlm.fd_gp = fopen(AGP_FILE, "w");

		if (tlm.fd_gp == NULL) {

			fprintf(stderr, "fopen: %s\n", strerror(errno));
			abort();
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

			abort();
		}
	}
}

void bench_script()
{
	blm_enable(&m);
	blm_restart(&m);

	tlm_restart();

	m.Rs = 20.e-3;
	m.Ld = 15.e-6;
	m.Lq = 25.e-6;
	m.Udc = 49.;
	m.Rdc = 0.1;
	m.Zp = 5;
	m.lambda = blm_Kv_lambda(&m, 58.);
	m.Jm = 17.e-3;

	ts_script_default();
	ts_script_base();
	blm_restart(&m);

	m.Jm = 5.e+7;

	pm.fsm_req = PM_STATE_PROBE_CONST_SATURATION;
	ts_wait_IDLE();

	printf("const_im_Ld = %.4e (H)\n", pm.const_im_Ld);
	printf("const_im_Lq = %.4e (H)\n", pm.const_im_Lq);

	pm.fsm_req = PM_STATE_PROBE_CONST_RESISTANCE;
	ts_wait_IDLE();

	pm.const_Rs = pm.const_im_Rz;

	printf("const_Rs = %.4e (Ohm)\n", pm.const_Rs);
	printf("self_DTu = %.4f (V)\n", pm.self_DTu);

	/*ts_adjust_sensor_hall();
	blm_restart(&m);

	pm.config_LU_SENSOR = PM_SENSOR_HALL;

	pm.watt_wA_maximal = 80.f;
	pm.watt_wA_reverse = 80.f;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_IDLE();

	pm.s_setpoint_speed = 800.f;
	sim_runtime(2.0);*/

	tlm_PWM_grab();
}

void mld_script()
{
	blm_enable(&m);
	blm_restart(&m);

	tlm_restart();

	m.Rs = 14.e-3;
	m.Ld = 10.e-6;
	m.Lq = 15.e-6;
	m.Udc = 22.;
	m.Rdc = 0.1;
	m.Zp = 14;
	m.lambda = blm_Kv_lambda(&m, 270.);
	m.Jm = 4.e-4;

	ts_script_default();
	ts_script_base();
	blm_restart(&m);

	pm.config_LU_ESTIMATE = PM_FLUX_KALMAN;
	pm.config_HFI_WAVETYPE = PM_HFI_SINE;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_IDLE();

	pm.s_setpoint_speed = 60.f;
	sim_runtime(1.0);

	tlm.fd_mld = fopen(MLD_FILE, "wb");

	if (tlm.fd_mld == NULL) {

		fprintf(stderr, "fopen: %s\n", strerror(errno));
		abort();
	}

	pm.s_setpoint_speed = 60.f;
	sim_runtime(1.0);

	pm.s_setpoint_speed = 900.f;
	sim_runtime(1.0);

	pm.s_setpoint_speed = - 60.f;
	sim_runtime(1.0);

	pm.s_setpoint_speed = - 900.f;
	sim_runtime(1.0);

	pm.s_setpoint_speed = 5500.f;
	sim_runtime(2.0);

	fclose(tlm.fd_mld);
}

int main(int argc, char *argv[])
{
	if (argc < 2) {

		abort();
	}

	lfg_start((int) time(NULL));

	if (strcmp(argv[1], "test") == 0) {

		ts_script_test();
	}
	else if (strcmp(argv[1], "bench") == 0) {

		bench_script();
	}
	else if (strcmp(argv[1], "data") == 0) {

		mld_script();
	}

	if (tlm.fd_tlm != NULL) {

		fclose(tlm.fd_tlm);
	}

	return 0;
}

