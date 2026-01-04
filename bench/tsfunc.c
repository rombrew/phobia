#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <errno.h>

#include "blm.h"
#include "lfg.h"
#include "pm.h"
#include "tsfunc.h"

#define TS_TICK_RATE		1000
#define TS_TOL			0.2

#define TS_printf(s)		fprintf(stderr, "%s in %s:%i\n", (s), __FILE__, __LINE__)
#define TS_assert(x)		if ((x) == 0) { TS_printf(#x); exit(-1); }

#define TS_assert_absolute(x, r, a)	TS_assert(fabs((x) - (r)) < fabs(a))
#define TS_assert_relative(x, r)	TS_assert(fabs((x) - (r)) < TS_TOL * fabs(r))

int ts_wait_IDLE()
{
	int			xTIME = 0;

	do {
		sim_runtime(10 / (double) TS_TICK_RATE);

		if (pm.fsm_state == PM_STATE_IDLE)
			break;

		if (xTIME > 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		xTIME += 10;
	}
	while (1);

	return pm.fsm_errno;
}

int ts_wait_motion()
{
	int			xTIME = 0;

	do {
		sim_runtime(50 / (double) TS_TICK_RATE);

		if (pm.fsm_errno != PM_OK)
			break;

		if (		m_fabsf(pm.zone_lpf_wS) > pm.zone_threshold
				&& pm.detach_TIM > PM_TSMS(&pm, pm.tm_transient_slow))
			break;

		if (xTIME > 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		xTIME += 50;
	}
	while (1);

	return pm.fsm_errno;
}

int ts_wait_spinup()
{
	int			xTIME = 0;

	do {
		sim_runtime(50 / (double) TS_TICK_RATE);

		if (pm.fsm_errno != PM_OK)
			break;

		if (m_fabsf(pm.s_setpoint_speed - pm.lu_wS) < pm.probe_speed_tol)
			break;

		if (		pm.lu_MODE == PM_LU_FORCED
				&& pm.vsi_lpf_DC > pm.forced_stop_DC)
			break;

		if (xTIME > 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		xTIME += 50;
	}
	while (1);

	return pm.fsm_errno;
}

void ts_self_adjust()
{
	double		usual_Mq;

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		ts_wait_IDLE();

		printf("const_fb_U = %.3f (V)\n", pm.const_fb_U);

		printf("self_STDi = %.3f %.3f %.3f (A)\n", pm.self_STDi[0],
				pm.self_STDi[1], pm.self_STDi[2]);

		printf("scale_iABC0 = %.3f %.3f %.3f (A)\n", pm.scale_iA[0],
				pm.scale_iB[0], pm.scale_iC[0]);

		printf("probe_current_hold = %.3f (A)\n", pm.probe_current_hold);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_ADJUST_ON_PCB_VOLTAGE;
			ts_wait_IDLE();

			printf("scale_uA = %.4E %.4f (V)\n", pm.scale_uA[1], pm.scale_uA[0]);
			printf("scale_uB = %.4E %.4f (V)\n", pm.scale_uB[1], pm.scale_uB[0]);
			printf("scale_uC = %.4E %.4f (V)\n", pm.scale_uC[1], pm.scale_uC[0]);

			printf("self_RMSu = %.4f (V)\n", pm.self_RMSu);
			printf("self_RMSt = %.4f %.4f %.4f (V)\n", pm.self_RMSt[0],
					pm.self_RMSt[1], pm.self_RMSt[2]);

			if (pm.fsm_errno != PM_OK)
				break;
		}

		if (pm.config_DCU_VOLTAGE == PM_ENABLED) {

			usual_Mq = m.Mq[3];
			m.Mq[3] = 5.E-1;

			pm.fsm_req = PM_STATE_ADJUST_DCU_VOLTAGE;
			ts_wait_IDLE();

			m.Mq[3] = usual_Mq;

			printf("const_im_Rz = %.4E (Ohm)\n", pm.const_im_Rz);
			printf("dcu_deadband = %.1f (ns)\n", pm.dcu_deadband);
			printf("self_DTu = %.4f (V)\n", pm.self_DTu);
		}
	}
	while (0);
}

void ts_probe_impedance()
{
	double		usual_Mq;

	do {
		usual_Mq = m.Mq[3];
		m.Mq[3] = 5.E-1;

		pm.fsm_req = PM_STATE_PROBE_CONST_RESISTANCE;

		printf("probe_current_hold = %.3f (A)\n", pm.probe_current_hold);
		printf("probe_current_sine = %.3f (A)\n", pm.probe_current_sine);
		printf("probe_current_bias = %.3f (A)\n", pm.probe_current_bias);
		printf("probe_freq_sine = %.1f (Hz)\n", pm.probe_freq_sine);
		printf("probe_loss_maximal = %.1f (W)\n", pm.probe_loss_maximal);

		if (ts_wait_IDLE() != PM_OK)
			break;

		m.Mq[3] = usual_Mq;

		pm.const_Rs = pm.const_im_Rz;

		printf("const_Rs = %.4E (Ohm)\n", pm.const_Rs);
		printf("self_DTu = %.4f (V)\n", pm.self_DTu);

		TS_assert_relative(pm.const_Rs, m.Rs);

		pm.fsm_req = PM_STATE_PROBE_CONST_INDUCTANCE;

		if (ts_wait_IDLE() != PM_OK)
			break;

		printf("const_im_Ld = %.4E (H)\n", pm.const_im_Ld);
		printf("const_im_Lq = %.4E (H)\n", pm.const_im_Lq);
		printf("const_im_Ag = %.2f (deg)\n", pm.const_im_Ag);
		printf("const_im_Rz = %.4E (Ohm)\n", pm.const_im_Rz);

		TS_assert_relative(pm.const_im_Ld, m.Ld);
		TS_assert_relative(pm.const_im_Lq, m.Lq);

		pm_auto(&pm, PM_AUTO_MAXIMAL_CURRENT);
		pm_auto(&pm, PM_AUTO_LOOP_CURRENT);

		printf("i_maixmal = %.3f (A) \n", pm.i_maximal);
		printf("i_gain_P = %.2E \n", pm.i_gain_P);
		printf("i_gain_I = %.2E \n", pm.i_gain_I);
		printf("i_slew_rate = %.1f (A/s)\n", pm.i_slew_rate);
	}
	while (0);
}

void ts_probe_spinup()
{
	int		backup_LU_DRIVE;
	float		Kv;

	backup_LU_DRIVE = pm.config_LU_DRIVE;
	pm.config_LU_DRIVE = PM_DRIVE_SPEED;

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (ts_wait_IDLE() != PM_OK)
			break;

		if (		pm.flux_LINKAGE != PM_ENABLED
				&& pm.config_EXCITATION == PM_MAGNET_PERMANENT) {

			pm.s_setpoint_speed = pm.probe_speed_hold;

			printf("probe_speed_hold = %.2f (rad/s)\n", pm.probe_speed_hold);

			if (ts_wait_spinup() != PM_OK)
				break;

			sim_runtime(200 / (double) TS_TICK_RATE);

			pm.fsm_req = PM_STATE_PROBE_CONST_FLUX_LINKAGE;

			if (ts_wait_IDLE() != PM_OK)
				break;

			Kv = 60. / (2. * M_PI * sqrt(3.)) / (pm.const_lambda * pm.const_Zp);

			printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);
			printf("const_lambda = %.4E (Wb) %.2f (rpm/v)\n", pm.const_lambda, Kv);
		}

		pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);
		pm_auto(&pm, PM_AUTO_PROBE_SPEED_HOLD);
		pm_auto(&pm, PM_AUTO_FORCED_MAXIMAL);

		printf("probe_speed_hold = %.2f (rad/s)\n", pm.probe_speed_hold);
		printf("forced_maximal = %.2f (rad/s)\n", pm.forced_maximal);

		printf("zone_threshold = %.2f (rad/s) %.3f (V)\n",
				pm.zone_threshold,
				pm.zone_threshold * pm.const_lambda / pm.k_EMAX);

		printf("zone_tol = %.2f (rad/s) %.3f (V)\n",
				pm.zone_tol,
				pm.zone_tol * pm.const_lambda / pm.k_EMAX);

		pm.s_setpoint_speed = pm.probe_speed_hold;

		if (ts_wait_spinup() != PM_OK)
			break;

		if (pm.flux_ZONE != PM_ZONE_HIGH) {

			pm.fsm_errno = PM_ERROR_NO_FLUX_CAUGHT;
			break;
		}

		if (pm.config_EXCITATION == PM_MAGNET_PERMANENT) {

			sim_runtime(200 / (double) TS_TICK_RATE);

			pm.fsm_req = PM_STATE_PROBE_CONST_FLUX_LINKAGE;

			if (ts_wait_IDLE() != PM_OK)
				break;

			Kv = 60. / (2. * M_PI * sqrt(3.)) / (pm.const_lambda * pm.const_Zp);

			printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);
			printf("const_lambda = %.4E (Wb) %.2f (rpm/v)\n", pm.const_lambda, Kv);

			TS_assert_relative(pm.const_lambda, m.lambda);
		}

		sim_runtime(200 / (double) TS_TICK_RATE);

		pm.fsm_req = PM_STATE_PROBE_THRESHOLD_TOL;

		if (ts_wait_IDLE() != PM_OK)
			break;

		pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);
		pm_auto(&pm, PM_AUTO_PROBE_SPEED_HOLD);
		pm_auto(&pm, PM_AUTO_FORCED_MAXIMAL);

		printf("probe_speed_hold = %.2f (rad/s)\n", pm.probe_speed_hold);
		printf("forced_maximal = %.2f (rad/s)\n", pm.forced_maximal);

		printf("zone_threshold = %.2f (rad/s) %.3f (V)\n",
				pm.zone_threshold,
				pm.zone_threshold * pm.const_lambda / pm.k_EMAX);

		printf("zone_tol = %.2f (rad/s) %.3f (V)\n",
				pm.zone_tol,
				pm.zone_tol * pm.const_lambda / pm.k_EMAX);

		pm.fsm_req = PM_STATE_PROBE_CONST_INERTIA;

		printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);

		sim_runtime(100 / (double) TS_TICK_RATE);

		pm.s_setpoint_speed = 110.f * pm.k_EMAX / 100.f
				* pm.const_fb_U / pm.const_lambda;

		sim_runtime(400 / (double) TS_TICK_RATE);

		printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);

		pm.s_setpoint_speed = pm.probe_speed_hold;

		sim_runtime(400 / (double) TS_TICK_RATE);

		if (ts_wait_IDLE() != PM_OK)
			break;

		printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);
		printf("const_Ja = %.4E (kgm2) \n", pm.const_Ja * pm.const_Zp * pm.const_Zp);

		TS_assert_relative(pm.const_Ja * pm.const_Zp * pm.const_Zp, m.Jm);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (ts_wait_IDLE() != PM_OK)
			break;

		pm_auto(&pm, PM_AUTO_FORCED_ACCEL);
		pm_auto(&pm, PM_AUTO_LOOP_SPEED);

		printf("forced_accel = %.1f (rad/s2)\n", pm.forced_accel);
		printf("lu_gain_mq_LP = %.2E\n", pm.lu_gain_mq_LP);
		printf("s_gain_P = %.2E\n", pm.s_gain_P);
		printf("s_gain_D = %.2E\n", pm.s_gain_D);
	}
	while (0);

	pm.config_LU_DRIVE = backup_LU_DRIVE;
}

void ts_adjust_sensor_hall()
{
	int		backup_LU_SENSOR, backup_LU_DRIVE, N;

	backup_LU_SENSOR = pm.config_LU_SENSOR;
	backup_LU_DRIVE = pm.config_LU_DRIVE;

	pm.config_LU_SENSOR = PM_SENSOR_NONE;
	pm.config_LU_DRIVE = PM_DRIVE_SPEED;

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		printf("probe_speed_hold = %.2f (rad/s)\n", pm.probe_speed_hold);

		if (ts_wait_IDLE() != PM_OK)
			break;

		pm.s_setpoint_speed = pm.probe_speed_hold;

		if (ts_wait_spinup() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_ADJUST_SENSOR_HALL;

		if (ts_wait_IDLE() != PM_OK)
			break;

		printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);

		for (N = 1; N < 7; ++N) {

			double		STg;

			STg = atan2(pm.hall_ST[N].Y, pm.hall_ST[N].X) * (180. / M_PI);

			printf("hall_ST[%i] = %.1f (deg)\n", N, STg);
		}

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (ts_wait_IDLE() != PM_OK)
			break;
	}
	while (0);

	pm.config_LU_SENSOR = backup_LU_SENSOR;
	pm.config_LU_DRIVE = backup_LU_DRIVE;
}

void ts_adjust_sensor_eabi()
{
	int		backup_LU_SENSOR, backup_LU_DRIVE;

	double		F0g;

	backup_LU_SENSOR = pm.config_LU_SENSOR;
	backup_LU_DRIVE = pm.config_LU_DRIVE;

	pm.config_LU_SENSOR = PM_SENSOR_NONE;
	pm.config_LU_DRIVE = PM_DRIVE_SPEED;

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		printf("zone_threshold = %.2f (rad/s)\n", pm.zone_threshold);

		if (ts_wait_IDLE() != PM_OK)
			break;

		pm.s_setpoint_speed = pm.zone_threshold;

		if (ts_wait_spinup() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_ADJUST_SENSOR_EABI;

		if (ts_wait_IDLE() != PM_OK)
			break;

		F0g = atan2(pm.eabi_F0[1], pm.eabi_F0[0]) * (180. / M_PI);

		printf("eabi_const_EP = %i\n", pm.eabi_const_EP);
		printf("eabi_const_Zs = %i\n", pm.eabi_const_Zs);
		printf("eabi_F0 = %.1f (deg)\n", F0g);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (ts_wait_IDLE() != PM_OK)
			break;
	}
	while (0);

	pm.config_LU_SENSOR = backup_LU_SENSOR;
	pm.config_LU_DRIVE = backup_LU_DRIVE;
}

void ts_adjust_sensor_sincos()
{
	int		backup_LU_SENSOR, backup_LU_DRIVE;

	int		N;

	backup_LU_SENSOR = pm.config_LU_SENSOR;
	backup_LU_DRIVE = pm.config_LU_DRIVE;

	pm.config_LU_SENSOR = PM_SENSOR_NONE;
	pm.config_LU_DRIVE = PM_DRIVE_SPEED;

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		printf("probe_speed_hold = %.2f (rad/s)\n", pm.probe_speed_hold);

		if (ts_wait_IDLE() != PM_OK)
			break;

		pm.s_setpoint_speed = pm.probe_speed_hold;

		if (ts_wait_spinup() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_ADJUST_SENSOR_SINCOS;

		printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);

		sim_runtime(400 / (double) TS_TICK_RATE);

		pm.s_setpoint_speed = 110.f * pm.k_EMAX / 100.f
				* pm.const_fb_U / pm.const_lambda;

		sim_runtime(400 / (double) TS_TICK_RATE);

		printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);

		pm.s_setpoint_speed = pm.probe_speed_hold;

		sim_runtime(400 / (double) TS_TICK_RATE);

		printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);

		pm.s_setpoint_speed = 110.f * pm.k_EMAX / 100.f
				* pm.const_fb_U / pm.const_lambda;

		sim_runtime(400 / (double) TS_TICK_RATE);

		printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);

		pm.s_setpoint_speed = pm.probe_speed_hold;

		sim_runtime(400 / (double) TS_TICK_RATE);

		if (ts_wait_IDLE() != PM_OK)
			break;

		printf("lu_wS = %.2f (rad/s)\n", pm.lu_wS);

		for (N = 0; N < 16; ++N) {

			printf("sincos_CONST[%i] = %.6f\n", N, pm.sincos_CONST[N]);
		}

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (ts_wait_IDLE() != PM_OK)
			break;
	}
	while (0);

	pm.config_LU_SENSOR = backup_LU_SENSOR;
	pm.config_LU_DRIVE = backup_LU_DRIVE;
}

static void
blm_proc_DC(int A, int B, int C)
{
	m.pwm_A = A;
	m.pwm_B = B;
	m.pwm_C = C;
}

static void
blm_proc_Z(int Z)
{
	m.pwm_Z = (Z != PM_Z_ABC) ? BLM_Z_NONE : BLM_Z_DETACHED;
}

void ts_script_default()
{
	pm.m_freq = (float) (1. / m.pwm_dT);
	pm.m_dT = 1.f / pm.m_freq;
	pm.dc_resolution = m.pwm_resolution;
	pm.proc_set_DC = &blm_proc_DC;
	pm.proc_set_Z = &blm_proc_Z;

	pm_auto(&pm, PM_AUTO_BASIC_DEFAULT);
	pm_auto(&pm, PM_AUTO_CONFIG_DEFAULT);
}

void ts_script_base()
{
	pm.const_Zp = m.Zp;

	ts_self_adjust();
	ts_probe_impedance();
	ts_probe_spinup();
}

static void
ts_script_speed()
{
	pm.config_LU_DRIVE = PM_DRIVE_SPEED;

	pm.s_accel_forward = 300000.f;
	pm.s_accel_reverse = pm.s_accel_forward;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_IDLE();

	m.unsync_flag = 1;

	pm.s_setpoint_speed = 50.f * pm.k_EMAX / 100.f
			* pm.const_fb_U / pm.const_lambda;

	ts_wait_spinup();
	sim_runtime(0.5);

	TS_assert(pm.lu_MODE == PM_LU_ESTIMATE);

	m.Mq[0] = - 1.5 * m.Zp * m.lambda * 20.f;
	sim_runtime(0.5);

	m.Mq[0] = 0.f;
	sim_runtime(0.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 10.f * pm.k_EMAX / 100.f
		* pm.const_fb_U / pm.const_lambda;

	ts_wait_spinup();
	sim_runtime(0.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	m.unsync_flag = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_IDLE();
}

static void
ts_script_hfi()
{
	pm.config_LU_ESTIMATE = PM_FLUX_KALMAN;
	pm.config_LU_DRIVE = PM_DRIVE_SPEED;
	pm.config_HFI_WAVETYPE = PM_HFI_SILENT;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_IDLE();

	m.unsync_flag = 1;

	pm.s_setpoint_speed = 1.f / m.lambda;
	sim_runtime(1.);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 0;
	sim_runtime(0.5);

	TS_assert(pm.lu_MODE == PM_LU_ON_HFI);

	pm.s_setpoint_speed = - 1.f / m.lambda;
	sim_runtime(0.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 0;
	sim_runtime(0.5);

	m.unsync_flag = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_IDLE();

	pm.config_HFI_WAVETYPE = PM_HFI_NONE;
}

static void
ts_script_weakening()
{
	pm.config_WEAKENING = PM_ENABLED;
	pm.config_LU_DRIVE = PM_DRIVE_SPEED;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_IDLE();

	m.unsync_flag = 1;

	pm.s_setpoint_speed = 200.f * pm.k_EMAX / 100.f
			* pm.const_fb_U / pm.const_lambda;

	ts_wait_spinup();
	sim_runtime(.5);

	TS_assert(pm.lu_MODE == PM_LU_ESTIMATE);

	m.Mq[0] = - 1.5 * m.Zp * m.lambda * 5.f;
	sim_runtime(0.5);

	m.Mq[0] = 0.f;
	sim_runtime(0.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 10.f * pm.k_EMAX / 100.f
		* pm.const_fb_U / pm.const_lambda;

	ts_wait_spinup();
	sim_runtime(0.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	m.unsync_flag = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_IDLE();
}

static void
ts_script_hall()
{
	int		backup_LU_ESTIMATE;

	ts_adjust_sensor_hall();
	blm_restart(&m);

	backup_LU_ESTIMATE = pm.config_LU_ESTIMATE;

	pm.config_LU_ESTIMATE = PM_FLUX_NONE;
	pm.config_LU_SENSOR = PM_SENSOR_HALL;

	pm.s_damping = 0.5f;

	pm_auto(&pm, PM_AUTO_LOOP_SPEED);

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_IDLE();

	m.unsync_flag = 1;

	pm.s_setpoint_speed = 50.f * pm.k_EMAX / 100.f
			* pm.const_fb_U / pm.const_lambda;

	ts_wait_spinup();
	sim_runtime(0.5);

	TS_assert(pm.lu_MODE == PM_LU_SENSOR_HALL);

	m.Mq[0] = - 1.5 * m.Zp * m.lambda * 20.f;
	sim_runtime(0.5);

	m.Mq[0] = 0.f;
	sim_runtime(0.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 10.f * pm.k_EMAX / 100.f
		* pm.const_fb_U / pm.const_lambda;

	ts_wait_spinup();
	sim_runtime(0.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	m.unsync_flag = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_IDLE();

	pm.config_LU_ESTIMATE = backup_LU_ESTIMATE;
	pm.config_LU_SENSOR = PM_SENSOR_NONE;
}

static void
ts_script_eabi(int knob_EABI)
{
	int		backup_LU_ESTIMATE;

	if (knob_EABI == PM_EABI_INCREMENTAL) {

		m.eabi_ERES = 2400;
		m.eabi_WRAP = 65536;

		pm.config_EABI_FRONTEND = PM_EABI_INCREMENTAL;
	}
	else if (knob_EABI == PM_EABI_ABSOLUTE) {

		m.eabi_ERES = 16384;
		m.eabi_WRAP = 16384;

		pm.config_EABI_FRONTEND = PM_EABI_ABSOLUTE;
	}

	pm.eabi_ADJUST = PM_DISABLED;

	ts_adjust_sensor_eabi();
	blm_restart(&m);

	backup_LU_ESTIMATE = pm.config_LU_ESTIMATE;

	pm.config_LU_ESTIMATE = PM_FLUX_NONE;
	pm.config_LU_SENSOR = PM_SENSOR_EABI;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_IDLE();

	m.unsync_flag = 1;

	pm.s_setpoint_speed = 50.f * pm.k_EMAX / 100.f
			* pm.const_fb_U / pm.const_lambda;

	ts_wait_spinup();
	sim_runtime(0.5);

	m.Mq[0] = - 1.5 * m.Zp * m.lambda * 20.f;
	sim_runtime(0.5);

	m.Mq[0] = 0.f;
	sim_runtime(0.5);

	TS_assert(pm.lu_MODE == PM_LU_SENSOR_EABI);
	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 10.f * pm.k_EMAX / 100.f
		* pm.const_fb_U / pm.const_lambda;

	ts_wait_spinup();
	sim_runtime(0.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	m.unsync_flag = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_IDLE();

	pm.config_LU_ESTIMATE = backup_LU_ESTIMATE;
	pm.config_LU_SENSOR = PM_SENSOR_NONE;
}

static void
ts_script_sincos()
{
	int		backup_LU_ESTIMATE;

	ts_adjust_sensor_sincos();
	blm_restart(&m);

	backup_LU_ESTIMATE = pm.config_LU_ESTIMATE;

	pm.config_LU_ESTIMATE = PM_FLUX_NONE;
	pm.config_LU_SENSOR = PM_SENSOR_SINCOS;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_IDLE();

	m.unsync_flag = 1;

	pm.s_setpoint_speed = 50.f * pm.k_EMAX / 100.f
			* pm.const_fb_U / pm.const_lambda;

	ts_wait_spinup();
	sim_runtime(0.5);

	m.Mq[0] = - 1.5 * m.Zp * m.lambda * 20.f;
	sim_runtime(0.5);

	m.Mq[0] = 0.f;
	sim_runtime(0.5);

	TS_assert(pm.lu_MODE == PM_LU_SENSOR_SINCOS);
	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 10.f * pm.k_EMAX / 100.f
		* pm.const_fb_U / pm.const_lambda;

	ts_wait_spinup();
	sim_runtime(0.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	m.unsync_flag = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_IDLE();

	pm.config_LU_ESTIMATE = backup_LU_ESTIMATE;
	pm.config_LU_SENSOR = PM_SENSOR_NONE;
}

void ts_script_test()
{
	blm_enable(&m);
	blm_restart(&m);

	printf("\n---- XNOVA Lightning 4530 ----\n");

	tlm_restart();

	m.Rs = 8.E-3;
	m.Ld = 3.E-6;
	m.Lq = 5.E-6;
	m.Udc = 48.;
	m.Rdc = 0.1;
	m.Zp = 5;
	m.lambda = blm_Kv_lambda(&m, 525.);
	m.Jm = 2.E-4;

	ts_script_default();
	ts_script_base();
	blm_restart(&m);

	ts_script_speed();
	blm_restart(&m);

	/*ts_script_hfi();
	  blm_restart(&m);*/

	printf("\n---- Turnigy RotoMax 1.20 ----\n");

	tlm_restart();

	m.Rs = 14.E-3;
	m.Ld = 10.E-6;
	m.Lq = 15.E-6;
	m.Udc = 22.;
	m.Rdc = 0.1;
	m.Zp = 14;
	m.lambda = blm_Kv_lambda(&m, 270.);
	m.Jm = 3.E-4;

	ts_script_default();
	ts_script_base();
	blm_restart(&m);

	ts_script_speed();
	blm_restart(&m);

	ts_script_hfi();
	blm_restart(&m);

	ts_script_eabi(PM_EABI_INCREMENTAL);
	blm_restart(&m);

	ts_script_eabi(PM_EABI_ABSOLUTE);
	blm_restart(&m);

	printf("\n---- Hub Motor (250W) ----\n");

	tlm_restart();

	m.Rs = 0.24;
	m.Ld = 520.E-6;
	m.Lq = 650.E-6;
	m.Udc = 48.;
	m.Rdc = 0.5;
	m.Zp = 15;
	m.lambda = blm_Kv_lambda(&m, 15.);
	m.Jm = 6.E-3;

	ts_script_default();
	ts_script_base();
	blm_restart(&m);

	ts_script_speed();
	blm_restart(&m);

	ts_script_weakening();
	blm_restart(&m);

	ts_script_hall();
	blm_restart(&m);

	printf("\n---- QS 138 (3000W) ----\n");

	tlm_restart();

	m.Rs = 4.E-3;
	m.Ld = 31.E-6;
	m.Lq = 44.E-6;
	m.Udc = 48.;
	m.Rdc = 0.1;
	m.Zp = 5;
	m.lambda = blm_Kv_lambda(&m, 58.);
	m.Jm = 15.E-3;

	ts_script_default();
	ts_script_base();
	blm_restart(&m);

	ts_script_speed();
	blm_restart(&m);

	/*ts_script_weakening();
	  blm_restart(&m);*/

	ts_script_hall();
	blm_restart(&m);

	ts_script_sincos();
	blm_restart(&m);
}

