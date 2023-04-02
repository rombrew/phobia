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

int ts_wait_for_idle()
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

int ts_wait_for_spinup(float ref)
{
	int			xTIME = 0;

	do {
		sim_runtime(50 / (double) TS_TICK_RATE);

		if (pm.fsm_errno != PM_OK)
			break;

		/* Check the target speed has reached.
		 * */
		if (m_fabsf(pm.lu_wS) + 10.f > ref)
			break;

		if (		pm.lu_MODE == PM_LU_FORCED
				&& pm.vsi_lpf_DC > pm.forced_maximal_DC) {

			ref = pm.lu_wS;
			break;
		}

		if (xTIME > 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}

		xTIME += 50;
	}
	while (1);

	sim_runtime(100 / (double) TS_TICK_RATE);

	return pm.fsm_errno;
}

int ts_wait_for_motion(float ref)
{
	int			xTIME = 0;
	int			revob = pm.lu_total_revol;

	do {
		sim_runtime(50 / (double) TS_TICK_RATE);

		if (pm.fsm_errno != PM_OK)
			break;

		if (pm.lu_total_revol != revob) {

			if (m_fabsf(pm.zone_lpf_wS) > ref)
				break;
		}

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
	double		tau_A, tau_B, tau_C;

	do {
		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		ts_wait_for_idle();

		printf("const_fb_U %.3f (V)\n", pm.const_fb_U);

		printf("ad_IABC0 %.3f %.3f %.3f (A)\n", pm.ad_IA[0],
				pm.ad_IB[0], pm.ad_IC[0]);

		printf("self_STDi %.3f %.3f %.3f (A)\n", pm.self_STDi[0],
				pm.self_STDi[1], pm.self_STDi[2]);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_ADJUST_VOLTAGE;
			ts_wait_for_idle();

			printf("ad_UA %.4E %.4f (V)\n", pm.ad_UA[1], pm.ad_UA[0]);
			printf("ad_UB %.4E %.4f (V)\n", pm.ad_UB[1], pm.ad_UB[0]);
			printf("ad_UC %.4E %.4f (V)\n", pm.ad_UC[1], pm.ad_UC[0]);

			tau_A = pm.m_dT / log(pm.tvm_FIR_A[0] / - pm.tvm_FIR_A[1]);
			tau_B = pm.m_dT / log(pm.tvm_FIR_B[0] / - pm.tvm_FIR_B[1]);
			tau_C = pm.m_dT / log(pm.tvm_FIR_C[0] / - pm.tvm_FIR_C[1]);

			printf("tau_A %.2f (us)\n", tau_A * 1000000.);
			printf("tau_B %.2f (us)\n", tau_B * 1000000.);
			printf("tau_C %.2f (us)\n", tau_C * 1000000.);

			printf("self_RMSu %.4f %.4f %.4f (V)\n", pm.self_RMSu[1],
					pm.self_RMSu[2], pm.self_RMSu[3]);

			TS_assert_relative(tau_A, m.tau_U);
			TS_assert_relative(tau_B, m.tau_U);
			TS_assert_relative(tau_C, m.tau_U);

			if (pm.fsm_errno != PM_OK)
				break;
		}
	}
	while (0);
}

void ts_probe_base()
{
	do {
		pm.fsm_req = PM_STATE_PROBE_CONST_R;

		if (ts_wait_for_idle() != PM_OK)
			break;

		pm.const_R = pm.const_im_R;

		printf("const_R %.4E (Ohm)\n", pm.const_R);

		TS_assert_relative(pm.const_R, m.R);

		pm.fsm_req = PM_STATE_PROBE_CONST_L;

		if (ts_wait_for_idle() != PM_OK)
			break;

		printf("const_im_L1 %.4E (H)\n", pm.const_im_L1);
		printf("const_im_L2 %.4E (H)\n", pm.const_im_L2);
		printf("const_im_B %.2f (g)\n", pm.const_im_B);
		printf("const_im_R %.4E (Ohm)\n", pm.const_im_R);

		TS_assert_relative(pm.const_im_L1, m.Ld);
		TS_assert_relative(pm.const_im_L2, m.Lq);

		pm_auto(&pm, PM_AUTO_MAXIMAL_CURRENT);
		pm_auto(&pm, PM_AUTO_LOOP_CURRENT);

		printf("i_maixmal %.3f (A) \n", pm.i_maximal);
		printf("i_gain_P %.2E \n", pm.i_gain_P);
		printf("i_gain_I %.2E \n", pm.i_gain_I);
		printf("i_slew_rate %.1f (A/s)\n", pm.i_slew_rate);
	}
	while (0);
}

void ts_probe_spinup()
{
	float		Kv;

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (ts_wait_for_idle() != PM_OK)
			break;

		if (pm.const_E < M_EPS_F) {

			pm.s_setpoint_speed = pm.probe_speed_hold;

			if (ts_wait_for_spinup(pm.probe_speed_hold) != PM_OK)
				break;

			printf("zone_lpf_wS %.2f (rad/s)\n", pm.zone_lpf_wS);

			pm.fsm_req = PM_STATE_PROBE_CONST_E;

			if (ts_wait_for_idle() != PM_OK)
				break;

			Kv = 60. / (2. * M_PI * sqrt(3.)) / (pm.const_E * pm.const_Zp);

			printf("const_E %.4E (Wb) %.2f (rpm/v)\n", pm.const_E, Kv);

			pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);
			pm_auto(&pm, PM_AUTO_PROBE_SPEED_HOLD);
			pm_auto(&pm, PM_AUTO_FORCED_MAXIMAL);

			printf("probe_speed_hold %.2f (rad/s)\n", pm.probe_speed_hold);

			printf("zone_threshold_NOISE %.2f (rad/s) %.3f (V)\n",
					pm.zone_threshold_NOISE,
					pm.zone_threshold_NOISE * pm.const_E);

			printf("zone_threshold_BASE %.2f (rad/s) %.3f (V)\n",
					pm.zone_threshold_BASE,
					pm.zone_threshold_BASE * pm.const_E);
		}

		pm.s_setpoint_speed = pm.probe_speed_hold;

		if (ts_wait_for_spinup(pm.probe_speed_hold) != PM_OK)
			break;

		printf("zone_lpf_wS %.2f (rad/s)\n", pm.zone_lpf_wS);

		if (pm.flux_ZONE != PM_ZONE_HIGH) {

			pm.fsm_errno = PM_ERROR_NO_FLUX_CAUGHT;
			break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (ts_wait_for_idle() != PM_OK)
			break;

		Kv = 60. / (2. * M_PI * sqrt(3.)) / (pm.const_E * pm.const_Zp);

		printf("const_E %.4E (Wb) %.2f (rpm/v)\n", pm.const_E, Kv);

		TS_assert_relative(pm.const_E, m.E);

		pm.fsm_req = PM_STATE_PROBE_NOISE_THRESHOLD;

		if (ts_wait_for_idle() != PM_OK)
			break;

		pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);

		printf("zone_threshold_NOISE %.2f (rad/s) %.3f (V)\n",
				pm.zone_threshold_NOISE,
				pm.zone_threshold_NOISE * pm.const_E);

		printf("zone_threshold_BASE %.2f (rad/s) %.3f (V)\n",
				pm.zone_threshold_BASE,
				pm.zone_threshold_BASE * pm.const_E);

		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		sim_runtime(100 / (double) TS_TICK_RATE);

		pm.s_setpoint_speed = 110.f * pm.k_EMAX / 100.f
				* pm.const_fb_U / pm.const_E;

		sim_runtime(300 / (double) TS_TICK_RATE);

		pm.s_setpoint_speed = pm.probe_speed_hold;

		sim_runtime(300 / (double) TS_TICK_RATE);

		if (ts_wait_for_idle() != PM_OK)
			break;

		printf("const_Ja %.4E (kgm2) \n", pm.const_Ja * pm.const_Zp * pm.const_Zp);

		TS_assert_relative(pm.const_Ja * pm.const_Zp * pm.const_Zp, m.J);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (ts_wait_for_idle() != PM_OK)
			break;

		pm_auto(&pm, PM_AUTO_FORCED_MAXIMAL);
		pm_auto(&pm, PM_AUTO_FORCED_ACCEL);
		pm_auto(&pm, PM_AUTO_LOOP_SPEED);

		printf("forced_maximal %.2f (rad/s)\n", pm.forced_maximal);
		printf("forced_accel %.1f (rad/s2)\n", pm.forced_accel);
		printf("lu_gain_mq_LP %.2E\n", pm.lu_gain_mq_LP);
		printf("s_gain_P %.2E\n", pm.s_gain_P);
		printf("s_gain_I %.2E\n", pm.s_gain_I);
	}
	while (0);
}

void ts_adjust_sensor_hall()
{
	int		N, ACTIVE = 0;
	double		ST;

	do {
		if (pm.lu_MODE == PM_LU_DISABLED) {

			pm.fsm_req = PM_STATE_LU_STARTUP;

			if (ts_wait_for_idle() != PM_OK)
				break;

			pm.s_setpoint_speed = pm.probe_speed_hold;

			if (ts_wait_for_spinup(pm.probe_speed_hold) != PM_OK)
				break;

			ACTIVE = 1;
		}

		pm.fsm_req = PM_STATE_ADJUST_SENSOR_HALL;

		if (ts_wait_for_idle() != PM_OK)
			break;

		for (N = 1; N < 7; ++N) {

			ST = atan2(pm.hall_ST[N].Y, pm.hall_ST[N].X) * (180. / M_PI);

			printf("hall_ST[%i] %.1f (g)\n", N, ST);
		}

		if (ACTIVE != 0) {

			pm.fsm_req = PM_STATE_LU_SHUTDOWN;

			if (ts_wait_for_idle() != PM_OK)
				break;
		}
	}
	while (0);
}

static void
blm_proc_DC(int A, int B, int C)
{
	m.PWM_A = A;
	m.PWM_B = B;
	m.PWM_C = C;
}

static void
blm_proc_Z(int Z)
{
	if (Z != 0) {

		m.HI_Z = 1;
	}
	else {
		m.HI_Z = 0;
	}
}

void ts_script_base()
{
	pm.m_freq = (float) (1. / m.dT);
	pm.m_dT = 1.f / pm.m_freq;
	pm.dc_resolution = m.PWM_R;
	pm.proc_set_DC = &blm_proc_DC;
	pm.proc_set_Z = &blm_proc_Z;

	pm_auto(&pm, PM_AUTO_BASIC_DEFAULT);
	pm_auto(&pm, PM_AUTO_CONFIG_DEFAULT);

	pm.const_Zp = m.Zp;

	sim_runtime(.001);

	ts_self_adjust();
	ts_probe_base();
	ts_probe_spinup();

	sim_runtime(.1);
}

void ts_script_speed()
{
	pm.config_LU_DRIVE = PM_DRIVE_SPEED;
	pm.s_accel = 300000.f;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_for_idle();

	sim_runtime(.1);

	m.sync_F = 1;

	pm.s_setpoint_speed = 50.f * pm.k_EMAX / 100.f
			* pm.const_fb_U / pm.const_E;

	ts_wait_for_spinup(pm.s_setpoint_speed);
	sim_runtime(.5);

	m.M[0] = - 1.5 * m.Zp * m.E * 20.f;
	sim_runtime(.5);

	m.M[0] = 0.f;
	sim_runtime(.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 0.f;
	sim_runtime(.5);

	m.sync_F = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_for_idle();
}

void ts_script_hfi()
{
	pm.config_LU_ESTIMATE = PM_FLUX_KALMAN;
	pm.config_LU_DRIVE = PM_DRIVE_SPEED;
	pm.config_HFI_WAVETYPE = PM_HFI_SINE;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_for_idle();

	sim_runtime(.5);

	TS_assert(pm.lu_MODE == PM_LU_ON_HFI);

	m.sync_F = 1;

	pm.s_setpoint_speed = 2.f / m.E;
	sim_runtime(.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 0;
	sim_runtime(.5);

	pm.s_setpoint_speed = - 2.f / m.E;
	sim_runtime(.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 0;
	sim_runtime(.5);

	m.sync_F = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_for_idle();

	pm.config_HFI_WAVETYPE = PM_HFI_NONE;
}

void ts_script_weakening()
{
	pm.config_WEAKENING = PM_ENABLED;
	pm.config_LU_DRIVE = PM_DRIVE_SPEED;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_for_idle();

	sim_runtime(.1);

	m.sync_F = 1;

	pm.s_setpoint_speed = 200.f * pm.k_EMAX / 100.f
			* pm.const_fb_U / pm.const_E;

	ts_wait_for_spinup(pm.s_setpoint_speed);

	m.M[0] = - 1.5 * m.Zp * m.E * 5.f;
	sim_runtime(.5);

	m.M[0] = 0.f;
	sim_runtime(.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 0.f;
	sim_runtime(.5);

	m.sync_F = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_for_idle();
}

void ts_script_hall()
{
	ts_adjust_sensor_hall();

	pm.config_LU_ESTIMATE = PM_FLUX_NONE;
	pm.config_LU_SENSOR = PM_SENSOR_HALL;

	pm.s_gain_P *= .5f;
	pm.s_gain_I *= .5f;
}

void ts_script_abi()
{
	//ts_adjust_sensor_abi();

	pm.config_LU_ESTIMATE = PM_FLUX_NONE;
	pm.config_LU_SENSOR = PM_SENSOR_ABI;
	pm.config_ABI_FRONTEND = PM_ABI_INCREMENTAL;
	pm.abi_USEABLE = PM_ENABLED;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_for_idle();

	sim_runtime(.5);

	TS_assert(pm.lu_MODE == PM_LU_SENSOR_ABI);

	m.sync_F = 1;

	pm.s_setpoint_speed = 50.f * pm.k_EMAX / 100.f
			* pm.const_fb_U / pm.const_E;

	ts_wait_for_spinup(pm.s_setpoint_speed);
	sim_runtime(.5);

	m.M[0] = - 1.5 * m.Zp * m.E * 20.f;
	sim_runtime(.5);

	m.M[0] = 0.f;
	sim_runtime(.5);

	TS_assert_absolute(pm.lu_wS, pm.s_setpoint_speed, 50.);

	pm.s_setpoint_speed = 0.f;
	sim_runtime(.5);

	m.sync_F = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_for_idle();

	pm.config_LU_ESTIMATE = PM_FLUX_ORTEGA;
	pm.config_LU_SENSOR = PM_SENSOR_NONE;
}

void ts_script_all()
{
	blm_enable(&m);
	blm_stop(&m);

	printf("\n---- XNOVA Lightning 4530 ----\n");

	sim_tlmdrop();

	m.R = 7E-3;
	m.Ld = 2E-6;
	m.Lq = 5E-6;
	m.U = 48.;
	m.Rs = 0.1;
	m.Zp = 5;
        m.E = blm_Kv_to_E(&m, 525.);
	m.J = 2E-4;

	ts_script_base();
	blm_stop(&m);

	ts_script_speed();
	blm_stop(&m);

	ts_script_hfi();
	blm_stop(&m);

	printf("\n---- Turnigy RotoMax 1.20 ----\n");

	sim_tlmdrop();

	m.R = 14E-3;
	m.Ld = 10E-6;
	m.Lq = 15E-6;
	m.U = 22.;
	m.Rs = 0.1;
	m.Zp = 14;
        m.E = blm_Kv_to_E(&m, 270.);
	m.J = 3E-4;

	ts_script_base();
	blm_stop(&m);

	ts_script_speed();
	blm_stop(&m);

	ts_script_hfi();
	blm_stop(&m);

	ts_script_abi();
	blm_stop(&m);

	printf("\n---- E-scooter Hub Motor (250W) ----\n");

	sim_tlmdrop();

	m.R = 2.4E-1;
	m.Ld = 5.2E-4;
	m.Lq = 6.5E-4;
	m.U = 48.;
	m.Rs = 0.5;
	m.Zp = 15;
        m.E = blm_Kv_to_E(&m, 15.7);
	m.J = 6E-3;

	ts_script_base();
	blm_stop(&m);

	ts_script_hall();
	blm_stop(&m);

	ts_script_speed();
	blm_stop(&m);

	pm.config_LU_ESTIMATE = PM_FLUX_ORTEGA;
	pm.config_LU_SENSOR = PM_SENSOR_NONE;

	ts_script_weakening();
	blm_stop(&m);
}

