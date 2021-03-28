#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include "blm.h"
#include "lfg.h"
#include "pm.h"
#include "tsfunc.h"

#define TS_TICK_RATE		1000
#define TS_REF_TOLERANCE	0.2

#define TS_xprintf(s)		fprintf(stderr, "failed %s in %s:%i\n", s, __FILE__, __LINE__)
#define TS_assert(x)		if ((x) == 0) { TS_xprintf(#x); exit(1); }
#define TS_assert_ref(x,ref)	TS_assert(fabs((x) - (ref)) / (ref) < TS_REF_TOLERANCE)

int ts_wait_for_IDLE()
{
	int		xTick = 0;

	do {
		sim_Run(10 / (double) TS_TICK_RATE);
		xTick += 10;

		if (pm.fsm_state == PM_STATE_IDLE)
			break;

		if (xTick > 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	return pm.fsm_errno;
}

int ts_wait_for_SPINUP()
{
	const float		wS_tol = 10.f;
	int			xTIME = 0;

	do {
		sim_Run(50 / (double) TS_TICK_RATE);
		xTIME += 50;

		if (pm.fsm_errno != PM_OK)
			break;

		/* Check the target speed has reached.
		 * */
		if (m_fabsf(pm.lu_wS) + wS_tol > pm.s_setpoint_speed)
			break;

		if (		pm.lu_mode == PM_LU_FORCED
				&& pm.vsi_lpf_DC > pm.probe_speed_maximal_pc / 100.f) {

			/* We are unable to keep such a high speed at
			 * this DC link voltage. Stop.
			 * */
			pm.s_setpoint_speed = pm.lu_wS;
			break;
		}

		if (xTIME > 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	sim_Run(100 / (double) TS_TICK_RATE);

	return pm.fsm_errno;
}

int ts_wait_for_MOTION(float s_ref)
{
	int			xTIME = 0;
	int			revol = pm.im_total_revol;

	do {
		sim_Run(50 / (double) TS_TICK_RATE);
		xTIME += 50;

		if (pm.fsm_errno != PM_OK)
			break;

		if (pm.im_total_revol != revol) {

			if (m_fabsf(pm.flux_lpf_wS) > s_ref)
				break;
		}

		if (xTIME > 10000) {

			pm.fsm_errno = PM_ERROR_TIMEOUT;
			break;
		}
	}
	while (1);

	return pm.fsm_errno;
}

void ts_reg_SET_SETPOINT_SPEED()
{
	float		s_pc;

	pm.s_setpoint_speed = pm.probe_speed_hold;

	s_pc = pm.s_setpoint_speed * pm.const_E / (pm.k_EMAX / 100.f * pm.const_fb_U);

	/* We check if speed percentage is higher than maximal allowed.
	 * */
	if (s_pc > pm.probe_speed_maximal_pc) {

		if (pm.const_E > M_EPS_F) {

			pm.s_setpoint_speed = pm.probe_speed_maximal_pc
				* pm.k_EMAX / 100.f
				* pm.const_fb_U / pm.const_E;
		}
	}
}

void ts_self_adjust()
{
	double		tau_A, tau_B, tau_C;

	do {
		printf("U %.3f (V)\n", pm.const_fb_U);

		pm.fsm_req = PM_STATE_ZERO_DRIFT;
		ts_wait_for_IDLE();

		printf("Z [AB] %.3f %.3f (A)\n", pm.ad_IA[0], pm.ad_IB[0]);

		if (pm.fsm_errno != PM_OK)
			break;

		if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_ADJUST_VOLTAGE;
			ts_wait_for_IDLE();

			printf("UA %.4E %.4f (V)\n", pm.ad_UA[1], pm.ad_UA[0]);
			printf("UB %.4E %.4f (V)\n", pm.ad_UB[1], pm.ad_UB[0]);
			printf("UC %.4E %.4f (V)\n", pm.ad_UC[1], pm.ad_UC[0]);

			tau_A = pm.dT / log(pm.tvm_FIR_A[0] / - pm.tvm_FIR_A[1]);
			tau_B = pm.dT / log(pm.tvm_FIR_B[0] / - pm.tvm_FIR_B[1]);
			tau_C = pm.dT / log(pm.tvm_FIR_C[0] / - pm.tvm_FIR_C[1]);

			printf("FIR [A] %.4E %.4E %.4E [tau] %.2f (us)\n", pm.tvm_FIR_A[0],
					pm.tvm_FIR_A[1], pm.tvm_FIR_A[2], tau_A * 1000000.);

			printf("FIR [B] %.4E %.4E %.4E [tau] %.2f (us)\n", pm.tvm_FIR_B[0],
					pm.tvm_FIR_B[1], pm.tvm_FIR_B[2], tau_B * 1000000.);

			printf("FIR [C] %.4E %.4E %.4E [tau] %.2f (us)\n", pm.tvm_FIR_C[0],
					pm.tvm_FIR_C[1], pm.tvm_FIR_C[2], tau_C * 1000000.);

			printf("RMSu %.4f %.4f %.4f (V)\n", pm.self_RMSu[0],
					pm.self_RMSu[1], pm.self_RMSu[2]);

			TS_assert_ref(tau_A, m.tau_U);
			TS_assert_ref(tau_B, m.tau_U);
			TS_assert_ref(tau_C, m.tau_U);

			if (pm.fsm_errno != PM_OK)
				break;
		}

		/* FIXME: Our BLM model is unable to operate in this mode.
		 * */
		/*pm.fsm_req = PM_STATE_ADJUST_CURRENT;
		ts_wait_for_IDLE();

		printf("I [AB] %.4E %.4E\n", pm.ad_IA[1], pm.ad_IB[1]);*/
	}
	while (0);
}

void ts_probe_base()
{
	do {
		/* FIXME: Our BLM model is unable to handle this mode.
		 * */
		/*if (PM_CONFIG_TVM(&pm) == PM_ENABLED) {

			pm.fsm_req = PM_STATE_SELF_TEST_POWER_STAGE;

			if (ts_wait_for_IDLE() != PM_OK)
				break;
		}*/

		pm.fsm_req = PM_STATE_PROBE_CONST_R;

		if (ts_wait_for_IDLE() != PM_OK)
			break;

		pm.const_R = pm.const_im_R;

		printf("R %.4E (Ohm)\n", pm.const_R);

		TS_assert_ref(pm.const_R, m.R);

		pm.fsm_req = PM_STATE_PROBE_CONST_L;

		if (ts_wait_for_IDLE() != PM_OK)
			break;

		printf("L %.4E (H)\n", pm.const_L);
		printf("im_L1 %.4E (H)\n", pm.const_im_L1);
		printf("im_L2 %.4E (H)\n", pm.const_im_L2);
		printf("im_B %.2f (g)\n", pm.const_im_B);
		printf("im_R %.4E (Ohm)\n", pm.const_im_R);

		TS_assert_ref(pm.const_im_L1, m.Ld);
		TS_assert_ref(pm.const_im_L2, m.Lq);

		pm_tune_loop_current(&pm);

		printf("i_gain_P %.2E \n", pm.i_gain_P);
		printf("i_gain_I %.2E \n", pm.i_gain_I);
		printf("i_slew_rate %.1f (A/s)\n", pm.i_slew_rate);
	}
	while (0);
}

void ts_probe_spinup()
{
	float		Kv, Ja_kgm2;

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (ts_wait_for_IDLE() != PM_OK)
			break;

		if (pm.const_E < M_EPS_F) {

			ts_reg_SET_SETPOINT_SPEED();

			if (ts_wait_for_SPINUP() != PM_OK)
				break;

			printf("lpf_wS %.2f (rad/s)\n", pm.flux_lpf_wS);

			pm.fsm_req = PM_STATE_PROBE_CONST_E;

			if (ts_wait_for_IDLE() != PM_OK)
				break;

			Kv = 60. / (2. * M_PI * sqrt(3.)) / (pm.const_E * pm.const_Zp);

			printf("Kv %.2f (rpm/v)\n", Kv);
		}

		ts_reg_SET_SETPOINT_SPEED();

		if (ts_wait_for_SPINUP() != PM_OK)
			break;

		printf("lpf_wS %.2f (rad/s)\n", pm.flux_lpf_wS);

		if (pm.flux_mode != PM_FLUX_HIGH) {

			pm.fsm_errno = PM_ERROR_NO_FLUX_CAUGHT;
			break;
		}

		pm.fsm_req = PM_STATE_PROBE_CONST_E;

		if (ts_wait_for_IDLE() != PM_OK)
			break;

		Kv = 60. / (2. * M_PI * sqrt(3.)) / (pm.const_E * pm.const_Zp);

		printf("Kv %.2f (rpm/v)\n", Kv);

		TS_assert_ref(pm.const_E, m.E);

		pm_tune_MPPE(&pm);

		printf("MPPE %.2f (rad/s)\n", pm.flux_MPPE);
		printf("TAKE_U %.3f (V)\n", pm.flux_gain_TAKE * pm.flux_MPPE * pm.const_E);
		printf("GIVE_U %.3f (V)\n", pm.flux_gain_GIVE * pm.flux_MPPE * pm.const_E);

		pm.fsm_req = PM_STATE_PROBE_CONST_J;

		sim_Run(100 / (double) TS_TICK_RATE);

		pm.s_setpoint_speed = 110.f * pm.k_EMAX / 100.f
				* pm.const_fb_U / pm.const_E;

		sim_Run(300 / (double) TS_TICK_RATE);

		ts_reg_SET_SETPOINT_SPEED();

		sim_Run(300 / (double) TS_TICK_RATE);

		if (ts_wait_for_IDLE() != PM_OK)
			break;

		Ja_kgm2 = pm.const_Ja * 1.5 * pm.const_Zp * pm.const_Zp * pm.const_E;

		printf("Ja %.4E (kgm2) \n", Ja_kgm2);

		TS_assert_ref(Ja_kgm2, m.J);

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (ts_wait_for_IDLE() != PM_OK)
			break;

		pm_tune_forced(&pm);
		pm_tune_loop_speed(&pm);

		printf("maximal %.2f (rad/s)\n", pm.forced_maximal);
		printf("accel %.1f (rad/s2)\n", pm.forced_accel);

		printf("lu_gain_TF %.2E \n", pm.lu_gain_TF);
		printf("s_gain_P %.2E \n", pm.s_gain_P);
	}
	while (0);
}

void ts_adjust_sensor_hall()
{
	double		hall_g;
	int		N;

	do {
		pm.fsm_req = PM_STATE_LU_STARTUP;

		if (ts_wait_for_IDLE() != PM_OK)
			break;

		ts_reg_SET_SETPOINT_SPEED();

		if (ts_wait_for_SPINUP() != PM_OK)
			break;

		pm.fsm_req = PM_STATE_ADJUST_SENSOR_HALL;

		if (ts_wait_for_IDLE() != PM_OK)
			break;

		for (N = 1; N < 7; ++N) {

			hall_g = atan2(pm.hall_ST[N].Y, pm.hall_ST[N].X) * 180. / M_PI;

			printf("hall_ST[%i] %.1f (g)\n", N, hall_g);
		}

		pm.fsm_req = PM_STATE_LU_SHUTDOWN;

		if (ts_wait_for_IDLE() != PM_OK)
			break;
	}
	while (0);
}

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

void ts_BASE()
{
	pm.freq_hz = (float) (1. / m.dT);
	pm.dT = 1.f / pm.freq_hz;
	pm.dc_resolution = m.PWM_R;
	pm.proc_set_DC = &blmDC;
	pm.proc_set_Z = &blmZ;

	pm_default(&pm);

	pm.const_Zp = m.Zp;

	sim_Run(.1);

	ts_self_adjust();
	ts_probe_base();
	ts_probe_spinup();

	sim_Run(.1);
}

void ts_SPEED()
{
	pm.config_DRIVE = PM_DRIVE_SPEED;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_for_IDLE();

	pm.s_setpoint_speed = 0.f;
	pm.s_accel = 300000.f;
	sim_Run(.1);

	m.sync_F = 1;

	pm.s_setpoint_speed = 50.f * pm.k_EMAX / 100.f
			* pm.const_fb_U / pm.const_E;

	ts_wait_for_SPINUP();
	sim_Run(.5);

	m.M[0] = - 1.5 * m.Zp * m.E * 20.f;
	sim_Run(.5);

	m.M[0] = 0.f;
	sim_Run(.5);

	TS_assert_ref(pm.lu_wS, pm.s_setpoint_speed);

	pm.s_setpoint_speed = 0.f;
	sim_Run(.5);

	m.sync_F = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_for_IDLE();
}

void ts_HFI()
{
	pm.config_HFI = PM_ENABLED;
	pm.config_DRIVE = PM_DRIVE_SPEED;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_for_IDLE();

	pm.s_setpoint_speed = 0.f;
	sim_Run(.1);

	m.sync_F = 1;

	pm.s_setpoint_speed = 2.f / m.E;
	sim_Run(.5);

	TS_assert_ref(pm.lu_wS, pm.s_setpoint_speed);

	pm.s_setpoint_speed = 0;
	sim_Run(.5);

	pm.s_setpoint_speed = - 2.f / m.E;
	sim_Run(.5);

	TS_assert_ref(pm.lu_wS, pm.s_setpoint_speed);

	pm.s_setpoint_speed = 0;
	sim_Run(.5);

	m.sync_F = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_for_IDLE();
}

void ts_HALL()
{
	ts_adjust_sensor_hall();

	pm.config_SENSOR = PM_SENSOR_HALL;
}

void ts_WEAK()
{
	pm.config_WEAK = PM_ENABLED;
	pm.config_DRIVE = PM_DRIVE_SPEED;

	pm.fsm_req = PM_STATE_LU_STARTUP;
	ts_wait_for_IDLE();

	pm.s_setpoint_speed = 0.f;
	sim_Run(.1);

	m.sync_F = 1;

	pm.s_setpoint_speed = 200.f * pm.k_EMAX / 100.f
			* pm.const_fb_U / pm.const_E;

	ts_wait_for_SPINUP();

	m.M[0] = - 1.5 * m.Zp * m.E * 5.f;
	sim_Run(.5);

	m.M[0] = 0.f;
	sim_Run(.5);

	TS_assert_ref(pm.lu_wS, pm.s_setpoint_speed);

	pm.s_setpoint_speed = 0.f;
	sim_Run(.5);

	m.sync_F = 0;

	pm.fsm_req = PM_STATE_LU_SHUTDOWN;
	ts_wait_for_IDLE();
}

void ts_START()
{
	blm_Enable(&m);
	blm_Stop(&m);
	sim_TlmDrop();

	printf("\n---- E-scooter Hub Motor (250W) ----\n");

	m.R = 2.4E-1;
	m.Ld = 5.2E-4;
	m.Lq = 6.5E-4;
	m.U = 48.;
	m.Rs = 0.5;
	m.Zp = 15;
        m.E = 60. / 2. / M_PI / sqrt(3.) / (15.7 * m.Zp);
	m.J = 6.2E-3;

	ts_BASE();
	blm_Stop(&m);

	ts_HALL();
	blm_Stop(&m);

	ts_SPEED();
	blm_Stop(&m);

	ts_WEAK();

	//exit(0);

	blm_Stop(&m);
	sim_TlmDrop();

	printf("\n---- Turnigy RotoMax 1.20 ----\n");

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

	ts_SPEED();
	blm_Stop(&m);

	ts_HFI();
	blm_Stop(&m);
}

