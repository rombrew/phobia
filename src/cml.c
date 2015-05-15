/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "hal/hal.h"
#include "lib.h"
#include "sh.h"
#include "pmc.h"
#include "task.h"
#include "tel.h"

static const char
MSG_ENABLED[] = "Enabled",
MSG_DISABLED[] = "Disabled";

void uptime(const char *s)
{
	int		Day, Hour, Min, Sec;

	Sec = td.uSEC;

	Day = Sec / 86400;
	Sec -= Day * 86400;
	Hour = Sec / 3600;
	Sec -= Hour * 3600;
	Min = Sec / 60;
	Sec -= Min * 60;

	printf("%id %ih %im %is" EOL,
			Day, Hour, Min, Sec);
}

void irqload(const char *s)
{
	int		Tirq, Tbase, Rpc;

	Tbase = 2 * halPWM.resolution;
	Tirq = td.Tirq;
	Rpc = 100 * Tirq / Tbase;

	printf("%i%% (%i/%i)" EOL, Rpc, Tirq, Tbase);
}

void reboot(const char *s)
{
	int		End, Del = 3;

	printf("Reboot in %i second" EOL, Del);

	End = td.uSEC + Del;

	do {
		taskIOMUX();
	}
	while (td.uSEC < End);

	halReset();
}

void pwm_freq_hz(const char *s)
{
	stoi(&halPWM.freq_hz, s);
	printf("%i (Hz)" EOL, halPWM.freq_hz);
}

void pwm_dead_time_ns(const char *s)
{
	stoi(&halPWM.dead_time_ns, s);
	printf("%i (ns)" EOL, halPWM.dead_time_ns);
}

void pm_pwm_resolution(const char *s)
{
	printf("%i" EOL, pm.pwm_resolution);
}

void pm_pwm_minimal_pulse(const char *s)
{
	float		conv_G;
	int		ns;

	conv_G = 1e-9f * pm.freq_hz * pm.pwm_resolution;

	if (stoi(&ns, s) != NULL)
		pm.pwm_minimal_pulse = (int) (ns * conv_G + .5f);

	ns = (int) (pm.pwm_minimal_pulse / conv_G + .5f);

	printf("%i (ns)" EOL, ns);
}

void pm_m_request_zero_drift(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_ZERO_DRIFT;
	}
}

void pm_m_request_wave_hold(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_WAVE_HOLD;
	}
}

void pm_m_request_wave_sine(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_WAVE_SINE;
	}
}

void pm_m_request_calibration(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_CALIBRATION;
	}
}

void pm_m_request_estimate_Q(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_ESTIMATE_Q;
	}
}

void pm_m_request_kalman_start(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_KALMAN_START;
	}
}

void pm_m_request_kalman_stop(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_KALMAN_STOP;
	}
}

static void
pm_m_bitmask(const char *s, int bit)
{
	int		flag;

	if (stoi(&flag, s) != NULL) {

		if (flag)
			pm.m_bitmask |= bit;
		else
			pm.m_bitmask &= ~bit;
	}

	flag = (pm.m_bitmask & bit) ? 1 : 0;

	printf("%i (%s)" EOL, flag, flag ? MSG_ENABLED : MSG_DISABLED);
}

void pm_m_bitmask_qaxis_drift(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_QAXIS_DRIFT);
}

void pm_m_bitmask_abzero_drift(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_ABZERO_DRIFT);
}

void pm_m_bitmask_efficient_modulation(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_EFFICIENT_MODULATION);
}

void pm_m_bitmask_speed_control_loop(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_SPEED_CONTROL_LOOP);
}

void pm_m_bitmask_direct_injection(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_DIRECT_INJECTION);
}

void pm_m_bitmask_frequency_injection(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_FREQUENCY_INJECTION);
}

void pm_m_bitmask_hold_update_R(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_HOLD_UPDATE_R);
}

void pm_m_bitmask_sine_update_L(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_SINE_UPDATE_L);
}

void pm_m_bitmask_estimate_R(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_ESTIMATE_R);
}

void pm_m_bitmask_estimate_E(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_ESTIMATE_E);
}

void pm_m_bitmask_estimate_Q(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_ESTIMATE_Q);
}

void pm_m_errno(const char *s)
{
	printf("%i" EOL, pm.m_errno);
}

void pm_T_drift(const char *s)
{
	stof(&pm.T_drift, s);
	printf("%3f (Sec)" EOL, &pm.T_drift);
}

void pm_T_hold(const char *s)
{
	stof(&pm.T_hold, s);
	printf("%3f (Sec)" EOL, &pm.T_hold);
}

void pm_T_rohm(const char *s)
{
	stof(&pm.T_rohm, s);
	printf("%3f (Sec)" EOL, &pm.T_rohm);
}

void pm_T_sine(const char *s)
{
	stof(&pm.T_sine, s);
	printf("%3f (Sec)" EOL, &pm.T_sine);
}

void pm_T_estq(const char *s)
{
	stof(&pm.T_estq, s);
	printf("%3f (Sec)" EOL, &pm.T_estq);
}

void pm_T_end(const char *s)
{
	stof(&pm.T_end, s);
	printf("%3f (Sec)" EOL, &pm.T_end);
}

void pm_i_hold(const char *s)
{
	stof(&pm.i_hold, s);
	printf("%3f (A)" EOL, &pm.i_hold);
}

void pm_i_sine(const char *s)
{
	stof(&pm.i_sine, s);
	printf("%3f (A)" EOL, &pm.i_sine);
}

void pm_i_offset_D(const char *s)
{
	stof(&pm.i_offset_D, s);
	printf("%3f (A)" EOL, &pm.i_offset_D);
}

void pm_i_offset_Q(const char *s)
{
	stof(&pm.i_offset_Q, s);
	printf("%3f (A)" EOL, &pm.i_offset_Q);
}

void pm_freq_sine_hz(const char *s)
{
	stof(&pm.freq_sine_hz, s);
	printf("%1f (Hz)" EOL, &pm.freq_sine_hz);
}

void pm_conv_A0(const char *s)
{
	stof(&pm.conv_A[0], s);
	printf("%3f (A)" EOL, &pm.conv_A[0]);
}

void pm_conv_A1(const char *s)
{
	stof(&pm.conv_A[1], s);
	printf("%4e" EOL, &pm.conv_A[1]);
}

void pm_conv_B0(const char *s)
{
	stof(&pm.conv_B[0], s);
	printf("%3f (A)" EOL, &pm.conv_B[0]);
}

void pm_conv_B1(const char *s)
{
	stof(&pm.conv_B[1], s);
	printf("%4e" EOL, &pm.conv_B[1]);
}

void pm_conv_U0(const char *s)
{
	stof(&pm.conv_U[0], s);
	printf("%3f (V)" EOL, &pm.conv_U[0]);
}

void pm_conv_U1(const char *s)
{
	stof(&pm.conv_U[1], s);
	printf("%4e" EOL, &pm.conv_U[1]);
}

void pm_residual_variance(const char *s)
{
	printf("%4e" EOL, &pm.residual_variance);
}

void pm_kalman_X0(const char *s)
{
	printf("%3f (A)" EOL, &pm.kalman_X[0]);
}

void pm_kalman_X1(const char *s)
{
	printf("%3f (A)" EOL, &pm.kalman_X[1]);
}

void pm_kalman_X23(const char *s)
{
	printf("%3f %3f" EOL, &pm.kalman_X[2], &pm.kalman_X[3]);
}

void pm_kalman_X4(const char *s)
{
	float			RPM;

	RPM = 9.5492969f * pm.kalman_X[4] / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM) " EOL, &pm.kalman_X[4], &RPM);
}

void pm_kalman_Q0(const char *s)
{
	stof(&pm.kalman_Q[0], s);
	printf("%2e" EOL, &pm.kalman_Q[0]);
}

void pm_kalman_Q1(const char *s)
{
	stof(&pm.kalman_Q[1], s);
	printf("%2e" EOL, &pm.kalman_Q[1]);
}

void pm_kalman_Q2(const char *s)
{
	stof(&pm.kalman_Q[2], s);
	printf("%2e" EOL, &pm.kalman_Q[2]);
}

void pm_kalman_Q3(const char *s)
{
	stof(&pm.kalman_Q[3], s);
	printf("%2e" EOL, &pm.kalman_Q[3]);
}

void pm_kalman_Q4(const char *s)
{
	stof(&pm.kalman_Q[4], s);
	printf("%2e" EOL, &pm.kalman_Q[4]);
}

void pm_kalman_Q5(const char *s)
{
	stof(&pm.kalman_Q[5], s);
	printf("%2e" EOL, &pm.kalman_Q[5]);
}

void pm_kalman_R(const char *s)
{
	stof(&pm.kalman_R, s);
	printf("%2e" EOL, &pm.kalman_R);
}

void pm_var_M(const char *s)
{
	printf("%4e (Nm)" EOL, &pm.var_M);
}

void pm_drift_A(const char *s)
{
	stof(&pm.drift_A, s);
	printf("%3f (A)" EOL, &pm.drift_A);
}

void pm_drift_B(const char *s)
{
	stof(&pm.drift_B, s);
	printf("%3f (A)" EOL, &pm.drift_B);
}

void pm_drift_Q(const char *s)
{
	stof(&pm.drift_Q, s);
	printf("%3f (V)" EOL, &pm.drift_Q);
}

void pm_drift_AB_maximal(const char *s)
{
	stof(&pm.drift_AB_maximal, s);
	printf("%3f (A)" EOL, &pm.drift_AB_maximal);
}

void pm_drift_Q_maximal(const char *s)
{
	stof(&pm.drift_Q_maximal, s);
	printf("%3f (V)" EOL, &pm.drift_Q_maximal);
}

void pm_const_U(const char *s)
{
	stof(&pm.const_U, s);
	printf("%3f (V)" EOL, &pm.const_U);
}

void pm_const_E_wb(const char *s)
{
	float		const_Kv;

	stof(&pm.const_E, s);
	const_Kv = 5.513289f / (pm.const_E * pm.const_Zp);

	printf("%4e (Wb) %1f (RPM/V)" EOL, &pm.const_E, &const_Kv);
}

void pm_const_E_kv(const char *s)
{
	float		const_Kv, Z;

	if (stof(&const_Kv, s) != NULL) {

		Z = const_Kv * pm.const_Zp;
		pm.const_E = (Z != 0.f) ? 5.513289f / Z : pm.const_E;
	}

	const_Kv = 5.513289f / (pm.const_E * pm.const_Zp);

	printf("%4e (Wb) %1f (RPM/V)" EOL, &pm.const_E, &const_Kv);
}

void pm_const_R(const char *s)
{
	stof(&pm.const_R, s);
	printf("%4e (Ohm)" EOL, &pm.const_R);
}

void pm_const_Q(const char *s)
{
	stof(&pm.const_Q, s);
	printf("%4e (Ohm)" EOL, &pm.const_Q);
}

void pm_const_Ld(const char *s)
{
	if (stof(&pm.const_Ld, s) != NULL)
		pm.const_ILd = 1.f / pm.const_Ld;

	printf("%4e (H)" EOL, &pm.const_Ld);
}

void pm_const_Lq(const char *s)
{
	if (stof(&pm.const_Lq, s) != NULL)
		pm.const_ILq = 1.f / pm.const_Lq;

	printf("%4e (H)" EOL, &pm.const_Lq);
}

void pm_const_Zp(const char *s)
{
	stoi(&pm.const_Zp, s);
	printf("%i" EOL, pm.const_Zp);
}

void pm_const_J(const char *s)
{
	float		const_J;

	if (stof(&const_J, s) != NULL)
		pm.const_IJ = (const_J != 0.f) ? 1.f / const_J : pm.const_IJ;

	const_J = 1.f / pm.const_IJ;

	printf("%4e (kgm2)" EOL, &const_J);
}

void pm_i_maximal(const char *s)
{
	stof(&pm.i_maximal, s);
	printf("%3f (A)" EOL, &pm.i_maximal);
}

void pm_i_power_watt(const char *s)
{
	printf("%1f (W)" EOL, &pm.i_power_watt);
}

void pm_i_power_consumption_maximal(const char *s)
{
	stof(&pm.i_power_consumption_maximal, s);
	printf("%1f (W)" EOL, &pm.i_power_consumption_maximal);
}

void pm_i_power_regeneration_maximal(const char *s)
{
	stof(&pm.i_power_regeneration_maximal, s);
	printf("%1f (W)" EOL, &pm.i_power_regeneration_maximal);
}

void pm_i_set_point_D(const char *s)
{
	stof(&pm.i_set_point_D, s);
	printf("%3f (A)" EOL, &pm.i_set_point_D);
}

void pm_i_set_point_Q(const char *s)
{
	stof(&pm.i_set_point_Q, s);
	printf("%3f (A)" EOL, &pm.i_set_point_Q);
}

void pm_i_KP(const char *s)
{
	stof(&pm.i_KP, s);
	printf("%2e" EOL, &pm.i_KP);
}

void pm_i_KI(const char *s)
{
	stof(&pm.i_KI, s);
	printf("%2e" EOL, &pm.i_KI);
}

void pm_i_inject_D(const char *s)
{
	stof(&pm.i_inject_D, s);
	printf("%3f (A)" EOL, &pm.i_inject_D);
}

void pm_i_inject_gain(const char *s)
{
	stof(&pm.i_inject_gain, s);
	printf("%2e" EOL, &pm.i_inject_gain);
}

void pm_h_freq_hz(const char *s)
{
	stof(&pm.h_freq_hz, s);
	printf("%1f (Hz)" EOL, &pm.h_freq_hz);
}

void pm_h_inject_D(const char *s)
{
	stof(&pm.h_inject_D, s);
	printf("%1f (Hz)" EOL, &pm.h_inject_D);
}

void pm_w_low_threshold(const char *s)
{
	float			RPM;

	stof(&pm.w_low_threshold, s);
	RPM = 9.5492969f * pm.w_low_threshold / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM) " EOL, &pm.w_low_threshold, &RPM);
}

void pm_w_low_hysteresis(const char *s)
{
	float			RPM;

	stof(&pm.w_low_hysteresis, s);
	RPM = 9.5492969f * pm.w_low_hysteresis / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM) " EOL, &pm.w_low_hysteresis, &RPM);
}

void pm_w_set_point_rpm(const char *s)
{
	float			RPM;

	if (stof(&RPM, s) != NULL)
		pm.w_set_point = .10471976f * RPM * pm.const_Zp;

	RPM = 9.5492969f * pm.w_set_point / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM) " EOL, &pm.w_set_point, &RPM);
}

void pm_w_KP(const char *s)
{
	stof(&pm.w_KP, s);
	printf("%2e" EOL, &pm.w_KP);
}

void tel_setup(const char *s)
{
	tel.s_clock_scale = 1;

	stoi(&tel.s_clock_scale, s);

	tel.pZ = tel.pD;
	tel.s_clock = 0;
}

void tel_enable(const char *s)
{
	tel.enabled = 1;
}

void tel_print(const char *s)
{
	tel_flush();
}

const shCMD_t		cmList[] = {

	{"uptime", &uptime},
	{"irqload", &irqload},
	{"reboot", &reboot},

	{"pwm_freq_hz", &pwm_freq_hz},
	{"pwm_dead_time_ns", &pwm_dead_time_ns},

	{"pm_pwm_resolution", &pm_pwm_resolution},
	{"pm_pwm_minimal_pulse", &pm_pwm_minimal_pulse},

	{"pm_m_request_zero_drift", &pm_m_request_zero_drift},
	{"pm_m_request_wave_hold", &pm_m_request_wave_hold},
	{"pm_m_request_wave_sine", &pm_m_request_wave_sine},
	{"pm_m_request_calibration", &pm_m_request_calibration},
	{"pm_m_request_estimate_Q", &pm_m_request_estimate_Q},
	{"pm_m_request_kalman_start", &pm_m_request_kalman_start},
	{"pm_m_request_kalman_stop", &pm_m_request_kalman_stop},

	{"pm_m_bitmask_qaxis_drift", &pm_m_bitmask_qaxis_drift},
	{"pm_m_bitmask_abzero_drift", &pm_m_bitmask_abzero_drift},
	{"pm_m_bitmask_efficient_modulation", &pm_m_bitmask_efficient_modulation},
	{"pm_m_bitmask_speed_control_loop", &pm_m_bitmask_speed_control_loop},
	{"pm_m_bitmask_direct_injection", &pm_m_bitmask_direct_injection},
	{"pm_m_bitmask_frequency_injection", &pm_m_bitmask_frequency_injection},
	{"pm_m_bitmask_hold_update_R", &pm_m_bitmask_hold_update_R},
	{"pm_m_bitmask_sine_update_L", &pm_m_bitmask_sine_update_L},
	{"pm_m_bitmask_estimate_R", &pm_m_bitmask_estimate_R},
	{"pm_m_bitmask_estimate_E", &pm_m_bitmask_estimate_E},
	{"pm_m_bitmask_estimate_Q", &pm_m_bitmask_estimate_Q},

	{"pm_m_errno", &pm_m_errno},

	{"pm_T_drift", &pm_T_drift},
	{"pm_T_hold", &pm_T_hold},
	{"pm_T_rohm", &pm_T_rohm},
	{"pm_T_sine", &pm_T_sine},
	{"pm_T_estq", &pm_T_estq},
	{"pm_T_end", &pm_T_end},

	{"pm_i_hold", &pm_i_hold},
	{"pm_i_sine", &pm_i_sine},
	{"pm_i_offset_D", &pm_i_offset_D},
	{"pm_i_offset_Q", &pm_i_offset_Q},
	{"pm_freq_sine_hz", &pm_freq_sine_hz},

	{"pm_conv_A0", &pm_conv_A0},
	{"pm_conv_A1", &pm_conv_A1},
	{"pm_conv_B0", &pm_conv_B0},
	{"pm_conv_B1", &pm_conv_B1},
	{"pm_conv_U0", &pm_conv_U0},
	{"pm_conv_U1", &pm_conv_U1},

	{"pm_residual_variance", &pm_residual_variance},

	{"pm_kalman_X0", &pm_kalman_X0},
	{"pm_kalman_X1", &pm_kalman_X1},
	{"pm_kalman_X23", &pm_kalman_X23},
	{"pm_kalman_X4", &pm_kalman_X4},

	{"pm_kalman_Q0", &pm_kalman_Q0},
	{"pm_kalman_Q1", &pm_kalman_Q1},
	{"pm_kalman_Q2", &pm_kalman_Q2},
	{"pm_kalman_Q3", &pm_kalman_Q3},
	{"pm_kalman_Q4", &pm_kalman_Q4},
	{"pm_kalman_Q5", &pm_kalman_Q5},
	{"pm_kalman_R", &pm_kalman_R},

	{"pm_var_M", &pm_var_M},

	{"pm_drift_A", &pm_drift_A},
	{"pm_drift_B", &pm_drift_B},
	{"pm_drift_Q", &pm_drift_Q},
	{"pm_drift_AB_maximal", &pm_drift_AB_maximal},
	{"pm_drift_Q_maximal", &pm_drift_Q_maximal},

	{"pm_const_U", &pm_const_U},
	{"pm_const_E_wb", &pm_const_E_wb},
	{"pm_const_E_kv", &pm_const_E_kv},
	{"pm_const_R", &pm_const_R},
	{"pm_const_Q", &pm_const_Q},
	{"pm_const_Ld", &pm_const_Ld},
	{"pm_const_Lq", &pm_const_Lq},
	{"pm_const_Zp", &pm_const_Zp},
	{"pm_const_J", &pm_const_J},

	{"pm_i_maximal", &pm_i_maximal},
	{"pm_i_power_watt", &pm_i_power_watt},
	{"pm_i_power_consumption_maximal", &pm_i_power_consumption_maximal},
	{"pm_i_power_regeneration_maximal", &pm_i_power_regeneration_maximal},
	{"pm_i_set_point_D", &pm_i_set_point_D},
	{"pm_i_set_point_Q", &pm_i_set_point_Q},
	{"pm_i_KP", &pm_i_KP},
	{"pm_i_KI", &pm_i_KI},

	{"pm_i_inject_D", &pm_i_inject_D},
	{"pm_i_inject_gain", &pm_i_inject_gain},

	{"pm_h_freq_hz", &pm_h_freq_hz},
	{"pm_h_inject_D", &pm_h_inject_D},

	{"pm_w_low_threshold", &pm_w_low_threshold},
	{"pm_w_low_hysteresis", &pm_w_low_hysteresis},
	{"pm_w_set_point_rpm", &pm_w_set_point_rpm},
	{"pm_w_KP", &pm_w_KP},

	{"tel_setup", &tel_setup},
	{"tel_enable", &tel_enable},
	{"tel_print", &tel_print},

	{NULL, NULL},
};

