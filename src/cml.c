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
#include "m.h"
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
	float		scal_G;
	int		ns;

	scal_G = 1e-9f * pm.freq_hz * pm.pwm_resolution;

	if (stoi(&ns, s) != NULL)
		pm.pwm_minimal_pulse = (int) (ns * scal_G + .5f);

	ns = (int) (pm.pwm_minimal_pulse / scal_G + .5f);

	printf("%i (ns)" EOL, ns);
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

void pm_m_bitmask_high_frequency_injection(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_HIGH_FREQUENCY_INJECTION);
}

void pm_m_bitmask_fewer_switching_modulation(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_FEWER_SWITCHING_MODULATION);
}

void pm_m_bitmask_position_control_loop(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_POSITION_CONTROL_LOOP);
}

void pm_m_bitmask_update_R_after_hold(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_UPDATE_R_AFTER_HOLD);
}

void pm_m_bitmask_update_L_after_sine(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_UPDATE_L_AFTER_SINE);
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

void pm_T_sine(const char *s)
{
	stof(&pm.T_sine, s);
	printf("%3f (Sec)" EOL, &pm.T_sine);
}

void pm_T_measure(const char *s)
{
	stof(&pm.T_measure, s);
	printf("%3f (Sec)" EOL, &pm.T_measure);
}

void pm_T_end(const char *s)
{
	stof(&pm.T_end, s);
	printf("%3f (Sec)" EOL, &pm.T_end);
}

void pm_wave_i_hold(const char *s)
{
	stof(&pm.wave_i_hold, s);
	printf("%3f (A)" EOL, &pm.wave_i_hold);
}

void pm_wave_i_sine(const char *s)
{
	stof(&pm.wave_i_sine, s);
	printf("%3f (A)" EOL, &pm.wave_i_sine);
}

void pm_wave_i_offset_D(const char *s)
{
	stof(&pm.wave_i_offset_D, s);
	printf("%3f (A)" EOL, &pm.wave_i_offset_D);
}

void pm_wave_i_offset_Q(const char *s)
{
	stof(&pm.wave_i_offset_Q, s);
	printf("%3f (A)" EOL, &pm.wave_i_offset_Q);
}

void pm_wave_freq_sine_hz(const char *s)
{
	stof(&pm.wave_freq_sine_hz, s);
	printf("%1f (Hz)" EOL, &pm.wave_freq_sine_hz);
}

void pm_scal_A0(const char *s)
{
	stof(&pm.scal_A[0], s);
	printf("%3f (A)" EOL, &pm.scal_A[0]);
}

void pm_scal_A1(const char *s)
{
	stof(&pm.scal_A[1], s);
	printf("%4e" EOL, &pm.scal_A[1]);
}

void pm_scal_B0(const char *s)
{
	stof(&pm.scal_B[0], s);
	printf("%3f (A)" EOL, &pm.scal_B[0]);
}

void pm_scal_B1(const char *s)
{
	stof(&pm.scal_B[1], s);
	printf("%4e" EOL, &pm.scal_B[1]);
}

void pm_scal_U0(const char *s)
{
	stof(&pm.scal_U[0], s);
	printf("%3f (V)" EOL, &pm.scal_U[0]);
}

void pm_scal_U1(const char *s)
{
	stof(&pm.scal_U[1], s);
	printf("%4e" EOL, &pm.scal_U[1]);
}

void pm_lu_X0(const char *s)
{
	printf("%3f (A)" EOL, &pm.lu_X[0]);
}

void pm_lu_X1(const char *s)
{
	printf("%3f (A)" EOL, &pm.lu_X[1]);
}

void pm_lu_X23(const char *s)
{
	printf("%3f %3f" EOL, &pm.lu_X[2], &pm.lu_X[3]);
}

void pm_lu_X4(const char *s)
{
	float			RPM;

	RPM = 9.5492969f * pm.lu_X[4] / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM) " EOL, &pm.lu_X[4], &RPM);
}

void pm_lu_gain_K0(const char *s)
{
	stof(&pm.lu_gain_K[0], s);
	printf("%2e" EOL, &pm.lu_gain_K[0]);
}

void pm_lu_gain_K1(const char *s)
{
	stof(&pm.lu_gain_K[1], s);
	printf("%2e" EOL, &pm.lu_gain_K[1]);
}

void pm_lu_gain_K2(const char *s)
{
	stof(&pm.lu_gain_K[2], s);
	printf("%2e" EOL, &pm.lu_gain_K[2]);
}

void pm_lu_gain_K3(const char *s)
{
	stof(&pm.lu_gain_K[3], s);
	printf("%2e" EOL, &pm.lu_gain_K[3]);
}

void pm_lu_gain_K4(const char *s)
{
	stof(&pm.lu_gain_K[4], s);
	printf("%2e" EOL, &pm.lu_gain_K[4]);
}

void pm_lu_gain_K5(const char *s)
{
	stof(&pm.lu_gain_K[5], s);
	printf("%2e" EOL, &pm.lu_gain_K[5]);
}

void pm_lu_gain_K6(const char *s)
{
	stof(&pm.lu_gain_K[6], s);
	printf("%2e" EOL, &pm.lu_gain_K[6]);
}

void pm_lu_gain_K7(const char *s)
{
	stof(&pm.lu_gain_K[7], s);
	printf("%2e" EOL, &pm.lu_gain_K[7]);
}

void pm_lu_low_threshold(const char *s)
{
	float			RPM;

	stof(&pm.lu_low_threshold, s);
	RPM = 9.5492969f * pm.lu_low_threshold / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM) " EOL, &pm.lu_low_threshold, &RPM);
}

void pm_lu_low_hysteresis(const char *s)
{
	float			RPM;

	stof(&pm.lu_low_hysteresis, s);
	RPM = 9.5492969f * pm.lu_low_hysteresis / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM) " EOL, &pm.lu_low_hysteresis, &RPM);
}

void pm_lu_residual_variance(const char *s)
{
	printf("%4e" EOL, &pm.lu_residual_variance);
}

void pm_hf_freq_hz(const char *s)
{
	stof(&pm.hf_freq_hz, s);
	printf("%1f (Hz)" EOL, &pm.hf_freq_hz);
}

void pm_hf_swing_D(const char *s)
{
	stof(&pm.hf_swing_D, s);
	printf("%3f (A)" EOL, &pm.hf_swing_D);
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

void pm_const_U_gain_F(const char *s)
{
	stof(&pm.const_U_gain_F, s);
	printf("%2e" EOL, &pm.const_U_gain_F);
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

void pm_const_Ld(const char *s)
{
	if (stof(&pm.const_Ld, s) != NULL)
		pm.const_Ld_inversed = 1.f / pm.const_Ld;

	printf("%4e (H)" EOL, &pm.const_Ld);
}

void pm_const_Lq(const char *s)
{
	if (stof(&pm.const_Lq, s) != NULL)
		pm.const_Lq_inversed = 1.f / pm.const_Lq;

	printf("%4e (H)" EOL, &pm.const_Lq);
}

void pm_const_Zp(const char *s)
{
	stoi(&pm.const_Zp, s);
	printf("%i" EOL, pm.const_Zp);
}

void pm_i_maximal(const char *s)
{
	stof(&pm.i_maximal, s);
	printf("%3f (A)" EOL, &pm.i_maximal);
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

void pm_i_slew_rate_D(const char *s)
{
	stof(&pm.i_slew_rate_D, s);
	printf("%3f (A)" EOL, &pm.i_slew_rate_D);
}

void pm_i_slew_rate_Q(const char *s)
{
	stof(&pm.i_slew_rate_Q, s);
	printf("%3f (A)" EOL, &pm.i_slew_rate_Q);
}

void pm_i_slew_rate_auto(const char *s)
{
	pm.i_slew_rate_D = .2f * pm.const_U / pm.const_Ld;
	pm.i_slew_rate_Q = .2f * pm.const_U / pm.const_Lq;

	printf(	"D %3f (A)" EOL
		"D %3f (A)" EOL,
		&pm.i_slew_rate_D,
		&pm.i_slew_rate_Q);
}

void pm_i_gain_P_D(const char *s)
{
	stof(&pm.i_gain_P_D, s);
	printf("%2e" EOL, &pm.i_gain_P_D);
}

void pm_i_gain_I_D(const char *s)
{
	stof(&pm.i_gain_I_D, s);
	printf("%2e" EOL, &pm.i_gain_I_D);
}

void pm_i_gain_P_Q(const char *s)
{
	stof(&pm.i_gain_P_Q, s);
	printf("%2e" EOL, &pm.i_gain_P_Q);
}

void pm_i_gain_I_Q(const char *s)
{
	stof(&pm.i_gain_I_Q, s);
	printf("%2e" EOL, &pm.i_gain_I_Q);
}

void pm_i_gain_auto(const char *s)
{
	pm.i_gain_P_D = (.5f * pm.const_Ld * pm.freq_hz - pm.const_R);
	pm.i_gain_I_D = 5e-2f * pm.const_Ld * pm.freq_hz;
	pm.i_gain_P_Q = (.5f * pm.const_Lq * pm.freq_hz - pm.const_R);
	pm.i_gain_I_Q = 5e-2f * pm.const_Lq * pm.freq_hz;

	printf(	"P_D %2e" EOL
		"I_D %2e" EOL
		"P_Q %2e" EOL
		"I_Q %2e" EOL,
		&pm.i_gain_P_D,
		&pm.i_gain_I_D,
		&pm.i_gain_P_Q,
		&pm.i_gain_I_Q);
}

void pm_p_set_point_x_g(const char *s)
{
	float			angle, g;
	int			revol;

	if (stof(&g, s) != NULL) {

		revol = (int) (g / 360.f);
		g -= (float) (revol * 360);

		if (g <= - 180.f) {

			revol -= 1;
			g += 360.f;
		}

		if (g >= 180.f) {

			revol += 1;
			g -= 360.f;
		}

		angle = g * (MPIF / 180.f);
		pm.p_set_point_x[0] = kcosf(angle);
		pm.p_set_point_x[1] = ksinf(angle);
		pm.p_set_point_revol = revol;
	}

	angle = arctanf(pm.p_set_point_x[1], pm.p_set_point_x[0]);
	g = angle * (180.f / MPIF) + (float) pm.p_set_point_revol * 360.f;

	printf("%2f (degree)" EOL, &g);
}

void pm_p_set_point_w_rpm(const char *s)
{
	float			RPM;

	if (stof(&RPM, s) != NULL)
		pm.p_set_point_w = .10471976f * RPM * pm.const_Zp;

	RPM = 9.5492969f * pm.p_set_point_w / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM)" EOL, &pm.p_set_point_w, &RPM);
}

void pm_p_gain_D(const char *s)
{
	stof(&pm.p_gain_D, s);
	printf("%2e" EOL, &pm.p_gain_D);
}

void pm_p_gain_P(const char *s)
{
	stof(&pm.p_gain_P, s);
	printf("%2e" EOL, &pm.p_gain_P);
}

void pm_i_power_watt(const char *s)
{
	printf("%1f (W)" EOL, &pm.i_power_watt);
}

void pm_request_zero_drift(const char *s)
{
	pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
}

void pm_request_wave_hold(const char *s)
{
	pmc_request(&pm, PMC_STATE_WAVE_HOLD);
}

void pm_request_wave_sine(const char *s)
{
	pmc_request(&pm, PMC_STATE_WAVE_SINE);
}

void pm_request_calibration(const char *s)
{
	pmc_request(&pm, PMC_STATE_CALIBRATION);
}

void pm_request_start(const char *s)
{
	pmc_request(&pm, PMC_STATE_START);
}

void pm_request_stop(const char *s)
{
	pmc_request(&pm, PMC_STATE_STOP);
}

static void
irq_telemetry_1()
{
	if (tel.iEN) {

		tel.pIN[0] = (short int) (pm.lu_X[0] * 1000.f);
		tel.pIN[1] = (short int) (pm.lu_X[1] * 1000.f);
		tel.pIN[2] = (short int) (pm.lu_X[2] * 1000.f);
		tel.pIN[3] = (short int) (pm.lu_X[3] * 1000.f);
		tel.pSZ = 4;

		telCapture();
	}
	else
		td.pIRQ = NULL;
}

void tel_start(const char *s)
{
	if (td.pIRQ == NULL) {

		tel.sDEC = 1;
		stoi(&tel.sDEC, s);

		tel.pZ = tel.pD;
		tel.sCNT = 0;
		tel.iEN = 1;

		halWFI();

		td.pIRQ = &irq_telemetry_1;
	}
}

void tel_flush(const char *s)
{
	telFlush();
}

static void
irq_avg_value_4()
{
	if (td.avgN < td.avgMAX) {

		td.avgSUM[0] += *td.avgIN[0];
		td.avgSUM[1] += *td.avgIN[1];
		td.avgSUM[2] += *td.avgIN[2];
		td.avgSUM[3] += *td.avgIN[3];

		td.avgN++;
	}
	else
		td.pIRQ = NULL;
}

void ap_update_const_E(const char *s)
{
	if (td.pIRQ == NULL && pm.lu_region != PMC_LU_DISABLED) {

		td.avgIN[0] = &pm.lu_X[0];
		td.avgIN[1] = &pm.lu_X[1];
		td.avgIN[2] = &pm.lu_X[4];
		td.avgIN[3] = &pm.drift_Q;

		td.avgSUM[0] = 0.f;
		td.avgSUM[1] = 0.f;
		td.avgSUM[2] = 0.f;
		td.avgSUM[3] = 0.f;

		td.avgN = 0;
		td.avgMAX = pm.freq_hz * pm.T_measure;

		td.pIRQ = &irq_avg_value_4;

		while (td.pIRQ != NULL)
			taskIOMUX();

		td.avgSUM[0] /= (float) td.avgN;
		td.avgSUM[1] /= (float) td.avgN;
		td.avgSUM[2] /= (float) td.avgN;
		td.avgSUM[3] /= (float) td.avgN;

		pm.const_E -= td.avgSUM[3] / td.avgSUM[2];
	}
}

void codes()
{
	int		j, c;

	for (j = 0; j < 5; ++j) {

		while ((c = shRecv()) < 0) 
			taskIOMUX();

		printf("-- %i" EOL, c);
	}
}

const shCMD_t		cmList[] = {

	{"uptime", &uptime},
	{"irqload", &irqload},
	{"reboot", &reboot},

	{"pwm_freq_hz", &pwm_freq_hz},
	{"pwm_dead_time_ns", &pwm_dead_time_ns},

	{"pm_pwm_resolution", &pm_pwm_resolution},
	{"pm_pwm_minimal_pulse", &pm_pwm_minimal_pulse},

	{"pm_m_bitmask_high_frequency_injection", &pm_m_bitmask_high_frequency_injection},
	{"pm_m_bitmask_fewer_switching_modulation", &pm_m_bitmask_fewer_switching_modulation},
	{"pm_m_bitmask_position_control_loop", &pm_m_bitmask_position_control_loop},
	{"pm_m_bitmask_update_R_after_hold", &pm_m_bitmask_update_R_after_hold},
	{"pm_m_bitmask_update_L_after_sine", &pm_m_bitmask_update_L_after_sine},

	{"pm_m_errno", &pm_m_errno},

	{"pm_T_drift", &pm_T_drift},
	{"pm_T_hold", &pm_T_hold},
	{"pm_T_sine", &pm_T_sine},
	{"pm_T_measure", &pm_T_measure},
	{"pm_T_end", &pm_T_end},

	{"pm_wave_i_hold", &pm_wave_i_hold},
	{"pm_wave_i_sine", &pm_wave_i_sine},
	{"pm_wave_i_offset_D", &pm_wave_i_offset_D},
	{"pm_wave_i_offset_Q", &pm_wave_i_offset_Q},
	{"pm_wave_freq_sine_hz", &pm_wave_freq_sine_hz},

	{"pm_scal_A0", &pm_scal_A0},
	{"pm_scal_A1", &pm_scal_A1},
	{"pm_scal_B0", &pm_scal_B0},
	{"pm_scal_B1", &pm_scal_B1},
	{"pm_scal_U0", &pm_scal_U0},
	{"pm_scal_U1", &pm_scal_U1},

	{"pm_lu_X0", &pm_lu_X0},
	{"pm_lu_X1", &pm_lu_X1},
	{"pm_lu_X23", &pm_lu_X23},
	{"pm_lu_X4", &pm_lu_X4},

	{"pm_lu_gain_K0", &pm_lu_gain_K0},
	{"pm_lu_gain_K1", &pm_lu_gain_K1},
	{"pm_lu_gain_K2", &pm_lu_gain_K2},
	{"pm_lu_gain_K3", &pm_lu_gain_K3},
	{"pm_lu_gain_K4", &pm_lu_gain_K4},
	{"pm_lu_gain_K5", &pm_lu_gain_K5},
	{"pm_lu_gain_K6", &pm_lu_gain_K6},
	{"pm_lu_gain_K7", &pm_lu_gain_K7},
	{"pm_lu_low_threshold", &pm_lu_low_threshold},
	{"pm_lu_low_hysteresis", &pm_lu_low_hysteresis},
	{"pm_lu_residual_variance", &pm_lu_residual_variance},

	{"pm_hf_freq_hz", &pm_hf_freq_hz},
	{"pm_hf_swing_D", &pm_hf_swing_D},

	{"pm_drift_A", &pm_drift_A},
	{"pm_drift_B", &pm_drift_B},
	{"pm_drift_Q", &pm_drift_Q},
	{"pm_drift_AB_maximal", &pm_drift_AB_maximal},
	{"pm_drift_Q_maximal", &pm_drift_Q_maximal},

	{"pm_const_U", &pm_const_U},
	{"pm_const_E_wb", &pm_const_E_wb},
	{"pm_const_E_kv", &pm_const_E_kv},
	{"pm_const_R", &pm_const_R},
	{"pm_const_Ld", &pm_const_Ld},
	{"pm_const_Lq", &pm_const_Lq},
	{"pm_const_Zp", &pm_const_Zp},

	{"pm_i_maximal", &pm_i_maximal},
	{"pm_i_power_consumption_maximal", &pm_i_power_consumption_maximal},
	{"pm_i_power_regeneration_maximal", &pm_i_power_regeneration_maximal},
	{"pm_i_set_point_D", &pm_i_set_point_D},
	{"pm_i_set_point_Q", &pm_i_set_point_Q},
	{"pm_i_slew_rate_D",&pm_i_slew_rate_D},
	{"pm_i_slew_rate_Q",&pm_i_slew_rate_Q},
	{"pm_i_slew_rate_auto", &pm_i_slew_rate_auto},
	{"pm_i_gain_P_D", &pm_i_gain_P_D},
	{"pm_i_gain_I_D", &pm_i_gain_I_D},
	{"pm_i_gain_P_Q", &pm_i_gain_P_Q},
	{"pm_i_gain_I_Q", &pm_i_gain_I_Q},
	{"pm_i_gain_auto", &pm_i_gain_auto},

	{"pm_p_set_point_x_g", &pm_p_set_point_x_g},
	{"pm_p_set_point_w_rpm", &pm_p_set_point_w_rpm},
	{"pm_p_gain_D", &pm_p_gain_D},
	{"pm_p_gain_P", &pm_p_gain_P},

	{"pm_i_power_watt", &pm_i_power_watt},

	{"pm_request_zero_drift", &pm_request_zero_drift},
	{"pm_request_wave_hold", &pm_request_wave_hold},
	{"pm_request_wave_sine", &pm_request_wave_sine},
	{"pm_request_calibration", &pm_request_calibration},
	{"pm_request_start", &pm_request_start},
	{"pm_request_stop", &pm_request_stop},

	{"tel_start", &tel_start},
	{"tel_flush", &tel_flush},

	{"ap_update_const_E", &ap_update_const_E},

	{"codes", &codes},

	{NULL, NULL},
};

