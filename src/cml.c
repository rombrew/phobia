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
#include "ap.h"
#include "lib.h"
#include "sh.h"
#include "pmc.h"
#include "m.h"
#include "task.h"
#include "tel.h"

void td_uptime(const char *s)
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

void td_irqload(const char *s)
{
	int		Tirq, Tbase, Rpc;

	Tbase = 2 * halPWM.resolution;
	Tirq = td.Tirq;
	Rpc = 100 * Tirq / Tbase;

	printf("%i %% (%i/%i)" EOL, Rpc, Tirq, Tbase);
}

void td_avg_default_time(const char *s)
{
	stof(&td.avg_default_time, s);
	printf("%3f (Sec)" EOL, &td.avg_default_time);
}

void td_reboot(const char *s)
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

void td_keycodes(const char *s)
{
	int		xC;

	do {
		while ((xC = shRecv()) < 0)
			taskIOMUX();

		if (xC == 3 || xC == 4)
			break;

		printf("-- %i" EOL, xC);
	}
	while (1);
}

void pwm_freq_hz(const char *s)
{
	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	if (stoi(&halPWM.freq_hz, s) != NULL) {

		pwmDisable();
		pwmEnable();

		pm.freq_hz = (float) halPWM.freq_hz;
		pm.dT = 1.f / pm.freq_hz;
		pm.pwm_resolution = halPWM.resolution;
	}

	printf("%i (Hz)" EOL, halPWM.freq_hz);
}

void pwm_dead_time_ns(const char *s)
{
	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	if (stoi(&halPWM.dead_time_ns, s) != NULL) {

		pwmDisable();
		pwmEnable();
	}

	printf("%i (tk) %i (ns)" EOL, halPWM.dead_time_tk, halPWM.dead_time_ns);
}

void pwm_DC(const char *s)
{
	const char	*tok;
	int		tokN = 0;
	int		xA, xB, xC, R;

	tok = s;
	s = stoi(&xA, tok);
	tokN += (s != NULL) ? 1 : 0;

	tok = strtok(tok, " ");
	s = stoi(&xB, tok);
	tokN += (s != NULL) ? 1 : 0;

	tok = strtok(tok, " ");
	s = stoi(&xC, tok);
	tokN += (s != NULL) ? 1 : 0;

	if (tokN == 3) {

		R = halPWM.resolution;

		xA = (xA < 0) ? 0 : (xA > R) ? R : xA;
		xB = (xB < 0) ? 0 : (xB > R) ? R : xB;
		xC = (xC < 0) ? 0 : (xC > R) ? R : xC;

		pwmDC(xA, xB, xC);

		printf("DC %i %i %i" EOL, xA, xB, xC);
	}
}

void pwm_Z(const char *s)
{
	int		Z;

	if (stoi(&Z, s) != NULL) {

		pwmZ(Z);

		printf("Z %i" EOL, Z);
	}
}

void pm_pwm_resolution(const char *s)
{
	printf("%i" EOL, pm.pwm_resolution);
}

static void
pm_pwm_int_modify(int *param, const char *s)
{
	float		scal_G;
	int		ns;

	scal_G = 1e-9f * pm.freq_hz * pm.pwm_resolution;

	if (stoi(&ns, s) != NULL)
		*param = (int) (ns * scal_G + .5f);

	ns = (int) (*param / scal_G + .5f);

	printf("%i (tk) %i (ns)" EOL, *param, ns);
}

void pm_pwm_minimal_pulse(const char *s)
{
	pm_pwm_int_modify(&pm.pwm_minimal_pulse, s);
}

void pm_pwm_dead_time(const char *s)
{
	pm_pwm_int_modify(&pm.pwm_dead_time, s);
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

	printf("%i (%s)" EOL, flag, flag ? "Enabled" : "Disabled");
}

void pm_m_bitmask_direct_current_injection(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_DIRECT_CURRENT_INJECTION);
}

void pm_m_bitmask_high_frequency_injection(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_HIGH_FREQUENCY_INJECTION);
}

void pm_m_bitmask_flux_polarity_detection(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_FLUX_POLARITY_DETECTION);
}

void pm_m_bitmask_servo_control_loop(const char *s)
{
	pm_m_bitmask(s, PMC_BIT_SERVO_CONTROL_LOOP);
}

void pm_m_errno(const char *s)
{
	int		flag;

	if (stoi(&flag, s) != NULL) {

		if (flag == 0)
			pm.m_errno = PMC_OK;
	}

	printf("%i: %s" EOL, pm.m_errno, pmc_strerror(pm.m_errno));
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

void pm_wave_i_hold_X(const char *s)
{
	stof(&pm.wave_i_hold_X, s);
	printf("%3f (A)" EOL, &pm.wave_i_hold_X);
}

void pm_wave_i_hold_Y(const char *s)
{
	stof(&pm.wave_i_hold_Y, s);
	printf("%3f (A)" EOL, &pm.wave_i_hold_Y);
}

void pm_wave_i_sine(const char *s)
{
	stof(&pm.wave_i_sine, s);
	printf("%3f (A)" EOL, &pm.wave_i_sine);
}

void pm_wave_freq_sine_hz(const char *s)
{
	stof(&pm.wave_freq_sine_hz, s);
	printf("%1f (Hz)" EOL, &pm.wave_freq_sine_hz);
}

void pm_wave_gain_P(const char *s)
{
	stof(&pm.wave_gain_P, s);
	printf("%2e" EOL, &pm.wave_gain_P);
}

void pm_wave_gain_I(const char *s)
{
	stof(&pm.wave_gain_I, s);
	printf("%2e" EOL, &pm.wave_gain_I);
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

void pm_fault_iab_maximal(const char *s)
{
	stof(&pm.fault_iab_maximal, s);
	printf("%3f (A)" EOL, &pm.fault_iab_maximal);
}

void pm_fault_residual_maximal(const char *s)
{
	stof(&pm.fault_residual_maximal, s);
	printf("%4e" EOL, &pm.fault_residual_maximal);
}

void pm_fault_drift_maximal(const char *s)
{
	stof(&pm.fault_drift_maximal, s);
	printf("%3f (A)" EOL, &pm.fault_drift_maximal);
}

void pm_fault_low_voltage(const char *s)
{
	stof(&pm.fault_low_voltage, s);
	printf("%3f (V)" EOL, &pm.fault_low_voltage);
}

void pm_fault_high_voltage(const char *s)
{
	stof(&pm.fault_high_voltage, s);
	printf("%3f (V)" EOL, &pm.fault_high_voltage);
}

void pm_vsi_u_maximal(const char *s)
{
	stof(&pm.vsi_u_maximal, s);
	printf("%4e" EOL, &pm.vsi_u_maximal);
}

void irq_avg_value_8()
{
	int			j;

	if (td.avg_N <= td.avg_MAX) {

		for (j = 0; j < td.avg_K; ++j)
			td.avg_SUM[j] += *td.avg_IN[j];

		td.avg_N++;
	}
	else
		td.pIRQ = NULL;
}

static float
pm_avg_float_1(float *param, float time)
{
	td.avg_IN[0] = param;
	td.avg_SUM[0] = 0.f;
	td.avg_K = 1;
	td.avg_N = 0;
	td.avg_MAX = pm.freq_hz * time;

	halWFI();

	td.pIRQ = &irq_avg_value_8;

	while (td.pIRQ != NULL)
		taskIOMUX();

	td.avg_SUM[0] /= (float) td.avg_N;

	return td.avg_SUM[0];
}

static float
pm_avg_float_arg_1(float *param, const char *s)
{
	float			time = td.avg_default_time;

	stof(&time, s);

	return pm_avg_float_1(param, time);
}

void pm_lu_X0(const char *s)
{
	float			avg;

	avg = pm_avg_float_arg_1(&pm.lu_X[0], s);
	printf("%3f (A)" EOL, &avg);
}

void pm_lu_X1(const char *s)
{
	float			avg;

	avg = pm_avg_float_arg_1(&pm.lu_X[1], s);
	printf("%3f (A)" EOL, &avg);
}

void pm_lu_X23(const char *s)
{
	float			g;

	g = matan2f(pm.lu_X[3], pm.lu_X[2]) * 180.f / MPIF;
	printf("%1f (degree) [%3f %3f]" EOL, &g, &pm.lu_X[2], &pm.lu_X[3]);
}

void pm_lu_X4(const char *s)
{
	float			avg, RPM;

	avg = pm_avg_float_arg_1(&pm.lu_X[4], s);
	RPM = 9.5492969f * avg / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM) " EOL, &avg, &RPM);
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

void pm_lu_threshold_low(const char *s)
{
	float			RPM;

	stof(&pm.lu_threshold_low, s);
	RPM = 9.5492969f * pm.lu_threshold_low / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM)" EOL, &pm.lu_threshold_low, &RPM);
}

void pm_lu_threshold_high(const char *s)
{
	float			RPM;

	stof(&pm.lu_threshold_high, s);
	RPM = 9.5492969f * pm.lu_threshold_high / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM)" EOL, &pm.lu_threshold_high, &RPM);
}

void pm_lu_threshold_auto(const char *s)
{
	pm.lu_threshold_low = 1E-1f / pm.const_E;
	pm.lu_threshold_high = pm.lu_threshold_low + 1E+2f;

	printf(	"%4e (Rad/S)" EOL
		"%4e (Rad/S)" EOL,
		&pm.lu_threshold_low,
		&pm.lu_threshold_high);
}

void pm_lu_low_D(const char *s)
{
	stof(&pm.lu_low_D, s);
	printf("%3f (A)" EOL, &pm.lu_low_D);
}

void pm_lu_residual_variance(const char *s)
{
	float			avg;

	avg = pm_avg_float_arg_1(&pm.lu_residual_variance, s);
	printf("%4e" EOL, &avg);
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

void pm_hf_gain_K0(const char *s)
{
	stof(&pm.hf_gain_K[0], s);
	printf("%2e" EOL, &pm.hf_gain_K[0]);
}

void pm_hf_gain_K1(const char *s)
{
	stof(&pm.hf_gain_K[1], s);
	printf("%2e" EOL, &pm.hf_gain_K[1]);
}

void pm_hf_gain_K2(const char *s)
{
	stof(&pm.hf_gain_K[2], s);
	printf("%2e" EOL, &pm.hf_gain_K[2]);
}

void pm_thermal_R(const char *s)
{
	float			avg;

	avg = pm_avg_float_arg_1(&pm.thermal_R, s);
	printf("%4e" EOL, &avg);
}

void pm_thermal_E(const char *s)
{
	float			avg;

	avg = pm_avg_float_arg_1(&pm.thermal_E, s);
	printf("%4e" EOL, &avg);
}

void pm_thermal_gain_R0(const char *s)
{
	stof(&pm.thermal_gain_R[0], s);
	printf("%1f" EOL, &pm.thermal_gain_R[0]);
}

void pm_thermal_gain_R1(const char *s)
{
	stof(&pm.thermal_gain_R[1], s);
	printf("%4e" EOL, &pm.thermal_gain_R[1]);
}

void pm_drift_A(const char *s)
{
	float			avg;

	avg = pm_avg_float_arg_1(&pm.drift_A, s);
	printf("%3f (A)" EOL, &avg);
}

void pm_drift_B(const char *s)
{
	float			avg;

	avg = pm_avg_float_arg_1(&pm.drift_B, s);
	printf("%3f (A)" EOL, &avg);
}

void pm_drift_Q(const char *s)
{
	float			avg;

	avg = pm_avg_float_arg_1(&pm.drift_Q, s);
	printf("%3f (V)" EOL, &avg);
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
	float			avg;

	avg = pm_avg_float_arg_1(&pm.const_U, s);
	printf("%3f (V)" EOL, &avg);
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

void pm_i_high_maximal(const char *s)
{
	stof(&pm.i_high_maximal, s);
	printf("%3f (A)" EOL, &pm.i_high_maximal);
}

void pm_i_low_maximal(const char *s)
{
	stof(&pm.i_low_maximal, s);
	printf("%3f (A)" EOL, &pm.i_low_maximal);
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
	printf("%2e (A/Sec)" EOL, &pm.i_slew_rate_D);
}

void pm_i_slew_rate_Q(const char *s)
{
	stof(&pm.i_slew_rate_Q, s);
	printf("%2e (A/Sec)" EOL, &pm.i_slew_rate_Q);
}

void pm_i_slew_rate_auto(const char *s)
{
	pm.i_slew_rate_D = .2f * pm.const_U / pm.const_Ld;
	pm.i_slew_rate_Q = .2f * pm.const_U / pm.const_Lq;

	printf(	"D %1f (A/Sec)" EOL
		"Q %1f (A/Sec)" EOL,
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

void pm_p_set_point_w_rpm(const char *s)
{
	float			RPM;

	if (stof(&RPM, s) != NULL)
		pm.p_set_point_w = .10471976f * RPM * pm.const_Zp;

	RPM = 9.5492969f * pm.p_set_point_w / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM)" EOL, &pm.p_set_point_w, &RPM);
}

void pm_p_track_point_x_g(const char *s)
{
	float			angle, g;
	int			revol, full;

	if (stof(&g, s) != NULL) {

		revol = (int) (g / 360.f);
		g -= (float) (revol * 360);

		if (stoi(&full, strtok(s, " ")) != NULL)
			revol += full;

		if (g <= - 180.f) {

			revol -= 1;
			g += 360.f;
		}

		if (g >= 180.f) {

			revol += 1;
			g -= 360.f;
		}

		angle = g * (MPIF / 180.f);
		pm.p_track_point_x[0] = mcosf(angle);
		pm.p_track_point_x[1] = msinf(angle);
		pm.p_track_point_revol = revol;
	}

	angle = matan2f(pm.p_track_point_x[1], pm.p_track_point_x[0]);
	g = angle * (180.f / MPIF);

	printf("%1f (degree) + %i (full revolutions)" EOL, &g, pm.p_track_point_revol);
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

void pm_lp_gain_0(const char *s)
{
	stof(&pm.lp_gain[0], s);
	printf("%2e" EOL, &pm.lp_gain[0]);
}

void pm_lp_gain_1(const char *s)
{
	stof(&pm.lp_gain[1], s);
	printf("%2e" EOL, &pm.lp_gain[1]);
}

void pm_n_power_watt(const char *s)
{
	float			avg;

	avg = pm_avg_float_arg_1(&pm.n_power_watt, s);
	printf("%1f (W)" EOL, &avg);
}

void pm_n_temperature_c(const char *s)
{
	float			avg;

	avg = pm_avg_float_arg_1(&pm.n_temperature_c, s);
	printf("%1f (C)" EOL, &avg);
}

void pm_default(const char *s)
{
	if (pm.lu_region == PMC_LU_DISABLED)
		pmc_default(&pm);
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

void pm_update_const_R(const char *s)
{
	float			R;

	if (pm.m_state == PMC_STATE_IDLE) {

		R = 0.f;

		pmc_resistance(pm.wave_DFT, &R);
		pm.const_R += R;

		pm.wave_DFT[0] = 0.f;
		pm.wave_DFT[1] = 0.f;

		printf("%4e (Ohm)" EOL, &pm.const_R);
	}
}

void pm_impedance(const char *s)
{
	float		IMP[6];

	pmc_impedance(pm.wave_DFT, pm.wave_freq_sine_hz, IMP);

	printf(	"L1 %4e (H)" EOL
		"L2 %4e (H)" EOL
		"E %1f (degree)" EOL
		"R1 %4e (Ohm)" EOL
		"R2 %4e (Ohm)" EOL
		"E %1f (degree)" EOL,
		IMP + 0, IMP + 1, IMP + 2, IMP + 3, IMP + 4, IMP + 5);
}

void pm_update_const_L(const char *s)
{
	float		IMP[6];

	if (pm.m_state == PMC_STATE_IDLE) {

		pmc_impedance(pm.wave_DFT, pm.wave_freq_sine_hz, IMP);

		if (fabsf(IMP[2]) < 45.f) {

			pm.const_Ld = IMP[0];
			pm.const_Lq = IMP[1];
		}
		else {
			pm.const_Ld = IMP[1];
			pm.const_Lq = IMP[0];
		}

		pm.const_Ld_inversed = 1.f / pm.const_Ld;
		pm.const_Lq_inversed = 1.f / pm.const_Lq;

		printf("Ld %4e (H)" EOL, &pm.const_Ld);
		printf("Lq %4e (H)" EOL, &pm.const_Lq);
	}
}

void pm_update_scal_AB(const char *s)
{
	float		gainA, gainB, fixA, fixB, ref;

	if (pm.m_state == PMC_STATE_IDLE) {

		if (stof(&ref, s) != NULL && ref != 0.f) {

			gainA = pm.wave_DFT[0] / ref;
			gainB = pm.wave_DFT[1] / ref;

			fixA = 100.f * (gainA - 1.f);
			fixB = 100.f * (gainB - 1.f);

			printf(	"FIX_A %2f %%" EOL
				"FIX_B %2f %%" EOL
				"A1 %4e" EOL
				"B1 %4e" EOL,
				&fixA, &fixB,
				&pm.scal_A[1], &pm.scal_B[1]);
		}
		else {
			gainA = pm.wave_DFT[0] / pm.wave_DFT[1];
			fixA = 100.f * (gainA - 1.f);

			gainA = sqrtf(gainA);
			pm.scal_A[1] /= gainA;
			pm.scal_B[1] *= gainA;

			printf(	"FIX %2f %%" EOL
				"A1 %4e" EOL
				"B1 %4e" EOL,
				&fixA, &pm.scal_A[1], &pm.scal_B[1]);
		}
	}
}

static void
irq_telemetry_0()
{
	if (tel.iEN) {

		tel.pIN[0] = (short int) (pm.fb_iA * 1000.f);
		tel.pIN[1] = (short int) (pm.fb_iB * 1000.f);
		tel.pSZ = 2;

		telCapture();
	}
	else
		td.pIRQ = NULL;
}

static void
irq_telemetry_1()
{
	if (tel.iEN) {

		tel.pIN[0] = (short int) (pm.lu_X[2] * 1000.f);
		tel.pIN[1] = (short int) (pm.lu_X[3] * 1000.f);
		tel.pSZ = 2;

		telCapture();
	}
	else
		td.pIRQ = NULL;
}

static void
irq_telemetry_2()
{
	if (tel.iEN) {

		tel.pIN[0] = (short int) (pm.lu_X[0] * 1000.f);
		tel.pIN[1] = (short int) (pm.lu_X[1] * 1000.f);
		tel.pIN[2] = (short int) (pm.lu_X[4] * 1.f);
		tel.pIN[3] = (short int) (pm.n_temperature_c * 1.f);
		tel.pSZ = 4;

		telCapture();
	}
	else
		td.pIRQ = NULL;
}

static void (* const irq_telemetry_list[]) () = {

	&irq_telemetry_0,
	&irq_telemetry_1,
	&irq_telemetry_2,
};

void tel_decimal(const char *s)
{
	stoi(&tel.sDEC, s);
	printf("%i" EOL, tel.sDEC);
}

void tel_capture(const char *s)
{
	const int	nMAX = sizeof(irq_telemetry_list) / sizeof(irq_telemetry_list[0]);
	void 		(* irqtel) ();
	int		nTEL = 0;

	if (td.pIRQ == NULL) {

		tel.iEN = 1;
		tel.sCNT = 0;
		tel.pZ = tel.pD;

		stoi(&nTEL, s);
		nTEL = (nTEL < 0) ? 0 : (nTEL > nMAX) ? nMAX : nTEL;
		irqtel = irq_telemetry_list[nTEL];

		halWFI();

		td.pIRQ = irqtel;
	}
}

void tel_disable(const char *s)
{
	tel.iEN = 0;
	tel.sCNT = 0;
	tel.pZ = tel.pD;
}

void tel_live(const char *s)
{
	const int	nMAX = sizeof(irq_telemetry_list) / sizeof(irq_telemetry_list[0]);
	void 		(* irqtel) ();
	int		nTEL = 0;
	int		xC, decmin;

	if (td.pIRQ == NULL) {

		decmin = (int) (pm.freq_hz / 50.f + .5f);
		tel.sDEC = (tel.sDEC < decmin) ? decmin : tel.sDEC;

		tel.iEN = 1;
		tel.sCNT = 0;
		tel.pZ = tel.pD;

		stoi(&nTEL, s);
		nTEL = (nTEL < 0) ? 0 : (nTEL > nMAX) ? nMAX : nTEL;
		irqtel = irq_telemetry_list[nTEL];

		halWFI();

		td.pIRQ = irqtel;

		do {
			taskIOMUX();
			xC = shRecv();

			if (xC == 3 || xC == 4)
				break;

			if (tel.pZ != tel.pD) {

				telFlush();
				tel.pZ = tel.pD;
			}
		}
		while (1);

		tel.iEN = 0;
	}
}

void tel_flush(const char *s)
{
	telFlush();
}

void tel_info(const char *s)
{
	float		freq, time;

	freq = pm.freq_hz / (float) tel.sDEC;
	time = TELSZ * (float) tel.sDEC / pm.freq_hz;

	printf(	"decimal %i" EOL
		"freq %1f (Hz)" EOL
		"time %3f (Sec)" EOL,
		tel.sDEC, &freq, &time);
}

const shCMD_t		cmList[] = {

	{"td_uptime", &td_uptime},
	{"td_irqload", &td_irqload},
	{"td_avg_default_time", &td_avg_default_time},
	{"td_reboot", &td_reboot},
	{"td_keycodes", &td_keycodes},

	{"pwm_freq_hz", &pwm_freq_hz},
	{"pwm_dead_time_ns", &pwm_dead_time_ns},
	{"pwm_DC", &pwm_DC},
	{"pwm_Z", &pwm_Z},

	{"pm_pwm_resolution", &pm_pwm_resolution},
	{"pm_pwm_minimal_pulse", &pm_pwm_minimal_pulse},
	{"pm_pwm_dead_time", &pm_pwm_dead_time},

	{"pm_m_bitmask_direct_current_injection", &pm_m_bitmask_direct_current_injection},
	{"pm_m_bitmask_high_frequency_injection", &pm_m_bitmask_high_frequency_injection},
	{"pm_m_bitmask_flux_polarity_detection", &pm_m_bitmask_flux_polarity_detection},
	{"pm_m_bitmask_servo_control_loop", &pm_m_bitmask_servo_control_loop},

	{"pm_m_errno", &pm_m_errno},

	{"pm_T_drift", &pm_T_drift},
	{"pm_T_hold", &pm_T_hold},
	{"pm_T_sine", &pm_T_sine},
	{"pm_T_measure", &pm_T_measure},
	{"pm_T_end", &pm_T_end},

	{"pm_wave_i_hold_X", &pm_wave_i_hold_X},
	{"pm_wave_i_hold_Y", &pm_wave_i_hold_Y},
	{"pm_wave_i_sine", &pm_wave_i_sine},
	{"pm_wave_freq_sine_hz", &pm_wave_freq_sine_hz},
	{"pm_wave_gain_P", &pm_wave_gain_P},
	{"pm_wave_gain_I", &pm_wave_gain_I},

	{"pm_scal_A0", &pm_scal_A0},
	{"pm_scal_A1", &pm_scal_A1},
	{"pm_scal_B0", &pm_scal_B0},
	{"pm_scal_B1", &pm_scal_B1},
	{"pm_scal_U0", &pm_scal_U0},
	{"pm_scal_U1", &pm_scal_U1},

	{"pm_fault_iab_maximal", &pm_fault_iab_maximal},
	{"pm_fault_residual_maximal", &pm_fault_residual_maximal},
	{"pm_fault_drift_maximal", &pm_fault_drift_maximal},
	{"pm_fault_low_voltage", &pm_fault_low_voltage},
	{"pm_fault_high_voltage", &pm_fault_high_voltage},

	{"pm_vsi_u_maximal", &pm_vsi_u_maximal},

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
	{"pm_lu_gain_K6", &pm_lu_gain_K5},
	{"pm_lu_threshold_low", &pm_lu_threshold_low},
	{"pm_lu_threshold_high", &pm_lu_threshold_high},
	{"pm_lu_threshold_auto", &pm_lu_threshold_auto},
	{"pm_lu_low_D", &pm_lu_low_D},
	{"pm_lu_residual_variance", &pm_lu_residual_variance},

	{"pm_hf_freq_hz", &pm_hf_freq_hz},
	{"pm_hf_swing_D", &pm_hf_swing_D},
	{"pm_hf_gain_K0", &pm_hf_gain_K0},
	{"pm_hf_gain_K1", &pm_hf_gain_K1},
	{"pm_hf_gain_K2", &pm_hf_gain_K2},

	{"pm_thermal_R", &pm_thermal_R},
	{"pm_thermal_E", &pm_thermal_E},
	{"pm_thermal_gain_R0", &pm_thermal_gain_R0},
	{"pm_thermal_gain_R1", &pm_thermal_gain_R1},

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

	{"pm_i_high_maximal", &pm_i_high_maximal},
	{"pm_i_low_maximal", &pm_i_low_maximal},
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

	{"pm_p_set_point_w_rpm", &pm_p_set_point_w_rpm},
	{"pm_p_track_point_x_g", &pm_p_track_point_x_g},
	{"pm_p_gain_D", &pm_p_gain_D},
	{"pm_p_gain_P", &pm_p_gain_P},

	{"pm_lp_gain_0", &pm_lp_gain_0},
	{"pm_lp_gain_1", &pm_lp_gain_1},

	{"pm_n_power_watt", &pm_n_power_watt},
	{"pm_n_temperature_c", &pm_n_temperature_c},

	{"pm_default", &pm_default},
	{"pm_request_zero_drift", &pm_request_zero_drift},
	{"pm_request_wave_hold", &pm_request_wave_hold},
	{"pm_request_wave_sine", &pm_request_wave_sine},
	{"pm_request_calibration", &pm_request_calibration},
	{"pm_request_start", &pm_request_start},
	{"pm_request_stop", &pm_request_stop},

	{"pm_update_const_R", &pm_update_const_R},
	{"pm_impedance", &pm_impedance},
	{"pm_update_const_L", &pm_update_const_L},
	{"pm_update_scal_AB", &pm_update_scal_AB},

	{"tel_decimal", &tel_decimal},
	{"tel_capture", &tel_capture},
	{"tel_disable", &tel_disable},
	{"tel_live", &tel_live},
	{"tel_flush", &tel_flush},
	{"tel_info", &tel_info},

	{"ap_identify_base", &ap_identify_base},
	{"ap_identify_const_R_abc", &ap_identify_const_R_abc},
	{"ap_identify_const_E", &ap_identify_const_E},

	{NULL, NULL},
};

