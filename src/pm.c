/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2016 Roman Belov <romblv@gmail.com>

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

#include <stddef.h>

#include "hal/hal.h"
#include "lib.h"
#include "m.h"
#include "main.h"
#include "pmc.h"
#include "sh.h"

SH_DEF(pm_pwm_resolution)
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

SH_DEF(pm_pwm_minimal_pulse)
{
	pm_pwm_int_modify(&pm.pwm_minimal_pulse, s);
}

SH_DEF(pm_pwm_dead_time)
{
	pm_pwm_int_modify(&pm.pwm_dead_time, s);
}

static int
pm_m_bitmask(const char *s, int bit)
{
	int		flag, mask = pm.m_bitmask;

	if (stoi(&flag, s) != NULL) {

		if (flag)
			mask |= bit;
		else
			mask &= ~bit;
	}

	flag = (mask & bit) ? 1 : 0;

	printf("%i (%s)" EOL, flag, flag ? "Enabled" : "Disabled");

	return mask;
}

SH_DEF(pm_m_bitmask_high_frequency_injection)
{
	int			mask;

	mask = pm_m_bitmask(s, PMC_BIT_HIGH_FREQUENCY_INJECTION);

	if (mask & PMC_BIT_HIGH_FREQUENCY_INJECTION) {

		pm.hf_CS[0] = 1.;
		pm.hf_CS[1] = 0.;
		pm.hf_flux_polarity = 0.f;

		halFence();
	}

	pm.m_bitmask = mask;
}

SH_DEF(pm_m_bitmask_flux_polarity_detection)
{
	pm.m_bitmask = pm_m_bitmask(s, PMC_BIT_FLUX_POLARITY_DETECTION);
}

SH_DEF(pm_m_bitmask_thermal_drift_estimation)
{
	pm.m_bitmask = pm_m_bitmask(s, PMC_BIT_THERMAL_DRIFT_ESTIMATION);
}

SH_DEF(pm_m_bitmask_bemf_waveform_compensation)
{
	int			mask;

	mask = pm_m_bitmask(s, PMC_BIT_BEMF_WAVEFORM_COMPENSATION);

	if (mask & PMC_BIT_BEMF_WAVEFORM_COMPENSATION) {

		pm.bemf_D = 0.f;
		pm.bemf_Q = 0.f;

		halFence();
	}

	pm.m_bitmask = mask;
}

SH_DEF(pm_m_bitmask_speed_control_loop)
{
	int			mask;

	mask = pm_m_bitmask(s, PMC_BIT_SPEED_CONTROL_LOOP);

	if (mask & PMC_BIT_SPEED_CONTROL_LOOP) {

		pm.s_set_point = pm.lu_X[4];
		pm.s_track_point = pm.lu_X[4];
		pm.s_nonl_X4 = pm.lu_X[4];
		pm.s_track_point_p[0] = pm.lu_X[2];
		pm.s_track_point_p[1] = pm.lu_X[3];

		halFence();
	}

	pm.m_bitmask = mask;
}

SH_DEF(pm_m_bitmask_position_control_loop)
{
	int			mask;

	mask = pm_m_bitmask(s, PMC_BIT_POSITION_CONTROL_LOOP);

	if (mask & PMC_BIT_POSITION_CONTROL_LOOP) {

		pm.p_set_point[0] = pm.lu_X[2];
		pm.p_set_point[1] = pm.lu_X[3];
		pm.p_set_point_s = pm.lu_X[4];
		pm.p_set_point_revol = pm.lu_revol;

		halFence();
	}

	pm.m_bitmask = mask;
}

SH_DEF(pm_m_bitmask_forced_control)
{
	pm.m_bitmask = pm_m_bitmask(s, PMC_BIT_FORCED_CONTROL);
}

SH_DEF(pm_m_bitmask_power_control_loop)
{
	pm.m_bitmask = pm_m_bitmask(s, PMC_BIT_POWER_CONTROL_LOOP);
}

SH_DEF(pm_m_errno)
{
	int		flag;

	if (stoi(&flag, s) != NULL) {

		if (flag == 0)
			pm.m_errno = PMC_OK;
	}

	printf("%i: %s" EOL, pm.m_errno, pmc_strerror(pm.m_errno));
}

SH_DEF(pm_T_drift)
{
	stof(&pm.T_drift, s);
	printf("%3f (Sec)" EOL, &pm.T_drift);
}

SH_DEF(pm_T_hold)
{
	stof(&pm.T_hold, s);
	printf("%3f (Sec)" EOL, &pm.T_hold);
}

SH_DEF(pm_T_sine)
{
	stof(&pm.T_sine, s);
	printf("%3f (Sec)" EOL, &pm.T_sine);
}

SH_DEF(pm_T_measure)
{
	stof(&pm.T_measure, s);
	printf("%3f (Sec)" EOL, &pm.T_measure);
}

SH_DEF(pm_T_end)
{
	stof(&pm.T_end, s);
	printf("%3f (Sec)" EOL, &pm.T_end);
}

SH_DEF(pm_wave_i_hold_X)
{
	stof(&pm.wave_i_hold_X, s);
	printf("%3f (A)" EOL, &pm.wave_i_hold_X);
}

SH_DEF(pm_wave_i_hold_Y)
{
	stof(&pm.wave_i_hold_Y, s);
	printf("%3f (A)" EOL, &pm.wave_i_hold_Y);
}

SH_DEF(pm_wave_i_sine)
{
	stof(&pm.wave_i_sine, s);
	printf("%3f (A)" EOL, &pm.wave_i_sine);
}

SH_DEF(pm_wave_freq_sine_hz)
{
	stof(&pm.wave_freq_sine_hz, s);
	printf("%1f (Hz)" EOL, &pm.wave_freq_sine_hz);
}

SH_DEF(pm_wave_gain_P)
{
	stof(&pm.wave_gain_P, s);
	printf("%2e" EOL, &pm.wave_gain_P);
}

SH_DEF(pm_wave_gain_I)
{
	stof(&pm.wave_gain_I, s);
	printf("%2e" EOL, &pm.wave_gain_I);
}

SH_DEF(pm_scal_A0)
{
	stof(&pm.scal_A[0], s);
	printf("%3f (A)" EOL, &pm.scal_A[0]);
}

SH_DEF(pm_scal_A1)
{
	stof(&pm.scal_A[1], s);
	printf("%4e" EOL, &pm.scal_A[1]);
}

SH_DEF(pm_scal_B0)
{
	stof(&pm.scal_B[0], s);
	printf("%3f (A)" EOL, &pm.scal_B[0]);
}

SH_DEF(pm_scal_B1)
{
	stof(&pm.scal_B[1], s);
	printf("%4e" EOL, &pm.scal_B[1]);
}

SH_DEF(pm_scal_U0)
{
	stof(&pm.scal_U[0], s);
	printf("%3f (V)" EOL, &pm.scal_U[0]);
}

SH_DEF(pm_scal_U1)
{
	stof(&pm.scal_U[1], s);
	printf("%4e" EOL, &pm.scal_U[1]);
}

SH_DEF(pm_fault_residual_maximal)
{
	stof(&pm.fault_residual_maximal, s);
	printf("%4e" EOL, &pm.fault_residual_maximal);
}

SH_DEF(pm_fault_drift_maximal)
{
	stof(&pm.fault_drift_maximal, s);
	printf("%3f (A)" EOL, &pm.fault_drift_maximal);
}

SH_DEF(pm_fault_low_voltage)
{
	stof(&pm.fault_low_voltage, s);
	printf("%3f (V)" EOL, &pm.fault_low_voltage);
}

SH_DEF(pm_fault_high_voltage)
{
	stof(&pm.fault_high_voltage, s);
	printf("%3f (V)" EOL, &pm.fault_high_voltage);
}

SH_DEF(pm_lu_X0)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.lu_X[0], s);
	printf("%3f (A)" EOL, &avg);
}

SH_DEF(pm_lu_X1)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.lu_X[1], s);
	printf("%3f (A)" EOL, &avg);
}

SH_DEF(pm_lu_X23)
{
	float			g;

	g = atan2f(pm.lu_X[3], pm.lu_X[2]) * 180.f / M_PI_F;
	printf("%1f (degree) [%3f %3f]" EOL, &g, &pm.lu_X[2], &pm.lu_X[3]);
}

SH_DEF(pm_lu_X4)
{
	float			avg, RPM;

	avg = ma_av_float_arg_1(&pm.lu_X[4], s);
	RPM = 9.5492969f * avg / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM) " EOL, &avg, &RPM);
}

SH_DEF(pm_lu_gain_K0)
{
	stof(&pm.lu_gain_K[0], s);
	printf("%2e" EOL, &pm.lu_gain_K[0]);
}

SH_DEF(pm_lu_gain_K1)
{
	stof(&pm.lu_gain_K[1], s);
	printf("%2e" EOL, &pm.lu_gain_K[1]);
}

SH_DEF(pm_lu_gain_K2)
{
	stof(&pm.lu_gain_K[2], s);
	printf("%2e" EOL, &pm.lu_gain_K[2]);
}

SH_DEF(pm_lu_gain_K3)
{
	stof(&pm.lu_gain_K[3], s);
	printf("%2e" EOL, &pm.lu_gain_K[3]);
}

SH_DEF(pm_lu_gain_K4)
{
	stof(&pm.lu_gain_K[4], s);
	printf("%2e" EOL, &pm.lu_gain_K[4]);
}

SH_DEF(pm_lu_gain_K5)
{
	stof(&pm.lu_gain_K[5], s);
	printf("%2e" EOL, &pm.lu_gain_K[5]);
}

SH_DEF(pm_lu_gain_K6)
{
	stof(&pm.lu_gain_K[6], s);
	printf("%2e" EOL, &pm.lu_gain_K[6]);
}

SH_DEF(pm_lu_threshold_low)
{
	float			RPM;

	stof(&pm.lu_threshold_low, s);
	RPM = 9.5492969f * pm.lu_threshold_low / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM)" EOL, &pm.lu_threshold_low, &RPM);
}

SH_DEF(pm_lu_threshold_high)
{
	float			RPM;

	stof(&pm.lu_threshold_high, s);
	RPM = 9.5492969f * pm.lu_threshold_high / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM)" EOL, &pm.lu_threshold_high, &RPM);
}

SH_DEF(pm_lu_threshold_auto)
{
	pm.lu_threshold_low = 5E-3f * pm.const_U / pm.const_E;
	pm.lu_threshold_high = pm.lu_threshold_low + 50.f;

	printf(	"LO %4e (Rad/S)" EOL
		"HI %4e (Rad/S)" EOL,
		&pm.lu_threshold_low,
		&pm.lu_threshold_high);
}

SH_DEF(pm_lu_residual_variance)
{
	float			avg;

	avg = sqrtf(ma_av_float_arg_1(&pm.lu_residual_variance, s));
	printf("%4e (SD)" EOL, &avg);
}

SH_DEF(pm_hf_freq_hz)
{
	stof(&pm.hf_freq_hz, s);
	printf("%1f (Hz)" EOL, &pm.hf_freq_hz);
}

SH_DEF(pm_hf_swing_D)
{
	stof(&pm.hf_swing_D, s);
	printf("%3f (A)" EOL, &pm.hf_swing_D);
}

SH_DEF(pm_hf_flux_polarity)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.hf_flux_polarity, s);
	printf("%4e" EOL, &avg);
}

SH_DEF(pm_hf_gain_K0)
{
	stof(&pm.hf_gain_K[0], s);
	printf("%2e" EOL, &pm.hf_gain_K[0]);
}

SH_DEF(pm_hf_gain_K1)
{
	stof(&pm.hf_gain_K[1], s);
	printf("%2e" EOL, &pm.hf_gain_K[1]);
}

SH_DEF(pm_hf_gain_K2)
{
	stof(&pm.hf_gain_K[2], s);
	printf("%2e" EOL, &pm.hf_gain_K[2]);
}

SH_DEF(pm_bemf_DFT)
{
	float			*DFT = pm.bemf_DFT;
	float			D[2], Q[2];
	int			i;

	for (i = 0; i < pm.bemf_N; ++i) {

		D[0] = 100.f * *DFT++;
		D[1] = 100.f * *DFT++;
		Q[0] = 100.f * *DFT++;
		Q[1] = 100.f * *DFT++;

		printf("%i: %1f %1f %1f %1f (%%)" EOL, 1 + i,
				D + 0, D + 1, Q + 0, Q + 1);
	}
}

SH_DEF(pm_bemf_gain_K)
{
	stof(&pm.bemf_gain_K, s);
	printf("%2e" EOL, &pm.bemf_gain_K);
}

SH_DEF(pm_bemf_N)
{
	stoi(&pm.bemf_N, s);

	pm.bemf_N = (pm.bemf_N < 1) ? 1 :
		(pm.bemf_N > 14) ? 14 : pm.bemf_N;

	printf("%i" EOL, pm.bemf_N);
}

SH_DEF(pm_bemf_tune_T)
{
	float		T;

	if (stof(&T, s) != NULL && T > 0.f) {

		pm.bemf_tune_T = pm.freq_hz * T;
		printf("+ %3f (Sec)" EOL, &T);
	}
}

SH_DEF(pm_thermal_R)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.thermal_R, s);
	printf("%4e" EOL, &avg);
}

SH_DEF(pm_thermal_E)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.thermal_E, s);
	printf("%4e" EOL, &avg);
}

SH_DEF(pm_thermal_gain_R0)
{
	stof(&pm.thermal_gain_R[0], s);
	printf("%1f" EOL, &pm.thermal_gain_R[0]);
}

SH_DEF(pm_thermal_gain_R1)
{
	stof(&pm.thermal_gain_R[1], s);
	printf("%4e" EOL, &pm.thermal_gain_R[1]);
}

SH_DEF(pm_drift_A)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.drift_A, s);
	printf("%3f (A)" EOL, &avg);
}

SH_DEF(pm_drift_B)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.drift_B, s);
	printf("%3f (A)" EOL, &avg);
}

SH_DEF(pm_drift_Q)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.drift_Q, s);
	printf("%3f (V)" EOL, &avg);
}

SH_DEF(pm_const_U)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.const_U, s);
	printf("%3f (V)" EOL, &avg);
}

SH_DEF(pm_const_E_wb)
{
	float		const_Kv;

	stof(&pm.const_E, s);
	const_Kv = 5.513289f / (pm.const_E * pm.const_Zp);

	printf("%4e (Wb) %1f (RPM/V)" EOL, &pm.const_E, &const_Kv);
}

SH_DEF(pm_const_E_kv)
{
	float		const_Kv, Z;

	if (stof(&const_Kv, s) != NULL) {

		Z = const_Kv * pm.const_Zp;
		pm.const_E = (Z != 0.f) ? 5.513289f / Z : pm.const_E;
	}

	const_Kv = 5.513289f / (pm.const_E * pm.const_Zp);

	printf("%4e (Wb) %1f (RPM/V)" EOL, &pm.const_E, &const_Kv);
}

SH_DEF(pm_const_R)
{
	stof(&pm.const_R, s);
	printf("%4e (Ohm)" EOL, &pm.const_R);
}

SH_DEF(pm_const_Ld)
{
	if (stof(&pm.const_Ld, s) != NULL)
		pm.const_Ld_inversed = 1.f / pm.const_Ld;

	printf("%4e (H)" EOL, &pm.const_Ld);
}

SH_DEF(pm_const_Lq)
{
	if (stof(&pm.const_Lq, s) != NULL)
		pm.const_Lq_inversed = 1.f / pm.const_Lq;

	printf("%4e (H)" EOL, &pm.const_Lq);
}

SH_DEF(pm_const_Zp)
{
	stoi(&pm.const_Zp, s);
	printf("%i" EOL, pm.const_Zp);
}

SH_DEF(pm_const_J)
{
	stof(&pm.const_J, s);
	printf("%4e (kgm2)" EOL, &pm.const_J);
}

SH_DEF(pm_i_high_maximal)
{
	stof(&pm.i_high_maximal, s);
	printf("%3f (A)" EOL, &pm.i_high_maximal);
}

SH_DEF(pm_i_low_maximal)
{
	stof(&pm.i_low_maximal, s);
	printf("%3f (A)" EOL, &pm.i_low_maximal);
}

SH_DEF(pm_i_power_consumption_maximal)
{
	stof(&pm.i_power_consumption_maximal, s);
	printf("%1f (W)" EOL, &pm.i_power_consumption_maximal);
}

SH_DEF(pm_i_power_regeneration_maximal)
{
	stof(&pm.i_power_regeneration_maximal, s);
	printf("%1f (W)" EOL, &pm.i_power_regeneration_maximal);
}

SH_DEF(pm_i_set_point_D)
{
	stof(&pm.i_set_point_D, s);
	printf("%3f (A)" EOL, &pm.i_set_point_D);
}

SH_DEF(pm_i_set_point_Q)
{
	stof(&pm.i_set_point_Q, s);
	printf("%3f (A)" EOL, &pm.i_set_point_Q);
}

SH_DEF(pm_i_slew_rate_D)
{
	stof(&pm.i_slew_rate_D, s);
	printf("%2e (A/Sec)" EOL, &pm.i_slew_rate_D);
}

SH_DEF(pm_i_slew_rate_Q)
{
	stof(&pm.i_slew_rate_Q, s);
	printf("%2e (A/Sec)" EOL, &pm.i_slew_rate_Q);
}

SH_DEF(pm_i_slew_rate_auto)
{
	pm.i_slew_rate_D = .2f * pm.const_U / pm.const_Ld;
	pm.i_slew_rate_Q = .2f * pm.const_U / pm.const_Lq;

	printf(	"D %1f (A/Sec)" EOL
		"Q %1f (A/Sec)" EOL,
		&pm.i_slew_rate_D,
		&pm.i_slew_rate_Q);
}

SH_DEF(pm_i_gain_P_D)
{
	stof(&pm.i_gain_P_D, s);
	printf("%2e" EOL, &pm.i_gain_P_D);
}

SH_DEF(pm_i_gain_I_D)
{
	stof(&pm.i_gain_I_D, s);
	printf("%2e" EOL, &pm.i_gain_I_D);
}

SH_DEF(pm_i_gain_P_Q)
{
	stof(&pm.i_gain_P_Q, s);
	printf("%2e" EOL, &pm.i_gain_P_Q);
}

SH_DEF(pm_i_gain_I_Q)
{
	stof(&pm.i_gain_I_Q, s);
	printf("%2e" EOL, &pm.i_gain_I_Q);
}

SH_DEF(pm_i_gain_auto)
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

SH_DEF(pm_s_maximal_rpm)
{
	float			RPM;

	if (stof(&RPM, s) != NULL)
		pm.s_maximal = .10471976f * RPM * pm.const_Zp;

	RPM = 9.5492969f * pm.s_maximal / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM)" EOL, &pm.s_maximal, &RPM);
}

SH_DEF(pm_s_set_point_rpm)
{
	float			RPM;

	if (stof(&RPM, s) != NULL)
		pm.s_set_point = .10471976f * RPM * pm.const_Zp;

	RPM = 9.5492969f * pm.s_set_point / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM)" EOL, &pm.s_set_point, &RPM);
}

SH_DEF(pm_s_slew_rate)
{
	stof(&pm.s_slew_rate, s);
	printf("%4e (Rad/S2)" EOL, &pm.s_slew_rate);
}

SH_DEF(pm_s_forced_D)
{
	stof(&pm.s_forced_D, s);
	printf("%3f (A)" EOL, &pm.s_forced_D);
}

SH_DEF(pm_s_forced_slew_rate)
{
	stof(&pm.s_forced_slew_rate, s);
	printf("%4e (Rad/S2)" EOL, &pm.s_forced_slew_rate);
}

SH_DEF(pm_s_slew_rate_auto)
{
	float			MT;

	if (pm.const_J > M_EPS_F) {

		MT = pm.const_Zp * pm.const_Zp * pm.const_E / pm.const_J;
		pm.s_slew_rate = 1.5f * MT * (pm.const_E / pm.const_Lq);
		pm.s_forced_slew_rate = .3f * MT * pm.s_forced_D;

		printf("%4e (Rad/S2)" EOL, &pm.s_slew_rate);
		printf("%4e (Rad/S2)" EOL, &pm.s_forced_slew_rate);
	}
}

SH_DEF(pm_s_nonl_gain_F)
{
	stof(&pm.s_nonl_gain_F, s);
	printf("%2e" EOL, &pm.s_nonl_gain_F);
}

SH_DEF(pm_s_nonl_range)
{
	stof(&pm.s_nonl_range, s);
	printf("%4e (Rad/S)" EOL, &pm.s_nonl_range);
}

SH_DEF(pm_s_gain_P)
{
	stof(&pm.s_gain_P, s);
	printf("%2e" EOL, &pm.s_gain_P);
}

SH_DEF(pm_p_set_point_g)
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

		angle = g * (M_PI_F / 180.f);
		pm.p_set_point[0] = cosf(angle);
		pm.p_set_point[1] = sinf(angle);
		pm.p_set_point_revol = revol;
	}

	angle = atan2f(pm.p_set_point[1], pm.p_set_point[0]);
	g = angle * (180.f / M_PI_F);

	printf("%1f (degree) + %i (full revolutions)" EOL, &g, pm.p_set_point_revol);
}

SH_DEF(pm_p_set_point_s_rpm)
{
	float			RPM;

	if (stof(&RPM, s) != NULL)
		pm.p_set_point_s = .10471976f * RPM * pm.const_Zp;

	RPM = 9.5492969f * pm.p_set_point_s / pm.const_Zp;

	printf("%4e (Rad/S) %1f (RPM)" EOL, &pm.p_set_point_s, &RPM);
}

SH_DEF(pm_p_gain_P)
{
	stof(&pm.p_gain_P, s);
	printf("%2e" EOL, &pm.p_gain_P);
}

SH_DEF(pm_w_set_point_watt)
{
	stof(&pm.w_set_point_watt, s);
	printf("%1f (W)" EOL, &pm.w_set_point_watt);
}

SH_DEF(pm_w_direction)
{
	stoi(&pm.w_direction, s);
	printf("%i" EOL, pm.w_direction);
}

SH_DEF(pm_lp_gain_0)
{
	stof(&pm.lp_gain[0], s);
	printf("%2e" EOL, &pm.lp_gain[0]);
}

SH_DEF(pm_lp_gain_1)
{
	stof(&pm.lp_gain[1], s);
	printf("%2e" EOL, &pm.lp_gain[1]);
}

SH_DEF(pm_n_power_watt)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.n_power_watt, s);
	printf("%1f (W)" EOL, &avg);
}

SH_DEF(pm_n_temperature_c)
{
	float			avg;

	avg = ma_av_float_arg_1(&pm.n_temperature_c, s);
	printf("%1f (C)" EOL, &avg);
}

SH_DEF(pm_default)
{
	if (pm.lu_region == PMC_LU_DISABLED)
		pmc_default(&pm);
}

SH_DEF(pm_request_zero_drift)
{
	pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
}

SH_DEF(pm_request_wave_hold)
{
	pmc_request(&pm, PMC_STATE_WAVE_HOLD);
}

SH_DEF(pm_request_wave_sine)
{
	pmc_request(&pm, PMC_STATE_WAVE_SINE);
}

SH_DEF(pm_request_calibration)
{
	pmc_request(&pm, PMC_STATE_CALIBRATION);
}

SH_DEF(pm_request_start)
{
	pmc_request(&pm, PMC_STATE_START);
}

SH_DEF(pm_request_stop)
{
	pmc_request(&pm, PMC_STATE_STOP);
}

SH_DEF(pm_update_const_R)
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

SH_DEF(pm_impedance)
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

SH_DEF(pm_update_const_L)
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

SH_DEF(pm_update_scal_AB)
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

