/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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

#ifndef _H_PM_CONTROL_
#define _H_PM_CONTROL_

enum {
	PM_LU_DISABLED				= 0,
	PM_LU_OPEN_LOOP,
	PM_LU_CLOSED_LOW,
	PM_LU_CLOSED_HIGH,
};

enum {
	PM_HALL_A				= 1,
	PM_HALL_B				= 2,
	PM_HALL_C				= 4,
};

typedef struct {

	float		current_A;
	float		current_B;
	float		voltage_U;

	float		voltage_A;
	float		voltage_B;
	float		voltage_C;

	int		hall;
	float		sensor;
}
pmfb_t;

typedef struct {

	float		freq_hz;
	float		dT;
	int		pwm_R;
	int		pwm_MP;

	int		error;
	int		rc[8];

	int		b_FORCED;
	int		b_HFI;
	int		b_SENSOR;
	int		b_LOOP;

	int		fsm_state;
	int		fsm_phase;
	int		fsm_phase_2;

	int		tm_value;
	int		tm_end;

	float		tm_skip;
	float		tm_probe;
	float		tm_hold;

	float		adjust_IA[2];
	float		adjust_IB[2];
	float		adjust_US[2];
	float		adjust_UA[2];
	float		adjust_UB[2];
	float		adjust_UC[2];

	float		fb_i_range;

	float		fb_iA;
	float		fb_iB;

	float		fb_uA;
	float		fb_uB;
	float		fb_uC;

	int		fb_hall;

	float		probe_i_hold;
	float		probe_i_hold_Q;
	float		probe_i_sine;
	float		probe_freq_sine_hz;
	float		probe_speed_ramp;
	float		probe_gain_P;
	float		probe_gain_I;
	float		probe_DFT[8];

	float		temp[8];

	float		fault_zero_drift_maximal;
	float		fault_voltage_tolerance;
	float		fault_current_tolerance;
	float		fault_adjust_tolerance;
	float		fault_lu_residual_maximal;
	float		fault_lu_drift_Q_maximal;
	float		fault_supply_voltage_low;
	float		fault_supply_voltage_high;

	float		vsi_X;
	float		vsi_Y;
	float		vsi_lpf_D;
	float		vsi_lpf_Q;

	float		lu_power_lpf;

	float		lu_residual_D;
	float		lu_residual_Q;
	float		lu_residual_lpf;

	int		lu_region;
	float		lu_X[5];
	float		lu_drift_Q;
	int		lu_revol;
	float		lu_gain_DA;
	float		lu_gain_QA;
	float		lu_gain_DP;
	float		lu_gain_DS;
	float		lu_gain_QS;
	float		lu_gain_QZ;
	float		lu_boost_slope;
	float		lu_boost_gain;
	float		lu_BEMF_low;
	float		lu_BEMF_high;
	float		lu_forced_D;
	float		lu_forced_accel;

	float		hf_freq_hz;
	float		hf_swing_D;
	float		hf_flux_polarity;
	float		hf_CS[2];
	float		hf_gain_P;
	float		hf_gain_S;
	float		hf_gain_F;

	float		hall_;

	float		const_lpf_U;
	float		const_E;
	float		const_R;
	float		const_Ld;
	float		const_Lq;

	int		const_Zp;
	float		const_J;

	float		i_maximal;
	float		i_maximal_weak;
	float		i_power_consumption_maximal;
	float		i_power_regeneration_maximal;
	float		i_set_point_D;
	float		i_set_point_Q;
	float		i_slew_rate_D;
	float		i_slew_rate_Q;
	float		i_track_point_D;
	float		i_track_point_Q;
	float		i_integral_D;
	float		i_integral_Q;
	float		i_gain_P_D;
	float		i_gain_I_D;
	float		i_gain_P_Q;
	float		i_gain_I_Q;

	float		s_maximal;
	float		s_set_point;
	float		s_slew_rate;
	float		s_track_point;
	float		s_integral;
	float		s_gain_P;
	float		s_gain_I;

	float		p_set_point_DQ[2];
	float		p_set_point_s;
	int		p_set_point_revol;
	float		p_gain_P;
	float		p_gain_I;

	float		lpf_gain_POWER;
	float		lpf_gain_LU;
	float		lpf_gain_VSI;
	float		lpf_gain_U;

	void 		(* pDC) (int, int, int);
	void 		(* pZ) (int);
}
pmc_t;

void pm_default(pmc_t *pm);
void pm_tune_current_loop(pmc_t *pm);

void pm_VSI_control(pmc_t *pm, float uX, float uY);
void pm_feedback(pmc_t *pm, pmfb_t *fb);

#endif /* _H_PM_CONTROL_ */

