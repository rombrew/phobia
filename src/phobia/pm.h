/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

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

#ifndef _H_PM_
#define _H_PM_

enum {
	PM_ABC_THREE_PHASE			= 0,
	PM_ABC_TWO_PHASE,
};

enum {
	PM_LDQ_NON_SALIENT_POLE			= 0,	// assumes (Ld = Lq)
	PM_LDQ_SALIENT_POLE,				// assumes (Ld > Lq)
	PM_LDQ_SATURATION_SALIENCY			// assumes (Ld < Lq)
};

enum {
	PM_LU_DISABLED				= 0,
	PM_LU_OPEN_LOOP,
	PM_LU_CLOSED_ESTIMATE_FLUX,
	PM_LU_CLOSED_ESTIMATE_HFI,
	PM_LU_CLOSED_SENSOR_HALL,
};

enum {
	PM_HALL_DISABLED			= 0,
	PM_HALL_ENABLED
};

enum {
	PM_HFI_DISABLED				= 0,
	PM_HFI_ENABLED
};

enum {
	PM_LOOP_CURRENT_CONTROL			= 0,
	PM_LOOP_SPEED_CONTROL,
};

enum {
	PM_STATE_IDLE				= 0,
	PM_STATE_ZERO_DRIFT,
	PM_STATE_POWER_STAGE_SELF_TEST,
	PM_STATE_SAMPLING_SELF_TEST,
	PM_STATE_ADJUST_VOLTAGE,
	PM_STATE_ADJUST_CURRENT,
	PM_STATE_PROBE_CONST_R,
	PM_STATE_PROBE_CONST_L,
	PM_STATE_LU_INITIATE,
	PM_STATE_LU_SHUTDOWN,
	PM_STATE_PROBE_CONST_E,
	PM_STATE_PROBE_CONST_J,
	PM_STATE_ADJUST_HALL,
	PM_STATE_ADJUST_SENSOR,
	PM_STATE_HALT,
};

enum {
	PM_OK					= 0,
	PM_ERROR_ZERO_DRIFT_FAULT,
	PM_ERROR_NO_MOTOR_CONNECTED,
	PM_ERORR_POWER_STAGE_FAULT,
	PM_ERROR_SAMPLING_FAULT,
	PM_ERROR_CURRENT_LOOP_FAULT,
	PM_ERROR_OVER_CURRENT,
	PM_ERROR_ADJUST_TOLERANCE_FAULT,
	PM_ERROR_LU_RESIDUAL_UNSTABLE,
	PM_ERROR_LU_INVALID_OPERATION,
};

typedef struct {

	float		current_A;
	float		current_B;
	float		voltage_U;

	float		voltage_A;
	float		voltage_B;
	float		voltage_C;

	int		hall_A;
	int		hall_B;
	int		hall_C;

	float		sensor;
}
pmfb_t;

typedef struct {

	float		freq_hz;
	float		dT;

	int		pwm_resolution;
	int		pwm_correction;
	float		pwm_tik_per_ns;
	int		pwm_minimal_pulse;
	int		pwm_sampling_gap;

	int		fail_reason;
	int		self_BM[8];
	float		self_SA[8];

	int		config_ABC;
	int		config_LDQ;

	int		config_HALL;
	int		config_HFI;
	int		config_LOOP;

	int		fsm_state;
	int		fsm_phase;
	int		fsm_phase_2;

	int		tm_value;
	int		tm_end;

	float		tm_transient_skip;
	float		tm_voltage_hold;
	float		tm_current_hold;
	float		tm_instant_probe;
	float		tm_average_probe;

	float		adjust_IA[2];
	float		adjust_IB[2];
	float		adjust_US[2];
	float		adjust_UA[2];
	float		adjust_UB[2];
	float		adjust_UC[2];

	float		fb_current_clamp;
	float		fb_current_A;
	float		fb_current_B;
	float		fb_voltage_A;
	float		fb_voltage_B;
	float		fb_voltage_C;
	int		fb_hall_A;
	int		fb_hall_B;
	int		fb_hall_C;

	float		probe_current_hold;
	float		probe_current_hold_Q;
	float		probe_current_sine;
	float		probe_freq_sine_hz;
	float		probe_speed_low;
	float		probe_speed_ramp;
	float		probe_gain_P;
	float		probe_gain_I;
	float		probe_DFT[8];

	float		temporal[8];

	float		fault_voltage_tolerance;
	float		fault_current_tolerance;
	float		fault_current_halt_level;
	float		fault_adjust_tolerance;
	float		fault_flux_residual_maximal;

	int		vsi_clamp_to_null;
	float		vsi_X;
	float		vsi_Y;
	float		vsi_lpf_D;
	float		vsi_lpf_Q;
	float		vsi_lpf_watt;
	float		vsi_gain_LP;
	float		vsi_gain_LW;

	float		lu_fb_X;
	float		lu_fb_Y;
	float		lu_X[5];
	int		lu_mode;
	int		lu_revol;

	float		forced_X[5];
	float		forced_hold_D;
	float		forced_accel;
	float		forced_setpoint;

	float		flux_X[5];
	float		flux_drift_Q;
	float		flux_residual_D;
	float		flux_residual_Q;
	float		flux_residual_lpf;
	float		flux_gain_LP;
	float		flux_gain_DA;
	float		flux_gain_QA;
	float		flux_gain_DP;
	float		flux_gain_DS;
	float		flux_gain_QS;
	float		flux_gain_QZ;
	float		flux_BEMF_low;
	float		flux_BEMF_high;

	float		hfi_X[5];
	float		hfi_freq_hz;
	float		hfi_swing_D;
	float		hfi_flux_polarity;
	float		hfi_CS[2];
	float		hfi_gain_P;
	float		hfi_gain_S;
	float		hfi_gain_F;

	float		hall_X[5];
	float		hall_;

	float		const_lpf_U;
	float		const_gain_LP;
	float		const_E;
	float		const_R;
	float		const_Ld;
	float		const_Lq;
	int		const_Zp;
	float		const_J;

	float		i_maximal;
	float		i_derated;
	float		i_watt_consumption_maximal;
	float		i_watt_regeneration_maximal;
	float		i_setpoint_D;
	float		i_setpoint_Q;
	float		i_slew_rate_D;
	float		i_slew_rate_Q;
	float		i_track_D;
	float		i_track_Q;
	float		i_integral_D;
	float		i_integral_Q;
	float		i_gain_PD;
	float		i_gain_ID;
	float		i_gain_PQ;
	float		i_gain_IQ;

	float		s_maximal;
	float		s_setpoint;
	float		s_accel;
	float		s_track;
	float		s_integral;
	float		s_gain_P;
	float		s_gain_I;

	float		p_setpoint_DQ[2];
	float		p_setpoint_s;
	int		p_setpoint_revol;
	float		p_gain_P;
	float		p_gain_I;

	void 		(* pDC) (int, int, int);
	void 		(* pZ) (int);
}
pmc_t;

void pm_config_default(pmc_t *pm);
void pm_config_tune_current_loop(pmc_t *pm);

void pm_voltage_control(pmc_t *pm, float uX, float uY);
void pm_feedback(pmc_t *pm, pmfb_t *fb);

void pm_FSM(pmc_t *pm);
void pm_fsm_req(pmc_t *pm, int req);

const char *pm_strerror(int n);

#endif /* _H_PM_ */

