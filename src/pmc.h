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

#ifndef _H_PMC_
#define _H_PMC_

enum {
	PMC_BIT_HIGH_FREQUENCY_INJECTION	= 0x0002,
	PMC_BIT_FEWER_SWITCHING_MODULATION	= 0x0004,
	PMC_BIT_POSITION_CONTROL_LOOP		= 0x0020,
	PMC_BIT_UPDATE_R_AFTER_HOLD		= 0x0100,
	PMC_BIT_UPDATE_L_AFTER_SINE		= 0x0200,
};

enum {
	PMC_STATE_IDLE				= 0,
	PMC_STATE_ZERO_DRIFT,
	PMC_STATE_WAVE_HOLD,
	PMC_STATE_WAVE_SINE,
	PMC_STATE_CALIBRATION,
	PMC_STATE_START,
	PMC_STATE_STOP,
	PMC_STATE_END
};

enum {
	PMC_LU_DISABLED				= 0,
	PMC_LU_LOW_REGION,
	PMC_LU_HIGH_REGION
};

enum {
	PMC_OK					= 0,
	PMC_ERROR_BROKEN_HARDWARE,
	PMC_ERROR_INVALID_CONFIGURATION,
	PMC_ERROR_SYNC_LOST
};

typedef struct {

	/* Frequency (Hz).
	 * */
	float		freq_hz;
	float		dT;

	/* PWM configuration.
	 * */
	int		pwm_resolution;
	int		pwm_minimal_pulse;

	/* FSM variables.
	 * */
	int		m_bitmask;
	int		m_state;
	int		m_phase;
	int		m_errno;

	/* Timer variables.
	 * */
	int		t_value;
	int		t_end;

	/* Timer configuration.
	 * */
	float		T_drift;
	float		T_hold;
	float		T_sine;
	float		T_measure;
	float		T_end;

	/* Wave configuration.
	 * */
	float		wave_i_hold;
	float		wave_i_sine;
	float		wave_i_offset_D;
	float		wave_i_offset_Q;
	float		wave_freq_sine_hz;
	float		wave_gain_P;
	float		wave_gain_I;

	/* Scale constants.
	 * */
	float		scal_A[2];
	float		scal_B[2];
	float		scal_U[2];

	/* Actual VSI voltage.
	 * */
	float		vsi_X;
	float		vsi_Y;

	/* Luenberger observer.
	 * */
	float		lu_X[5];
	int		lu_revol;
	float		lu_gain_K[8];
	float		lu_low_threshold;
	float		lu_low_hysteresis;
	int		lu_region;

	/* Measurement residual.
	 * */
	float		lu_residual_D;
	float		lu_residual_Q;
	float		lu_residual_variance;

	/* HFI observer.
	 * */
	float		hf_freq_hz;
	float		hf_swing_D;
	float		hf_CS[2];

	/* Temporal.
	 * */
	float		temp_A[4];
	float		temp_B[8];

	/* Zero Drift.
	 * */
	float		drift_A;
	float		drift_B;
	float		drift_Q;
	float		drift_AB_maximal;
	float		drift_Q_maximal;

	/* Supply voltage.
	 * */
	float		const_U;
	float		const_U_inversed;
	float		const_U_gain_F;

	/* Electrical constants.
	 * */
	float		const_E;
	float		const_R;
	float		const_Ld;
	float		const_Ld_inversed;
	float		const_Lq;
	float		const_Lq_inversed;

	/* Mechanical constants.
	 * */
	int		const_Zp;

	/* Current control loop.
	 * */
	float		i_maximal;
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

	/* Position control loop.
	 * */
	float		p_set_point_x[2];
	int		p_set_point_revol;
	float		p_set_point_w;
	float		p_gain_D;
	float		p_gain_P;

	/* Informational.
	 * */
	float		i_power_watt;

	/* Control interface.
	 * */
	void 		(* pDC) (int, int, int);
	void 		(* pZ) (int);
}
pmc_t;

void pmc_default(pmc_t *pm);
void pmc_feedback(pmc_t *pm, int xA, int xB);
void pmc_voltage(pmc_t *pm, int xU);

void pmc_request(pmc_t *pm, int req);

#endif /* _H_PMC_ */

