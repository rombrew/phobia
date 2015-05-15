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
	PMC_BIT_KALMAN_FILTER			= 0x0001,
	PMC_BIT_QAXIS_DRIFT			= 0x0002,
	PMC_BIT_ABZERO_DRIFT			= 0x0004,

	PMC_BIT_LOW_REGION			= 0x0008,
	PMC_BIT_EFFICIENT_MODULATION		= 0x0010,
	PMC_BIT_SPEED_CONTROL_LOOP		= 0x0020,
	PMC_BIT_POSITION_CONTROL_LOOP		= 0x0040,
	PMC_BIT_DIRECT_INJECTION		= 0x0080,
	PMC_BIT_FREQUENCY_INJECTION		= 0x0100,

	PMC_BIT_HOLD_UPDATE_R			= 0x0200,
	PMC_BIT_SINE_UPDATE_L			= 0x0400,
	PMC_BIT_ESTIMATE_R			= 0x0800,
	PMC_BIT_ESTIMATE_E			= 0x1000,
	PMC_BIT_ESTIMATE_Q			= 0x2000,
};

enum {
	PMC_STATE_IDLE				= 0,
	PMC_STATE_ZERO_DRIFT,
	PMC_STATE_WAVE_HOLD,
	PMC_STATE_WAVE_SINE,
	PMC_STATE_CALIBRATION,
	PMC_STATE_ESTIMATE_Q,
	PMC_STATE_ESTIMATE_J,
	PMC_STATE_KALMAN_START,
	PMC_STATE_KALMAN_STOP,
	PMC_STATE_END
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
	int		m_request;
	int		m_bitmask;
	int		m_bitmode;
	int		m_state;
	int		m_phase;
	int		m_errno;

	/* Timer variables.
	 * */
	int		t_value;
	int		t_end;
	int		t_clock;

	/* Timer configuration.
	 * */
	float		T_drift;
	float		T_hold;
	float		T_rohm;
	float		T_sine;
	float		T_estq;
	float		T_end;

	/* Other configuration.
	 * */
	float		i_hold;
	float		i_sine;
	float		i_offset_D;
	float		i_offset_Q;
	float		freq_sine_hz;

	/* Conversion constants.
	 * */
	float		conv_A[2];
	float		conv_B[2];
	float		conv_U[2];

	/* Actual VSI voltage.
	 * */
	float		vsi_X;
	float		vsi_Y;

	/* Measurement residual.
	 * */
	float		residual_D;
	float		residual_Q;
	float		residual_variance;

	/* Extended Kalman Filter.
	 * */
	float		kalman_X[5];
	float		kalman_P[21];
	float		kalman_Q[6];
	float		kalman_R;

	/* Load torque.
	 * */
	float		var_M;

	/* Temporal.
	 * */
	float		temp_A[5];
	float		temp_B[8];

	/* Zero Drift.
	 * */
	float		drift_A;
	float		drift_B;
	float		drift_Q;
	float		drift_AB_maximal;
	float		drift_Q_maximal;

	/* Electrical constants.
	 * */
	float		const_U;
	float		const_E;
	float		const_R;
	float		const_Q;
	float		const_Ld, const_ILd;
	float		const_Lq, const_ILq;

	/* Mechanical constants.
	 * */
	int		const_Zp;
	float		const_M[3];
	float		const_IJ;

	/* Current control loop.
	 * */
	float		i_maximal;
	float		i_power_watt;
	float		i_power_consumption_maximal;
	float		i_power_regeneration_maximal;
	float		i_set_point_D;
	float		i_set_point_Q;
	float		i_integral_D;
	float		i_integral_Q;
	float		i_KP;
	float		i_KI;

	/* Direct injection.
	 * */
	float		i_inject_D;
	float		i_inject_gain;

	/* Frequency injection.
	 * */
	float		h_freq_hz;
	float		h_inject_D;
	float		h_CS[2];

	/* Speed control loop.
	 * */
	float		w_low_threshold;
	float		w_low_hysteresis;
	float		w_set_point;
	float		w_average[2];
	float		w_KP;

	/* Control interface.
	 * */
	void 		(* pDC) (int, int, int);
	void 		(* pZ) (int);
}
pmc_t;

void pmc_default(pmc_t *pm);
void pmc_feedback(pmc_t *pm, int xA, int xB);
void pmc_voltage(pmc_t *pm, int xU);
void pmc_gain_tune();

#endif /* _H_PMC_ */

