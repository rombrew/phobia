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

#include "pm.h"
#include "pm_m.h"

static void
pm_fsm_current_halt(pmc_t *pm, float i_halt)
{
	if (pm_fabsf(pm->fb_current_A) > i_halt) {

		pm->fail_reason = PM_ERROR_OVER_CURRENT;
		pm->fsm_state = PM_STATE_HALT;
		pm->fsm_phase = 0;
	}

	if (pm_fabsf(pm->fb_current_B) > i_halt) {

		pm->fail_reason = PM_ERROR_OVER_CURRENT;
		pm->fsm_state = PM_STATE_HALT;
		pm->fsm_phase = 0;
	}
}

static void
pm_fsm_state_zero_drift(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(7);

			pm->temporal[0] = 0.f;
			pm->temporal[1] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_skip;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_average_probe;

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			pm->temporal[0] += pm->fb_current_A;
			pm->temporal[1] += pm->fb_current_B;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->temporal[0] /= pm->tm_end;
				pm->temporal[1] /= pm->tm_end;

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			pm->adjust_IA[0] += - pm->temporal[0];
			pm->adjust_IB[0] += - pm->temporal[1];

			if (pm_fabsf(pm->adjust_IA[0]) > pm->fault_current_tolerance)
				pm->fail_reason = PM_ERROR_ZERO_DRIFT_FAULT;
			else if (pm_fabsf(pm->adjust_IB[0]) > pm->fault_current_tolerance)
				pm->fail_reason = PM_ERROR_ZERO_DRIFT_FAULT;

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	pm_fsm_current_halt(pm, pm->fault_current_halt_level);
}

static void
pm_fsm_state_power_stage_self_test(pmc_t *pm)
{
	int			bm;

	switch (pm->fsm_phase) {

		case 0:
			pm->fail_reason = PM_OK;

			pm->fsm_phase = 1;
			pm->fsm_phase_2 = 0;
			break;

		case 1:
			switch (pm->fsm_phase_2) {

				case 0:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(7);
					break;

				case 1:
					pm->proc_set_DC(pm->pwm_resolution, 0, 0);
					pm->proc_set_Z(6);
					break;

				case 2:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(6);
					break;

				case 3:
					pm->proc_set_DC(0, pm->pwm_resolution, 0);
					pm->proc_set_Z(5);
					break;

				case 4:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(5);
					break;

				case 5:
					pm->proc_set_DC(0, 0, pm->pwm_resolution);
					pm->proc_set_Z(3);
					break;

				case 6:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(3);
					break;
			}

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_voltage_hold;

			pm->fsm_phase = 2;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->temporal[0] = 0.f;
				pm->temporal[1] = 0.f;
				pm->temporal[2] = 0.f;

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_instant_probe;

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			pm->temporal[0] += pm->fb_voltage_A;
			pm->temporal[1] += pm->fb_voltage_B;
			pm->temporal[2] += pm->fb_voltage_C;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->temporal[0] /= pm->tm_end;
				pm->temporal[1] /= pm->tm_end;
				pm->temporal[2] /= pm->tm_end;

				pm->fsm_phase = 4;
			}
			break;

		case 4:
			bm = 0;

			bm |= (pm_fabsf(pm->temporal[0] - pm->const_lpf_U) < pm->fault_voltage_tolerance)
				? 1 : (pm_fabsf(pm->temporal[0]) < pm->fault_voltage_tolerance) ? 0 : 0x10;
			bm |= (pm_fabsf(pm->temporal[1] - pm->const_lpf_U) < pm->fault_voltage_tolerance)
				? 2 : (pm_fabsf(pm->temporal[1]) < pm->fault_voltage_tolerance) ? 0 : 0x20;
			bm |= (pm_fabsf(pm->temporal[2] - pm->const_lpf_U) < pm->fault_voltage_tolerance)
				? 4 : (pm_fabsf(pm->temporal[2]) < pm->fault_voltage_tolerance) ? 0 : 0x40;

			pm->self_BM[pm->fsm_phase_2] = bm;

			if (pm->fsm_phase_2 < 6) {

				pm->fsm_phase = 1;
				pm->fsm_phase_2++;
			}
			else {
				pm->fsm_phase = 5;
			}
			break;

		case 5:
			if (1		&& pm->self_BM[1] == 7
					&& pm->self_BM[2] == 0
					&& pm->self_BM[3] == 7
					&& pm->self_BM[4] == 0
					&& pm->self_BM[5] == 7
					&& pm->self_BM[6] == 0) {

				pm->fail_reason = PM_OK;
			}
			else if (1	&& (pm->self_BM[1] & 1) != 0
					&& (pm->self_BM[2] & 1) == 0
					&& (pm->self_BM[3] & 2) != 0
					&& (pm->self_BM[4] & 2) == 0
					&& (pm->self_BM[5] & 4) != 0
					&& (pm->self_BM[6] & 4) == 0) {

				pm->fail_reason = PM_ERROR_NO_MOTOR_CONNECTED;
			}
			else {
				pm->fail_reason = PM_ERORR_POWER_STAGE_FAULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	pm_fsm_current_halt(pm, pm->fault_current_halt_level);
}

static void
pm_fsm_state_sampling_accuracy_self_test(pmc_t *pm)
{
	int				xDC;

	switch (pm->fsm_phase) {

		case 0:
			pm->fail_reason = PM_OK;

			pm->fsm_phase = 1;
			pm->fsm_phase_2 = 0;
			break;

		case 1:
			switch (pm->fsm_phase_2) {

				case 0:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(7);
					break;

				case 1:
					xDC = pm->pwm_resolution - pm->pwm_silence_gap;
					pm->proc_set_DC(xDC, xDC, xDC);
					pm->proc_set_Z(0);
					break;
			}

			pm->temporal[0] = 0.f;
			pm->temporal[1] = 0.f;
			pm->temporal[2] = 0.f;
			pm->temporal[3] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_skip;

			pm->fsm_phase = 2;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_average_probe;

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			pm->temporal[0] += pm->fb_current_A;
			pm->temporal[1] += pm->fb_current_B;
			pm->temporal[2] += pm->fb_current_A * pm->fb_current_A;
			pm->temporal[3] += pm->fb_current_B * pm->fb_current_B;

			pm->tm_value++;

			if (pm->tm_value & 1) {

				xDC = pm->pwm_resolution - pm->pwm_silence_gap;
				pm->proc_set_DC(xDC, xDC, xDC);
			}
			else {
				xDC = pm->pwm_resolution;
				pm->proc_set_DC(xDC, xDC, xDC);
			}

			if (pm->tm_value >= pm->tm_end) {

				pm->temporal[0] = pm->temporal[0] / pm->tm_end;
				pm->temporal[1] = pm->temporal[1] / pm->tm_end;
				pm->temporal[2] = pm_sqrtf(pm->temporal[2] / pm->tm_end);
				pm->temporal[3] = pm_sqrtf(pm->temporal[3] / pm->tm_end);

				pm->fsm_phase = 4;
			}
			break;

		case 4:
			if (pm_fabsf(pm->temporal[2]) > pm->fault_current_tolerance)
				pm->fail_reason = PM_ERROR_SAMPLING_ACCURACY_FAULT;
			else if (pm_fabsf(pm->temporal[3]) > pm->fault_current_tolerance)
				pm->fail_reason = PM_ERROR_SAMPLING_ACCURACY_FAULT;

			switch (pm->fsm_phase_2) {

				case 0:
					pm->self_SA[0] = pm->temporal[0];
					pm->self_SA[1] = pm->temporal[1];
					pm->self_SA[2] = pm->temporal[2];
					pm->self_SA[3] = pm->temporal[3];

					pm->fsm_phase = 1;
					pm->fsm_phase_2++;
					break;

				case 1:
					pm->self_SA[4] = pm->temporal[0];
					pm->self_SA[5] = pm->temporal[1];
					pm->self_SA[6] = pm->temporal[2];
					pm->self_SA[7] = pm->temporal[3];

					pm->fsm_state = PM_STATE_HALT;
					pm->fsm_phase = 0;
					break;
			}
			break;
	}

	pm_fsm_current_halt(pm, pm->fault_current_halt_level);
}

static void
pm_fsm_state_adjust_current(pmc_t *pm)
{
	float			eX, uX, mean;
	float			uMAX;

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(4);

			pm->vsi_clamp_to_null = 0;

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;

			pm->temporal[0] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_skip;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			pm->probe_DFT[0] += pm->fb_current_A;
			pm->probe_DFT[1] += - pm->fb_current_B;

		case 1:
			eX = pm->probe_current_hold - pm->fb_current_A;
			pm->temporal[0] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->temporal[0];

			uMAX = (2.f / 3.f) * pm->const_lpf_U;

			if (pm_fabsf(uX) > uMAX) {

				pm->fail_reason = PM_ERROR_CURRENT_LOOP_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}

			pm_voltage_control(pm, uX, 0.f);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_average_probe;

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm->probe_DFT[0] /= pm->tm_end;
			pm->probe_DFT[1] /= pm->tm_end;

			mean = (pm->probe_DFT[1] + pm->probe_DFT[0]) / 2.f;
			pm->adjust_IA[1] *= mean / pm->probe_DFT[0];
			pm->adjust_IB[1] *= mean / pm->probe_DFT[1];

			if (pm_fabsf(pm->adjust_IA[1] - 1.f) > pm->fault_adjust_tolerance)
				pm->fail_reason = PM_ERROR_ADJUST_TOLERANCE_FAULT;
			else if (pm_fabsf(pm->adjust_IB[1] - 1.f) > pm->fault_adjust_tolerance)
				pm->fail_reason = PM_ERROR_ADJUST_TOLERANCE_FAULT;

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	pm_fsm_current_halt(pm, pm_fabsf(pm->probe_current_hold)
			+ pm->fault_current_halt_level);
}

static void
pm_fsm_state_probe_const_r(pmc_t *pm)
{
	float			eX, eY, uX, uY;
	float			uMAX;

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(0);

			pm->vsi_clamp_to_null = 0;

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = pm->probe_current_hold;
			pm->probe_DFT[3] = pm->probe_current_hold_Q;

			pm->temporal[0] = 0.f;
			pm->temporal[1] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_current_hold;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			pm->probe_DFT[0] += pm->vsi_X - pm->probe_current_hold * pm->const_R;
			pm->probe_DFT[1] += pm->vsi_Y - pm->probe_current_hold_Q * pm->const_R;

		case 1:
			pm_lu_get_current(pm);

			eX = pm->probe_current_hold - pm->lu_fb_X;
			eY = pm->probe_current_hold_Q - pm->lu_fb_Y;

			pm->temporal[0] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->temporal[0];

			pm->temporal[1] += pm->probe_gain_I * eY;
			uY = pm->probe_gain_P * eY + pm->temporal[1];

			uMAX = (2.f / 3.f) * pm->const_lpf_U;

			if (pm_fabsf(uX) > uMAX || pm_fabsf(uY) > uMAX) {

				pm->fail_reason = PM_ERROR_CURRENT_LOOP_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}

			pm_voltage_control(pm, uX, uY);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_average_probe;

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm->probe_DFT[0] /= pm->tm_end;
			pm->probe_DFT[1] /= pm->tm_end;

			pm->const_R += pm_DFT_const_R(pm->probe_DFT);

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	pm_fsm_current_halt(pm, pm_fabsf(pm->probe_current_hold)
			+ pm->fault_current_halt_level);
}

static void
pm_fsm_state_probe_const_l(pmc_t *pm)
{
	float			uX, uY;
	float			imp_Z, LDQ[4];

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(0);

			pm->vsi_clamp_to_null = 0;

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = 0.f;
			pm->probe_DFT[3] = 0.f;
			pm->probe_DFT[4] = 0.f;
			pm->probe_DFT[5] = 0.f;
			pm->probe_DFT[6] = 0.f;
			pm->probe_DFT[7] = 0.f;

			pm->temporal[0] = 1.f;
			pm->temporal[1] = 0.f;
			pm->temporal[2] = 2.f * M_PI_F * pm->probe_freq_sine_hz / pm->freq_hz;

			imp_Z = (pm->const_Ld < pm->const_Lq) ? pm->const_Ld : pm->const_Lq;
			imp_Z = 2.f * M_PI_F * imp_Z * pm->probe_freq_sine_hz;
			imp_Z = pm->const_R * pm->const_R + imp_Z * imp_Z;
			pm->temporal[3] = pm->probe_current_sine * pm_sqrtf(imp_Z);

			if (pm->probe_current_hold_Q != 0.f) {

				pm->temporal[4] = pm->probe_current_hold * pm->const_R;
				pm->temporal[5] = pm->probe_current_hold_Q * pm->const_R;
			}
			else {
				pm->temporal[4] = 0.f;
				pm->temporal[5] = 0.f;
			}

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_skip;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			pm_lu_get_current(pm);

			pm->probe_DFT[0] += pm->lu_fb_X * pm->temporal[0];
			pm->probe_DFT[1] += pm->lu_fb_X * pm->temporal[1];
			pm->probe_DFT[2] += pm->vsi_X * pm->temporal[0];
			pm->probe_DFT[3] += pm->vsi_X * pm->temporal[1];
			pm->probe_DFT[4] += pm->lu_fb_Y * pm->temporal[0];
			pm->probe_DFT[5] += pm->lu_fb_Y * pm->temporal[1];
			pm->probe_DFT[6] += pm->vsi_Y * pm->temporal[0];
			pm->probe_DFT[7] += pm->vsi_Y * pm->temporal[1];

		case 1:
			uX = pm->temporal[4] + pm->temporal[3] * pm->temporal[0];
			uY = pm->temporal[5] + pm->temporal[3] * pm->temporal[1];

			pm_voltage_control(pm, uX, uY);

			pm_rotf(pm->temporal, pm->temporal[2], pm->temporal);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_average_probe;

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm_DFT_const_L(pm->probe_DFT, pm->probe_freq_sine_hz, LDQ);

			if (pm->config_LDQ == PM_LDQ_SALIENT_POLE) {

				pm->const_Ld = LDQ[1];
				pm->const_Lq = LDQ[0];
			}
			else if (pm->config_LDQ == PM_LDQ_SATURATION_SALIENCY) {

				pm->const_Ld = LDQ[0];
				pm->const_Lq = LDQ[1];
			}
			else {
				pm->const_Ld = LDQ[0];
				pm->const_Lq = LDQ[1];
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	pm_fsm_current_halt(pm, pm_fabsf(pm->probe_current_sine)
			+ pm->fault_current_halt_level);
}

static void
pm_fsm_state_lu_initiate(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			if (pm->const_R != 0.f && pm->const_Ld != 0.f && pm->const_Lq != 0.f) {

				pm->lu_mode = PM_LU_DETACHED;
				pm->lu_revol = 0;

				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(7);

				pm->temporal[0] = 0.f;
				pm->temporal[1] = 0.f;

				pm->vsi_lpf_D = 0.f;
				pm->vsi_lpf_Q = 0.f;
				pm->vsi_lpf_watt = 0.f;
				pm->vsi_clamp_to_null = 1;

				pm->flux_X[0] = 0.f;
				pm->flux_X[1] = 0.f;
				pm->flux_X[2] = 1.f;
				pm->flux_X[3] = 0.f;
				pm->flux_X[4] = 0.f;
				pm->flux_drift_Q = 0.f;
				pm->flux_residual_lpf = 0.f;

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_startup;

				pm->fail_reason = PM_OK;
				pm->fsm_phase = 1;
			}
			else {
				pm->fail_reason = PM_ERROR_LU_INVALID_OPERATION;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}
			break;

		case 1:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			pm->lu_mode = PM_LU_ESTIMATE_FLUX;
			
			pm->proc_set_Z(0);

			pm->forced_setpoint = 0.f;

			pm->hfi_CS[0] = 1.f;
			pm->hfi_CS[1] = 0.f;
			pm->hfi_flux_polarity = 0.f;

			pm->i_derated = pm->i_maximal;
			pm->i_setpoint_D = 0.f;
			pm->i_setpoint_Q = 0.f;
			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			pm->s_setpoint = 0.f;
			pm->s_track = 0.f;
			pm->s_integral = 0.f;

			/*pm->p_setpoint[0] = 1.f;
			pm->p_setpoint[1] = 0.f;
			pm->p_setpoint_s = 0.f;
			pm->p_setpoint_revol = 0;*/

			pm->fsm_state = PM_STATE_IDLE;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_lu_shutdown(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			pm->i_setpoint_D = 0.f;
			pm->i_setpoint_Q = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_skip;

			pm->fsm_phase = 1;

		case 1:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->lu_mode = PM_LU_DISABLED;

				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(7);

				pm->fsm_state = PM_STATE_IDLE;
				pm->fsm_phase = 0;
			}
			break;
	}
}

static void
pm_fsm_state_probe_const_e(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_average_probe;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm->probe_DFT[0] += pm->flux_X[4];
			pm->probe_DFT[1] += pm->flux_drift_Q;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end)
				pm->fsm_phase = 2;
			break;

		case 2:
			pm->const_E += - pm->probe_DFT[1] / pm->probe_DFT[0];

			pm->fsm_state = PM_STATE_IDLE;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_probe_const_j(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			break;

		case 3:
			/*J0 = pm->const_E * (X1 * pm->tm_probe)
				* 1.5f * (pm->const_Zp * pm->const_Zp)
				* pm->dT / (X41 - X40);*/
			break;
	}
}

static void
pm_fsm_state_halt(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			pm->lu_mode = PM_LU_DISABLED;

			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(7);

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_skip;

			pm->fsm_phase = 1;

		case 1:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_state = PM_STATE_IDLE;
				pm->fsm_phase = 0;
			}
			break;
	}
}

void pm_FSM(pmc_t *pm)
{
	switch (pm->fsm_state) {

		case PM_STATE_IDLE:
			break;

		case PM_STATE_ZERO_DRIFT:
			pm_fsm_state_zero_drift(pm);
			break;

		case PM_STATE_POWER_STAGE_SELF_TEST:
			pm_fsm_state_power_stage_self_test(pm);
			break;

		case PM_STATE_SAMPLING_ACCURACY_SELF_TEST:
			pm_fsm_state_sampling_accuracy_self_test(pm);
			break;

		case PM_STATE_ADJUST_VOLTAGE:
			break;

		case PM_STATE_ADJUST_CURRENT:
			pm_fsm_state_adjust_current(pm);
			break;

		case PM_STATE_PROBE_CONST_R:
			pm_fsm_state_probe_const_r(pm);
			break;

		case PM_STATE_PROBE_CONST_L:
			pm_fsm_state_probe_const_l(pm);
			break;

		case PM_STATE_LU_INITIATE:
			pm_fsm_state_lu_initiate(pm);
			break;

		case PM_STATE_LU_SHUTDOWN:
			pm_fsm_state_lu_shutdown(pm);
			break;

		case PM_STATE_PROBE_CONST_E:
			pm_fsm_state_probe_const_e(pm);
			break;

		case PM_STATE_PROBE_CONST_J:
			pm_fsm_state_probe_const_j(pm);
			break;

		case PM_STATE_HALT:
		default:
			pm_fsm_state_halt(pm);
	}
}

void pm_fsm_req(pmc_t *pm, int req)
{
	switch (req) {

		case PM_STATE_ZERO_DRIFT:
		case PM_STATE_POWER_STAGE_SELF_TEST:
		case PM_STATE_SAMPLING_ACCURACY_SELF_TEST:
		case PM_STATE_ADJUST_VOLTAGE:
		case PM_STATE_ADJUST_CURRENT:
		case PM_STATE_PROBE_CONST_R:
		case PM_STATE_PROBE_CONST_L:
		case PM_STATE_LU_INITIATE:

			if (pm->fsm_state != PM_STATE_IDLE)
				break;

			if (pm->lu_mode != PM_LU_DISABLED)
				break;

			pm->fsm_state = req;
			pm->fsm_phase = 0;
			break;

		case PM_STATE_LU_SHUTDOWN:
		case PM_STATE_PROBE_CONST_E:
		case PM_STATE_PROBE_CONST_J:

			if (pm->fsm_state != PM_STATE_IDLE)
				break;

			if (pm->lu_mode == PM_LU_DISABLED)
				break;

			pm->fsm_state = req;
			pm->fsm_phase = 0;
			break;

		case PM_STATE_HALT:

			pm->fsm_state = req;
			pm->fsm_phase = 0;
			break;

		default:
			break;
	}
}

const char *pm_strerror(int n)
{
	const char	*list[] = {

		PM_SFI(PM_OK),
		PM_SFI(PM_ERROR_ZERO_DRIFT_FAULT),
		PM_SFI(PM_ERROR_NO_MOTOR_CONNECTED),
		PM_SFI(PM_ERORR_POWER_STAGE_FAULT),
		PM_SFI(PM_ERROR_SAMPLING_ACCURACY_FAULT),
		PM_SFI(PM_ERROR_CURRENT_LOOP_FAULT),
		PM_SFI(PM_ERROR_OVER_CURRENT),
		PM_SFI(PM_ERROR_ADJUST_TOLERANCE_FAULT),
		PM_SFI(PM_ERROR_LU_RESIDUAL_UNSTABLE),
		PM_SFI(PM_ERROR_LU_INVALID_OPERATION)
	};

	const int 	lmax = sizeof(list) / sizeof(list[0]);

	return (n >= 0 && n < lmax) ? list[n] : "";
}

