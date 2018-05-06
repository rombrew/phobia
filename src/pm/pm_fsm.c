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
pm_fsm_state_zero_drift(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			pm->pDC(0, 0, 0);
			pm->pZ(7);

			pm->temp[0] = 0.f;
			pm->temp[1] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_skip;

			pm->err_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_probe;

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			pm->temp[0] += pm->fb_iA;
			pm->temp[1] += pm->fb_iB;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end)
				pm->fsm_phase = 3;
			break;

		case 3:
			pm->adjust_IA[0] -= pm->temp[0] / pm->tm_end;
			pm->adjust_IB[0] -= pm->temp[1] / pm->tm_end;

			if (fabsf(pm->adjust_IA[0]) > pm->fault_zero_drift_maximal)
				pm->err_reason = PM_ERROR_ZERO_DRIFT_FAULT;
			else if (fabsf(pm->adjust_IB[0]) > pm->fault_zero_drift_maximal) 
				pm->err_reason = PM_ERROR_ZERO_DRIFT_FAULT;

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_power_stage_test(pmc_t *pm)
{
	int			bits;

	switch (pm->fsm_phase) {

		case 0:
			pm->fsm_phase_2 = 0;
			pm->fsm_phase = 1;
			break;

		case 1:
			switch (pm->fsm_phase_2) {

				case 0:
					pm->pDC(0, 0, 0);
					pm->pZ(7);
					break;

				case 1:
					pm->pDC(pm->pwm_R, 0, 0);
					pm->pZ(6);
					break;

				case 2:
					pm->pDC(0, 0, 0);
					pm->pZ(6);
					break;

				case 3:
					pm->pDC(0, pm->pwm_R, 0);
					pm->pZ(5);
					break;

				case 4:
					pm->pDC(0, 0, 0);
					pm->pZ(5);
					break;

				case 5:
					pm->pDC(0, 0, pm->pwm_R);
					pm->pZ(3);
					break;

				case 6:
					pm->pDC(0, 0, 0);
					pm->pZ(3);
					break;
			}

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_skip;

			pm->fsm_phase = 2;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->temp[0] = 0.f;
				pm->temp[1] = 0.f;
				pm->temp[2] = 0.f;

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_probe;

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			pm->temp[0] += pm->fb_uA;
			pm->temp[1] += pm->fb_uB;
			pm->temp[2] += pm->fb_uC;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->temp[0] /= pm->tm_end;
				pm->temp[1] /= pm->tm_end;
				pm->temp[2] /= pm->tm_end;

				pm->fsm_phase = 4;
			}
			break;

		case 4:
			bits = 0;

			bits |= (fabsf(pm->temp[0] - pm->const_lpf_U) < pm->fault_voltage_tolerance)
				? 1 : (fabsf(pm->temp[0]) < pm->fault_voltage_tolerance) ? 0 : 16;
			bits |= (fabsf(pm->temp[1] - pm->const_lpf_U) < pm->fault_voltage_tolerance)
				? 2 : (fabsf(pm->temp[1]) < pm->fault_voltage_tolerance) ? 0 : 32;
			bits |= (fabsf(pm->temp[2] - pm->const_lpf_U) < pm->fault_voltage_tolerance)
				? 4 : (fabsf(pm->temp[2]) < pm->fault_voltage_tolerance) ? 0 : 64;

			pm->err_bb[pm->fsm_phase_2] = bits;

			if (pm->fsm_phase_2 < 6) {

				pm->fsm_phase = 1;
				pm->fsm_phase_2++;
			}
			else {
				pm->fsm_phase = 5;
			}
			break;

		case 5:
			if (		pm->err_bb[0] == 0
					&& pm->err_bb[1] == 7
					&& pm->err_bb[2] == 0
					&& pm->err_bb[3] == 7
					&& pm->err_bb[4] == 0
					&& pm->err_bb[5] == 7
					&& pm->err_bb[6] == 0) {

				pm->err_reason = PM_OK;
			}
			else if (	pm->err_bb[0] == 0
					&& (pm->err_bb[1] & 1) != 0
					&& (pm->err_bb[2] & 1) == 0
					&& (pm->err_bb[3] & 2) != 0
					&& (pm->err_bb[4] & 2) == 0
					&& (pm->err_bb[5] & 4) != 0
					&& (pm->err_bb[6] & 4) == 0) {

				pm->err_reason = PM_ERROR_NO_MOTOR_CONNECTED;
			}
			else {
				pm->err_reason = PM_ERORR_POWER_STAGE_FAULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	if (fabsf(pm->fb_iA) > pm->fault_current_tolerance) {

		pm->err_reason = PM_ERROR_OVER_CURRENT;
		pm->fsm_state = PM_STATE_HALT;
		pm->fsm_phase = 0;
	}

	if (fabsf(pm->fb_iB) > pm->fault_current_tolerance) {

		pm->err_reason = PM_ERROR_OVER_CURRENT;
		pm->fsm_state = PM_STATE_HALT;
		pm->fsm_phase = 0;
	}
}

static void
pm_fsm_state_adjust_current(pmc_t *pm)
{
	float			eA, uX, mean;
	float			uMAX;

	switch (pm->fsm_phase) {

		case 0:
			pm->pDC(0, 0, 0);
			pm->pZ(4);

			pm->temp[0] = 0.f;
			pm->temp[1] = 0.f;
			pm->temp[2] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_skip;

			pm->err_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			pm->temp[1] += pm->fb_iA;
			pm->temp[2] += - pm->fb_iB;

		case 1:
			eA = pm->probe_i_hold - pm->fb_iA;
			pm->temp[0] += pm->probe_gain_I * eA;
			uX = pm->probe_gain_P * eA + pm->temp[0];

			uMAX = (2.f / 3.f) * pm->const_lpf_U;

			if (fabsf(uX) > uMAX) {

				pm->err_reason = PM_ERROR_CURRENT_LOOP_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}

			pm_VSI_control(pm, uX / pm->const_lpf_U, 0.f);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_probe;

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm->temp[1] /= pm->tm_end;
			pm->temp[2] /= pm->tm_end;

			mean = (pm->temp[2] + pm->temp[1]) / 2.f;
			pm->adjust_IA[1] *= mean / pm->temp[1];
			pm->adjust_IB[1] *= mean / pm->temp[2];

			if (fabsf(pm->adjust_IA[1] - 1.f) > pm->fault_adjust_tolerance)
				pm->err_reason = PM_ERROR_ADJUST_TOLERANCE_FAULT;
			else if (fabsf(pm->adjust_IB[1] - 1.f) > pm->fault_adjust_tolerance)
				pm->err_reason = PM_ERROR_ADJUST_TOLERANCE_FAULT;

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_probe_const_r(pmc_t *pm)
{
	float			iX, iY, eX, eY, uX, uY;
	float			uMAX;

	switch (pm->fsm_phase) {

		case 0:
			pm->pDC(0, 0, 0);
			pm->pZ(0);

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = pm->probe_i_hold;
			pm->probe_DFT[3] = pm->probe_i_hold_Q;

			pm->temp[0] = 0.f;
			pm->temp[1] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_hold;

			pm->err_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			pm->probe_DFT[0] += pm->vsi_X - pm->probe_i_hold * pm->const_R;
			pm->probe_DFT[1] += pm->vsi_Y - pm->probe_i_hold_Q * pm->const_R;

		case 1:
			iX = pm->fb_iA;
			iY = .57735027f * pm->fb_iA + 1.1547005f * pm->fb_iB;

			eX = pm->probe_i_hold - iX;
			eY = pm->probe_i_hold_Q - iY;

			pm->temp[0] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->temp[0];

			pm->temp[1] += pm->probe_gain_I * eY;
			uY = pm->probe_gain_P * eY + pm->temp[1];

			uMAX = (2.f / 3.f) * pm->const_lpf_U;

			if (fabsf(uX) > uMAX || fabsf(uY) > uMAX) {

				pm->err_reason = PM_ERROR_CURRENT_LOOP_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}

			pm_VSI_control(pm, uX / pm->const_lpf_U,
					uY / pm->const_lpf_U);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_probe;

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
}

static void
pm_fsm_state_probe_const_l(pmc_t *pm)
{
	float			iX, iY, uX, uY;
	float			Z, LDQ[3];

	switch (pm->fsm_phase) {

		case 0:
			pm->pDC(0, 0, 0);
			pm->pZ(0);

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = 0.f;
			pm->probe_DFT[3] = 0.f;
			pm->probe_DFT[4] = 0.f;
			pm->probe_DFT[5] = 0.f;
			pm->probe_DFT[6] = 0.f;
			pm->probe_DFT[7] = 0.f;

			pm->temp[0] = 1.f;
			pm->temp[1] = 0.f;
			pm->temp[2] = 2.f * M_PI_F * pm->probe_freq_sine_hz / pm->freq_hz;

			Z = (pm->const_Ld < pm->const_Lq) ? pm->const_Ld : pm->const_Lq;
			Z = 2.f * M_PI_F * Z * pm->probe_freq_sine_hz;
			Z = pm->const_R * pm->const_R + Z * Z;
			pm->temp[3] = pm->probe_i_sine * sqrtf(Z);

			if (fabsf(pm->probe_i_hold_Q) > M_EPS_F) {

				pm->temp[4] = pm->probe_i_hold * pm->const_R;
				pm->temp[5] = pm->probe_i_hold_Q * pm->const_R;
			}
			else {
				pm->temp[4] = 0.f;
				pm->temp[5] = 0.f;
			}

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_skip;

			pm->err_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			iX = pm->fb_iA;
			iY = .57735027f * pm->fb_iA + 1.1547005f * pm->fb_iB;

			pm->probe_DFT[0] += iX * pm->temp[0];
			pm->probe_DFT[1] += iX * pm->temp[1];
			pm->probe_DFT[2] += pm->vsi_X * pm->temp[0];
			pm->probe_DFT[3] += pm->vsi_X * pm->temp[1];
			pm->probe_DFT[4] += iY * pm->temp[0];
			pm->probe_DFT[5] += iY * pm->temp[1];
			pm->probe_DFT[6] += pm->vsi_Y * pm->temp[0];
			pm->probe_DFT[7] += pm->vsi_Y * pm->temp[1];

		case 1:
			uX = pm->temp[4] + pm->temp[3] * pm->temp[0];
			uY = pm->temp[5] + pm->temp[3] * pm->temp[1];

			pm_VSI_control(pm, uX / pm->const_lpf_U,
					uY / pm->const_lpf_U);

			pm_rotf(pm->temp, pm->temp[2], pm->temp);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_probe;

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm_DFT_const_L(pm->probe_DFT, pm->probe_freq_sine_hz, LDQ);

			if (fabsf(LDQ[2]) < (M_PI_F / 4.f)) {

				pm->const_Ld = LDQ[0];
				pm->const_Lq = LDQ[1];
			}
			else {
				pm->const_Ld = LDQ[1];
				pm->const_Lq = LDQ[0];
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_lu_initiate(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			pm->temp[0] = 0.f;
			pm->temp[1] = 0.f;

			pm->lu_region = (pm->b_FORCED != 0)
				? PM_LU_OPEN_LOOP : PM_LU_CLOSED_LOW;

			pm->lu_power_lpf = 0.f;
			pm->lu_residual_lpf = 0.f;
			
			pm->lu_X[0] = 0.f;
			pm->lu_X[1] = 0.f;
			pm->lu_X[2] = 1.f;
			pm->lu_X[3] = 0.f;
			pm->lu_X[4] = 0.f;
			pm->lu_drift_Q = 0.f;
			pm->lu_revol = 0;

			pm->hf_CS[0] = 1.;
			pm->hf_CS[1] = 0.;
			pm->hf_flux_polarity = 0.f;

			pm->i_set_point_D = 0.f;
			pm->i_set_point_Q = 0.f;
			pm->i_track_point_D = 0.f;
			pm->i_track_point_Q = 0.f;
			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			pm->s_set_point = 0.f;
			pm->s_track_point = 0.f;
			pm->s_integral = 0.f;

			/*pm->p_set_point[0] = 1.f;
			pm->p_set_point[1] = 0.f;
			pm->p_set_point_s = 0.f;
			pm->p_set_point_revol = 0;*/

			pm->pDC(0, 0, 0);
			pm->pZ(0);

			pm->err_reason = PM_OK;
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
			pm->i_set_point_D = 0.f;
			pm->i_set_point_Q = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_skip;

			pm->fsm_phase = 1;

		case 1:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->lu_region = PM_LU_DISABLED;

				pm->pDC(0, 0, 0);
				pm->pZ(7);

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
			pm->temp[0] = 0.f;
			pm->temp[1] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_probe;

			pm->err_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm->temp[0] += pm->lu_X[4];
			pm->temp[1] += pm->lu_drift_Q;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end)
				pm->fsm_phase = 2;
			break;

		case 2:
			pm->const_E += - pm->temp[1] / pm->temp[0];

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
			pm->lu_region = PM_LU_DISABLED;

			pm->pDC(0, 0, 0);
			pm->pZ(7);

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_skip;

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

		case PM_STATE_POWER_STAGE_TEST:
			pm_fsm_state_power_stage_test(pm);
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
		case PM_STATE_POWER_STAGE_TEST:
		case PM_STATE_ADJUST_VOLTAGE:
		case PM_STATE_ADJUST_CURRENT:
		case PM_STATE_PROBE_CONST_R:
		case PM_STATE_PROBE_CONST_L:
		case PM_STATE_LU_INITIATE:

			if (pm->fsm_state != PM_STATE_IDLE)
				break;

			if (pm->lu_region != PM_LU_DISABLED)
				break;

			pm->fsm_state = req;
			pm->fsm_phase = 0;
			break;

		case PM_STATE_LU_SHUTDOWN:

			if (pm->lu_region == PM_LU_DISABLED)
				break;

			pm->fsm_state = req;
			pm->fsm_phase = 0;
			break;

		case PM_STATE_PROBE_CONST_E:
		case PM_STATE_PROBE_CONST_J:

			if (pm->fsm_state != PM_STATE_IDLE)
				break;

			if (pm->lu_region == PM_LU_DISABLED)
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

		"OK",
		"Zero Drift Fault",
		"No Motor Connected",
		"Power Stage Fault",
		"Current Loop Fault",
		"Over Current",
		"Adjust Tolerance Fault",
		"Supply Voltage LOW",
		"Supply Voltage HIGH",
		"LU Residual Unstable",
		"LU Invalid Operation",
		"LU Speed HIGH",
		"LU Drift HIGH"
	};

	const int 	lmax = sizeof(list) / sizeof(list[0]);

	return (n >= 0 && n < lmax) ? list[n] : "";
}

