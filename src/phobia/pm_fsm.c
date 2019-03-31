#include "libm.h"
#include "pm.h"

static float
pm_DFT_R(const float DFT[8])
{
	float			D, X, Y, E, R = 0.;

	D = m_sqrtf(DFT[2] * DFT[2] + DFT[3] * DFT[3]);

	if (D > 0.f) {

		X = DFT[2] / D;
		Y = DFT[3] / D;

		E = DFT[0] * X + DFT[1] * Y;
		R = E / D;
	}

	return R;
}

static void
pm_DFT_EIG(float XY[3], float DQ[4])
{
	float		B, D, la1, la2;

	B = XY[0] + XY[2];
	D = B * B - 4.f * (XY[0] * XY[2] - XY[1] * XY[1]);

	if (D > 0.f) {

		D = m_sqrtf(D);
		la1 = (B - D) / 2.f;
		la2 = (B + D) / 2.f;

		B = XY[2] - la1;
		D = m_sqrtf(B * B + XY[1] * XY[1]);

		DQ[0] = la1;
		DQ[1] = la2;
		DQ[2] = B / D;
		DQ[3] = XY[1] / D;
	}
}

static void
pm_LSQ_3(const float LSQ[9], float X[3])
{
	float		LD[6], B[3];

	LD[0] = LSQ[0];
	LD[1] = LSQ[1] / LD[0];
	LD[3] = LSQ[3] / LD[0];
	LD[2] = LSQ[2] - LD[0] * LD[1] * LD[1];
	LD[4] = (LSQ[4] - LD[0] * LD[3] * LD[1]) / LD[2];
	LD[5] = LSQ[5] - (LD[0] * LD[3] * LD[3] + LD[2] * LD[4] * LD[4]);

	B[0] = LSQ[6];
	B[1] = LSQ[7] - LD[1] * B[0];
	B[2] = LSQ[8] - (LD[3] * B[0] + LD[4] * B[1]);

	X[2] = B[2] / LD[5];
	X[1] = B[1] / LD[2] - LD[4] * X[2];
	X[0] = B[0] / LD[0] - (LD[1] * X[1] + LD[3] * X[2]);
}

static void
pm_DFT_LDQ(const float DFT[8], float HZ, float LDQ[5])
{
	float		LSQ[9], B[4], LXY[3], R, WF;

	LSQ[0] = DFT[1] * DFT[1] + DFT[0] * DFT[0];
	LSQ[1] = - DFT[1] * DFT[5] - DFT[0] * DFT[4];
	LSQ[5] = DFT[5] * DFT[5] + DFT[4] * DFT[4];
	LSQ[2] = LSQ[5] + LSQ[0];
	LSQ[3] = 0.f;
	LSQ[4] = LSQ[1];

	R = (DFT[2] * DFT[0] + DFT[3] * DFT[1] + DFT[6] * DFT[4] + DFT[7] * DFT[5]) / LSQ[2];

	B[0] = DFT[2] - DFT[0] * R;
	B[1] = DFT[3] - DFT[1] * R;
	B[2] = DFT[6] - DFT[4] * R;
	B[3] = DFT[7] - DFT[5] * R;

	LSQ[6] = DFT[1] * B[0] - DFT[0] * B[1];
	LSQ[7] = - DFT[5] * B[0] + DFT[4] * B[1] - DFT[1] * B[2] + DFT[0] * B[3];
	LSQ[8] = DFT[5] * B[2] - DFT[4] * B[3];

	pm_LSQ_3(LSQ, LXY);

	WF = 2.f * M_PI_F * HZ;

	LXY[0] /= WF;
	LXY[1] /= WF;
	LXY[2] /= WF;

	pm_DFT_EIG(LXY, LDQ);

	LDQ[4] = R;
}

static void
pm_ADD(float *S, float *C, float X)
{
	float		fixed_X, up_S;

	/* Kahan summation algorithm.
	 * */

	fixed_X = X - *C;
	up_S = *S + fixed_X;

	*C = (up_S - *S) - fixed_X;
	*S = up_S;
}

static void
pm_fsm_current_halt(pmc_t *pm, float i_halt)
{
	if (m_fabsf(pm->fb_current_A) > i_halt) {

		pm->fail_reason = PM_ERROR_OVER_CURRENT;
		pm->fsm_state = PM_STATE_HALT;
		pm->fsm_phase = 0;
	}

	if (m_fabsf(pm->fb_current_B) > i_halt) {

		pm->fail_reason = PM_ERROR_OVER_CURRENT;
		pm->fsm_state = PM_STATE_HALT;
		pm->fsm_phase = 0;
	}
}

static void
pm_fsm_current_probe(pmc_t *pm)
{
	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		pm->probe_fb_X = pm->fb_current_A;
		pm->probe_fb_Y = .57735027f * pm->fb_current_A + 1.1547005f * pm->fb_current_B;
	}
	else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

		pm->probe_fb_X = pm->fb_current_A;
		pm->probe_fb_Y = pm->fb_current_B;
	}
}

static void
pm_fsm_state_zero_drift(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(7);

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->FIX[0] = 0.f;
			pm->FIX[1] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_skip;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_average_drift;

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->fb_current_A);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], pm->fb_current_B);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->probe_DFT[0] /= pm->tm_end;
				pm->probe_DFT[1] /= pm->tm_end;

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			pm->adjust_IA[0] += - pm->probe_DFT[0];
			pm->adjust_IB[0] += - pm->probe_DFT[1];

			if (		m_fabsf(pm->adjust_IA[0]) > pm->fault_current_tolerance
					|| m_fabsf(pm->adjust_IB[0]) > pm->fault_current_tolerance) {

				pm->fail_reason = PM_ERROR_ZERO_DRIFT_FAULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	pm_fsm_current_halt(pm, pm->fault_current_halt_level);
}

static void
pm_fsm_state_self_test_power_stage(pmc_t *pm)
{
	int			bm;

	switch (pm->fsm_phase) {

		case 0:
			pm->fail_reason = PM_OK;

			if (PM_CONFIG_VOLT(pm) == PM_ENABLED) {

				pm->fsm_phase = 1;
				pm->fsm_phase_2 = 0;
			}
			else {
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;

			}
			break;

		case 1:
			switch (pm->fsm_phase_2) {

				case 0:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(7);
					break;

				case 1:
					pm->proc_set_DC(pm->dc_resolution, 0, 0);
					pm->proc_set_Z(6);
					break;

				case 2:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(6);
					break;

				case 3:
					pm->proc_set_DC(0, pm->dc_resolution, 0);
					pm->proc_set_Z(5);
					break;

				case 4:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(5);
					break;

				case 5:
					pm->proc_set_DC(0, 0, pm->dc_resolution);
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

				pm->probe_DFT[0] = 0.f;
				pm->probe_DFT[1] = 0.f;
				pm->probe_DFT[2] = 0.f;
				pm->FIX[0] = 0.f;
				pm->FIX[1] = 0.f;
				pm->FIX[2] = 0.f;

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_instant_probe;

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->fb_voltage_A);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], pm->fb_voltage_B);
			pm_ADD(&pm->probe_DFT[2], &pm->FIX[2], pm->fb_voltage_C);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->probe_DFT[0] /= pm->tm_end;
				pm->probe_DFT[1] /= pm->tm_end;
				pm->probe_DFT[2] /= pm->tm_end;

				pm->fsm_phase = 4;
			}
			break;

		case 4:
			bm = 0;

			bm |= (m_fabsf(pm->probe_DFT[0] - pm->const_lpf_U) < pm->fault_voltage_tolerance)
				? 1 : (m_fabsf(pm->probe_DFT[0]) < pm->fault_voltage_tolerance) ? 0 : 16;
			bm |= (m_fabsf(pm->probe_DFT[1] - pm->const_lpf_U) < pm->fault_voltage_tolerance)
				? 2 : (m_fabsf(pm->probe_DFT[1]) < pm->fault_voltage_tolerance) ? 0 : 32;
			bm |= (m_fabsf(pm->probe_DFT[2] - pm->const_lpf_U) < pm->fault_voltage_tolerance)
				? 4 : (m_fabsf(pm->probe_DFT[2]) < pm->fault_voltage_tolerance) ? 0 : 64;

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
			if (		   pm->self_BM[1] == 7
					&& pm->self_BM[2] == 0
					&& pm->self_BM[3] == 7
					&& pm->self_BM[4] == 0
					&& pm->self_BM[5] == 7
					&& pm->self_BM[6] == 0) {

				pm->fail_reason = PM_OK;
			}
			else if (	   (pm->self_BM[1] & 17) == 1
					&& (pm->self_BM[2] & 17) == 0
					&& (pm->self_BM[3] & 34) == 2
					&& (pm->self_BM[4] & 34) == 0
					&& (pm->self_BM[5] & 68) == 4
					&& (pm->self_BM[6] & 68) == 0) {

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
pm_fsm_state_self_test_sampling_accuracy(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			pm->self_RMS[0] = 0.f;
			pm->self_RMS[1] = 0.f;
			pm->FIX[0] = 0.f;
			pm->FIX[1] = 0.f;

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
			pm_ADD(&pm->self_RMS[0], &pm->FIX[0], pm->fb_current_A * pm->fb_current_A);
			pm_ADD(&pm->self_RMS[1], &pm->FIX[1], pm->fb_current_B * pm->fb_current_B);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->self_RMS[0] = m_sqrtf(pm->self_RMS[0] / pm->tm_end);
				pm->self_RMS[1] = m_sqrtf(pm->self_RMS[1] / pm->tm_end);

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			if (		pm->self_RMS[0] > pm->fault_current_tolerance
					|| pm->self_RMS[1] > pm->fault_current_tolerance) {

				pm->fail_reason = PM_ERROR_ACCURACY_FAULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	pm_fsm_current_halt(pm, pm->fault_current_halt_level);
}

static void
pm_fsm_state_adjust_voltage(pmc_t *pm)
{
	int		N, xDC, xMIN, xMAX;
	float		REF;

	switch (pm->fsm_phase) {

		case 0:
			pm->fail_reason = PM_OK;

			if (PM_CONFIG_VOLT(pm) == PM_ENABLED) {

				pm->fsm_phase = 1;
				pm->fsm_phase_2 = 0;
			}
			else {
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}
			break;

		case 1:
			xDC = (pm->fsm_phase_2 == 0) ? 0 : pm->dc_resolution;

			pm->proc_set_DC(xDC, xDC, xDC);
			pm->proc_set_Z(0);

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = 0.f;
			pm->probe_DFT[3] = 0.f;
			pm->FIX[0] = 0.f;
			pm->FIX[1] = 0.f;
			pm->FIX[2] = 0.f;
			pm->FIX[3] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_skip;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 2;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_instant_probe;

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->fb_voltage_A);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], pm->fb_voltage_B);
			pm_ADD(&pm->probe_DFT[2], &pm->FIX[2], pm->fb_voltage_C);
			pm_ADD(&pm->probe_DFT[3], &pm->FIX[3], pm->const_lpf_U);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->probe_DFT[0] /= pm->tm_end;
				pm->probe_DFT[1] /= pm->tm_end;
				pm->probe_DFT[2] /= pm->tm_end;
				pm->probe_DFT[3] /= pm->tm_end;

				pm->fsm_phase = (pm->fsm_phase_2 == 0) ? 4 : 5;
			}
			break;

		case 4:
			pm->adjust_UA[0] += - pm->probe_DFT[0];
			pm->adjust_UB[0] += - pm->probe_DFT[1];
			pm->adjust_UC[0] += - pm->probe_DFT[2];

			if (		m_fabsf(pm->adjust_UA[0]) > pm->fault_voltage_tolerance
					|| m_fabsf(pm->adjust_UB[0]) > pm->fault_voltage_tolerance
					|| m_fabsf(pm->adjust_UC[0]) > pm->fault_voltage_tolerance) {

				pm->fail_reason = PM_ERROR_ACCURACY_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm->fsm_phase = 1;
			pm->fsm_phase_2 = 1;
			break;

		case 5:
			pm->adjust_UA[1] *= pm->probe_DFT[3] / pm->probe_DFT[0];
			pm->adjust_UB[1] *= pm->probe_DFT[3] / pm->probe_DFT[1];
			pm->adjust_UC[1] *= pm->probe_DFT[3] / pm->probe_DFT[2];

			if (		m_fabsf(pm->adjust_UA[1] - 1.f) > pm->fault_adjust_tolerance
					|| m_fabsf(pm->adjust_UB[1] - 1.f) > pm->fault_adjust_tolerance
					|| m_fabsf(pm->adjust_UC[1] - 1.f) > pm->fault_adjust_tolerance) {

				pm->fail_reason = PM_ERROR_ACCURACY_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}

			for (N = 0; N < 9; ++N) {

				pm->probe_LSQ_A[N] = 0.f;
				pm->probe_LSQ_B[N] = 0.f;
				pm->probe_LSQ_C[N] = 0.f;
				pm->FIX[N] = 0.f;
				pm->FIX[N + 9] = 0.f;
				pm->FIX[N + 18] = 0.f;
			}

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_average_probe;

			pm->fsm_phase = 6;
			pm->fsm_phase_2 = 0;
			break;

		case 6:
			xMIN = pm->dc_minimal;
			xMAX = (int) (pm->dc_resolution * pm->volt_maximal / pm->const_lpf_U);

			switch (pm->fsm_phase_2) {

				case 0:
				case 1:
				case 2:
				case 6:
				case 8:
					xDC = xMIN;
					break;

				case 3:
				case 4:
				case 5:
				case 7:
				case 9:
					xDC = xMAX;
					break;

				case 10:
				case 11:
				case 12:
					xDC = (xMIN + xMAX) / 2;
					break;

				case 13:
				case 14:
				case 15:
				case 19:
				case 21:
				case 23:
					xDC = xMIN + (xMAX - xMIN) / 4;
					break;

				case 16:
				case 17:
				case 18:
				case 20:
				case 22:
					xDC = xMIN + 3 * (xMAX - xMIN) / 4;
					break;
			}

			pm->fsm_phase_2 = (pm->fsm_phase_2 < 23) ? pm->fsm_phase_2 + 1 : 0;

			pm->proc_set_DC(xDC, xDC, xDC);

			if (pm->tm_value >= 2) {

				REF = pm->probe_DFT[0] * pm->const_lpf_U / pm->dc_resolution;

				pm_ADD(&pm->probe_LSQ_A[0], &pm->FIX[0], pm->fb_voltage_A * pm->fb_voltage_A);
				pm_ADD(&pm->probe_LSQ_A[1], &pm->FIX[1], pm->fb_voltage_A * pm->probe_DFT[2]);
				pm_ADD(&pm->probe_LSQ_A[2], &pm->FIX[2], pm->probe_DFT[2] * pm->probe_DFT[2]);
				pm_ADD(&pm->probe_LSQ_A[3], &pm->FIX[3], pm->fb_voltage_A);
				pm_ADD(&pm->probe_LSQ_A[4], &pm->FIX[4], pm->probe_DFT[2]);
				pm_ADD(&pm->probe_LSQ_A[5], &pm->FIX[5], 1.f);
				pm_ADD(&pm->probe_LSQ_A[6], &pm->FIX[6], pm->fb_voltage_A * REF);
				pm_ADD(&pm->probe_LSQ_A[7], &pm->FIX[7], pm->probe_DFT[2] * REF);
				pm_ADD(&pm->probe_LSQ_A[8], &pm->FIX[8], REF);

				pm_ADD(&pm->probe_LSQ_B[0], &pm->FIX[9], pm->fb_voltage_B * pm->fb_voltage_B);
				pm_ADD(&pm->probe_LSQ_B[1], &pm->FIX[10], pm->fb_voltage_B * pm->probe_DFT[3]);
				pm_ADD(&pm->probe_LSQ_B[2], &pm->FIX[11], pm->probe_DFT[3] * pm->probe_DFT[3]);
				pm_ADD(&pm->probe_LSQ_B[3], &pm->FIX[12], pm->fb_voltage_B);
				pm_ADD(&pm->probe_LSQ_B[4], &pm->FIX[13], pm->probe_DFT[3]);
				pm_ADD(&pm->probe_LSQ_B[5], &pm->FIX[14], 1.f);
				pm_ADD(&pm->probe_LSQ_B[6], &pm->FIX[15], pm->fb_voltage_B * REF);
				pm_ADD(&pm->probe_LSQ_B[7], &pm->FIX[16], pm->probe_DFT[3] * REF);
				pm_ADD(&pm->probe_LSQ_B[8], &pm->FIX[17], REF);

				pm_ADD(&pm->probe_LSQ_C[0], &pm->FIX[18], pm->fb_voltage_C * pm->fb_voltage_C);
				pm_ADD(&pm->probe_LSQ_C[1], &pm->FIX[19], pm->fb_voltage_C * pm->probe_DFT[4]);
				pm_ADD(&pm->probe_LSQ_C[2], &pm->FIX[20], pm->probe_DFT[4] * pm->probe_DFT[4]);
				pm_ADD(&pm->probe_LSQ_C[3], &pm->FIX[21], pm->fb_voltage_C);
				pm_ADD(&pm->probe_LSQ_C[4], &pm->FIX[22], pm->probe_DFT[4]);
				pm_ADD(&pm->probe_LSQ_C[5], &pm->FIX[23], 1.f);
				pm_ADD(&pm->probe_LSQ_C[6], &pm->FIX[24], pm->fb_voltage_C * REF);
				pm_ADD(&pm->probe_LSQ_C[7], &pm->FIX[25], pm->probe_DFT[4] * REF);
				pm_ADD(&pm->probe_LSQ_C[8], &pm->FIX[26], REF);
			}

			pm->probe_DFT[0] = pm->probe_DFT[1];
			pm->probe_DFT[1] = (float) xDC;
			pm->probe_DFT[2] = pm->fb_voltage_A;
			pm->probe_DFT[3] = pm->fb_voltage_B;
			pm->probe_DFT[4] = pm->fb_voltage_C;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 7;
			}
			break;

		case 7:
			pm_LSQ_3(pm->probe_LSQ_A, pm->volt_FIR_A);
			pm_LSQ_3(pm->probe_LSQ_B, pm->volt_FIR_B);
			pm_LSQ_3(pm->probe_LSQ_C, pm->volt_FIR_C);

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
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

			pm->vsi_precise_MODE = PM_DISABLED;

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->FIX[0] = 0.f;
			pm->FIX[1] = 0.f;
			pm->FIX[2] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_skip;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->fb_current_A);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], - pm->fb_current_B);

		case 1:
			eX = pm->probe_current_hold_X - pm->fb_current_A;

			pm->FIX[2] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->FIX[2];

			uMAX = (2.f / 3.f) * pm->const_lpf_U;

			if (m_fabsf(uX) > uMAX) {

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

			if (		m_fabsf(pm->adjust_IA[1] - 1.f) > pm->fault_adjust_tolerance
					|| m_fabsf(pm->adjust_IB[1] - 1.f) > pm->fault_adjust_tolerance) {

				pm->fail_reason = PM_ERROR_ACCURACY_FAULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	pm_fsm_current_halt(pm, pm->fault_current_halt_level);
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

			pm->vsi_precise_MODE = PM_ENABLED;

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = pm->probe_current_hold_X;
			pm->probe_DFT[3] = pm->probe_current_hold_Y;
			pm->probe_DFT[4] = pm->probe_current_hold_X * pm->const_R;
			pm->probe_DFT[5] = pm->probe_current_hold_Y * pm->const_R;

			pm->FIX[0] = 0.f;
			pm->FIX[1] = 0.f;
			pm->FIX[2] = 0.f;
			pm->FIX[3] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_current_hold;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			uX = pm->vsi_X - pm->probe_DFT[4];
			uY = pm->vsi_Y - pm->probe_DFT[5];

			if (PM_CONFIG_VOLT(pm) == PM_ENABLED) {

				pm_volt_residue(pm);

				uX += pm->volt_residue_X;
				uY += pm->volt_residue_Y;
			}

			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], uX);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], uY);

		case 1:
			pm_fsm_current_probe(pm);

			eX = pm->probe_current_hold_X - pm->probe_fb_X;
			eY = pm->probe_current_hold_Y - pm->probe_fb_Y;

			pm->FIX[2] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->FIX[2];

			pm->FIX[3] += pm->probe_gain_I * eY;
			uY = pm->probe_gain_P * eY + pm->FIX[3];

			uMAX = (2.f / 3.f) * pm->const_lpf_U;

			if (m_fabsf(uX) > uMAX || m_fabsf(uY) > uMAX) {

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

			pm->const_R += pm_DFT_R(pm->probe_DFT);

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	pm_fsm_current_halt(pm, pm->fault_current_halt_level);
}

static void
pm_fsm_state_probe_const_l(pmc_t *pm)
{
	float			tX, tY, uX, uY, eX, eY, uMAX;
	float			imp_Z, LDQ[5];

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(0);

			pm->vsi_precise_MODE = PM_ENABLED;

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = 0.f;
			pm->probe_DFT[3] = 0.f;
			pm->probe_DFT[4] = 0.f;
			pm->probe_DFT[5] = 0.f;
			pm->probe_DFT[6] = 0.f;
			pm->probe_DFT[7] = 0.f;

			pm->FIX[0] = 0.f;
			pm->FIX[1] = 0.f;
			pm->FIX[2] = 0.f;
			pm->FIX[3] = 0.f;
			pm->FIX[4] = 0.f;
			pm->FIX[5] = 0.f;
			pm->FIX[6] = 0.f;
			pm->FIX[7] = 0.f;

			pm->FIX[8] = 1.f;
			pm->FIX[9] = 0.f;
			pm->FIX[10] = 2.f * M_PI_F * pm->probe_freq_sine_hz * pm->dT;

			imp_Z = (pm->const_Ld < pm->const_Lq) ? pm->const_Ld : pm->const_Lq;
			imp_Z = (imp_Z < M_EPS_F) ? 1E-6f : imp_Z;

			imp_Z = 2.f * M_PI_F * imp_Z * pm->probe_freq_sine_hz;
			imp_Z = pm->const_R * pm->const_R + imp_Z * imp_Z;
			pm->FIX[11] = pm->probe_current_sine * m_sqrtf(imp_Z);

			if (pm->FIX[11] < pm->const_lpf_U / pm->dc_resolution) {

				pm->fail_reason = PM_ERROR_INVALID_OPERATION;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}

			pm->FIX[12] = m_cosf(pm->FIX[10] * .5f);
			pm->FIX[13] = m_sinf(pm->FIX[10] * .5f);

			pm->FIX[14] = 0.f;
			pm->FIX[15] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_skip;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			pm_fsm_current_probe(pm);

			tX = pm->vsi_DX;
			tY = pm->vsi_DY;

			if (PM_CONFIG_VOLT(pm) == PM_ENABLED) {

				pm_volt_residue(pm);

				tX += pm->volt_residue_X;
				tY += pm->volt_residue_Y;
			}

			uX = tX * pm->FIX[12] - tY * pm->FIX[13];
			uY = tX * pm->FIX[13] + tY * pm->FIX[12];

			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->probe_fb_X * pm->FIX[8]);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], pm->probe_fb_X * pm->FIX[9]);
			pm_ADD(&pm->probe_DFT[2], &pm->FIX[2], uX * pm->FIX[8]);
			pm_ADD(&pm->probe_DFT[3], &pm->FIX[3], uX * pm->FIX[9]);
			pm_ADD(&pm->probe_DFT[4], &pm->FIX[4], pm->probe_fb_Y * pm->FIX[8]);
			pm_ADD(&pm->probe_DFT[5], &pm->FIX[5], pm->probe_fb_Y * pm->FIX[9]);
			pm_ADD(&pm->probe_DFT[6], &pm->FIX[6], uY * pm->FIX[8]);
			pm_ADD(&pm->probe_DFT[7], &pm->FIX[7], uY * pm->FIX[9]);

		case 1:
			m_rotf(pm->FIX + 8, pm->FIX[10], pm->FIX + 8);

			pm_fsm_current_probe(pm);

			if (pm->probe_current_hold_Y != 0.f) {

				eX = pm->probe_current_hold_X - pm->probe_fb_X;
				eY = pm->probe_current_hold_Y - pm->probe_fb_Y;
			}
			else {
				eX = 0.f - pm->probe_fb_X;
				eY = 0.f - pm->probe_fb_Y;
			}

			pm->FIX[14] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->FIX[14];

			pm->FIX[15] += pm->probe_gain_I * eY;
			uY = pm->probe_gain_P * eY + pm->FIX[15];

			uMAX = (2.f / 3.f) * pm->const_lpf_U;

			if (m_fabsf(uX) > uMAX || m_fabsf(uY) > uMAX) {

				pm->fail_reason = PM_ERROR_CURRENT_LOOP_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}

			uX += pm->FIX[11] * pm->FIX[8];
			uY += pm->FIX[11] * pm->FIX[9];

			pm_voltage_control(pm, uX, uY);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_average_probe;

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm_DFT_LDQ(pm->probe_DFT, pm->probe_freq_sine_hz, LDQ);

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

			pm->probe_impedance_R = LDQ[4];
			pm->probe_rotation_DQ = m_atan2f(LDQ[3], LDQ[2]) * (180.f / M_PI_F);

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}

	pm_fsm_current_halt(pm, pm->fault_current_halt_level);
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

				pm->vsi_precise_MODE = PM_DISABLED;
				pm->vsi_bit_ZONE = 0x8UL;
				pm->vsi_lpf_D = 0.f;
				pm->vsi_lpf_Q = 0.f;
				pm->vsi_lpf_watt = 0.f;

				pm->flux_X[0] = 0.f;
				pm->flux_X[1] = 0.f;
				pm->flux_X[2] = 1.f;
				pm->flux_X[3] = 0.f;
				pm->flux_X[4] = 0.f;
				pm->flux_drift_Q = 0.f;
				pm->flux_residue_lpf = 0.f;

				pm->fail_reason = PM_OK;

				if (PM_CONFIG_VOLT(pm) == PM_ENABLED) {

					pm->tm_value = 0;
					pm->tm_end = pm->freq_hz * pm->tm_startup;

					pm->fsm_phase = 1;
				}
				else {
					pm->fsm_phase = 2;
				}
			}
			else {
				pm->fail_reason = PM_ERROR_INVALID_OPERATION;
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

			pm->forced_setpoint = pm->flux_X[4];

			pm->hfi_wave[0] = 1.f;
			pm->hfi_wave[1] = 0.f;
			pm->hfi_flux = 0.f;

			pm->i_derated = pm->i_maximal;
			pm->i_setpoint_D = 0.f;
			pm->i_setpoint_Q = 0.f;
			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			pm->s_setpoint = pm->flux_X[4];
			pm->s_track = pm->flux_X[4];
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
			pm->FIX[0] = 0.f;
			pm->FIX[1] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_average_probe;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->flux_X[4]);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], pm->flux_drift_Q);

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

		case PM_STATE_SELF_TEST_POWER_STAGE:
			pm_fsm_state_self_test_power_stage(pm);
			break;

		case PM_STATE_SELF_TEST_SAMPLING_ACCURACY:
			pm_fsm_state_self_test_sampling_accuracy(pm);
			break;

		case PM_STATE_ADJUST_VOLTAGE:
			pm_fsm_state_adjust_voltage(pm);
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
		case PM_STATE_SELF_TEST_POWER_STAGE:
		case PM_STATE_SELF_TEST_SAMPLING_ACCURACY:
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
		PM_SFI(PM_ERROR_ACCURACY_FAULT),
		PM_SFI(PM_ERROR_CURRENT_LOOP_FAULT),
		PM_SFI(PM_ERROR_OVER_CURRENT),
		PM_SFI(PM_ERROR_RESIDUE_UNSTABLE),
		PM_SFI(PM_ERROR_INVALID_OPERATION)
	};

	const int 	lmax = sizeof(list) / sizeof(list[0]);

	return (n >= 0 && n < lmax) ? list[n] : "";
}

