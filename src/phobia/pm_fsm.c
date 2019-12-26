#include "libm.h"
#include "pm.h"

static float
pm_DFT_R(const float DFT[8])
{
	float			D, X, Y, E, R = 0.;

	/* Here we make a simple resistance calculation on constant current
	 * condition. R = U / I.
	 * */

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

	/* Here we calculate the eigenvalues of the inductance matrix in order
	 * to find the inductance in DQ-axes.
	 *
	 * R * [XY[0] XY[1]] * R' = [DQ[0] 0    ]
	 *     [XY[1] XY[2]]        [0     DQ[1]]
	 *
	 * R = [DQ[2] -DQ[3]]
	 *     [DQ[3]  DQ[2]].
	 * */

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
	else {
		/* Looks like we have a problem.
		 * */
		DQ[0] = XY[0];
		DQ[1] = XY[2];
		DQ[2] = 1.f;
		DQ[3] = 0.f;
	}
}

static void
pm_LSQ_3(const float LSQ[9], float X[3])
{
	float		LD[6], B[3];

	/* This function solves the eqaution A*X = B by LDL' decomposition.
	 *
	 *     [LSQ[0] LSQ[1] LSQ[3]]      [LSQ[6]]
	 * A = [LSQ[1] LSQ[2] LSQ[4]]  B = [LSQ[7]].
	 *     [LSQ[3] LSQ[4] LSQ[5]]      [LSQ[8]]
	 * */

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

	/* The initial expression is Z * I = U.
	 *
	 * [R-j*WF*LXY[0]   j*WF*LXY[1]] * [IX] = [UX]
	 * [  j*WF*LXY[1] R-j*WF*LXY[2]] * [IY]   [UY]
	 *
	 * IX = [DFT[0]+j*DFT[1]], UX = [DFT[2]+j*DFT[3]]
	 * IY = [DFT[4]+j*DFT[5]], UY = [DFT[6]+j*DFT[7]]
	 *
	 * We rewrite previous equation with respect to the impedance
	 * components.
	 *
	 * [DFT[0]  DFT[1] -DFT[5]  0     ] * [R        ]   [DFT[2]]
	 * [DFT[1] -DFT[0]  DFT[4]  0     ]   [WF*LXY[0]] = [DFT[3]]
	 * [DFT[4]  0      -DFT[1]  DFT[5]]   [WF*LXY[1]]   [DFT[6]].
	 * [DFT[5]  0       DFT[0] -DFT[4]]   [WF*LYY[2]]   [DFT[7]]
	 *
	 * */

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

void pm_ADD(float *S, float *C, float X)
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
			pm->tm_end = pm->freq_hz * pm->tm_average_drift;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->fb_iA);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], pm->fb_iB);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->probe_DFT[0] /= pm->tm_end;
				pm->probe_DFT[1] /= pm->tm_end;

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			pm->ad_IA[0] += - pm->probe_DFT[0];
			pm->ad_IB[0] += - pm->probe_DFT[1];

			if (		m_fabsf(pm->ad_IA[0]) > pm->fault_current_tol
					|| m_fabsf(pm->ad_IB[0]) > pm->fault_current_tol) {

				pm->fail_reason = PM_ERROR_ZERO_DRIFT_FAULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_self_test_bootstrap(pmc_t *pm)
{
	float		uA, uB, uC;

	switch (pm->fsm_phase) {

		case 0:
			pm->fail_reason = PM_OK;

			if (		PM_CONFIG_TVM(pm) == PM_ENABLED
					&& pm->ts_bootstrap != 0) {

				pm->fsm_phase = 1;
			}
			else {
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}
			break;

		case 1:
			pm->self_BST[0] = 0.f;
			pm->self_BST[1] = 0.f;
			pm->self_BST[2] = 0.f;

			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(0);

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_fast;

			pm->fsm_phase = 2;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->proc_set_DC(pm->dc_resolution, pm->dc_resolution, pm->dc_resolution);

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_average_probe;

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			uA = pm->fb_uA - pm->const_lpf_U;
			uB = pm->fb_uB - pm->const_lpf_U;
			uC = pm->fb_uC - pm->const_lpf_U;

			pm->self_BST[0] += (m_fabsf(uA) < pm->fault_voltage_tol) ? 1.f : 0.f;
			pm->self_BST[1] += (m_fabsf(uB) < pm->fault_voltage_tol) ? 1.f : 0.f;
			pm->self_BST[2] += (m_fabsf(uC) < pm->fault_voltage_tol) ? 1.f : 0.f;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->self_BST[0] /= pm->freq_hz;
				pm->self_BST[1] /= pm->freq_hz;
				pm->self_BST[2] /= pm->freq_hz;

				pm->fsm_phase = 4;
			}
			break;

		case 4:
			if (		pm->self_BST[0] < pm->dc_bootstrap
					|| pm->self_BST[1] < pm->dc_bootstrap
					|| pm->self_BST[2] < pm->dc_bootstrap) {

				pm->fail_reason = PM_ERROR_POWER_STAGE_FAULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_self_test_power_stage(pmc_t *pm)
{
	float			uA, uB, uC;
	int			bm;

	switch (pm->fsm_phase) {

		case 0:
			pm->fail_reason = PM_OK;

			if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

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
			pm->tm_end = pm->freq_hz * pm->tm_transient_fast;

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
			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->fb_uA);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], pm->fb_uB);
			pm_ADD(&pm->probe_DFT[2], &pm->FIX[2], pm->fb_uC);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->probe_DFT[0] /= pm->tm_end;
				pm->probe_DFT[1] /= pm->tm_end;
				pm->probe_DFT[2] /= pm->tm_end;

				pm->fsm_phase = 4;
			}
			break;

		case 4:
			uA = pm->probe_DFT[0];
			uB = pm->probe_DFT[1];
			uC = pm->probe_DFT[2];
			bm = 0UL;

			bm |= (m_fabsf(uA - pm->const_lpf_U) < pm->fault_voltage_tol)
				? 1UL : (m_fabsf(uA) < pm->fault_voltage_tol) ? 0UL : 16UL;
			bm |= (m_fabsf(uB - pm->const_lpf_U) < pm->fault_voltage_tol)
				? 2UL : (m_fabsf(uB) < pm->fault_voltage_tol) ? 0UL : 32UL;
			bm |= (m_fabsf(uC - pm->const_lpf_U) < pm->fault_voltage_tol)
				? 4UL : (m_fabsf(uC) < pm->fault_voltage_tol) ? 0UL : 64UL;

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
				pm->fail_reason = PM_ERROR_POWER_STAGE_FAULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_self_test_clearance(pmc_t *pm)
{
	float			RMS, AVG;

	switch (pm->fsm_phase) {

		case 0:
			pm->self_RMSi[0] = 0.f;
			pm->self_RMSi[1] = 0.f;
			pm->self_RMSi[2] = 0.f;
			pm->self_RMSu[0] = 0.f;
			pm->self_RMSu[1] = 0.f;
			pm->self_RMSu[2] = 0.f;

			pm->FIX[0] = 0.f;
			pm->FIX[1] = 0.f;
			pm->FIX[2] = 0.f;
			pm->FIX[3] = 0.f;
			pm->FIX[4] = 0.f;
			pm->FIX[5] = 0.f;
			pm->FIX[6] = 0.f;
			pm->FIX[7] = 0.f;
			pm->FIX[8] = 0.f;
			pm->FIX[9] = 0.f;
			pm->FIX[10] = 0.f;
			pm->FIX[11] = 0.f;
			pm->FIX[12] = 0.f;
			pm->FIX[13] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_slow;

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
			pm_ADD(&pm->self_RMSi[0], &pm->FIX[0], pm->fb_iA * pm->fb_iA);
			pm_ADD(&pm->self_RMSi[1], &pm->FIX[1], pm->fb_iB * pm->fb_iB);
			pm_ADD(&pm->self_RMSi[2], &pm->FIX[2], pm->const_lpf_U * pm->const_lpf_U);
			pm_ADD(&pm->FIX[6], &pm->FIX[10], pm->const_lpf_U);

			if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				pm_ADD(&pm->self_RMSu[0], &pm->FIX[3], pm->fb_uA * pm->fb_uA);
				pm_ADD(&pm->self_RMSu[1], &pm->FIX[4], pm->fb_uB * pm->fb_uB);
				pm_ADD(&pm->self_RMSu[2], &pm->FIX[5], pm->fb_uC * pm->fb_uC);

				pm_ADD(&pm->FIX[7], &pm->FIX[11], pm->fb_uA);
				pm_ADD(&pm->FIX[8], &pm->FIX[12], pm->fb_uB);
				pm_ADD(&pm->FIX[9], &pm->FIX[13], pm->fb_uC);
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				RMS = pm->self_RMSi[0] / pm->tm_end;
				pm->self_RMSi[0] = m_sqrtf(RMS);

				RMS = pm->self_RMSi[1] / pm->tm_end;
				pm->self_RMSi[1] = m_sqrtf(RMS);

				RMS = pm->self_RMSi[2] / pm->tm_end;
				AVG = pm->FIX[6] / pm->tm_end;
				pm->self_RMSi[2] = m_sqrtf(RMS - AVG * AVG);

				RMS = pm->self_RMSu[0] / pm->tm_end;
				AVG = pm->FIX[7] / pm->tm_end;
				pm->self_RMSu[0] = m_sqrtf(RMS - AVG * AVG);

				RMS = pm->self_RMSu[1] / pm->tm_end;
				AVG = pm->FIX[8] / pm->tm_end;
				pm->self_RMSu[1] = m_sqrtf(RMS - AVG * AVG);

				RMS = pm->self_RMSu[2] / pm->tm_end;
				AVG = pm->FIX[9] / pm->tm_end;
				pm->self_RMSu[2] = m_sqrtf(RMS - AVG * AVG);

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			if (		pm->self_RMSi[0] > pm->fault_current_tol
					|| pm->self_RMSi[1] > pm->fault_current_tol) {

				pm->fail_reason = PM_ERROR_ACCURACY_FAULT;
			}

			if (		pm->self_RMSi[2] > pm->fault_voltage_tol
					|| pm->self_RMSu[0] > pm->fault_voltage_tol
					|| pm->self_RMSu[1] > pm->fault_voltage_tol
					|| pm->self_RMSu[2] > pm->fault_voltage_tol) {

				pm->fail_reason = PM_ERROR_ACCURACY_FAULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_adjust_voltage(pmc_t *pm)
{
	int		N, xDC, xMIN, xMAX;
	float		REF;

	switch (pm->fsm_phase) {

		case 0:
			pm->fail_reason = PM_OK;

			if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

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
			pm->tm_end = pm->freq_hz * pm->tm_transient_fast;

			pm->fsm_phase = 2;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_voltage_hold;
				pm->tm_end = (pm->tm_end > pm->ts_bootstrap)
					? pm->ts_bootstrap : pm->tm_end;

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->fb_uA);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], pm->fb_uB);
			pm_ADD(&pm->probe_DFT[2], &pm->FIX[2], pm->fb_uC);
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
			pm->ad_UA[0] += - pm->probe_DFT[0];
			pm->ad_UB[0] += - pm->probe_DFT[1];
			pm->ad_UC[0] += - pm->probe_DFT[2];

			if (		m_fabsf(pm->ad_UA[0]) > pm->fault_voltage_tol
					|| m_fabsf(pm->ad_UB[0]) > pm->fault_voltage_tol
					|| m_fabsf(pm->ad_UC[0]) > pm->fault_voltage_tol) {

				pm->fail_reason = PM_ERROR_ACCURACY_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm->fsm_phase = 1;
			pm->fsm_phase_2 = 1;
			break;

		case 5:
			pm->ad_UA[1] *= pm->probe_DFT[3] / pm->probe_DFT[0];
			pm->ad_UB[1] *= pm->probe_DFT[3] / pm->probe_DFT[1];
			pm->ad_UC[1] *= pm->probe_DFT[3] / pm->probe_DFT[2];

			if (		m_fabsf(pm->ad_UA[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->ad_UB[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->ad_UC[1] - 1.f) > pm->fault_accuracy_tol) {

				pm->fail_reason = PM_ERROR_ACCURACY_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
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
			xMIN = pm->ts_minimal;
			xMAX = (int) (pm->dc_resolution * pm->tvm_range);

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

				default:
					xDC = 0;
					break;
			}

			pm->fsm_phase_2 = (pm->fsm_phase_2 < 23) ? pm->fsm_phase_2 + 1 : 0;

			pm->proc_set_DC(xDC, xDC, xDC);

			if (pm->tm_value >= 2) {

				REF = pm->probe_DFT[0] * pm->const_lpf_U / pm->dc_resolution;

				pm_ADD(&pm->probe_LSQ_A[0], &pm->FIX[0], pm->fb_uA * pm->fb_uA);
				pm_ADD(&pm->probe_LSQ_A[1], &pm->FIX[1], pm->fb_uA * pm->probe_DFT[2]);
				pm_ADD(&pm->probe_LSQ_A[2], &pm->FIX[2], pm->probe_DFT[2] * pm->probe_DFT[2]);
				pm_ADD(&pm->probe_LSQ_A[3], &pm->FIX[3], pm->fb_uA);
				pm_ADD(&pm->probe_LSQ_A[4], &pm->FIX[4], pm->probe_DFT[2]);
				pm_ADD(&pm->probe_LSQ_A[5], &pm->FIX[5], 1.f);
				pm_ADD(&pm->probe_LSQ_A[6], &pm->FIX[6], pm->fb_uA * REF);
				pm_ADD(&pm->probe_LSQ_A[7], &pm->FIX[7], pm->probe_DFT[2] * REF);
				pm_ADD(&pm->probe_LSQ_A[8], &pm->FIX[8], REF);

				pm_ADD(&pm->probe_LSQ_B[0], &pm->FIX[9], pm->fb_uB * pm->fb_uB);
				pm_ADD(&pm->probe_LSQ_B[1], &pm->FIX[10], pm->fb_uB * pm->probe_DFT[3]);
				pm_ADD(&pm->probe_LSQ_B[2], &pm->FIX[11], pm->probe_DFT[3] * pm->probe_DFT[3]);
				pm_ADD(&pm->probe_LSQ_B[3], &pm->FIX[12], pm->fb_uB);
				pm_ADD(&pm->probe_LSQ_B[4], &pm->FIX[13], pm->probe_DFT[3]);
				pm_ADD(&pm->probe_LSQ_B[5], &pm->FIX[14], 1.f);
				pm_ADD(&pm->probe_LSQ_B[6], &pm->FIX[15], pm->fb_uB * REF);
				pm_ADD(&pm->probe_LSQ_B[7], &pm->FIX[16], pm->probe_DFT[3] * REF);
				pm_ADD(&pm->probe_LSQ_B[8], &pm->FIX[17], REF);

				pm_ADD(&pm->probe_LSQ_C[0], &pm->FIX[18], pm->fb_uC * pm->fb_uC);
				pm_ADD(&pm->probe_LSQ_C[1], &pm->FIX[19], pm->fb_uC * pm->probe_DFT[4]);
				pm_ADD(&pm->probe_LSQ_C[2], &pm->FIX[20], pm->probe_DFT[4] * pm->probe_DFT[4]);
				pm_ADD(&pm->probe_LSQ_C[3], &pm->FIX[21], pm->fb_uC);
				pm_ADD(&pm->probe_LSQ_C[4], &pm->FIX[22], pm->probe_DFT[4]);
				pm_ADD(&pm->probe_LSQ_C[5], &pm->FIX[23], 1.f);
				pm_ADD(&pm->probe_LSQ_C[6], &pm->FIX[24], pm->fb_uC * REF);
				pm_ADD(&pm->probe_LSQ_C[7], &pm->FIX[25], pm->probe_DFT[4] * REF);
				pm_ADD(&pm->probe_LSQ_C[8], &pm->FIX[26], REF);
			}

			pm->probe_DFT[0] = pm->probe_DFT[1];
			pm->probe_DFT[1] = (float) xDC;
			pm->probe_DFT[2] = pm->fb_uA;
			pm->probe_DFT[3] = pm->fb_uB;
			pm->probe_DFT[4] = pm->fb_uC;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 7;
			}
			break;

		case 7:
			pm_LSQ_3(pm->probe_LSQ_A, pm->tvm_FIR_A);
			pm_LSQ_3(pm->probe_LSQ_B, pm->tvm_FIR_B);
			pm_LSQ_3(pm->probe_LSQ_C, pm->tvm_FIR_C);

			if (		   m_isfinitef(pm->tvm_FIR_A[0]) == 0
					|| m_isfinitef(pm->tvm_FIR_A[1]) == 0
					|| m_isfinitef(pm->tvm_FIR_A[2]) == 0
					|| m_isfinitef(pm->tvm_FIR_B[0]) == 0
					|| m_isfinitef(pm->tvm_FIR_B[1]) == 0
					|| m_isfinitef(pm->tvm_FIR_B[2]) == 0
					|| m_isfinitef(pm->tvm_FIR_C[0]) == 0
					|| m_isfinitef(pm->tvm_FIR_C[1]) == 0
					|| m_isfinitef(pm->tvm_FIR_C[2]) == 0) {

				pm->fail_reason = PM_ERROR_INVALID_OPERATION;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_adjust_current(pmc_t *pm)
{
	float			eX, uX, REF;
	float			uMAX;

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(4);

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->FIX[0] = 0.f;
			pm->FIX[1] = 0.f;
			pm->FIX[2] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_slow;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->fb_iA);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], - pm->fb_iB);

		case 1:
			eX = pm->probe_current_hold - pm->fb_iA;

			pm->FIX[2] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->FIX[2];

			uMAX = PM_UMAX(pm) * pm->const_lpf_U;

			if (m_fabsf(uX) > uMAX) {

				pm->fail_reason = PM_ERROR_CURRENT_LOOP_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm_voltage(pm, uX, 0.f);

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

			REF = (pm->probe_DFT[1] + pm->probe_DFT[0]) / 2.f;
			pm->ad_IA[1] *= REF / pm->probe_DFT[0];
			pm->ad_IB[1] *= REF / pm->probe_DFT[1];

			if (		m_fabsf(pm->ad_IA[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->ad_IB[1] - 1.f) > pm->fault_accuracy_tol) {

				pm->fail_reason = PM_ERROR_ACCURACY_FAULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
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

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = pm->probe_current_hold;
			pm->probe_DFT[3] = pm->probe_current_bias_Q;
			pm->probe_DFT[4] = pm->probe_current_hold * pm->const_R;
			pm->probe_DFT[5] = pm->probe_current_bias_Q * pm->const_R;

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
			uX = pm->tvm_DX - pm->probe_DFT[4];
			uY = pm->tvm_DY - pm->probe_DFT[5];

			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], uX);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], uY);

		case 1:
			eX = pm->probe_current_hold - pm->lu_iX;
			eY = pm->probe_current_bias_Q - pm->lu_iY;

			pm->FIX[2] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->FIX[2];

			pm->FIX[3] += pm->probe_gain_I * eY;
			uY = pm->probe_gain_P * eY + pm->FIX[3];

			uMAX = PM_UMAX(pm) * pm->const_lpf_U;

			if (m_fabsf(uX) > uMAX || m_fabsf(uY) > uMAX) {

				pm->fail_reason = PM_ERROR_CURRENT_LOOP_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm_voltage(pm, uX, uY);

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
}

static void
pm_fsm_state_probe_const_l(pmc_t *pm)
{
	float			uX, uY, eX, eY, uMAX;
	float			imp_Z, LDQ[5];

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(0);

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

			/* Assume minimal inductance.
			 * */
			imp_Z = 1E-6f;

			/* The estimated impedance.
			 * */
			imp_Z = (pm->const_L > imp_Z) ? pm->const_L : imp_Z;
			imp_Z = 2.f * M_PI_F * imp_Z * pm->probe_freq_sine_hz;
			imp_Z = pm->const_R * pm->const_R + imp_Z * imp_Z;

			pm->FIX[11] = pm->probe_current_sine * m_sqrtf(imp_Z);

			if (pm->FIX[11] < pm->const_lpf_U / pm->dc_resolution) {

				pm->fail_reason = PM_ERROR_INVALID_OPERATION;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm->FIX[12] = m_cosf(pm->FIX[10] * .5f);
			pm->FIX[13] = m_sinf(pm->FIX[10] * .5f);

			pm->FIX[14] = 0.f;
			pm->FIX[15] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_slow;

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			uX = pm->tvm_DX * pm->FIX[12] - pm->tvm_DY * pm->FIX[13];
			uY = pm->tvm_DX * pm->FIX[13] + pm->tvm_DY * pm->FIX[12];

			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->lu_iX * pm->FIX[8]);
			pm_ADD(&pm->probe_DFT[1], &pm->FIX[1], pm->lu_iX * pm->FIX[9]);
			pm_ADD(&pm->probe_DFT[2], &pm->FIX[2], uX * pm->FIX[8]);
			pm_ADD(&pm->probe_DFT[3], &pm->FIX[3], uX * pm->FIX[9]);
			pm_ADD(&pm->probe_DFT[4], &pm->FIX[4], pm->lu_iY * pm->FIX[8]);
			pm_ADD(&pm->probe_DFT[5], &pm->FIX[5], pm->lu_iY * pm->FIX[9]);
			pm_ADD(&pm->probe_DFT[6], &pm->FIX[6], uY * pm->FIX[8]);
			pm_ADD(&pm->probe_DFT[7], &pm->FIX[7], uY * pm->FIX[9]);

		case 1:
			m_rotf(pm->FIX + 8, pm->FIX[10], pm->FIX + 8);

			if (m_fabsf(pm->probe_current_bias_Q) > M_EPS_F) {

				eX = pm->probe_current_hold - pm->lu_iX;
				eY = pm->probe_current_bias_Q - pm->lu_iY;
			}
			else {
				eX = 0.f - pm->lu_iX;
				eY = 0.f - pm->lu_iY;
			}

			pm->FIX[14] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->FIX[14];

			pm->FIX[15] += pm->probe_gain_I * eY;
			uY = pm->probe_gain_P * eY + pm->FIX[15];

			uMAX = PM_UMAX(pm) * pm->const_lpf_U;

			if (m_fabsf(uX) > uMAX || m_fabsf(uY) > uMAX) {

				pm->fail_reason = PM_ERROR_CURRENT_LOOP_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			uX += pm->FIX[11] * pm->FIX[8];
			uY += pm->FIX[11] * pm->FIX[9];

			pm_voltage(pm, uX, uY);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_average_probe;

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm_DFT_LDQ(pm->probe_DFT, pm->probe_freq_sine_hz, LDQ);

			pm->const_L = (LDQ[0] + LDQ[1]) * .5f;

			pm->const_im_LD = LDQ[0];
			pm->const_im_LQ = LDQ[1];
			pm->const_im_B = m_atan2f(LDQ[3], LDQ[2]) * (180.f / M_PI_F);
			pm->const_im_R = LDQ[4];

			pm->i_gain_P = .2f * pm->const_L * pm->freq_hz - pm->const_R;
			pm->i_gain_I = 1E-2f * pm->const_L * pm->freq_hz;

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_lu_startup(pmc_t *pm)
{
	int			N;

	switch (pm->fsm_phase) {

		case 0:
			if (pm->const_L > M_EPS_F) {

				pm->vsi_SA = 0;
				pm->vsi_SB = 0;
				pm->vsi_SC = 0;
				pm->vsi_IF = 3;
				pm->vsi_UF = 3;

				pm->forced_F[0] = 1.f;
				pm->forced_F[1] = 0.f;
				pm->forced_wS = 0.f;

				for (N = 0; N < PM_FLUX_MAX; N++) {

					pm->flux[N].X = pm->const_E;
					pm->flux[N].Y = 0.f;
					pm->flux[N].lpf_E = .1f;
				}

				pm->flux_E = 0.f;
				pm->flux_H = 0;
				pm->flux_F[0] = 1.f;
				pm->flux_F[1] = 0.f;
				pm->flux_wS = 0.f;

				pm->hfi_iD = 0.f;
				pm->hfi_iQ = 0.f;
				pm->hfi_F[0] = 1.f;
				pm->hfi_F[1] = 0.f;
				pm->hfi_wS = 0.f;
				pm->hfi_wave[0] = 1.f;
				pm->hfi_wave[1] = 0.f;
				pm->hfi_polarity = 0.f;

				pm->hall_F[0] = 1.f;
				pm->hall_F[0] = 0.f;
				pm->hall_TIM = PM_MAX_I;

				pm->watt_lpf_D = 0.f;
				pm->watt_lpf_Q = 0.f;
				pm->watt_lpf_wP = 0.f;

				pm->i_derated_1 = PM_MAX_F;
				pm->i_setpoint_D = 0.f;
				pm->i_setpoint_Q = 0.f;
				pm->i_integral_D = 0.f;
				pm->i_integral_Q = 0.f;

				pm->weak_D = 0.f;

				pm->s_setpoint = 0.f;
				pm->s_track = 0.f;
				pm->s_integral = 0.f;

				pm->x_setpoint_F[0] = 1.f;
				pm->x_setpoint_F[1] = 0.f;
				pm->x_setpoint_revol = 0;
				pm->x_lu_F1 = pm->lu_F[1];
				pm->x_lu_revol = 0;

				pm->fail_reason = PM_OK;

				if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

					pm->lu_mode = PM_LU_DETACHED;
					pm->lu_TIM = (pm->fsm_state != PM_STATE_LU_DETACHED)
						? pm->freq_hz * pm->tm_startup : PM_MAX_I;

					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(7);
				}
				else {
					pm->lu_mode = PM_LU_ESTIMATE_FLUX;
					pm->lu_TIM = 0;

					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(0);
				}

				pm->fsm_state = PM_STATE_IDLE;
				pm->fsm_phase = 0;
			}
			else {
				pm->fail_reason = PM_ERROR_INVALID_OPERATION;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}
			break;
	}
}

static void
pm_fsm_state_lu_shutdown(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_slow;

			pm->fsm_phase = 1;

		case 1:
			pm->i_derated_1 = 0.f;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->lu_mode = PM_LU_DISABLED;

				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(7);

				pm->vsi_IF = 0;
				pm->vsi_UF = 0;
				pm->vsi_AZ = 0;
				pm->vsi_BZ = 0;
				pm->vsi_CZ = 0;

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
			pm->FIX[0] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * ((pm->lu_mode == PM_LU_DETACHED)
					? pm->tm_average_drift : pm->tm_average_probe);

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm_ADD(&pm->probe_DFT[0], &pm->FIX[0], pm->flux_E);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->probe_DFT[0] /= pm->tm_end;
				pm->fsm_phase = 2;
			}
			break;

		case 2:
			pm->const_E = pm->probe_DFT[0];

			pm_build(pm);

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
pm_fsm_state_adjust_hall(pmc_t *pm)
{
	int		HS, N, min_S;
	float		D;

	switch (pm->fsm_phase) {

		case 0:
			if (pm->lu_mode == PM_LU_ESTIMATE_FLUX) {

				for (N = 0; N < 8; ++N) {

					pm->hall_ST[N].X = 0.f;
					pm->hall_ST[N].Y = 0.f;

					pm->probe_DFT[N] = 0.f;
					pm->FIX[N] = 0.f;
					pm->FIX[N + 8] = 0.f;
				}

				pm->tm_value = 0;
				pm->tm_end = pm->freq_hz * pm->tm_average_probe;

				pm->fail_reason = PM_OK;
				pm->fsm_phase = 1;
			}
			else {
				pm->fsm_state = PM_STATE_IDLE;
				pm->fsm_phase = 0;
			}
			break;

		case 1:
			HS = pm->fb_HS;

			if (HS >= 1 && HS <= 6) {

				pm_ADD(&pm->hall_ST[HS].X, &pm->FIX[HS], pm->flux_F[0]);
				pm_ADD(&pm->hall_ST[HS].Y, &pm->FIX[HS + 8], pm->flux_F[1]);

				pm->probe_DFT[HS] += 1.f;
			}
			else {
				pm->hall_IF = 0;

				pm->fail_reason = PM_ERROR_SENSOR_HALL_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			min_S = (float) pm->tm_end / 12.f;
			N = 0;

			for (HS = 1; HS < 7; ++HS) {

				if (pm->probe_DFT[HS] > min_S) {

					D = pm->probe_DFT[HS];

					pm->hall_ST[HS].X /= D;
					pm->hall_ST[HS].Y /= D;

					D = m_sqrtf(pm->hall_ST[HS].X * pm->hall_ST[HS].X
						+ pm->hall_ST[HS].Y * pm->hall_ST[HS].Y);

					if (D > .5f) {

						pm->hall_ST[HS].X /= D;
						pm->hall_ST[HS].Y /= D;

						N += 1;
					}
				}
			}

			if (N < 6) {

				pm->hall_IF = 0;

				pm->fail_reason = PM_ERROR_SENSOR_HALL_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}
			else {
				pm->hall_IF = 1;

				pm->fsm_state = PM_STATE_IDLE;
				pm->fsm_phase = 0;
			}
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

			pm->vsi_IF = 0;
			pm->vsi_UF = 0;
			pm->vsi_AZ = 0;
			pm->vsi_BZ = 0;
			pm->vsi_CZ = 0;

			pm->tm_value = 0;
			pm->tm_end = pm->freq_hz * pm->tm_transient_slow;

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
	switch (pm->fsm_req) {

		case PM_STATE_ZERO_DRIFT:
		case PM_STATE_SELF_TEST_BOOTSTRAP:
		case PM_STATE_SELF_TEST_POWER_STAGE:
		case PM_STATE_SELF_TEST_CLEARANCE:
		case PM_STATE_ADJUST_VOLTAGE:
		case PM_STATE_ADJUST_CURRENT:
		case PM_STATE_PROBE_CONST_R:
		case PM_STATE_PROBE_CONST_L:
		case PM_STATE_LU_DETACHED:
		case PM_STATE_LU_STARTUP:

			if (pm->fsm_state != PM_STATE_IDLE)
				break;

			if (pm->lu_mode != PM_LU_DISABLED)
				break;

			pm_build(pm);

			pm->fsm_state = pm->fsm_req;
			pm->fsm_phase = 0;
			break;

		case PM_STATE_LU_SHUTDOWN:
		case PM_STATE_PROBE_CONST_E:
		case PM_STATE_PROBE_CONST_J:
		case PM_STATE_ADJUST_HALL:

			if (pm->fsm_state != PM_STATE_IDLE)
				break;

			if (pm->lu_mode == PM_LU_DISABLED)
				break;

			pm->fsm_state = pm->fsm_req;
			pm->fsm_phase = 0;
			break;

		case PM_STATE_HALT:

			if (pm->fsm_state == PM_STATE_HALT)
				break;

			if (pm->fsm_state == PM_STATE_IDLE
			&& pm->lu_mode == PM_LU_DISABLED)
				break;

			pm->fsm_state = pm->fsm_req;
			pm->fsm_phase = 0;
			break;

		default:
			break;
	}

	pm->fsm_req = PM_STATE_IDLE;

	switch (pm->fsm_state) {

		case PM_STATE_IDLE:
			break;

		case PM_STATE_ZERO_DRIFT:
			pm_fsm_state_zero_drift(pm);
			break;

		case PM_STATE_SELF_TEST_BOOTSTRAP:
			pm_fsm_state_self_test_bootstrap(pm);
			break;

		case PM_STATE_SELF_TEST_POWER_STAGE:
			pm_fsm_state_self_test_power_stage(pm);
			break;

		case PM_STATE_SELF_TEST_CLEARANCE:
			pm_fsm_state_self_test_clearance(pm);
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

		case PM_STATE_LU_DETACHED:
		case PM_STATE_LU_STARTUP:
			pm_fsm_state_lu_startup(pm);
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

		case PM_STATE_ADJUST_HALL:
			pm_fsm_state_adjust_hall(pm);
			break;

		case PM_STATE_HALT:
		default:
			pm_fsm_state_halt(pm);
	}
}

const char *pm_strerror(int n)
{
	const char	*list[] = {

		PM_SFI(PM_OK),

		PM_SFI(PM_ERROR_ZERO_DRIFT_FAULT),
		PM_SFI(PM_ERROR_NO_MOTOR_CONNECTED),
		PM_SFI(PM_ERROR_POWER_STAGE_FAULT),
		PM_SFI(PM_ERROR_ACCURACY_FAULT),
		PM_SFI(PM_ERROR_CURRENT_LOOP_FAULT),
		PM_SFI(PM_ERROR_INLINE_OVER_CURRENT),
		PM_SFI(PM_ERROR_DC_LINK_OVER_VOLTAGE),
		PM_SFI(PM_ERROR_FLUX_UNSTABLE),
		PM_SFI(PM_ERROR_INVALID_OPERATION),
		PM_SFI(PM_ERROR_SENSOR_HALL_FAULT),
		PM_SFI(PM_ERROR_SENSOR_QENC_FAULT),

		PM_SFI(PM_ERROR_TIMEOUT),
	};

	const int 	lmax = sizeof(list) / sizeof(list[0]);

	return (n >= 0 && n < lmax) ? list[n] : "";
}

