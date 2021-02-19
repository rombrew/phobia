#include "libm.h"
#include "pm.h"

static float
pm_DFT_R(const float DFT[8])
{
	float			D, X, Y, E, R = 0.;

	/* Here is resistance calculation on DC condition.
	 * R = U / I.
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

void pm_DFT_LDQ(const float DFT[8], float WF, float LDQ[5])
{
	float		LSQ[9], B[4], LXY[3], R;

	/* The primary equation is Z * I = U,
	 *
	 * [R-j*LXY[0]   j*LXY[1]] * [IX] = [UX]
	 * [  j*LXY[1] R-j*LXY[2]] * [IY]   [UY], where
	 *
	 * IX = [DFT[0]+j*DFT[1]], UX = [DFT[2]+j*DFT[3]]
	 * IY = [DFT[4]+j*DFT[5]], UY = [DFT[6]+j*DFT[7]].
	 *
	 * Then rewrite it with respect to the impedance components.
	 *
	 * [DFT[0]  DFT[1] -DFT[5]  0     ] * [R     ]   [DFT[2]]
	 * [DFT[1] -DFT[0]  DFT[4]  0     ]   [LXY[0]] = [DFT[3]]
	 * [DFT[4]  0      -DFT[1]  DFT[5]]   [LXY[1]]   [DFT[6]].
	 * [DFT[5]  0       DFT[0] -DFT[4]]   [LYY[2]]   [DFT[7]]
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

	m_la_LSQ_3(LSQ, LXY);

	WF = 1.f / WF;

	LXY[0] *= WF;
	LXY[1] *= WF;
	LXY[2] *= WF;

	m_la_EIG(LXY, LDQ);

	LDQ[4] = R;
}

static void
pm_fsm_state_idle(pmc_t *pm)
{
	if (		pm->lu_mode == PM_LU_DISABLED
			&& pm->config_BOOST == PM_ENABLED) {

		switch (pm->fsm_phase) {

			case 0:
				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(5);

				pm->fsm_phase = 1;
				break;

			case 1:
				if (m_fabsf(pm->fb_iB) > pm->fault_current_tol) {

					pm->fsm_state = PM_STATE_LOOP_BOOST;
					pm->fsm_phase = 0;
				}
				break;
		}
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
			pm->REM[0] = 0.f;
			pm->REM[1] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_drift);

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			m_rsum(&pm->probe_DFT[0], &pm->REM[0], pm->fb_iA);
			m_rsum(&pm->probe_DFT[1], &pm->REM[1], pm->fb_iB);

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
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(6);

			pm->self_BST[0] = 0.f;
			pm->self_BST[1] = 0.f;
			pm->self_BST[2] = 0.f;

			pm->vsi_SA = PM_TSMS(pm, pm->tm_transient_fast);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

			pm->fsm_phase = 2;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->proc_set_DC(pm->dc_resolution, 0, 0);

				pm->vsi_TIM = 0;

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			if (m_fabsf(pm->fb_uA - pm->const_fb_U) < pm->fault_voltage_tol) {

				pm->vsi_TIM = 0;

				pm->self_BST[0] += 1.f;
			}
			else {
				pm->vsi_TIM++;

				if (pm->vsi_TIM >= pm->vsi_SA) {

					pm->tm_value = pm->tm_end;
				}
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->self_BST[0] *= 1000.f / pm->freq_hz;

				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(5);

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

				pm->fsm_phase = 4;
			}
			break;

		case 4:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->proc_set_DC(0, pm->dc_resolution, 0);

				pm->vsi_TIM = 0;

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_phase = 5;
			}
			break;

		case 5:
			if (m_fabsf(pm->fb_uB - pm->const_fb_U) < pm->fault_voltage_tol) {

				pm->vsi_TIM = 0;

				pm->self_BST[1] += 1.f;
			}
			else {
				pm->vsi_TIM++;

				if (pm->vsi_TIM >= pm->vsi_SA) {

					pm->tm_value = pm->tm_end;
				}
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->self_BST[1] *= 1000.f / pm->freq_hz;

				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(3);

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

				pm->fsm_phase = 6;
			}
			break;

		case 6:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->proc_set_DC(0, 0, pm->dc_resolution);

				pm->vsi_TIM = 0;

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_phase = 7;
			}
			break;

		case 7:
			if (m_fabsf(pm->fb_uC - pm->const_fb_U) < pm->fault_voltage_tol) {

				pm->vsi_TIM = 0;

				pm->self_BST[2] += 1.f;
			}
			else {
				pm->vsi_TIM++;

				if (pm->vsi_TIM >= pm->vsi_SA) {

					pm->tm_value = pm->tm_end;
				}
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->self_BST[2] *= 1000.f / pm->freq_hz;

				if (		pm->self_BST[0] < pm->dc_bootstrap
						|| pm->self_BST[1] < pm->dc_bootstrap
						|| pm->self_BST[2] < pm->dc_bootstrap) {

					pm->fail_reason = PM_ERROR_BOOTSTRAP_TIME;
				}

				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}
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
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

			pm->fsm_phase = 2;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->probe_DFT[0] = 0.f;
				pm->probe_DFT[1] = 0.f;
				pm->probe_DFT[2] = 0.f;
				pm->REM[0] = 0.f;
				pm->REM[1] = 0.f;
				pm->REM[2] = 0.f;

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_instant_probe);

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			m_rsum(&pm->probe_DFT[0], &pm->REM[0], pm->fb_uA);
			m_rsum(&pm->probe_DFT[1], &pm->REM[1], pm->fb_uB);
			m_rsum(&pm->probe_DFT[2], &pm->REM[2], pm->fb_uC);

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

			bm  = (m_fabsf(uA - pm->const_fb_U) < pm->fault_voltage_tol)
				? 1UL : (m_fabsf(uA) < pm->fault_voltage_tol) ? 0UL : 16UL;
			bm |= (m_fabsf(uB - pm->const_fb_U) < pm->fault_voltage_tol)
				? 2UL : (m_fabsf(uB) < pm->fault_voltage_tol) ? 0UL : 32UL;
			bm |= (m_fabsf(uC - pm->const_fb_U) < pm->fault_voltage_tol)
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
				pm->fail_reason = PM_ERROR_POWER_STAGE_DAMAGED;
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

			pm->REM[0] = 0.f;
			pm->REM[1] = 0.f;
			pm->REM[2] = 0.f;
			pm->REM[3] = 0.f;
			pm->REM[4] = 0.f;
			pm->REM[5] = 0.f;
			pm->REM[6] = 0.f;
			pm->REM[7] = 0.f;
			pm->REM[8] = 0.f;
			pm->REM[9] = 0.f;
			pm->REM[10] = 0.f;
			pm->REM[11] = 0.f;
			pm->REM[12] = 0.f;
			pm->REM[13] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_slow);

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			m_rsum(&pm->self_RMSi[0], &pm->REM[0], pm->fb_iA * pm->fb_iA);
			m_rsum(&pm->self_RMSi[1], &pm->REM[1], pm->fb_iB * pm->fb_iB);
			m_rsum(&pm->self_RMSi[2], &pm->REM[2], pm->const_fb_U * pm->const_fb_U);
			m_rsum(&pm->REM[6], &pm->REM[10], pm->const_fb_U);

			if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				m_rsum(&pm->self_RMSu[0], &pm->REM[3], pm->fb_uA * pm->fb_uA);
				m_rsum(&pm->self_RMSu[1], &pm->REM[4], pm->fb_uB * pm->fb_uB);
				m_rsum(&pm->self_RMSu[2], &pm->REM[5], pm->fb_uC * pm->fb_uC);

				m_rsum(&pm->REM[7], &pm->REM[11], pm->fb_uA);
				m_rsum(&pm->REM[8], &pm->REM[12], pm->fb_uB);
				m_rsum(&pm->REM[9], &pm->REM[13], pm->fb_uC);
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				RMS = pm->self_RMSi[0] / pm->tm_end;
				pm->self_RMSi[0] = m_sqrtf(RMS);

				RMS = pm->self_RMSi[1] / pm->tm_end;
				pm->self_RMSi[1] = m_sqrtf(RMS);

				RMS = pm->self_RMSi[2] / pm->tm_end;
				AVG = pm->REM[6] / pm->tm_end;
				pm->self_RMSi[2] = m_sqrtf(RMS - AVG * AVG);

				RMS = pm->self_RMSu[0] / pm->tm_end;
				AVG = pm->REM[7] / pm->tm_end;
				pm->self_RMSu[0] = m_sqrtf(RMS - AVG * AVG);

				RMS = pm->self_RMSu[1] / pm->tm_end;
				AVG = pm->REM[8] / pm->tm_end;
				pm->self_RMSu[1] = m_sqrtf(RMS - AVG * AVG);

				RMS = pm->self_RMSu[2] / pm->tm_end;
				AVG = pm->REM[9] / pm->tm_end;
				pm->self_RMSu[2] = m_sqrtf(RMS - AVG * AVG);

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			if (		   m_isfinitef(pm->self_RMSi[0]) != 0
					&& m_isfinitef(pm->self_RMSi[1]) != 0
					&& m_isfinitef(pm->self_RMSi[2]) != 0
					&& m_isfinitef(pm->self_RMSu[0]) != 0
					&& m_isfinitef(pm->self_RMSu[1]) != 0
					&& m_isfinitef(pm->self_RMSu[2]) != 0) {

				if (		pm->self_RMSi[0] > pm->fault_current_tol
						|| pm->self_RMSi[1] > pm->fault_current_tol) {

					pm->fail_reason = PM_ERROR_LOW_ACCURACY;
				}
				else if (	pm->self_RMSi[2] > pm->fault_voltage_tol
						|| pm->self_RMSu[0] > pm->fault_voltage_tol
						|| pm->self_RMSu[1] > pm->fault_voltage_tol
						|| pm->self_RMSu[2] > pm->fault_voltage_tol) {

					pm->fail_reason = PM_ERROR_LOW_ACCURACY;
				}
			}
			else {
				pm->fail_reason = PM_ERROR_INVALID_OPERATION;
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
			pm->proc_set_Z(pm->k_ZNUL);

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = 0.f;
			pm->probe_DFT[3] = 0.f;
			pm->REM[0] = 0.f;
			pm->REM[1] = 0.f;
			pm->REM[2] = 0.f;
			pm->REM[3] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

			pm->fsm_phase = 2;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_voltage_hold);

				if (pm->ts_bootstrap != 0) {

					pm->tm_end = (pm->tm_end > pm->ts_bootstrap)
						? pm->ts_bootstrap : pm->tm_end;
				}

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			m_rsum(&pm->probe_DFT[0], &pm->REM[0], pm->fb_uA);
			m_rsum(&pm->probe_DFT[1], &pm->REM[1], pm->fb_uB);
			m_rsum(&pm->probe_DFT[2], &pm->REM[2], pm->fb_uC);
			m_rsum(&pm->probe_DFT[3], &pm->REM[3], pm->const_fb_U);

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

				pm->tvm_ENABLED = PM_DISABLED;

				pm->fail_reason = PM_ERROR_LOW_ACCURACY;
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

				pm->tvm_ENABLED = PM_DISABLED;

				pm->fail_reason = PM_ERROR_LOW_ACCURACY;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			for (N = 0; N < 9; ++N) {

				pm->probe_LSQ_A[N] = 0.f;
				pm->probe_LSQ_B[N] = 0.f;
				pm->probe_LSQ_C[N] = 0.f;
				pm->REM[N] = 0.f;
				pm->REM[N + 9] = 0.f;
				pm->REM[N + 18] = 0.f;
			}

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

			pm->fsm_phase = 6;
			pm->fsm_phase_2 = 0;
			break;

		case 6:
			xMIN = pm->ts_minimal;
			xMAX = (int) (pm->dc_resolution * pm->tvm_range_DC);

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

				REF = pm->probe_DFT[0] * pm->const_fb_U * pm->ts_inverted;

				m_rsum(&pm->probe_LSQ_A[0], &pm->REM[0], pm->fb_uA * pm->fb_uA);
				m_rsum(&pm->probe_LSQ_A[1], &pm->REM[1], pm->fb_uA * pm->probe_DFT[2]);
				m_rsum(&pm->probe_LSQ_A[2], &pm->REM[2], pm->probe_DFT[2] * pm->probe_DFT[2]);
				m_rsum(&pm->probe_LSQ_A[3], &pm->REM[3], pm->fb_uA);
				m_rsum(&pm->probe_LSQ_A[4], &pm->REM[4], pm->probe_DFT[2]);
				m_rsum(&pm->probe_LSQ_A[5], &pm->REM[5], 1.f);
				m_rsum(&pm->probe_LSQ_A[6], &pm->REM[6], pm->fb_uA * REF);
				m_rsum(&pm->probe_LSQ_A[7], &pm->REM[7], pm->probe_DFT[2] * REF);
				m_rsum(&pm->probe_LSQ_A[8], &pm->REM[8], REF);

				m_rsum(&pm->probe_LSQ_B[0], &pm->REM[9], pm->fb_uB * pm->fb_uB);
				m_rsum(&pm->probe_LSQ_B[1], &pm->REM[10], pm->fb_uB * pm->probe_DFT[3]);
				m_rsum(&pm->probe_LSQ_B[2], &pm->REM[11], pm->probe_DFT[3] * pm->probe_DFT[3]);
				m_rsum(&pm->probe_LSQ_B[3], &pm->REM[12], pm->fb_uB);
				m_rsum(&pm->probe_LSQ_B[4], &pm->REM[13], pm->probe_DFT[3]);
				m_rsum(&pm->probe_LSQ_B[5], &pm->REM[14], 1.f);
				m_rsum(&pm->probe_LSQ_B[6], &pm->REM[15], pm->fb_uB * REF);
				m_rsum(&pm->probe_LSQ_B[7], &pm->REM[16], pm->probe_DFT[3] * REF);
				m_rsum(&pm->probe_LSQ_B[8], &pm->REM[17], REF);

				m_rsum(&pm->probe_LSQ_C[0], &pm->REM[18], pm->fb_uC * pm->fb_uC);
				m_rsum(&pm->probe_LSQ_C[1], &pm->REM[19], pm->fb_uC * pm->probe_DFT[4]);
				m_rsum(&pm->probe_LSQ_C[2], &pm->REM[20], pm->probe_DFT[4] * pm->probe_DFT[4]);
				m_rsum(&pm->probe_LSQ_C[3], &pm->REM[21], pm->fb_uC);
				m_rsum(&pm->probe_LSQ_C[4], &pm->REM[22], pm->probe_DFT[4]);
				m_rsum(&pm->probe_LSQ_C[5], &pm->REM[23], 1.f);
				m_rsum(&pm->probe_LSQ_C[6], &pm->REM[24], pm->fb_uC * REF);
				m_rsum(&pm->probe_LSQ_C[7], &pm->REM[25], pm->probe_DFT[4] * REF);
				m_rsum(&pm->probe_LSQ_C[8], &pm->REM[26], REF);
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
			m_la_LSQ_3(pm->probe_LSQ_A, pm->tvm_FIR_A);
			m_la_LSQ_3(pm->probe_LSQ_B, pm->tvm_FIR_B);
			m_la_LSQ_3(pm->probe_LSQ_C, pm->tvm_FIR_C);

			if (		   m_isfinitef(pm->tvm_FIR_A[0]) != 0
					&& m_isfinitef(pm->tvm_FIR_A[1]) != 0
					&& m_isfinitef(pm->tvm_FIR_A[2]) != 0
					&& m_isfinitef(pm->tvm_FIR_B[0]) != 0
					&& m_isfinitef(pm->tvm_FIR_B[1]) != 0
					&& m_isfinitef(pm->tvm_FIR_B[2]) != 0
					&& m_isfinitef(pm->tvm_FIR_C[0]) != 0
					&& m_isfinitef(pm->tvm_FIR_C[1]) != 0
					&& m_isfinitef(pm->tvm_FIR_C[2]) != 0) {

				pm->tvm_ENABLED = PM_ENABLED;
			}
			else {
				pm->tvm_ENABLED = PM_DISABLED;

				pm->fail_reason = PM_ERROR_INVALID_OPERATION;
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
			pm->REM[0] = 0.f;
			pm->REM[1] = 0.f;
			pm->REM[2] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_slow);

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			m_rsum(&pm->probe_DFT[0], &pm->REM[0], pm->fb_iA);
			m_rsum(&pm->probe_DFT[1], &pm->REM[1], - pm->fb_iB);

		case 1:
			eX = pm->probe_current_hold - pm->fb_iA;

			pm->REM[2] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->REM[2];

			uMAX = pm->k_UMAX * pm->const_fb_U;

			if (m_fabsf(pm->REM[2]) > uMAX) {

				pm->fail_reason = PM_ERROR_CURRENT_LOOP_IS_OPEN;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm_voltage(pm, uX, 0.f);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

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

				pm->fail_reason = PM_ERROR_LOW_ACCURACY;
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
	float			uMAX, hold_R;

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(pm->k_ZNUL);

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;

			hold_R = pm->probe_hold_angle * (M_PI_F / 180.f);
			hold_R = (hold_R < - M_PI_F) ? - M_PI_F :
				(hold_R > M_PI_F) ? M_PI_F : hold_R;

			pm->lu_F[0] = 1.f;
			pm->lu_F[1] = 0.f;

			pm->i_track_D = m_cosf(hold_R) * pm->probe_current_hold;
			pm->i_track_Q = m_sinf(hold_R) * pm->probe_current_hold;

			pm->probe_DFT[2] = pm->i_track_D;
			pm->probe_DFT[3] = pm->i_track_Q;

			pm->REM[0] = 0.f;
			pm->REM[1] = 0.f;
			pm->REM[2] = 0.f;
			pm->REM[3] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_current_hold);

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			m_rsum(&pm->probe_DFT[0], &pm->REM[0], pm->tvm_DX);
			m_rsum(&pm->probe_DFT[1], &pm->REM[1], pm->tvm_DY);

		case 1:
			eX = pm->i_track_D - pm->lu_iX;
			eY = pm->i_track_Q - pm->lu_iY;

			pm->REM[2] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->REM[2];

			pm->REM[3] += pm->probe_gain_I * eY;
			uY = pm->probe_gain_P * eY + pm->REM[3];

			uMAX = pm->k_UMAX * pm->const_fb_U;

			if (m_fabsf(pm->REM[2]) > uMAX || m_fabsf(pm->REM[3]) > uMAX) {

				pm->fail_reason = PM_ERROR_CURRENT_LOOP_IS_OPEN;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm_voltage(pm, uX, uY);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm->probe_DFT[0] /= pm->tm_end;
			pm->probe_DFT[1] /= pm->tm_end;

			pm->const_im_R = pm_DFT_R(pm->probe_DFT);

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_probe_const_l(pmc_t *pm)
{
	float			iX, iY, eHF, eX, eY, uHF, uX, uY;
	float			uMAX, hold_R, LDQ[5];

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(pm->k_ZNUL);

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = 0.f;
			pm->probe_DFT[3] = 0.f;
			pm->probe_DFT[4] = 0.f;
			pm->probe_DFT[5] = 0.f;
			pm->probe_DFT[6] = 0.f;
			pm->probe_DFT[7] = 0.f;

			pm->REM[0] = 0.f;
			pm->REM[1] = 0.f;
			pm->REM[2] = 0.f;
			pm->REM[3] = 0.f;
			pm->REM[4] = 0.f;
			pm->REM[5] = 0.f;
			pm->REM[6] = 0.f;
			pm->REM[7] = 0.f;

			pm->hfi_wave[0] = 1.f;
			pm->hfi_wave[1] = 0.f;

			pm->temp_HFI_wS = 2.f * M_PI_F * pm->probe_freq_sine_hz;
			pm->temp_HFI_HT[0] = m_cosf(pm->temp_HFI_wS * pm->dT * .5f);
			pm->temp_HFI_HT[1] = m_sinf(pm->temp_HFI_wS * pm->dT * .5f);

			pm->REM[9] = 0.f;
			pm->REM[10] = 0.f;
			pm->REM[11] = 0.f;

			hold_R = pm->probe_hold_angle * (M_PI_F / 180.f);
			hold_R = (hold_R < - M_PI_F) ? - M_PI_F :
				(hold_R > M_PI_F) ? M_PI_F : hold_R;

			pm->lu_F[0] = 1.f;
			pm->lu_F[1] = 0.f;

			pm->i_track_D = m_cosf(hold_R) * pm->probe_current_weak;
			pm->i_track_Q = m_sinf(hold_R) * pm->probe_current_weak;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_current_hold);

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			iX = pm->REM[14];
			iY = pm->REM[15];

			uX = pm->tvm_DX * pm->temp_HFI_HT[0] + pm->tvm_DY * pm->temp_HFI_HT[1];
			uY = pm->tvm_DY * pm->temp_HFI_HT[0] - pm->tvm_DX * pm->temp_HFI_HT[1];

			m_rsum(&pm->probe_DFT[0], &pm->REM[0], iX * pm->hfi_wave[0]);
			m_rsum(&pm->probe_DFT[1], &pm->REM[1], iX * pm->hfi_wave[1]);
			m_rsum(&pm->probe_DFT[2], &pm->REM[2], uX * pm->hfi_wave[0]);
			m_rsum(&pm->probe_DFT[3], &pm->REM[3], uX * pm->hfi_wave[1]);
			m_rsum(&pm->probe_DFT[4], &pm->REM[4], iY * pm->hfi_wave[0]);
			m_rsum(&pm->probe_DFT[5], &pm->REM[5], iY * pm->hfi_wave[1]);
			m_rsum(&pm->probe_DFT[6], &pm->REM[6], uY * pm->hfi_wave[0]);
			m_rsum(&pm->probe_DFT[7], &pm->REM[7], uY * pm->hfi_wave[1]);

		case 1:
			m_rotf(pm->hfi_wave, pm->temp_HFI_wS * pm->dT, pm->hfi_wave);

			eX = pm->i_track_D - pm->lu_iX;
			eY = pm->i_track_Q - pm->lu_iY;

			eHF = pm->probe_current_sine - m_sqrtf(eX * eX + eY * eY);

			pm->REM[9] += pm->probe_gain_I * eHF;
			uHF = pm->probe_gain_P * eHF + pm->REM[9];

			pm->REM[10] += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->REM[10];

			pm->REM[11] += pm->probe_gain_I * eY;
			uY = pm->probe_gain_P * eY + pm->REM[11];

			uMAX = pm->k_UMAX * pm->const_fb_U;

			if (		m_fabsf(pm->REM[9]) > uMAX
					|| m_fabsf(pm->REM[10]) > uMAX
					|| m_fabsf(pm->REM[11]) > uMAX) {

				pm->fail_reason = PM_ERROR_CURRENT_LOOP_IS_OPEN;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm->REM[14] = pm->lu_iX;
			pm->REM[15] = pm->lu_iY;

			uX += uHF * pm->hfi_wave[0];
			uY += uHF * pm->hfi_wave[1];

			pm_voltage(pm, uX, uY);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm_DFT_LDQ(pm->probe_DFT, pm->temp_HFI_wS, LDQ);

			if (m_isfinitef(LDQ[3]) != 0 && LDQ[3] > M_EPS_F) {

				pm->const_L = LDQ[3];

				pm->const_im_L1 = LDQ[2];
				pm->const_im_L2 = LDQ[3];
				pm->const_im_B = m_atan2f(LDQ[1], LDQ[0]) * (180.f / M_PI_F);
				pm->const_im_R = LDQ[4];

				/* Tune current loop.
				 * */
				pm->i_gain_P = 2E-1f * pm->const_L * pm->freq_hz - pm->const_R;
				pm->i_gain_I = 1E-2f * pm->const_L * pm->freq_hz;
			}
			else {
				pm->fail_reason = PM_ERROR_INVALID_OPERATION;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_lu_startup(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			if (pm->const_L > M_EPS_F) {

				pm->vsi_SA = 0;
				pm->vsi_SB = 0;
				pm->vsi_SC = 0;
				pm->vsi_TIM = 0;
				pm->vsi_AG = 0;
				pm->vsi_BG = 0;
				pm->vsi_CG = 0;
				pm->vsi_AF = 0;
				pm->vsi_BF = 0;
				pm->vsi_SF = 0;
				pm->vsi_UF = 0;
				pm->vsi_AZ = 0;
				pm->vsi_BZ = 0;
				pm->vsi_CZ = 0;

				pm->vsi_lpf_DC = 0.f;

				pm->lu_iX = 0.f;
				pm->lu_iY = 0.f;
				pm->lu_iD = 0.f;
				pm->lu_iQ = 0.f;
				pm->lu_F[0] = 1.f;
				pm->lu_F[1] = 0.f;
				pm->lu_wS = 0.f;
				pm->lu_mode = PM_LU_DETACHED;

				pm->lu_lpf_torque = 0.f;
				pm->lu_base_wS = 0.f;

				pm->forced_F[0] = 1.f;
				pm->forced_F[1] = 0.f;
				pm->forced_wS = 0.f;

				pm->detach_TIM = 0;
				pm->detach_SKIP = 0;

				pm->flux_X = pm->const_E;
				pm->flux_Y = 0.f;
				pm->flux_E = 0.f;
				pm->flux_F[0] = 1.f;
				pm->flux_F[1] = 0.f;
				pm->flux_wS = 0.f;

				pm->flux_lpf_wS = 0.f;
				pm->flux_mode = PM_FLUX_UNCERTAIN;
				pm->flux_locked = PM_FLUX_UNCERTAIN;

				pm->hfi_F[0] = 1.f;
				pm->hfi_F[1] = 0.f;
				pm->hfi_wS = 0.f;
				pm->hfi_wave[0] = 1.f;
				pm->hfi_wave[1] = 0.f;
				pm->hfi_DFT_N = 0;
				pm->hfi_DFT_P = 0;

				pm->hall_F[0] = 1.f;
				pm->hall_F[1] = 0.f;
				pm->hall_wS = 0.f;

				pm->abi_F[0] = 1.f;
				pm->abi_F[1] = 0.f;
				pm->abi_wS = 0.f;

				pm->watt_lpf_D = 0.f;
				pm->watt_lpf_Q = 0.f;
				pm->watt_lpf_wP = 0.f;

				pm->i_derated_PCB = PM_MAX_F;
				pm->i_derated_WEAK = PM_MAX_F;
				pm->i_setpoint_torque = 0.f;
				pm->i_track_D = 0.f;
				pm->i_track_Q = 0.f;
				pm->i_integral_D = 0.f;
				pm->i_integral_Q = 0.f;
				pm->i_slew_dT = 0.f;

				pm->weak_D = 0.f;

				pm->s_setpoint_speed = 0.f;
				pm->s_track = 0.f;
				pm->s_iSP = 0.f;

				pm->x_setpoint_F[0] = 1.f;
				pm->x_setpoint_F[1] = 0.f;
				pm->x_setpoint_speed = 0.f;
				pm->x_setpoint_revol = 0;
				pm->x_lu_F1 = pm->lu_F[1];
				pm->x_lu_revol = 0;
				pm->x_residual = 0.f;

				pm->fail_reason = PM_OK;

				if (pm->fsm_state == PM_STATE_LU_DETACHED) {

					pm->flux_mode = PM_FLUX_LOCKED_IN_DETACH;
				}

				if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(7);
				}
				else {
					pm->lu_mode = PM_LU_ESTIMATE_FLUX;

					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(pm->k_ZNUL);
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
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_slow);

			pm->fsm_phase = 1;

		case 1:
			pm->i_derated_PCB = 0.f;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->lu_mode = PM_LU_DISABLED;

				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(7);

				pm->vsi_AF = 0;
				pm->vsi_BF = 0;
				pm->vsi_SF = 0;
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
	float			temp_E;

	switch (pm->fsm_phase) {

		case 0:
			pm->probe_DFT[0] = 0.f;
			pm->REM[0] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, (pm->lu_mode == PM_LU_DETACHED)
					? pm->tm_average_drift : pm->tm_average_probe);

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			m_rsum(&pm->probe_DFT[0], &pm->REM[0], pm->flux_E);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->probe_DFT[0] /= pm->tm_end;
				pm->fsm_phase = 2;
			}
			break;

		case 2:
			temp_E = pm->probe_DFT[0];

			if (m_isfinitef(temp_E) != 0 && temp_E > M_EPS_F) {

				pm->const_E = temp_E;
				pm->temp_const_iE = 1.f / temp_E;
			}
			else {
				pm->fail_reason = PM_ERROR_INVALID_OPERATION;
			}

			pm->fsm_state = PM_STATE_IDLE;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_probe_const_ja(pmc_t *pm)
{
	float			temp_Ja;

	switch (pm->fsm_phase) {

		case 0:
			pm->fail_reason = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;

			pm->probe_LSQ_A[0] = 0.f;
			pm->probe_LSQ_A[1] = 0.f;
			pm->probe_LSQ_A[2] = 0.f;
			pm->probe_LSQ_A[3] = 0.f;
			pm->probe_LSQ_A[4] = 0.f;
			pm->probe_LSQ_A[5] = 0.f;
			pm->probe_LSQ_A[6] = 0.f;
			pm->probe_LSQ_A[7] = 0.f;
			pm->probe_LSQ_A[8] = 0.f;

			pm->REM[0] = 0.f;
			pm->REM[1] = 0.f;
			pm->REM[2] = 0.f;
			pm->REM[3] = 0.f;
			pm->REM[4] = 0.f;
			pm->REM[5] = 0.f;
			pm->REM[6] = 0.f;
			pm->REM[7] = 0.f;
			pm->REM[8] = 0.f;
			pm->REM[9] = 0.f;
			pm->REM[10] = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_inertia);

			pm->fail_reason = PM_OK;
			pm->fsm_phase = 2;
			break;

		case 2:
			m_rsum(&pm->probe_DFT[0], &pm->REM[0], pm->lu_iQ);
			m_rsum(&pm->probe_DFT[1], &pm->REM[1], 1.f);

			m_rsum(&pm->probe_LSQ_A[0], &pm->REM[2], pm->probe_DFT[0] * pm->probe_DFT[0]);
			m_rsum(&pm->probe_LSQ_A[1], &pm->REM[3], pm->probe_DFT[1] * pm->probe_DFT[0]);
			m_rsum(&pm->probe_LSQ_A[2], &pm->REM[4], pm->probe_DFT[1] * pm->probe_DFT[1]);
			m_rsum(&pm->probe_LSQ_A[3], &pm->REM[5], pm->probe_DFT[0]);
			m_rsum(&pm->probe_LSQ_A[4], &pm->REM[6], pm->probe_DFT[1]);
			m_rsum(&pm->probe_LSQ_A[5], &pm->REM[7], 1.f);
			m_rsum(&pm->probe_LSQ_A[6], &pm->REM[8], pm->probe_DFT[0] * pm->lu_wS);
			m_rsum(&pm->probe_LSQ_A[7], &pm->REM[9], pm->probe_DFT[1] * pm->lu_wS);
			m_rsum(&pm->probe_LSQ_A[8], &pm->REM[10], pm->lu_wS);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			m_la_LSQ_3(pm->probe_LSQ_A, pm->probe_DFT);

			temp_Ja = pm->dT / pm->probe_DFT[0];

			if (m_isfinitef(temp_Ja) != 0 && temp_Ja > M_EPS_F) {

				pm->const_Ja = temp_Ja;
			}
			else {
				pm->fail_reason = PM_ERROR_INVALID_OPERATION;
			}

			pm->fsm_state = PM_STATE_IDLE;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_adjust_sensor_hall(pmc_t *pm)
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
					pm->REM[N] = 0.f;
					pm->REM[N + 8] = 0.f;
				}

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

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

				m_rsum(&pm->hall_ST[HS].X, &pm->REM[HS], pm->lu_F[0]);
				m_rsum(&pm->hall_ST[HS].Y, &pm->REM[HS + 8], pm->lu_F[1]);

				pm->probe_DFT[HS] += 1.f;
			}
			else {
				pm->hall_ENABLED = PM_DISABLED;

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
			min_S = pm->tm_end / 12;
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

				pm->hall_ENABLED = PM_DISABLED;

				pm->fail_reason = PM_ERROR_SENSOR_HALL_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}
			else {
				pm->hall_ENABLED = PM_ENABLED;

				pm->fsm_state = PM_STATE_IDLE;
				pm->fsm_phase = 0;
			}
			break;
	}
}

static void
pm_fsm_state_loop_boost(pmc_t *pm)
{
	/* TODO */
}

static void
pm_fsm_state_halt(pmc_t *pm)
{
	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(7);

			pm->vsi_AF = 0;
			pm->vsi_BF = 0;
			pm->vsi_SF = 0;
			pm->vsi_UF = 0;
			pm->vsi_AZ = 0;
			pm->vsi_BZ = 0;
			pm->vsi_CZ = 0;

			if (pm->lu_mode != PM_LU_DISABLED) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_halt_pause);

				pm->lu_mode = PM_LU_DISABLED;
			}
			else {
				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_transient_slow);
			}

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
		case PM_STATE_ADJUST_SENSOR_HALL:

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
			pm_fsm_state_idle(pm);
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
			pm_fsm_state_probe_const_ja(pm);
			break;

		case PM_STATE_ADJUST_SENSOR_HALL:
			pm_fsm_state_adjust_sensor_hall(pm);
			break;

		case PM_STATE_LOOP_BOOST:
			pm_fsm_state_loop_boost(pm);
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
		PM_SFI(PM_ERROR_BOOTSTRAP_TIME),
		PM_SFI(PM_ERROR_POWER_STAGE_DAMAGED),
		PM_SFI(PM_ERROR_LOW_ACCURACY),
		PM_SFI(PM_ERROR_CURRENT_LOOP_IS_OPEN),
		PM_SFI(PM_ERROR_INLINE_OVERCURRENT),
		PM_SFI(PM_ERROR_DC_LINK_OVERVOLTAGE),
		PM_SFI(PM_ERROR_INVALID_OPERATION),
		PM_SFI(PM_ERROR_SENSOR_HALL_FAULT),
		PM_SFI(PM_ERROR_SENSOR_ABI_FAULT),

		PM_SFI(PM_ERROR_TIMEOUT),
		PM_SFI(PM_ERROR_NO_FLUX_CAUGHT),
		PM_SFI(PM_ERROR_LOSS_OF_SYNC),
	};

	const int 	lmax = sizeof(list) / sizeof(list[0]);

	return (n >= 0 && n < lmax) ? list[n] : "";
}

