#include "libm.h"
#include "pm.h"

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
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[3];

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(7);

			lse_initiate(ls, 1, 2);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_drift);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			v[0] = 1.f;
			v[1] = pm->fb_iA;
			v[2] = pm->fb_iB;

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			lse_finalise(ls);

			pm->ad_IA[0] += - ls->b[0];
			pm->ad_IB[0] += - ls->b[1];

			if (		m_fabsf(pm->ad_IA[0]) > pm->fault_current_tol
					|| m_fabsf(pm->ad_IB[0]) > pm->fault_current_tol) {

				pm->fsm_errno = PM_ERROR_ZERO_DRIFT_FAULT;
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
			pm->fsm_errno = PM_OK;

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

					pm->fsm_errno = PM_ERROR_BOOTSTRAP_TIME;
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
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[4];

	float			uA, uB, uC;
	int			bm;

	switch (pm->fsm_phase) {

		case 0:
			pm->fsm_errno = PM_OK;

			if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				pm->fsm_phase = 1;
				pm->fsm_subi = 0;
			}
			else {
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}
			break;

		case 1:
			switch (pm->fsm_subi) {

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

				lse_initiate(ls, 1, 3);

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_instant_probe);

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			v[0] = 1.f;
			v[1] = pm->fb_uA;
			v[2] = pm->fb_uB;
			v[3] = pm->fb_uC;

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 4;
			}
			break;

		case 4:
			lse_finalise(ls);

			uA = ls->b[0];
			uB = ls->b[1];
			uC = ls->b[2];

			bm  = (m_fabsf(uA - pm->const_fb_U) < pm->fault_voltage_tol)
				? 1UL : (m_fabsf(uA) < pm->fault_voltage_tol) ? 0UL : 16UL;
			bm |= (m_fabsf(uB - pm->const_fb_U) < pm->fault_voltage_tol)
				? 2UL : (m_fabsf(uB) < pm->fault_voltage_tol) ? 0UL : 32UL;
			bm |= (m_fabsf(uC - pm->const_fb_U) < pm->fault_voltage_tol)
				? 4UL : (m_fabsf(uC) < pm->fault_voltage_tol) ? 0UL : 64UL;

			pm->self_BM[pm->fsm_subi] = bm;

			if (pm->fsm_subi < 6) {

				pm->fsm_phase = 1;
				pm->fsm_subi++;
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

				pm->fsm_errno = PM_OK;
			}
			else if (	   (pm->self_BM[1] & 17) == 1
					&& (pm->self_BM[2] & 17) == 0
					&& (pm->self_BM[3] & 34) == 2
					&& (pm->self_BM[4] & 34) == 0
					&& (pm->self_BM[5] & 68) == 4
					&& (pm->self_BM[6] & 68) == 0) {

				pm->fsm_errno = PM_ERROR_NO_MOTOR_CONNECTED;
			}
			else {
				pm->fsm_errno = PM_ERROR_POWER_STAGE_DAMAGED;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_self_test_clearance(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[7];

	switch (pm->fsm_phase) {

		case 0:
			lse_initiate(ls, 1, 6);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_slow);

			pm->fsm_errno = PM_OK;
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
			v[0] = 1.f;
			v[1] = pm->fb_iA;
			v[2] = pm->fb_iB;
			v[3] = pm->const_fb_U;

			if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				v[4] = pm->fb_uA;
				v[5] = pm->fb_uB;
				v[6] = pm->fb_uC;
			}
			else {
				v[4] = 0.f;
				v[5] = 0.f;
				v[6] = 0.f;
			}

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			lse_finalise(ls);

			pm->self_RMSi[0] = m_sqrtf(ls->b[0] * ls->b[0] + ls->e[0] * ls->e[0]);
			pm->self_RMSi[1] = m_sqrtf(ls->b[1] * ls->b[1] + ls->e[1] * ls->e[1]);
			pm->self_RMSi[2] = ls->e[2];
			pm->self_RMSu[0] = ls->e[3];
			pm->self_RMSu[1] = ls->e[4];
			pm->self_RMSu[2] = ls->e[5];

			if (		   m_isfinitef(pm->self_RMSi[0]) != 0
					&& m_isfinitef(pm->self_RMSi[1]) != 0
					&& m_isfinitef(pm->self_RMSi[2]) != 0
					&& m_isfinitef(pm->self_RMSu[0]) != 0
					&& m_isfinitef(pm->self_RMSu[1]) != 0
					&& m_isfinitef(pm->self_RMSu[2]) != 0) {

				if (		pm->self_RMSi[0] > pm->fault_current_tol
						|| pm->self_RMSi[1] > pm->fault_current_tol) {

					pm->fsm_errno = PM_ERROR_LOW_ACCURACY;
				}
				else if (	pm->self_RMSi[2] > pm->fault_voltage_tol
						|| pm->self_RMSu[0] > pm->fault_voltage_tol
						|| pm->self_RMSu[1] > pm->fault_voltage_tol
						|| pm->self_RMSu[2] > pm->fault_voltage_tol) {

					pm->fsm_errno = PM_ERROR_LOW_ACCURACY;
				}
			}
			else {
				pm->fsm_errno = PM_ERROR_INVALID_OPERATION;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_adjust_voltage(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[5];

	int			xDC, xMIN, xMAX;
	float			REF;

	switch (pm->fsm_phase) {

		case 0:
			pm->fsm_errno = PM_OK;

			if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				pm->fsm_phase = 1;
				pm->fsm_subi = 0;
			}
			else {
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}
			break;

		case 1:
			xDC = (pm->fsm_subi == 0) ? 0 : pm->dc_resolution;

			pm->proc_set_DC(xDC, xDC, xDC);
			pm->proc_set_Z(pm->k_ZNUL);

			lse_initiate(ls, 1, 4);

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
			v[0] = 1.f;
			v[1] = pm->fb_uA;
			v[2] = pm->fb_uB;
			v[3] = pm->fb_uC;
			v[4] = pm->const_fb_U;

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = (pm->fsm_subi == 0) ? 4 : 5;
			}
			break;

		case 4:
			lse_finalise(ls);

			pm->ad_UA[0] += - ls->b[0];
			pm->ad_UB[0] += - ls->b[1];
			pm->ad_UC[0] += - ls->b[2];

			if (		m_fabsf(pm->ad_UA[0]) > pm->fault_voltage_tol
					|| m_fabsf(pm->ad_UB[0]) > pm->fault_voltage_tol
					|| m_fabsf(pm->ad_UC[0]) > pm->fault_voltage_tol) {

				pm->tvm_ALLOWED = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_LOW_ACCURACY;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm->fsm_phase = 1;
			pm->fsm_subi = 1;
			break;

		case 5:
			lse_finalise(ls);

			pm->ad_UA[1] *= ls->b[3] / ls->b[0];
			pm->ad_UB[1] *= ls->b[3] / ls->b[1];
			pm->ad_UC[1] *= ls->b[3] / ls->b[2];

			if (		m_fabsf(pm->ad_UA[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->ad_UB[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->ad_UC[1] - 1.f) > pm->fault_accuracy_tol) {

				pm->tvm_ALLOWED = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_LOW_ACCURACY;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			lse_initiate(&pm->probe_LS[0], 3, 1);
			lse_initiate(&pm->probe_LS[1], 3, 1);
			lse_initiate(&pm->probe_LS[2], 3, 1);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

			pm->fsm_phase = 6;
			pm->fsm_subi = 0;
			break;

		case 6:
			xMIN = pm->ts_minimal;
			xMAX = (int) (pm->dc_resolution * pm->tvm_range_DC);

			switch (pm->fsm_subi) {

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

			pm->fsm_subi = (pm->fsm_subi < 23) ? pm->fsm_subi + 1 : 0;

			pm->proc_set_DC(xDC, xDC, xDC);

			if (pm->tm_value >= 2) {

				REF = pm->hfi_REM[0] * pm->const_fb_U * pm->ts_inverted;

				v[0] = pm->fb_uA;
				v[1] = pm->hfi_REM[2];
				v[2] = 1.f;
				v[3] = REF;

				lse_insert(&pm->probe_LS[0], v);

				v[0] = pm->fb_uB;
				v[1] = pm->hfi_REM[3];
				v[2] = 1.f;
				v[3] = REF;

				lse_insert(&pm->probe_LS[1], v);

				v[0] = pm->fb_uC;
				v[1] = pm->hfi_REM[4];
				v[2] = 1.f;
				v[3] = REF;

				lse_insert(&pm->probe_LS[2], v);
			}

			pm->hfi_REM[0] = pm->hfi_REM[1];
			pm->hfi_REM[1] = (float) xDC;
			pm->hfi_REM[2] = pm->fb_uA;
			pm->hfi_REM[3] = pm->fb_uB;
			pm->hfi_REM[4] = pm->fb_uC;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 7;
			}
			break;

		case 7:
			lse_finalise(&pm->probe_LS[0]);
			lse_finalise(&pm->probe_LS[1]);
			lse_finalise(&pm->probe_LS[2]);

			pm->tvm_FIR_A[0] = pm->probe_LS[0].b[0];
			pm->tvm_FIR_A[1] = pm->probe_LS[0].b[1];
			pm->tvm_FIR_A[2] = pm->probe_LS[0].b[2];

			pm->tvm_FIR_B[0] = pm->probe_LS[1].b[0];
			pm->tvm_FIR_B[1] = pm->probe_LS[1].b[1];
			pm->tvm_FIR_B[2] = pm->probe_LS[1].b[2];

			pm->tvm_FIR_C[0] = pm->probe_LS[2].b[0];
			pm->tvm_FIR_C[1] = pm->probe_LS[2].b[1];
			pm->tvm_FIR_C[2] = pm->probe_LS[2].b[2];

			pm->self_RMSu[0] = pm->probe_LS[0].e[0];
			pm->self_RMSu[1] = pm->probe_LS[1].e[0];
			pm->self_RMSu[2] = pm->probe_LS[2].e[0];

			if (		   m_isfinitef(pm->tvm_FIR_A[0]) != 0
					&& m_isfinitef(pm->tvm_FIR_A[1]) != 0
					&& m_isfinitef(pm->tvm_FIR_A[2]) != 0
					&& m_isfinitef(pm->tvm_FIR_B[0]) != 0
					&& m_isfinitef(pm->tvm_FIR_B[1]) != 0
					&& m_isfinitef(pm->tvm_FIR_B[2]) != 0
					&& m_isfinitef(pm->tvm_FIR_C[0]) != 0
					&& m_isfinitef(pm->tvm_FIR_C[1]) != 0
					&& m_isfinitef(pm->tvm_FIR_C[2]) != 0) {

				pm->tvm_ALLOWED = PM_ENABLED;
			}
			else {
				pm->tvm_ALLOWED = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_INVALID_OPERATION;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_adjust_current(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[3];

	float			eX, uX, REF;
	float			uMAX;

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(4);

			lse_initiate(ls, 1, 2);

			pm->i_integral_D = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_slow);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			v[0] = 1.f;
			v[1] = pm->fb_iA;
			v[2] = - pm->fb_iB;

			lse_insert(ls, v);

		case 1:
			eX = pm->probe_current_hold - pm->fb_iA;

			pm->i_integral_D += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->i_integral_D;

			uMAX = pm->k_UMAX * pm->const_fb_U;

			if (m_fabsf(pm->i_integral_D) > uMAX) {

				pm->fsm_errno = PM_ERROR_CURRENT_LOOP_IS_OPEN;
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
			lse_finalise(ls);

			REF = (ls->b[1] + ls->b[0]) / 2.f;
			pm->ad_IA[1] *= REF / ls->b[0];
			pm->ad_IB[1] *= REF / ls->b[1];

			if (		m_fabsf(pm->ad_IA[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->ad_IB[1] - 1.f) > pm->fault_accuracy_tol) {

				pm->fsm_errno = PM_ERROR_LOW_ACCURACY;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_probe_const_r(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[2];

	float			eX, eY, uX, uY;
	float			uMAX, hold_R;

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(pm->k_ZNUL);

			lse_initiate(ls, 1, 1);

			pm->lu_F[0] = 1.f;
			pm->lu_F[1] = 0.f;

			hold_R = pm->probe_hold_angle * (M_PI_F / 180.f);
			hold_R = (hold_R < - M_PI_F) ? - M_PI_F :
				(hold_R > M_PI_F) ? M_PI_F : hold_R;

			pm->i_track_D = m_cosf(hold_R) * pm->probe_current_hold;
			pm->i_track_Q = m_sinf(hold_R) * pm->probe_current_hold;

			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_current_hold);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			v[0] = pm->i_track_D;
			v[1] = pm->tvm_DX;

			lse_insert(ls, v);

			v[0] = pm->i_track_Q;
			v[1] = pm->tvm_DY;

			lse_insert(ls, v);

		case 1:
			eX = pm->i_track_D - pm->lu_iX;
			eY = pm->i_track_Q - pm->lu_iY;

			pm->i_integral_D += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->i_integral_D;

			pm->i_integral_Q += pm->probe_gain_I * eY;
			uY = pm->probe_gain_P * eY + pm->i_integral_Q;

			uMAX = pm->k_UMAX * pm->const_fb_U;

			if (		m_fabsf(pm->i_integral_D) > uMAX
					|| m_fabsf(pm->i_integral_Q) > uMAX) {

				pm->fsm_errno = PM_ERROR_CURRENT_LOOP_IS_OPEN;
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
			lse_finalise(ls);

			pm->const_im_R = ls->b[0];

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_probe_const_l(pmc_t *pm)
{
	float			iX, iY, eHF, eX, eY, uHF, uX, uY;
	float			uMAX, hold_R, la[5];

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(pm->k_ZNUL);

			pm->lu_F[0] = 1.f;
			pm->lu_F[1] = 0.f;

			pm->hfi_DFT[0] = 0.f;
			pm->hfi_DFT[1] = 0.f;
			pm->hfi_DFT[2] = 0.f;
			pm->hfi_DFT[3] = 0.f;
			pm->hfi_DFT[4] = 0.f;
			pm->hfi_DFT[5] = 0.f;
			pm->hfi_DFT[6] = 0.f;
			pm->hfi_DFT[7] = 0.f;

			pm->hfi_REM[0] = 0.f;
			pm->hfi_REM[1] = 0.f;
			pm->hfi_REM[2] = 0.f;
			pm->hfi_REM[3] = 0.f;
			pm->hfi_REM[4] = 0.f;
			pm->hfi_REM[5] = 0.f;
			pm->hfi_REM[6] = 0.f;
			pm->hfi_REM[7] = 0.f;
			pm->hfi_REM[9] = 0.f;

			pm->hfi_wave[0] = 1.f;
			pm->hfi_wave[1] = 0.f;

			pm->temp_HFI_wS = 2.f * M_PI_F * pm->probe_freq_sine_hz;
			pm->temp_HFI_HT[0] = m_cosf(pm->temp_HFI_wS * pm->dT * .5f);
			pm->temp_HFI_HT[1] = m_sinf(pm->temp_HFI_wS * pm->dT * .5f);

			hold_R = pm->probe_hold_angle * (M_PI_F / 180.f);
			hold_R = (hold_R < - M_PI_F) ? - M_PI_F :
				(hold_R > M_PI_F) ? M_PI_F : hold_R;

			pm->i_track_D = m_cosf(hold_R) * pm->probe_current_weak;
			pm->i_track_Q = m_sinf(hold_R) * pm->probe_current_weak;

			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_current_hold);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			iX = pm->hfi_REM[10];
			iY = pm->hfi_REM[11];

			uX = pm->tvm_DX * pm->temp_HFI_HT[0] + pm->tvm_DY * pm->temp_HFI_HT[1];
			uY = pm->tvm_DY * pm->temp_HFI_HT[0] - pm->tvm_DX * pm->temp_HFI_HT[1];

			m_rsumf(&pm->hfi_DFT[0], &pm->hfi_REM[0], iX * pm->hfi_wave[0]);
			m_rsumf(&pm->hfi_DFT[1], &pm->hfi_REM[1], iX * pm->hfi_wave[1]);
			m_rsumf(&pm->hfi_DFT[2], &pm->hfi_REM[2], uX * pm->hfi_wave[0]);
			m_rsumf(&pm->hfi_DFT[3], &pm->hfi_REM[3], uX * pm->hfi_wave[1]);
			m_rsumf(&pm->hfi_DFT[4], &pm->hfi_REM[4], iY * pm->hfi_wave[0]);
			m_rsumf(&pm->hfi_DFT[5], &pm->hfi_REM[5], iY * pm->hfi_wave[1]);
			m_rsumf(&pm->hfi_DFT[6], &pm->hfi_REM[6], uY * pm->hfi_wave[0]);
			m_rsumf(&pm->hfi_DFT[7], &pm->hfi_REM[7], uY * pm->hfi_wave[1]);

		case 1:
			m_rotatef(pm->hfi_wave, pm->temp_HFI_wS * pm->dT);

			eX = pm->i_track_D - pm->lu_iX;
			eY = pm->i_track_Q - pm->lu_iY;

			eHF = pm->probe_current_sine - m_sqrtf(eX * eX + eY * eY);

			pm->hfi_REM[9] += pm->probe_gain_I * eHF;
			uHF = pm->probe_gain_P * eHF + pm->hfi_REM[9];

			pm->i_integral_D += pm->probe_gain_I * eX;
			uX = pm->probe_gain_P * eX + pm->i_integral_D;

			pm->i_integral_Q += pm->probe_gain_I * eY;
			uY = pm->probe_gain_P * eY + pm->i_integral_Q;

			uMAX = pm->k_UMAX * pm->const_fb_U;

			if (		m_fabsf(pm->hfi_REM[9]) > uMAX
					|| m_fabsf(pm->i_integral_D) > uMAX
					|| m_fabsf(pm->i_integral_Q) > uMAX) {

				pm->fsm_errno = PM_ERROR_CURRENT_LOOP_IS_OPEN;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm->hfi_REM[10] = pm->lu_iX;
			pm->hfi_REM[11] = pm->lu_iY;

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
			pm_hfi_DFT(pm, la);

			if (		m_isfinitef(la[2]) != 0 && la[2] > M_EPS_F
					&& m_isfinitef(la[3]) != 0 && la[3] > M_EPS_F) {

				pm->const_im_L1 = la[2];
				pm->const_im_L2 = la[3];
				pm->const_im_B = m_atan2f(la[1], la[0]) * (180.f / M_PI_F);
				pm->const_im_R = la[4];
			}
			else {
				pm->fsm_errno = PM_ERROR_INVALID_OPERATION;
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
			if (pm->const_im_L2 > M_EPS_F) {

				pm->debug_locked_HFI = PM_DISABLED;
				pm->debug_locked_SENSOR = PM_SENSOR_DISABLED;

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

				pm->lu_F1 = pm->lu_F[1];
				pm->lu_revol = 0;
				pm->lu_revob = 0;

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
				pm->flux_LOCKED = PM_FLUX_UNCERTAIN;

				pm->skew_ONFLAG = 0;
				pm->skew_TIM = 0;
				pm->skew_END = 0;

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
				pm->x_residual = 0.f;

				pm->im_base_revol = 0;
				pm->im_REM[0] = 0.f;
				pm->im_REM[1] = 0.f;
				pm->im_REM[2] = 0.f;
				pm->im_REM[3] = 0.f;

				pm->fsm_errno = PM_OK;

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
				pm->fsm_errno = PM_ERROR_INVALID_OPERATION;
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
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[2];

	switch (pm->fsm_phase) {

		case 0:
			lse_initiate(ls, 1, 1);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, (pm->lu_mode == PM_LU_DETACHED)
					? pm->tm_average_drift : pm->tm_average_probe);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			v[0] = 1.f;
			v[1] = pm->flux_E;

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			lse_finalise(ls);

			if (m_isfinitef(ls->b[0]) != 0 && ls->b[0] > M_EPS_F) {

				pm->const_E = ls->b[0];
				pm->temp_const_iE = 1.f / ls->b[0];
			}
			else {
				pm->fsm_errno = PM_ERROR_INVALID_OPERATION;
			}

			pm->fsm_state = PM_STATE_IDLE;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_probe_const_ja(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[4];

	float			*m = pm->probe_LS[1].vm;
	float			temp_Ja;

	switch (pm->fsm_phase) {

		case 0:
			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			m[0] = 0.f;
			m[1] = 0.f;
			m[2] = 0.f;
			m[3] = 0.f;

			lse_initiate(ls, 3, 1);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_inertia);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 2;
			break;

		case 2:
			m_rsumf(&m[0], &m[2], pm->lu_iQ);
			m_rsumf(&m[1], &m[3], 1.f);

			v[0] = m[0];
			v[1] = m[1];
			v[2] = 1.f;
			v[3] = pm->lu_wS;

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 3;
			}
			break;

		case 3:
			lse_finalise(ls);

			temp_Ja = pm->dT / ls->b[0];

			if (m_isfinitef(temp_Ja) != 0 && temp_Ja > M_EPS_F) {

				pm->const_Ja = temp_Ja;
			}
			else {
				pm->fsm_errno = PM_ERROR_INVALID_OPERATION;
			}

			pm->fsm_state = PM_STATE_IDLE;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_adjust_sensor_hall(pmc_t *pm)
{
	float			*NUM = pm->probe_LS[1].vm;
	float			*REM = pm->probe_LS[2].vm;

	int			HS, N, min_S;
	float			D;

	switch (pm->fsm_phase) {

		case 0:
			if (pm->lu_mode == PM_LU_ESTIMATE_FLUX) {

				for (N = 0; N < 8; ++N) {

					pm->hall_ST[N].X = 0.f;
					pm->hall_ST[N].Y = 0.f;

					NUM[N] = 0.f;
					REM[N] = 0.f;
					REM[N + 8] = 0.f;
				}

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_errno = PM_OK;
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

				m_rsumf(&pm->hall_ST[HS].X, &REM[HS], pm->lu_F[0]);
				m_rsumf(&pm->hall_ST[HS].Y, &REM[HS + 8], pm->lu_F[1]);

				NUM[HS] += 1.f;
			}
			else {
				pm->hall_ALLOWED = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_SENSOR_HALL_FAULT;
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

				if (NUM[HS] > min_S) {

					D = NUM[HS];

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

				pm->hall_ALLOWED = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_SENSOR_HALL_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}
			else {
				pm->hall_ALLOWED = PM_ENABLED;

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

