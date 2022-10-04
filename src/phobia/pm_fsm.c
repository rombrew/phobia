#include "libm.h"
#include "pm.h"

static void
pm_fsm_state_idle(pmc_t *pm)
{
	/* TODO
	 * */

	if (		pm->lu_MODE == PM_LU_DISABLED
			&& pm->config_BOOST_CHARGE == PM_ENABLED) {

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
	lse_float_t		v[4];

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(PM_Z_ABC);

			lse_initiate(ls, LSE_CASCADE_MAX, 1, 3);

			pm_clearance(pm, 0, 0, 0);
			pm_clearance(pm, 0, 0, 0);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_drift);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			v[0] = 1.f;
			v[1] = (pm->vsi_AF == 0) ? pm->fb_iA : 0.f;
			v[2] = (pm->vsi_BF == 0) ? pm->fb_iB : 0.f;
			v[3] = (pm->vsi_CF == 0) ? pm->fb_iC : 0.f;

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			lse_finalise(ls);

			pm->ad_IA[0] += (pm->vsi_AF == 0) ? - ls->b[0] : 0.f;
			pm->ad_IB[0] += (pm->vsi_BF == 0) ? - ls->b[1] : 0.f;
			pm->ad_IC[0] += (pm->vsi_CF == 0) ? - ls->b[2] : 0.f;

			if (		   m_fabsf(pm->ad_IA[0]) > pm->fault_current_tol
					|| m_fabsf(pm->ad_IB[0]) > pm->fault_current_tol
					|| m_fabsf(pm->ad_IC[0]) > pm->fault_current_tol) {

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

			if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				pm->fsm_phase = 1;
			}
			else {
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}
			break;

		case 1:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(PM_Z_BC);

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
				pm->proc_set_Z(PM_Z_AC);

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
				pm->proc_set_Z(PM_Z_AB);

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

				if (		   pm->self_BST[0] < pm->dc_bootstrap
						|| pm->self_BST[1] < pm->dc_bootstrap
						|| pm->self_BST[2] < pm->dc_bootstrap) {

					pm->fsm_errno = PM_ERROR_BOOTSTRAP_FAULT;
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
					pm->proc_set_Z(PM_Z_ABC);
					break;

				case 1:
					pm->proc_set_DC(pm->dc_resolution, 0, 0);
					pm->proc_set_Z(PM_Z_BC);
					break;

				case 2:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(PM_Z_BC);
					break;

				case 3:
					pm->proc_set_DC(0, pm->dc_resolution, 0);
					pm->proc_set_Z(PM_Z_AC);
					break;

				case 4:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(PM_Z_AC);
					break;

				case 5:
					pm->proc_set_DC(0, 0, pm->dc_resolution);
					pm->proc_set_Z(PM_Z_AB);
					break;

				case 6:
					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(PM_Z_AB);
					break;
			}

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

			pm->fsm_phase = 2;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				lse_initiate(ls, LSE_CASCADE_MAX, 1, 3);

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
				? 1U : (m_fabsf(uA) < pm->fault_voltage_tol) ? 0U : 0x10U;
			bm |= (m_fabsf(uB - pm->const_fb_U) < pm->fault_voltage_tol)
				? 2U : (m_fabsf(uB) < pm->fault_voltage_tol) ? 0U : 0x20U;
			bm |= (m_fabsf(uC - pm->const_fb_U) < pm->fault_voltage_tol)
				? 4U : (m_fabsf(uC) < pm->fault_voltage_tol) ? 0U : 0x40U;

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
			if (		   pm->self_BM[1] == 7U
					&& pm->self_BM[2] == 0U
					&& pm->self_BM[3] == 7U
					&& pm->self_BM[4] == 0U
					&& pm->self_BM[5] == 7U
					&& pm->self_BM[6] == 0U) {

				pm->fsm_errno = PM_OK;
			}
			else if (	   (pm->self_BM[1] & 17U) == 1U
					&& (pm->self_BM[2] & 17U) == 0U
					&& (pm->self_BM[3] & 34U) == 2U
					&& (pm->self_BM[4] & 34U) == 0U
					&& (pm->self_BM[5] & 68U) == 4U
					&& (pm->self_BM[6] & 68U) == 0U) {

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
	lse_float_t		v[8];

	switch (pm->fsm_phase) {

		case 0:
			lse_initiate(ls, LSE_CASCADE_MAX, 1, 7);

			pm_clearance(pm, 0, 0, 0);
			pm_clearance(pm, 0, 0, 0);

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
			v[1] = (pm->vsi_AF == 0) ? pm->fb_iA : 0.f;
			v[2] = (pm->vsi_BF == 0) ? pm->fb_iB : 0.f;
			v[3] = (pm->vsi_CF == 0) ? pm->fb_iC : 0.f;
			v[4] = pm->const_fb_U;

			if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				v[5] = pm->fb_uA;
				v[6] = pm->fb_uB;
				v[7] = pm->fb_uC;
			}
			else {
				v[5] = 0.f;
				v[6] = 0.f;
				v[7] = 0.f;
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
			pm->self_RMSi[2] = m_sqrtf(ls->b[2] * ls->b[2] + ls->e[2] * ls->e[2]);
			pm->self_RMSu[0] = ls->e[3];
			pm->self_RMSu[1] = ls->e[4];
			pm->self_RMSu[2] = ls->e[5];
			pm->self_RMSu[3] = ls->e[6];

			if (		   m_isfinitef(pm->self_RMSi[0]) != 0
					&& m_isfinitef(pm->self_RMSi[1]) != 0
					&& m_isfinitef(pm->self_RMSi[2]) != 0
					&& m_isfinitef(pm->self_RMSu[0]) != 0
					&& m_isfinitef(pm->self_RMSu[1]) != 0
					&& m_isfinitef(pm->self_RMSu[2]) != 0
					&& m_isfinitef(pm->self_RMSu[3]) != 0) {

				if (		   pm->self_RMSi[0] > pm->fault_current_tol
						|| pm->self_RMSi[1] > pm->fault_current_tol
						|| pm->self_RMSi[2] > pm->fault_current_tol) {

					pm->fsm_errno = PM_ERROR_INSUFFICIENT_ACCURACY;
				}
				else if (	   pm->self_RMSu[0] > pm->fault_voltage_tol
						|| pm->self_RMSu[1] > pm->fault_voltage_tol
						|| pm->self_RMSu[2] > pm->fault_voltage_tol
						|| pm->self_RMSu[3] > pm->fault_voltage_tol) {

					pm->fsm_errno = PM_ERROR_INSUFFICIENT_ACCURACY;
				}
			}
			else {
				pm->fsm_errno = PM_ERROR_UNCERTAIN_RESULT;
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
			pm->proc_set_Z(PM_Z_NUL);

			lse_initiate(ls, LSE_CASCADE_MAX, 1, 4);

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

			if (		   m_fabsf(pm->ad_UA[0]) > pm->fault_voltage_tol
					|| m_fabsf(pm->ad_UB[0]) > pm->fault_voltage_tol
					|| m_fabsf(pm->ad_UC[0]) > pm->fault_voltage_tol) {

				pm->tvm_INUSE = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_INSUFFICIENT_ACCURACY;
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

			if (		   m_fabsf(pm->ad_UA[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->ad_UB[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->ad_UC[1] - 1.f) > pm->fault_accuracy_tol) {

				pm->tvm_INUSE = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_INSUFFICIENT_ACCURACY;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			lse_initiate(&pm->probe_LS[0], LSE_CASCADE_MAX, 3, 1);
			lse_initiate(&pm->probe_LS[1], LSE_CASCADE_MAX, 3, 1);
			lse_initiate(&pm->probe_LS[2], LSE_CASCADE_MAX, 3, 1);

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

			pm->self_RMSu[1] = pm->probe_LS[0].e[0];
			pm->self_RMSu[2] = pm->probe_LS[1].e[0];
			pm->self_RMSu[3] = pm->probe_LS[2].e[0];

			if (		   m_isfinitef(pm->tvm_FIR_A[0]) != 0
					&& m_isfinitef(pm->tvm_FIR_A[1]) != 0
					&& m_isfinitef(pm->tvm_FIR_A[2]) != 0
					&& m_isfinitef(pm->tvm_FIR_B[0]) != 0
					&& m_isfinitef(pm->tvm_FIR_B[1]) != 0
					&& m_isfinitef(pm->tvm_FIR_B[2]) != 0
					&& m_isfinitef(pm->tvm_FIR_C[0]) != 0
					&& m_isfinitef(pm->tvm_FIR_C[1]) != 0
					&& m_isfinitef(pm->tvm_FIR_C[2]) != 0) {

				if (		   pm->self_RMSu[1] < pm->fault_terminal_tol
						&& pm->self_RMSu[2] < pm->fault_terminal_tol
						&& pm->self_RMSu[3] < pm->fault_terminal_tol) {

					pm->tvm_INUSE = PM_ENABLED;
				}
				else {
					pm->tvm_INUSE = PM_DISABLED;
				}
			}
			else {
				pm->tvm_INUSE = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_UNCERTAIN_RESULT;
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
	lse_float_t		v[4];

	float			iZ, eZ, uZ, REF;
	float			uMAX;

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(PM_Z_ABC);

			pm->fsm_subi = 0;

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			pm_clearance(pm, 0, 0, 0);
			pm_clearance(pm, 0, 0, 0);

			while (pm->fsm_subi < 3) {

				if (pm->fsm_subi == 0) {

					if (		   pm->vsi_AF == 0
							&& pm->vsi_BF == 0) {

						pm->proc_set_Z(PM_Z_C);
						break;
					}
				}
				else if (pm->fsm_subi == 1) {

					if (		   pm->vsi_AF == 0
							&& pm->vsi_CF == 0) {

						pm->proc_set_Z(PM_Z_B);
						break;
					}
				}
				else if (pm->fsm_subi == 2) {

					if (		   pm->vsi_BF == 0
							&& pm->vsi_CF == 0) {

						pm->proc_set_Z(PM_Z_A);
						break;
					}
				}

				pm->fsm_subi++;
			}

			if (pm->fsm_subi < 3) {

				lse_initiate(ls, LSE_CASCADE_MAX, 1, 3);

				pm->i_integral_D = 0.f;

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_current_ramp);

				pm->fsm_phase = 2;
			}
			else {
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}
			break;

		case 4:
			v[0] = 1.f;
			v[1] = (pm->vsi_AF == 0) ? pm->fb_iA : 0.f;
			v[2] = (pm->vsi_BF == 0) ? pm->fb_iB : 0.f;
			v[3] = (pm->vsi_CF == 0) ? pm->fb_iC : 0.f;

			lse_insert(ls, v);

		case 2:
		case 3:
			if (pm->fsm_phase == 2) {

				pm->i_track_D = (float) (pm->tm_value + 1)
					* pm->probe_current_hold / (float) pm->tm_end;
			}

			iZ =	  (pm->fsm_subi == 0) ? pm->fb_iA
				: (pm->fsm_subi == 1) ? pm->fb_iA
				: (pm->fsm_subi == 2) ? pm->fb_iB : 0.f;

			eZ = pm->probe_current_hold - iZ;

			pm->i_integral_D += pm->probe_gain_I * eZ;
			uZ = pm->probe_gain_P * eZ + pm->i_integral_D;

			uMAX = pm->k_UMAX * pm->const_fb_U;

			if (m_fabsf(pm->i_integral_D) > uMAX) {

				pm->fsm_errno = PM_ERROR_CURRENT_LOOP_IS_OPEN;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			if      (pm->fsm_subi == 0) { pm_voltage(pm, uZ, 0.f); }
			else if (pm->fsm_subi == 1) { pm_voltage(pm, uZ, 0.f); }
			else if (pm->fsm_subi == 2) { pm_voltage(pm, 0.f, uZ); }

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				if (pm->fsm_phase == 2) {

					pm->tm_value = 0;
					pm->tm_end = PM_TSMS(pm, pm->tm_current_hold);
				}
				else {
					pm->tm_value = 0;
					pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);
				}

				pm->fsm_phase += 1;
			}
			break;

		case 5:
			lse_finalise(ls);

			if (pm->fsm_subi == 0) {

				REF = (ls->b[0] - ls->b[1]) / 2.f;

				pm->ad_IA[1] *= REF / ls->b[0];
				pm->ad_IB[1] *= - REF / ls->b[1];
			}
			else if (pm->fsm_subi == 1) {

				REF = (ls->b[0] - ls->b[2]) / 2.f;

				pm->ad_IA[1] *= REF / ls->b[0];
				pm->ad_IC[1] *= - REF / ls->b[2];
			}
			else if (pm->fsm_subi == 2) {

				REF = (ls->b[1] - ls->b[2]) / 2.f;

				pm->ad_IB[1] *= REF / ls->b[1];
				pm->ad_IC[1] *= - REF / ls->b[2];
			}

			if (		   m_fabsf(pm->ad_IA[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->ad_IB[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->ad_IC[1] - 1.f) > pm->fault_accuracy_tol) {

				pm->fsm_errno = PM_ERROR_INSUFFICIENT_ACCURACY;
			}

			pm->fsm_subi++;
			pm->fsm_phase = 1;
			break;
	}
}

static void
pm_fsm_probe_loop_current(pmc_t *pm, float track_D, float track_Q, float track_HF)
{
	float		eD, eQ, eHF, uD, uQ, uHF, uMAX;

	eD = track_D - pm->lu_iX;
	eQ = track_Q - pm->lu_iY;

	if (track_HF > M_EPS_F) {

		eHF = track_HF - m_sqrtf(eD * eD + eQ * eQ);

		pm->hfi_REM[8] += (eHF - pm->hfi_REM[8]) * pm->hfi_DFT[8];
		pm->hfi_REM[9] += pm->probe_gain_I * pm->hfi_REM[8];
		uHF = pm->probe_gain_P * pm->hfi_REM[8] + pm->hfi_REM[9];
	}

	pm->i_integral_D += pm->probe_gain_I * eD;
	uD = pm->probe_gain_P * eD + pm->i_integral_D;

	pm->i_integral_Q += pm->probe_gain_I * eQ;
	uQ = pm->probe_gain_P * eQ + pm->i_integral_Q;

	uMAX = pm->k_UMAX * pm->const_fb_U;

	if (		m_fabsf(pm->hfi_REM[9]) > uMAX
			|| m_fabsf(pm->i_integral_D) > uMAX
			|| m_fabsf(pm->i_integral_Q) > uMAX) {

		pm->fsm_errno = PM_ERROR_CURRENT_LOOP_IS_OPEN;
		pm->fsm_state = PM_STATE_HALT;
		pm->fsm_phase = 0;

		return ;
	}

	if (track_HF > M_EPS_F) {

		m_rotatef(pm->hfi_wave, pm->quick_hfwS * pm->dT);

		uD += uHF * pm->hfi_wave[0];
		uQ += uHF * pm->hfi_wave[1];
	}

	pm_voltage(pm, uD, uQ);
}

static void
pm_fsm_state_probe_const_r(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[3];

	float			hold_R, ramp_A;

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(PM_Z_NUL);

			lse_initiate(ls, LSE_CASCADE_MAX, 2, 1);

			pm->lu_F[0] = 1.f;
			pm->lu_F[1] = 0.f;

			hold_R = pm->probe_hold_angle * (M_PI_F / 180.f);
			hold_R = (hold_R < - M_PI_F) ? - M_PI_F :
				(hold_R > M_PI_F) ? M_PI_F : hold_R;

			pm->hfi_REM[0] = m_cosf(hold_R);
			pm->hfi_REM[1] = m_sinf(hold_R);

			pm->hfi_REM[9] = 0.f;

			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_current_ramp);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			ramp_A = (float) (pm->tm_value + 1) * pm->probe_current_hold
				/ (float) pm->tm_end;

			pm->i_track_D = pm->hfi_REM[0] * ramp_A;
			pm->i_track_Q = pm->hfi_REM[1] * ramp_A;

		case 2:
			pm_fsm_probe_loop_current(pm, pm->i_track_D, pm->i_track_Q, 0.f);

			if (pm->fsm_errno != PM_OK)
				break;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				if (pm->fsm_phase == 1) {

					pm->tm_value = 0;
					pm->tm_end = PM_TSMS(pm, pm->tm_current_hold);
				}
				else {
					pm->tm_value = 0;
					pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);
				}

				pm->fsm_phase += 1;
			}
			break;

		case 3:
		case 4:
			v[0] = pm->hfi_REM[0] * pm->lu_iX
				+ pm->hfi_REM[1] * pm->lu_iY;

			v[1] = 1.f;
			v[2] = pm->hfi_REM[0] * pm->tvm_DX
				+ pm->hfi_REM[1] * pm->tvm_DY;

			lse_insert(ls, v);

			pm_fsm_probe_loop_current(pm, pm->i_track_D, pm->i_track_Q, 0.f);

			if (pm->fsm_errno != PM_OK)
				break;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->i_track_D = pm->hfi_REM[0] * pm->probe_current_weak;
				pm->i_track_Q = pm->hfi_REM[1] * pm->probe_current_weak;

				pm->fsm_phase += 1;
			}
			break;

		case 5:
			lse_finalise(ls);

			if (		m_isfinitef(ls->b[0]) != 0
					&& ls->b[0] > M_EPS_F) {

				pm->const_im_R = ls->b[0];
			}
			else {
				pm->fsm_errno = PM_ERROR_UNCERTAIN_RESULT;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_probe_const_l(pmc_t *pm)
{
	float			iX, iY, uX, uY;
	float			hold_R, tau_HF, la[5];

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(PM_Z_NUL);

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

			pm->hfi_REM[8] = 0.f;
			pm->hfi_REM[9] = 0.f;

			pm->quick_hfwS = 2.f * M_PI_F * pm->probe_freq_sine_hz;
			pm->quick_hfSC[0] = m_cosf(pm->quick_hfwS * pm->dT * .5f);
			pm->quick_hfSC[1] = m_sinf(pm->quick_hfwS * pm->dT * .5f);

			tau_HF = 4.f / pm->quick_hfwS;
			pm->hfi_DFT[8] = 1.f - m_expf(- pm->dT / tau_HF);

			pm->hfi_wave[0] = 1.f;
			pm->hfi_wave[1] = 0.f;

			hold_R = pm->probe_hold_angle * (M_PI_F / 180.f);
			hold_R = (hold_R < - M_PI_F) ? - M_PI_F :
				(hold_R > M_PI_F) ? M_PI_F : hold_R;

			pm->i_track_D = m_cosf(hold_R) * pm->probe_current_bias;
			pm->i_track_Q = m_sinf(hold_R) * pm->probe_current_bias;

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

			uX = pm->tvm_DX * pm->quick_hfSC[0] + pm->tvm_DY * pm->quick_hfSC[1];
			uY = pm->tvm_DY * pm->quick_hfSC[0] - pm->tvm_DX * pm->quick_hfSC[1];

			m_rsumf(&pm->hfi_DFT[0], &pm->hfi_REM[0], iX * pm->hfi_wave[0]);
			m_rsumf(&pm->hfi_DFT[1], &pm->hfi_REM[1], iX * pm->hfi_wave[1]);
			m_rsumf(&pm->hfi_DFT[2], &pm->hfi_REM[2], uX * pm->hfi_wave[0]);
			m_rsumf(&pm->hfi_DFT[3], &pm->hfi_REM[3], uX * pm->hfi_wave[1]);
			m_rsumf(&pm->hfi_DFT[4], &pm->hfi_REM[4], iY * pm->hfi_wave[0]);
			m_rsumf(&pm->hfi_DFT[5], &pm->hfi_REM[5], iY * pm->hfi_wave[1]);
			m_rsumf(&pm->hfi_DFT[6], &pm->hfi_REM[6], uY * pm->hfi_wave[0]);
			m_rsumf(&pm->hfi_DFT[7], &pm->hfi_REM[7], uY * pm->hfi_wave[1]);

		case 1:
			pm->hfi_REM[10] = pm->lu_iX;
			pm->hfi_REM[11] = pm->lu_iY;

			pm_fsm_probe_loop_current(pm, pm->i_track_D, pm->i_track_Q,
					pm->probe_current_sine);

			if (pm->fsm_errno != PM_OK)
				break;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm_hfi_DFT(pm, la);

			if (		   m_isfinitef(la[2]) != 0 && la[2] > M_EPS_F
					&& m_isfinitef(la[3]) != 0 && la[3] > M_EPS_F) {

				pm->const_im_L1 = la[2];
				pm->const_im_L2 = la[3];
				pm->const_im_B = m_atan2f(la[1], la[0]) * (180.f / M_PI_F);
				pm->const_im_R = la[4];
			}
			else {
				pm->fsm_errno = PM_ERROR_UNCERTAIN_RESULT;
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
			if (		   m_isfinitef(pm->const_im_L1) != 0
					&& m_isfinitef(pm->const_im_L2) != 0
					&& pm->const_im_L1 > M_EPS_F
					&& pm->const_im_L2 > M_EPS_F) {

				pm->fsm_errno = PM_OK;

				pm->vsi_SA = 0;
				pm->vsi_SB = 0;
				pm->vsi_SC = 0;
				pm->vsi_TIM = 0;
				pm->vsi_AG = 0;
				pm->vsi_BG = 0;
				pm->vsi_CG = 0;
				pm->vsi_AF = 0;
				pm->vsi_BF = 0;
				pm->vsi_CF = 0;
				pm->vsi_XXF = 0;
				pm->vsi_SF = 0;
				pm->vsi_UF = 0;
				pm->vsi_AZ = 0;
				pm->vsi_BZ = 0;
				pm->vsi_CZ = 0;
				pm->vsi_lpf_DC = 0.f;

				pm->lu_MODE = PM_LU_DETACHED;
				pm->lu_iX = 0.f;
				pm->lu_iY = 0.f;
				pm->lu_iD = 0.f;
				pm->lu_iQ = 0.f;
				pm->lu_F[0] = 1.f;
				pm->lu_F[1] = 0.f;
				pm->lu_F[2] = 0.f;
				pm->lu_wS = 0.f;
				pm->lu_location = 0.f;
				pm->lu_revol = 0;
				pm->lu_revob = 0;
				pm->lu_load_torque = 0.f;
				pm->lu_base_wS = 0.f;

				pm->forced_F[0] = 1.f;
				pm->forced_F[1] = 0.f;
				pm->forced_wS = 0.f;
				pm->forced_track_D = 0.f;

				pm->detach_LOCK = 0;
				pm->detach_TIM = 0;

				pm->flux_ESTIMATE = PM_ESTIMATE_NONE;
				pm->flux_ZONE = PM_ZONE_NONE;

				pm->flux_X[0] = 0.f;
				pm->flux_X[1] = 0.f;
				pm->flux_E = 0.f;
				pm->flux_F[0] = 1.f;
				pm->flux_F[1] = 0.f;
				pm->flux_wS = 0.f;

				pm->kalman_P[0] = 1.f;
				pm->kalman_P[1] = 0.f;
				pm->kalman_P[2] = 1.f;
				pm->kalman_P[3] = 0.f;
				pm->kalman_P[4] = 0.f;
				pm->kalman_P[5] = 1.f;
				pm->kalman_P[6] = 0.f;
				pm->kalman_P[7] = 0.f;
				pm->kalman_P[8] = 0.f;
				pm->kalman_P[9] = 1.f;
				pm->kalman_P[10] = 0.f;
				pm->kalman_P[11] = 0.f;
				pm->kalman_P[12] = 0.f;
				pm->kalman_P[13] = 0.f;
				pm->kalman_P[14] = 0.f;
				pm->kalman_Z = 0.f;

				pm->zone_lpf_wS = 0.f;

				pm->hfi_F[0] = 1.f;
				pm->hfi_F[1] = 0.f;
				pm->hfi_wS = 0.f;
				pm->hfi_wave[0] = 1.f;
				pm->hfi_wave[1] = 0.f;
				pm->hfi_IN = 0;

				pm->hall_F[0] = 1.f;
				pm->hall_F[1] = 0.f;
				pm->hall_wS = 0.f;

				pm->abi_ONCE = 0;
				pm->abi_revol = 0;
				pm->abi_unwrap = 0;
				pm->abi_F[0] = 1.f;
				pm->abi_F[1] = 0.f;
				pm->abi_wS = 0.f;
				pm->abi_location = 0.f;

				pm->sincos_ONCE = 0;
				pm->sincos_revol = 0;
				pm->sincos_unwrap = 0;
				pm->sincos_F[0] = 1.f;
				pm->sincos_F[1] = 0.f;
				pm->sincos_wS = 0.f;
				pm->sincos_location = 0.f;

				pm->watt_lpf_D = 0.f;
				pm->watt_lpf_Q = 0.f;
				pm->watt_lpf_wP = 0.f;

				pm->i_setpoint_current = 0.f;
				pm->i_derated_PCB = PM_MAX_F;
				pm->i_derated_WEAK = PM_MAX_F;
				pm->i_track_D = 0.f;
				pm->i_track_Q = 0.f;
				pm->i_integral_D = 0.f;
				pm->i_integral_Q = 0.f;

				pm->weak_D = 0.f;

				pm->s_setpoint_speed = 0.f;
				pm->s_track = 0.f;

				pm->x_setpoint_location = 0.f;
				pm->x_setpoint_speed = 0.f;
				pm->x_home_location = 0.f;

				pm->im_REM[0] = 0.f;
				pm->im_REM[1] = 0.f;
				pm->im_REM[2] = 0.f;
				pm->im_REM[3] = 0.f;

				if (pm->fsm_state == PM_STATE_LU_DETACHED) {

					pm->flux_ZONE = PM_ZONE_LOCKED_IN_DETACH;
				}

				if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(PM_Z_ABC);
				}
				else {
					pm->lu_MODE = PM_LU_ESTIMATE_FLUX;

					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(PM_Z_NUL);
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

				pm->lu_MODE = PM_LU_DISABLED;

				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(PM_Z_ABC);

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
			lse_initiate(ls, LSE_CASCADE_MAX, 1, 1);

			if (pm->lu_MODE == PM_LU_DETACHED) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_drift);
			}
			else {
				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);
			}

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			if (pm->flux_ESTIMATE == PM_ESTIMATE_KALMAN) {

				v[0] = 1.f;
				v[1] = pm->const_E;

				if (m_fabsf(pm->flux_wS) > M_EPS_F) {

					v[1] += - pm->kalman_Z / pm->flux_wS;
				}
			}
			else {
				v[0] = 1.f;
				v[1] = pm->flux_E;
			}

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			lse_finalise(ls);

			if (		m_isfinitef(ls->b[0]) != 0
					&& ls->b[0] > M_EPS_F) {

				pm->const_E = ls->b[0];
				pm->kalman_Z = 0.f;

				pm_build(pm);
			}
			else {
				pm->fsm_errno = PM_ERROR_UNCERTAIN_RESULT;
			}

			pm->fsm_state = PM_STATE_IDLE;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_probe_const_j(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[4];

	float			*m = pm->probe_LS[1].vm;
	float			mQ, temp_Ja;

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

			lse_initiate(ls, LSE_CASCADE_MAX, 3, 1);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_inertia);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 2;
			break;

		case 2:
			mQ = pm_torque_equation(pm, pm->lu_iD, pm->lu_iQ);

			m_rsumf(&m[0], &m[2], mQ);
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

			if (		m_isfinitef(temp_Ja) != 0
					&& temp_Ja > M_EPS_F) {

				pm->const_Ja = temp_Ja;
			}
			else {
				pm->fsm_errno = PM_ERROR_UNCERTAIN_RESULT;
			}

			pm->fsm_state = PM_STATE_IDLE;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_probe_noise_threshold(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_LS[0];
	lse_float_t		v[3];

	switch (pm->fsm_phase) {

		case 0:
			lse_initiate(ls, LSE_CASCADE_MAX, 2, 1);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_drift);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			v[0] = 1.f;
			v[1] = (float) pm->tm_value;
			v[2] = pm->flux_wS;

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = 2;
			}
			break;

		case 2:
			lse_finalise(ls);

			if (		m_isfinitef(ls->e[0]) != 0
					&& ls->e[0] > M_EPS_F) {

				pm->zone_threshold_NOISE = ls->e[0] * 5.f;
			}
			else {
				pm->fsm_errno = PM_ERROR_UNCERTAIN_RESULT;
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
			if (pm->lu_MODE == PM_LU_ESTIMATE_FLUX) {

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
				pm->hall_INUSE = PM_DISABLED;

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

				pm->hall_INUSE = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_SENSOR_HALL_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}
			else {
				pm->hall_INUSE = PM_ENABLED;

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
			pm->proc_set_Z(PM_Z_ABC);

			pm->vsi_AF = 0;
			pm->vsi_BF = 0;
			pm->vsi_CF = 0;
			pm->vsi_SF = 0;
			pm->vsi_UF = 0;
			pm->vsi_AZ = 0;
			pm->vsi_BZ = 0;
			pm->vsi_CZ = 0;

			if (pm->lu_MODE != PM_LU_DISABLED) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_halt_pause);

				pm->lu_MODE = PM_LU_DISABLED;
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

			if (pm->lu_MODE != PM_LU_DISABLED)
				break;

			pm_build(pm);

			pm->fsm_state = pm->fsm_req;
			pm->fsm_phase = 0;
			break;

		case PM_STATE_LU_SHUTDOWN:
		case PM_STATE_PROBE_CONST_E:
		case PM_STATE_PROBE_CONST_J:
		case PM_STATE_PROBE_NOISE_THRESHOLD:
		case PM_STATE_ADJUST_SENSOR_HALL:
		case PM_STATE_ADJUST_SENSOR_ABI:
		case PM_STATE_ADJUST_SENSOR_SINCOS:

			if (pm->fsm_state != PM_STATE_IDLE)
				break;

			if (pm->lu_MODE == PM_LU_DISABLED)
				break;

			pm->fsm_state = pm->fsm_req;
			pm->fsm_phase = 0;
			break;

		case PM_STATE_HALT:

			if (pm->fsm_state == PM_STATE_HALT)
				break;

			if (		pm->fsm_state == PM_STATE_IDLE
					&& pm->lu_MODE == PM_LU_DISABLED)
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
			pm_fsm_state_probe_const_j(pm);
			break;

		case PM_STATE_PROBE_NOISE_THRESHOLD:
			pm_fsm_state_probe_noise_threshold(pm);
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

#define PM_SFI_CASE(fsm_errno)	case fsm_errno: sym = PM_SFI(fsm_errno); break

const char *pm_strerror(int fsm_errno)
{
	const char		*sym;

	switch (fsm_errno) {

		PM_SFI_CASE(PM_OK);

		PM_SFI_CASE(PM_ERROR_ZERO_DRIFT_FAULT);
		PM_SFI_CASE(PM_ERROR_NO_MOTOR_CONNECTED);
		PM_SFI_CASE(PM_ERROR_BOOTSTRAP_FAULT);
		PM_SFI_CASE(PM_ERROR_POWER_STAGE_DAMAGED);
		PM_SFI_CASE(PM_ERROR_INSUFFICIENT_ACCURACY);
		PM_SFI_CASE(PM_ERROR_CURRENT_LOOP_IS_OPEN);
		PM_SFI_CASE(PM_ERROR_INSTANT_OVERCURRENT);
		PM_SFI_CASE(PM_ERROR_DC_LINK_OVERVOLTAGE);
		PM_SFI_CASE(PM_ERROR_UNCERTAIN_RESULT);
		PM_SFI_CASE(PM_ERROR_INVALID_OPERATION);
		PM_SFI_CASE(PM_ERROR_SENSOR_HALL_FAULT);

		PM_SFI_CASE(PM_ERROR_TIMEOUT);
		PM_SFI_CASE(PM_ERROR_NO_FLUX_CAUGHT);
		PM_SFI_CASE(PM_ERROR_LOSS_OF_SYNC);

		PM_SFI_CASE(PM_ERROR_HW_OVERCURRENT);
		PM_SFI_CASE(PM_ERROR_HW_OVERTEMPERATURE);
		PM_SFI_CASE(PM_ERROR_HW_EMERGENCY_STOP);

		default: sym = "";
	}

	return sym;
}

