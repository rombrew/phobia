#include "libm.h"
#include "pm.h"

static void
pm_fsm_state_idle(pmc_t *pm)
{
	/* TODO */
}

static void
pm_fsm_state_zero_drift(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_lse[0];
	lse_float_t		v[4];

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(PM_Z_ABC);

			lse_construct(ls, LSE_CASCADE_MAX, 1, 3);

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

				pm->fsm_phase += 1;
			}
			break;

		case 2:
			lse_solve(ls);

			pm->scale_iA[0] = (pm->vsi_AF == 0) ? pm->scale_iA[0] - ls->sol.m[0] : 0.f;
			pm->scale_iB[0] = (pm->vsi_BF == 0) ? pm->scale_iB[0] - ls->sol.m[1] : 0.f;
			pm->scale_iC[0] = (pm->vsi_CF == 0) ? pm->scale_iC[0] - ls->sol.m[2] : 0.f;

			if (		   m_fabsf(pm->scale_iA[0]) > pm->fault_current_tol
					|| m_fabsf(pm->scale_iB[0]) > pm->fault_current_tol
					|| m_fabsf(pm->scale_iC[0]) > pm->fault_current_tol) {

				pm->fsm_errno = PM_ERROR_ZERO_DRIFT_FAULT;
			}

			pm->fsm_phase += 1;
			break;

		case 3:
			lse_std(ls);

			pm->self_STDi[0] = (pm->vsi_AF == 0) ? ls->std.m[0] : 0.f;
			pm->self_STDi[1] = (pm->vsi_BF == 0) ? ls->std.m[1] : 0.f;
			pm->self_STDi[2] = (pm->vsi_CF == 0) ? ls->std.m[2] : 0.f;

			if (		   m_isfinitef(pm->self_STDi[0]) != 0
					&& m_isfinitef(pm->self_STDi[1]) != 0
					&& m_isfinitef(pm->self_STDi[2]) != 0) {

				if (		   pm->self_STDi[0] > pm->fault_current_tol
						|| pm->self_STDi[1] > pm->fault_current_tol
						|| pm->self_STDi[2] > pm->fault_current_tol) {

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

			pm->vsi_AT = PM_TSMS(pm, pm->tm_transient_fast);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

			pm->fsm_phase += 1;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->proc_set_DC(pm->dc_resolution, 0, 0);

				pm->vsi_BT = 0;

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			if (m_fabsf(pm->fb_uA - pm->const_fb_U) < pm->fault_voltage_tol) {

				pm->vsi_BT = 0;

				pm->self_BST[0] += 1.f;
			}
			else {
				pm->vsi_BT++;

				if (pm->vsi_BT >= pm->vsi_AT) {

					pm->tm_value = pm->tm_end;
				}
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->self_BST[0] *= 1000.f / pm->m_freq;

				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(PM_Z_AC);

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

				pm->fsm_phase += 1;
			}
			break;

		case 4:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->proc_set_DC(0, pm->dc_resolution, 0);

				pm->vsi_BT = 0;

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_phase += 1;
			}
			break;

		case 5:
			if (m_fabsf(pm->fb_uB - pm->const_fb_U) < pm->fault_voltage_tol) {

				pm->vsi_BT = 0;

				pm->self_BST[1] += 1.f;
			}
			else {
				pm->vsi_BT++;

				if (pm->vsi_BT >= pm->vsi_AT) {

					pm->tm_value = pm->tm_end;
				}
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->self_BST[1] *= 1000.f / pm->m_freq;

				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(PM_Z_AB);

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

				pm->fsm_phase += 1;
			}
			break;

		case 6:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->proc_set_DC(0, 0, pm->dc_resolution);

				pm->vsi_BT = 0;

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

				pm->fsm_phase += 1;
			}
			break;

		case 7:
			if (m_fabsf(pm->fb_uC - pm->const_fb_U) < pm->fault_voltage_tol) {

				pm->vsi_BT = 0;

				pm->self_BST[2] += 1.f;
			}
			else {
				pm->vsi_BT++;

				if (pm->vsi_BT >= pm->vsi_AT) {

					pm->tm_value = pm->tm_end;
				}
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->self_BST[2] *= 1000.f / pm->m_freq;

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
	lse_t			*ls = &pm->probe_lse[0];
	lse_float_t		v[4];

	float			uA, uB, uC, tol;
	int			xIST;

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

			pm->fsm_phase += 1;
			break;

		case 2:
			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				lse_construct(ls, LSE_CASCADE_MAX, 1, 3);

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_instant_probe);

				pm->fsm_phase += 1;
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

				pm->fsm_phase += 1;
			}
			break;

		case 4:
			lse_solve(ls);

			tol = pm->fault_voltage_tol;

			uA = ls->sol.m[0];
			uB = ls->sol.m[1];
			uC = ls->sol.m[2];

			xIST  = (m_fabsf(uA - pm->const_fb_U) < tol) ? 1U
				: (m_fabsf(uA) < tol) ? 0U : 0x10U;
			xIST |= (m_fabsf(uB - pm->const_fb_U) < tol) ? 2U
				: (m_fabsf(uB) < tol) ? 0U : 0x20U;
			xIST |= (m_fabsf(uC - pm->const_fb_U) < tol) ? 4U
				: (m_fabsf(uC) < tol) ? 0U : 0x40U;

			pm->self_IST[pm->fsm_subi] = xIST;

			if (pm->fsm_subi < 6) {

				pm->fsm_phase = 1;
				pm->fsm_subi++;
			}
			else {
				pm->fsm_phase += 1;
			}
			break;

		case 5:
			if (		   pm->self_IST[1] == 7U
					&& pm->self_IST[2] == 0U
					&& pm->self_IST[3] == 7U
					&& pm->self_IST[4] == 0U
					&& pm->self_IST[5] == 7U
					&& pm->self_IST[6] == 0U) {

				pm->fsm_errno = PM_OK;
			}
			else if (	   (pm->self_IST[1] & 0x11U) == 1U
					&& (pm->self_IST[2] & 0x11U) == 0U
					&& (pm->self_IST[3] & 0x22U) == 2U
					&& (pm->self_IST[4] & 0x22U) == 0U
					&& (pm->self_IST[5] & 0x44U) == 4U
					&& (pm->self_IST[6] & 0x44U) == 0U) {

				pm->fsm_errno = PM_ERROR_NO_MOTOR_CONNECTED;
			}
			else {
				pm->fsm_errno = PM_ERROR_POWER_STAGE_BROKEN;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_state_self_test_clearance(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_lse[0];
	lse_float_t		v[8];

	switch (pm->fsm_phase) {

		case 0:
			lse_construct(ls, LSE_CASCADE_MAX, 1, 7);

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

				pm->fsm_phase += 1;
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

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			lse_solve(ls);

			pm->fsm_phase += 1;
			break;

		case 4:
			lse_std(ls);

			pm->self_RMSi[0] = m_sqrtf(ls->sol.m[0] * ls->sol.m[0] + ls->std.m[0] * ls->std.m[0]);
			pm->self_RMSi[1] = m_sqrtf(ls->sol.m[1] * ls->sol.m[1] + ls->std.m[1] * ls->std.m[1]);
			pm->self_RMSi[2] = m_sqrtf(ls->sol.m[2] * ls->sol.m[2] + ls->std.m[2] * ls->std.m[2]);
			pm->self_RMSu[0] = ls->std.m[3];
			pm->self_RMSu[1] = ls->std.m[4];
			pm->self_RMSu[2] = ls->std.m[5];
			pm->self_RMSu[3] = ls->std.m[6];

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
	float			*rem = pm->probe_REM;
	lse_t			*ls = &pm->probe_lse[0];
	lse_float_t		v[5];

	int			xDC, xMIN, xMAX;
	float			REF;

	switch (pm->fsm_phase) {

		case 0:
			pm->fsm_errno = PM_OK;

			if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				lse_construct(ls, LSE_CASCADE_MAX, 2, 3);

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
			pm->proc_set_Z(PM_Z_NONE);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

			pm->fsm_phase += 1;
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

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			v[0] = 1.f;
			v[1] = (pm->fsm_subi == 0) ? 0.f : pm->const_fb_U;
			v[2] = pm->fb_uA;
			v[3] = pm->fb_uB;
			v[4] = pm->fb_uC;

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase = (pm->fsm_subi == 0) ? 1 : 4;
				pm->fsm_subi = 1;
			}
			break;

		case 4:
			lse_solve(ls);

			if (		   m_isfinitef(ls->sol.m[1]) != 0
					&& m_isfinitef(ls->sol.m[3]) != 0
					&& m_isfinitef(ls->sol.m[5]) != 0
					&& ls->sol.m[1] > M_EPSILON
					&& ls->sol.m[3] > M_EPSILON
					&& ls->sol.m[5] > M_EPSILON) {

				ls->sol.m[0] = pm->scale_uA[0] - ls->sol.m[0] / ls->sol.m[1];
				ls->sol.m[1] = pm->scale_uA[1] / ls->sol.m[1];

				ls->sol.m[2] = pm->scale_uB[0] - ls->sol.m[2] / ls->sol.m[3];
				ls->sol.m[3] = pm->scale_uB[1] / ls->sol.m[3];

				ls->sol.m[4] = pm->scale_uC[0] - ls->sol.m[4] / ls->sol.m[5];
				ls->sol.m[5] = pm->scale_uC[1] / ls->sol.m[5];
			}
			else {
				pm->tvm_ACTIVE = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_UNCERTAIN_RESULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			if (		   m_fabsf(ls->sol.m[0]) > pm->fault_voltage_tol
					|| m_fabsf(ls->sol.m[2]) > pm->fault_voltage_tol
					|| m_fabsf(ls->sol.m[4]) > pm->fault_voltage_tol) {

				pm->tvm_ACTIVE = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_INSUFFICIENT_ACCURACY;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			if (		   m_fabsf(ls->sol.m[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(ls->sol.m[3] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(ls->sol.m[5] - 1.f) > pm->fault_accuracy_tol) {

				pm->tvm_ACTIVE = PM_DISABLED;

				pm->fsm_errno = PM_ERROR_INSUFFICIENT_ACCURACY;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			pm->scale_uA[0] = ls->sol.m[0];
			pm->scale_uA[1] = ls->sol.m[1];
			pm->scale_uB[0] = ls->sol.m[2];
			pm->scale_uB[1] = ls->sol.m[3];
			pm->scale_uC[0] = ls->sol.m[4];
			pm->scale_uC[1] = ls->sol.m[5];

			lse_construct(&pm->probe_lse[0], LSE_CASCADE_MAX, 3, 1);
			lse_construct(&pm->probe_lse[1], LSE_CASCADE_MAX, 3, 1);
			lse_construct(&pm->probe_lse[2], LSE_CASCADE_MAX, 3, 1);

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

			pm->fsm_phase = 5;
			pm->fsm_subi = 0;
			break;

		case 5:
			xMIN = pm->ts_minimal;
			xMAX = (int) (pm->dc_resolution * pm->tvm_clean_zone);

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

				REF = rem[0] * pm->const_fb_U * pm->ts_inverted;

				v[0] = pm->fb_uA;
				v[1] = rem[2];
				v[2] = 1.f;
				v[3] = REF;

				lse_insert(&pm->probe_lse[0], v);

				v[0] = pm->fb_uB;
				v[1] = rem[3];
				v[2] = 1.f;
				v[3] = REF;

				lse_insert(&pm->probe_lse[1], v);

				v[0] = pm->fb_uC;
				v[1] = rem[4];
				v[2] = 1.f;
				v[3] = REF;

				lse_insert(&pm->probe_lse[2], v);
			}

			rem[0] = rem[1];
			rem[1] = (float) xDC;
			rem[2] = pm->fb_uA;
			rem[3] = pm->fb_uB;
			rem[4] = pm->fb_uC;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase += 1;
			}
			break;

		case 6:
			lse_solve(&pm->probe_lse[0]);
			lse_solve(&pm->probe_lse[1]);
			lse_solve(&pm->probe_lse[2]);

			pm->tvm_FIR_A[0] = pm->probe_lse[0].sol.m[0];
			pm->tvm_FIR_A[1] = pm->probe_lse[0].sol.m[1];
			pm->tvm_FIR_A[2] = pm->probe_lse[0].sol.m[2];

			pm->tvm_FIR_B[0] = pm->probe_lse[1].sol.m[0];
			pm->tvm_FIR_B[1] = pm->probe_lse[1].sol.m[1];
			pm->tvm_FIR_B[2] = pm->probe_lse[1].sol.m[2];

			pm->tvm_FIR_C[0] = pm->probe_lse[2].sol.m[0];
			pm->tvm_FIR_C[1] = pm->probe_lse[2].sol.m[1];
			pm->tvm_FIR_C[2] = pm->probe_lse[2].sol.m[2];

			pm->fsm_phase += 1;
			break;

		case 7:
			lse_std(&pm->probe_lse[0]);
			lse_std(&pm->probe_lse[1]);
			lse_std(&pm->probe_lse[2]);

			pm->self_RMSu[1] = pm->probe_lse[0].std.m[0];
			pm->self_RMSu[2] = pm->probe_lse[1].std.m[0];
			pm->self_RMSu[3] = pm->probe_lse[2].std.m[0];

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

					pm->tvm_ACTIVE = PM_ENABLED;
				}
				else {
					pm->tvm_ACTIVE = PM_DISABLED;
				}
			}
			else {
				pm->tvm_ACTIVE = PM_DISABLED;

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
	lse_t			*ls = &pm->probe_lse[0];
	lse_t			*lb = &pm->probe_lse[1];
	lse_float_t		v[4];

	float			eA, uA, uMAX, iA, REF;

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(PM_Z_ABC);

			lse_construct(lb, LSE_CASCADE_MAX, 3, 1);

			pm_clearance(pm, 0, 0, 0);
			pm_clearance(pm, 0, 0, 0);

			v[0] = (pm->vsi_AF == 0) ? 1.f : 0.f;
			v[1] = (pm->vsi_BF == 0) ? 1.f : 0.f;
			v[2] = (pm->vsi_CF == 0) ? 1.f : 0.f;
			v[3] = v[0] + v[1] + v[2];

			lse_insert(lb, v);

			pm->fsm_subi = 0;

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			do {
				if (pm->fsm_subi == 0) {

					if (		   pm->vsi_AF == 0
							&& pm->vsi_BF == 0) {

						pm->proc_set_Z(PM_Z_C);

						lse_construct(ls, LSE_CASCADE_MAX, 1, 3);

						pm->i_integral_D = 0.f;

						pm->tm_value = 0;
						pm->tm_end = PM_TSMS(pm, pm->tm_current_ramp);

						pm->fsm_phase = 2;
						break;
					}
				}
				else if (pm->fsm_subi == 1) {

					if (		   pm->vsi_AF == 0
							&& pm->vsi_CF == 0) {

						pm->proc_set_Z(PM_Z_B);

						lse_construct(ls, LSE_CASCADE_MAX, 1, 3);

						pm->i_integral_D = 0.f;

						pm->tm_value = 0;
						pm->tm_end = PM_TSMS(pm, pm->tm_current_ramp);

						pm->fsm_phase = 2;
						break;
					}
				}
				else if (pm->fsm_subi == 2) {

					if (		   pm->vsi_BF == 0
							&& pm->vsi_CF == 0) {

						pm->proc_set_Z(PM_Z_A);

						lse_construct(ls, LSE_CASCADE_MAX, 1, 3);

						pm->i_integral_D = 0.f;

						pm->tm_value = 0;
						pm->tm_end = PM_TSMS(pm, pm->tm_current_ramp);

						pm->fsm_phase = 2;
						break;
					}
				}
				else {
					pm->tm_value = 0;
					pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);

					pm->fsm_phase = 6;
					break;
				}

				pm->fsm_subi++;
			}
			while (1);
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

			iA =	  (pm->fsm_subi == 0) ? pm->fb_iA
				: (pm->fsm_subi == 1) ? pm->fb_iA
				: (pm->fsm_subi == 2) ? pm->fb_iB : 0.f;

			eA = pm->i_track_D - iA;

			pm->i_integral_D += pm->probe_gain_I * eA;
			uA = pm->probe_gain_P * eA + pm->i_integral_D;

			uMAX = pm->k_UMAX * pm->const_fb_U;

			if (		m_fabsf(pm->i_integral_D) > uMAX
					&& m_fabsf(iA) < pm->fault_current_tol) {

				pm->fsm_errno = PM_ERROR_CURRENT_LOOP_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
				break;
			}

			if      (pm->fsm_subi == 0) { pm_voltage(pm, uA, 0.f); }
			else if (pm->fsm_subi == 1) { pm_voltage(pm, uA, 0.f); }
			else if (pm->fsm_subi == 2) { pm_voltage(pm, 0.f, uA); }

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
			lse_solve(ls);

			if (pm->fsm_subi == 0) {

				if (pm->vsi_CF == 0) {

					pm->self_RMSi[0] = ls->sol.m[2];
				}

				REF = (ls->sol.m[0] - ls->sol.m[1]) / 2.f;

				v[0] = REF / ls->sol.m[0];
				v[1] = REF / ls->sol.m[1];
				v[2] = 0.f;
				v[3] = 0.f;

				lse_insert(lb, v);
			}
			else if (pm->fsm_subi == 1) {

				if (pm->vsi_BF == 0) {

					pm->self_RMSi[1] = ls->sol.m[1];
				}

				REF = (ls->sol.m[0] - ls->sol.m[2]) / 2.f;

				v[0] = REF / ls->sol.m[0];
				v[1] = 0.f;
				v[2] = REF / ls->sol.m[2];
				v[3] = 0.f;

				lse_insert(lb, v);
			}
			else if (pm->fsm_subi == 2) {

				if (pm->vsi_AF == 0) {

					pm->self_RMSi[2] = ls->sol.m[0];
				}

				REF = (ls->sol.m[1] - ls->sol.m[2]) / 2.f;

				v[0] = 0.f;
				v[1] = REF / ls->sol.m[1];
				v[2] = REF / ls->sol.m[2];
				v[3] = 0.f;

				lse_insert(lb, v);
			}

			pm->fsm_subi++;
			pm->fsm_phase = 1;
			break;

		case 6:
			pm_voltage(pm, 0.f, 0.f);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				REF = 0.01f;	/* regularization constant */

				v[0] = REF;
				v[1] = 0.f;
				v[2] = 0.f;
				v[3] = REF;

				lse_insert(lb, v);

				v[0] = 0.f;
				v[1] = REF;
				v[2] = 0.f;
				v[3] = REF;

				lse_insert(lb, v);

				v[0] = 0.f;
				v[1] = 0.f;
				v[2] = REF;
				v[3] = REF;

				lse_insert(lb, v);

				pm->fsm_phase += 1;
			}
			break;

		case 7:
			lse_solve(lb);

			if (		   m_isfinitef(lb->sol.m[0]) != 0
					&& m_isfinitef(lb->sol.m[1]) != 0
					&& m_isfinitef(lb->sol.m[2]) != 0
					&& lb->sol.m[0] > M_EPSILON
					&& lb->sol.m[1] > M_EPSILON
					&& lb->sol.m[2] > M_EPSILON) {

				pm->scale_iA[1] /= lb->sol.m[0];
				pm->scale_iB[1] /= lb->sol.m[1];
				pm->scale_iC[1] /= lb->sol.m[2];
			}
			else {
				pm->fsm_errno = PM_ERROR_UNCERTAIN_RESULT;
			}

			if (		   m_fabsf(pm->scale_iA[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->scale_iB[1] - 1.f) > pm->fault_accuracy_tol
					|| m_fabsf(pm->scale_iC[1] - 1.f) > pm->fault_accuracy_tol) {

				pm->fsm_errno = PM_ERROR_INSUFFICIENT_ACCURACY;
			}

			pm->fsm_state = PM_STATE_HALT;
			pm->fsm_phase = 0;
			break;
	}
}

static void
pm_fsm_probe_impedance_DFT(pmc_t *pm, float la[5])
{
	lse_t		*ls = &pm->probe_lse[0];
	lse_float_t	v[5];

	float		*DFT = pm->probe_DFT;
	float		Z[3], iW;

	/* The primary impedance equation is \Z * \I = \U,
	 *
	 * [R - iZ(0)      iZ(1)] * [IX] = [UX]
	 * [    iZ(1)  R - iZ(2)]   [IY]   [UY], where
	 *
	 * IX = [DFT(0) + iDFT(1)],  UX = [DFT(2) + iDFT(3)],
	 * IY = [DFT(4) + iDFT(5)],  UY = [DFT(6) + iDFT(7)].
	 *
	 * We rewrite it with respect to the impedance components.
	 *
	 * [DFT(0)  DFT(1) -DFT(5)  0     ]   [R   ]   [DFT(2)]
	 * [DFT(1) -DFT(0)  DFT(4)  0     ] * [Z(0)] = [DFT(3)]
	 * [DFT(4)  0      -DFT(1)  DFT(5)]   [Z(1)]   [DFT(6)].
	 * [DFT(5)  0       DFT(0) -DFT(4)]   [Z(2)]   [DFT(7)]
	 *
	 * */

	lse_construct(ls, 1, 4, 1);

	v[0] = DFT[0];
	v[1] = DFT[1];
	v[2] = - DFT[5];
	v[3] = 0.f;
	v[4] = DFT[2];

	lse_insert(ls, v);

	v[0] = DFT[1];
	v[1] = - DFT[0];
	v[2] = DFT[4];
	v[3] = 0.f;
	v[4] = DFT[3];

	lse_insert(ls, v);

	v[0] = DFT[4];
	v[1] = 0.f;
	v[2] = - DFT[1];
	v[3] = DFT[5];
	v[4] = DFT[6];

	lse_insert(ls, v);

	v[0] = DFT[5];
	v[1] = 0.f;
	v[2] = DFT[0];
	v[3] = - DFT[4];
	v[4] = DFT[7];

	lse_insert(ls, v);
	lse_solve(ls);

	iW = 1.f / pm->quick_HFwS;

	la[4] = ls->sol.m[0];

	Z[0] = ls->sol.m[1] * iW;
	Z[1] = ls->sol.m[2] * iW;
	Z[2] = ls->sol.m[3] * iW;

	if (pm->config_SALIENCY == PM_SALIENCY_NEGATIVE) {

		m_la_eigf(Z, la, 0);
	}
	else if (pm->config_SALIENCY == PM_SALIENCY_POSITIVE) {

		m_la_eigf(Z, la, 1);
	}
	else {
		la[0] = 1.f;
		la[1] = 0.f;
		la[2] = (Z[0] + Z[2]) / 2.f;
		la[3] = la[2];
	}
}

static void
pm_fsm_probe_loop_current(pmc_t *pm, float track_HF)
{
	float		eD, eQ, eHF, uD, uQ, uHF, uMAX;

	eD = pm->i_track_D - pm->lu_iX;
	eQ = pm->i_track_Q - pm->lu_iY;

	uMAX = pm->k_EMAX * pm->const_fb_U;

	if (track_HF > M_EPSILON) {

		eHF = track_HF - m_sqrtf(eD * eD + eQ * eQ);

		pm->probe_HF_lpf_track += (eHF - pm->probe_HF_lpf_track) * pm->probe_LP;
		pm->probe_HF_integral += pm->probe_gain_I * pm->probe_HF_lpf_track;

		pm->probe_HF_integral = (pm->probe_HF_integral > uMAX) ? uMAX
			: (pm->probe_HF_integral < - uMAX) ? - uMAX : pm->probe_HF_integral;

		uHF = pm->probe_gain_P * pm->probe_HF_lpf_track + pm->probe_HF_integral;
	}

	pm->i_integral_D += pm->probe_gain_I * eD;
	uD = pm->probe_gain_P * eD + pm->i_integral_D;

	pm->i_integral_Q += pm->probe_gain_I * eQ;
	uQ = pm->probe_gain_P * eQ + pm->i_integral_Q;

	if (		m_fabsf(pm->i_integral_D) > uMAX
			&& m_fabsf(pm->lu_iX) < pm->fault_current_tol) {

		pm->fsm_errno = PM_ERROR_CURRENT_LOOP_FAULT;
		pm->fsm_state = PM_STATE_HALT;
		pm->fsm_phase = 0;
		return ;
	}

	if (		m_fabsf(pm->i_integral_Q) > uMAX
			&& m_fabsf(pm->lu_iY) < pm->fault_current_tol) {

		pm->fsm_errno = PM_ERROR_CURRENT_LOOP_FAULT;
		pm->fsm_state = PM_STATE_HALT;
		pm->fsm_phase = 0;
		return ;
	}

	if (track_HF > M_EPSILON) {

		m_rotatef(pm->hfi_wave, pm->quick_HFwS * pm->m_dT);
		m_normalizef(pm->hfi_wave);

		uD += uHF * pm->hfi_wave[0];
		uQ += uHF * pm->hfi_wave[1];
	}

	pm_voltage(pm, uD, uQ);
}

static void
pm_fsm_state_probe_const_resistance(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_lse[0];
	lse_float_t		v[3];

	float			hold_A, ramp_A;

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(PM_Z_NONE);

			lse_construct(ls, LSE_CASCADE_MAX, 2, 1);

			pm->probe_HF_lpf_track = 0.f;
			pm->probe_HF_integral = 0.f;

			hold_A = pm->probe_hold_angle * (M_PI_F / 180.f);

			pm->probe_TEMP[0] = m_cosf(hold_A);
			pm->probe_TEMP[1] = m_sinf(hold_A);

			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_current_ramp);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			ramp_A = (float) (pm->tm_value + 1)
				* pm->probe_current_hold / (float) pm->tm_end;

			pm->i_track_D = pm->probe_TEMP[0] * ramp_A;
			pm->i_track_Q = pm->probe_TEMP[1] * ramp_A;

		case 2:
			pm_fsm_probe_loop_current(pm, 0.f);

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
			v[0] = pm->probe_TEMP[0] * pm->lu_iX
				+ pm->probe_TEMP[1] * pm->lu_iY;

			v[1] = 1.f;
			v[2] = pm->probe_TEMP[0] * pm->tvm_X0
				+ pm->probe_TEMP[1] * pm->tvm_Y0;

			lse_insert(ls, v);

			pm_fsm_probe_loop_current(pm, 0.f);

			if (pm->fsm_errno != PM_OK)
				break;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				if (pm->fsm_phase == 3) {

					pm->tm_value = 0;
					pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);
				}
				else {
					pm->tm_value = 0;
					pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);
				}

				pm->i_track_D = pm->probe_TEMP[0] * pm->probe_current_weak;
				pm->i_track_Q = pm->probe_TEMP[1] * pm->probe_current_weak;

				pm->fsm_phase += 1;
			}
			break;

		case 5:
			pm_voltage(pm, 0.f, 0.f);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase += 1;
			}
			break;

		case 6:
			lse_solve(ls);

			if (		m_isfinitef(ls->sol.m[0]) != 0
					&& ls->sol.m[0] > M_EPSILON) {

				pm->const_im_R = ls->sol.m[0];
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
pm_fsm_state_probe_const_inductance(pmc_t *pm)
{
	float			iX, iY, uX, uY;
	float			hold_A, la[5];

	switch (pm->fsm_phase) {

		case 0:
			pm->proc_set_DC(0, 0, 0);
			pm->proc_set_Z(PM_Z_NONE);

			pm->probe_DFT[0] = 0.f;
			pm->probe_DFT[1] = 0.f;
			pm->probe_DFT[2] = 0.f;
			pm->probe_DFT[3] = 0.f;
			pm->probe_DFT[4] = 0.f;
			pm->probe_DFT[5] = 0.f;
			pm->probe_DFT[6] = 0.f;
			pm->probe_DFT[7] = 0.f;

			pm->probe_REM[0] = 0.f;
			pm->probe_REM[1] = 0.f;
			pm->probe_REM[2] = 0.f;
			pm->probe_REM[3] = 0.f;
			pm->probe_REM[4] = 0.f;
			pm->probe_REM[5] = 0.f;
			pm->probe_REM[6] = 0.f;
			pm->probe_REM[7] = 0.f;

			pm->quick_HFwS = M_2_PI_F * pm->probe_freq_sine;

			pm->probe_SC[0] = m_cosf(pm->quick_HFwS * pm->m_dT * 0.5f);
			pm->probe_SC[1] = m_sinf(pm->quick_HFwS * pm->m_dT * 0.5f);

			pm->probe_HF_lpf_track = 0.f;
			pm->probe_HF_integral = 0.f;
			pm->probe_LP = pm->quick_HFwS * pm->m_dT / 4.f;

			pm->hfi_wave[0] = 1.f;
			pm->hfi_wave[1] = 0.f;

			hold_A = pm->probe_hold_angle * (M_PI_F / 180.f);

			pm->i_track_D = m_cosf(hold_A) * pm->probe_current_bias;
			pm->i_track_Q = m_sinf(hold_A) * pm->probe_current_bias;

			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_current_hold);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 2:
			iX = pm->probe_TEMP[0];
			iY = pm->probe_TEMP[1];

			uX = pm->tvm_X0 * pm->probe_SC[0] + pm->tvm_Y0 * pm->probe_SC[1];
			uY = pm->tvm_Y0 * pm->probe_SC[0] - pm->tvm_X0 * pm->probe_SC[1];

			m_rsumf(&pm->probe_DFT[0], &pm->probe_REM[0], iX * pm->hfi_wave[0]);
			m_rsumf(&pm->probe_DFT[1], &pm->probe_REM[1], iX * pm->hfi_wave[1]);
			m_rsumf(&pm->probe_DFT[2], &pm->probe_REM[2], uX * pm->hfi_wave[0]);
			m_rsumf(&pm->probe_DFT[3], &pm->probe_REM[3], uX * pm->hfi_wave[1]);
			m_rsumf(&pm->probe_DFT[4], &pm->probe_REM[4], iY * pm->hfi_wave[0]);
			m_rsumf(&pm->probe_DFT[5], &pm->probe_REM[5], iY * pm->hfi_wave[1]);
			m_rsumf(&pm->probe_DFT[6], &pm->probe_REM[6], uY * pm->hfi_wave[0]);
			m_rsumf(&pm->probe_DFT[7], &pm->probe_REM[7], uY * pm->hfi_wave[1]);

		case 1:
			pm->probe_TEMP[0] = pm->lu_iX;
			pm->probe_TEMP[1] = pm->lu_iY;

			pm_fsm_probe_loop_current(pm, pm->probe_current_sine);

			if (pm->fsm_errno != PM_OK)
				break;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				if (pm->fsm_phase == 1) {

					pm->tm_value = 0;
					pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);
				}
				else {
					pm->tm_value = 0;
					pm->tm_end = PM_TSMS(pm, pm->tm_transient_fast);
				}

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			pm_voltage(pm, 0.f, 0.f);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase += 1;
			}
			break;

		case 4:
			pm_fsm_probe_impedance_DFT(pm, la);

			if (		   m_isfinitef(la[2]) != 0 && la[2] > M_EPSILON
					&& m_isfinitef(la[3]) != 0 && la[3] > M_EPSILON) {

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
pm_fsm_state_lu_startup(pmc_t *pm, int in_ZONE)
{
	switch (pm->fsm_phase) {

		case 0:
			if (		   m_isfinitef(pm->const_im_L1) != 0
					&& m_isfinitef(pm->const_im_L2) != 0
					&& pm->const_im_L1 > M_EPSILON
					&& pm->const_im_L2 > M_EPSILON) {

				pm->vsi_DC = 0.f;
				pm->vsi_lpf_DC = 0.f;
				pm->vsi_X = 0.f;
				pm->vsi_Y = 0.f;
				pm->vsi_X0 = 0.f;
				pm->vsi_Y0 = 0.f;

				pm_clearance(pm, 0, 0, 0);
				pm_clearance(pm, 0, 0, 0);

				pm->vsi_AT = 0;
				pm->vsi_BT = 0;
				pm->vsi_CT = 0;

				pm->lu_iX = 0.f;
				pm->lu_iY = 0.f;
				pm->lu_iD = 0.f;
				pm->lu_iQ = 0.f;
				pm->lu_uD = 0.f;
				pm->lu_uQ = 0.f;
				pm->lu_F[0] = 1.f;
				pm->lu_F[1] = 0.f;
				pm->lu_F[2] = 0.f;
				pm->lu_wS = 0.f;
				pm->lu_location = 0.f;
				pm->lu_revol = 0;
				pm->lu_revob = 0;
				pm->lu_mq_produce = 0.f;
				pm->lu_mq_load = 0.f;
				pm->lu_wS_prev = 0.f;

				pm->base_TIM = 0;
				pm->hold_TIM = 0;

				pm->forced_F[0] = 1.f;
				pm->forced_F[1] = 0.f;
				pm->forced_wS = 0.f;
				pm->forced_track_D = 0.f;

				pm->flux_DETACH = PM_DISABLED;

				if (pm->config_LU_ESTIMATE == PM_FLUX_NONE) {

					pm->flux_LINKAGE = PM_ENABLED;
				}
				else if (pm->config_EXCITATION == PM_EXCITATION_NONE) {

					pm->flux_LINKAGE = PM_ENABLED;
				}
				else if (pm->const_lambda > M_EPSILON) {

					pm->flux_LINKAGE = PM_ENABLED;
				}
				else {
					/* So we indicate that flux linkage
					 * is to be estimated further.
					 * */
					pm->flux_LINKAGE = PM_DISABLED;
				}

				pm->detach_TIM = 0;

				pm->flux_TYPE = PM_FLUX_NONE;
				pm->flux_ZONE = in_ZONE;

				pm->flux_X[0] = 0.f;
				pm->flux_X[1] = 0.f;
				pm->flux_lambda = 0.f;
				pm->flux_F[0] = 1.f;
				pm->flux_F[1] = 0.f;
				pm->flux_wS = 0.f;

				pm->kalman_POSTPONED = PM_DISABLED;
				pm->kalman_lpf_wS = 0.f;

				pm->zone_lpf_wS = 0.f;

				pm->hfi_wave[0] = 0.f;
				pm->hfi_wave[1] = 1.f;

				pm->hall_ERN = 0;
				pm->hall_F[0] = 1.f;
				pm->hall_F[1] = 0.f;
				pm->hall_wS = 0.f;

				pm->eabi_RECENT = PM_DISABLED;

				if (pm->config_EABI_FRONTEND == PM_EABI_INCREMENTAL) {

					/* We need to adjust the position again
					 * after loss of tracking.
					 * */
					pm->eabi_ADJUST = PM_DISABLED;
				}

				pm->eabi_F[0] = 1.f;
				pm->eabi_F[1] = 0.f;
				pm->eabi_wS = 0.f;
				pm->eabi_location = 0.f;

				pm->sincos_RECENT = PM_DISABLED;
				pm->sincos_revol = 0;
				pm->sincos_unwrap = 0;
				pm->sincos_F[0] = 1.f;
				pm->sincos_F[1] = 0.f;
				pm->sincos_wS = 0.f;
				pm->sincos_location = 0.f;

				pm->watt_DC_MAX = PM_DISABLED;
				pm->watt_DC_MIN = PM_DISABLED;

				pm->watt_lpf_D = 0.f;
				pm->watt_lpf_Q = 0.f;
				pm->watt_drain_wP = 0.f;
				pm->watt_drain_wA = 0.f;
				pm->watt_integral = 0.f;

				pm->i_setpoint_current = 0.f;
				pm->i_maximal_on_PCB = PM_MAX_F;
				pm->i_track_D = 0.f;
				pm->i_track_Q = 0.f;
				pm->i_integral_D = 0.f;
				pm->i_integral_Q = 0.f;

				pm->mtpa_approx_D = 0.f;
				pm->mtpa_D = 0.f;
				pm->weak_D = 0.f;

				pm->s_setpoint_speed = 0.f;
				pm->s_track = 0.f;
				pm->s_integral = 0.f;

				pm->l_track = 0.f;
				pm->l_blend = 0.f;

				pm->x_setpoint_location = 0.f;
				pm->x_setpoint_speed = 0.f;

				if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

					pm->lu_MODE = PM_LU_DETACHED;

					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(PM_Z_ABC);
				}
				else {
					if (pm->config_LU_ESTIMATE != PM_FLUX_NONE) {

						pm->lu_MODE = PM_LU_ESTIMATE;
					}
					else if (pm->config_LU_SENSOR == PM_SENSOR_HALL) {

						pm->lu_MODE = PM_LU_SENSOR_HALL;
					}
					else if (	pm->config_LU_SENSOR == PM_SENSOR_EABI
							&& pm->eabi_ADJUST == PM_ENABLED) {

						pm->lu_MODE = PM_LU_SENSOR_EABI;
					}
					else if (pm->config_LU_FORCED == PM_ENABLED) {

						pm->lu_MODE = PM_LU_FORCED;
					}
					else {
						pm->fsm_errno = PM_ERROR_INVALID_OPERATION;
						pm->fsm_state = PM_STATE_HALT;
						pm->fsm_phase = 0;
					}

					pm->proc_set_DC(0, 0, 0);
					pm->proc_set_Z(PM_Z_NONE);
				}

				pm->fsm_errno = PM_OK;
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
			pm->i_maximal_on_PCB = 0.f;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->lu_MODE = PM_LU_DISABLED;

				pm->proc_set_DC(0, 0, 0);
				pm->proc_set_Z(PM_Z_ABC);

				pm_clearance(pm, 0, 0, 0);
				pm_clearance(pm, 0, 0, 0);

				pm->fsm_state = PM_STATE_IDLE;
				pm->fsm_phase = 0;
			}
			break;
	}
}

static void
pm_fsm_state_probe_const_flux_linkage(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_lse[0];
	lse_float_t		v[2];

	switch (pm->fsm_phase) {

		case 0:
			lse_construct(ls, LSE_CASCADE_MAX, 1, 1);

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
			if (pm->lu_MODE == PM_LU_DETACHED) {

				if (pm->detach_TIM > PM_TSMS(pm, pm->tm_transient_slow)) {

					v[0] = 1.f;
					v[1] = pm->flux_lambda;

					lse_insert(ls, v);
				}
			}
			else if (pm->flux_TYPE == PM_FLUX_ORTEGA) {

				v[0] = 1.f;
				v[1] = pm->flux_lambda;

				lse_insert(ls, v);
			}
			else if (pm->flux_TYPE == PM_FLUX_KALMAN) {

				if (m_fabsf(pm->flux_wS) > pm->zone_threshold) {

					v[0] = 1.f;
					v[1] = pm->const_lambda
						- pm->kalman_bias_Q / pm->flux_wS;

					lse_insert(ls, v);
				}
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase += 1;
			}
			break;

		case 2:
			if (ls->n_total > PM_TSMS(pm, pm->tm_transient_fast)) {

				lse_solve(ls);

				if (		m_isfinitef(ls->sol.m[0]) != 0
						&& ls->sol.m[0] > M_EPSILON) {

					pm->const_lambda = ls->sol.m[0];
					pm->kalman_bias_Q = 0.f;

					pm_quick_build(pm);
				}
				else {
					pm->fsm_errno = PM_ERROR_UNCERTAIN_RESULT;
				}
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
pm_fsm_state_probe_const_inertia(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_lse[0];
	lse_float_t		v[4];

	float			*m = pm->probe_lse[1].vm;
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

			lse_construct(ls, LSE_CASCADE_MAX, 3, 1);

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

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			lse_solve(ls);

			temp_Ja = pm->m_dT / ls->sol.m[0];

			if (		m_isfinitef(temp_Ja) != 0
					&& temp_Ja > 0.f) {

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
	lse_t			*ls = &pm->probe_lse[0];
	lse_float_t		v[3];

	switch (pm->fsm_phase) {

		case 0:
			lse_construct(ls, LSE_CASCADE_MAX, 2, 1);

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

				pm->fsm_phase += 1;
			}
			break;

		case 2:
			lse_std(ls);

			if (		m_isfinitef(ls->std.m[0]) != 0
					&& ls->std.m[0] > M_EPSILON) {

				pm->zone_noise = ls->std.m[0] * 5.f;
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
	int			*dnum = (int *) pm->probe_lse[1].vm;
	float			*rem0 = pm->probe_lse[1].vm + 10;
	float			*rem1 = pm->probe_lse[1].vm + 18;

	int			HS, N, thld;

	switch (pm->fsm_phase) {

		case 0:
			pm->hall_ERN = 0;

			for (N = 0; N < 8; ++N) {

				pm->hall_ST[N].X = 0.f;
				pm->hall_ST[N].Y = 0.f;

				dnum[N] = 0;
				rem0[N] = 0.f;
				rem1[N] = 0.f;
			}

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, pm->tm_average_probe);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			HS = pm->fb_HS;

			if (HS >= 1 && HS <= 6) {

				pm->hall_ERN = 0;

				dnum[HS] += 1;

				m_rsumf(&pm->hall_ST[HS].X, &rem0[HS], pm->lu_F[0]);
				m_rsumf(&pm->hall_ST[HS].Y, &rem1[HS], pm->lu_F[1]);
			}
			else {
				pm->hall_ERN++;

				if (pm->hall_ERN >= 10) {

					pm->fsm_errno = PM_ERROR_SENSOR_HALL_FAULT;
					pm->fsm_state = PM_STATE_HALT;
					pm->fsm_phase = 0;
				}
				break;
			}

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase += 1;
			}
			break;

		case 2:
			thld = pm->tm_end / 12;
			N = 0;

			for (HS = 1; HS < 7; ++HS) {

				if (dnum[HS] > thld) {

					float		l;

					l = m_fast_reciprocalf((float) dnum[HS]);

					pm->hall_ST[HS].X *= l;
					pm->hall_ST[HS].Y *= l;

					l = m_sqrtf(pm->hall_ST[HS].X * pm->hall_ST[HS].X
						  + pm->hall_ST[HS].Y * pm->hall_ST[HS].Y);

					if (l > 0.5f) {

						l = 1.f / l;

						pm->hall_ST[HS].X *= l;
						pm->hall_ST[HS].Y *= l;

						N += 1;
					}
				}
			}

			if (N < 6) {

				pm->fsm_errno = PM_ERROR_SENSOR_HALL_FAULT;
				pm->fsm_state = PM_STATE_HALT;
				pm->fsm_phase = 0;
			}
			else {
				pm->fsm_state = PM_STATE_IDLE;
				pm->fsm_phase = 0;
			}
			break;
	}
}

static void
pm_fsm_state_adjust_sensor_eabi(pmc_t *pm)
{
	lse_t			*ls = &pm->probe_lse[0];
	lse_float_t		v[4];

	int			*range_bEP = (int *) pm->probe_lse[1].vm;

	int			relEP, WRAP, N;

	switch (pm->fsm_phase) {

		case 0:
			pm->eabi_bEP = pm->fb_EP;
			pm->eabi_lEP = 0;

			range_bEP[0] = pm->fb_EP;
			range_bEP[1] = pm->fb_EP;

			pm->tm_value = 0;
			pm->tm_end = PM_TSMS(pm, 10.f * pm->tm_average_probe);

			pm->fsm_errno = PM_OK;
			pm->fsm_phase = 1;
			break;

		case 1:
			range_bEP[0] = (pm->fb_EP < range_bEP[0])
				? pm->fb_EP : range_bEP[0];

			range_bEP[1] = (pm->fb_EP > range_bEP[1])
				? pm->fb_EP : range_bEP[1];

			relEP = pm->fb_EP - pm->eabi_bEP;

			pm->eabi_lEP += (relEP < 0) ? - 1 : (relEP > 0) ? 1 : 0;
			pm->eabi_bEP = pm->fb_EP;

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->eabi_const_Zs = (pm->eabi_lEP < 0) ? - 1 : 1;

				if (pm->config_EABI_FRONTEND == PM_EABI_INCREMENTAL) {

					pm->eabi_const_EP = 1;

					pm->eabi_bEP = pm->fb_EP;
					pm->eabi_lEP = 0;
				}
				else if (pm->config_EABI_FRONTEND == PM_EABI_ABSOLUTE) {

					if (		range_bEP[1] - range_bEP[0] > 100
							&& range_bEP[0] < 100) {

						for (N = 0; N < 32; ++N) {

							if ((range_bEP[1] & 0x1FU) == 0)
								break;

							range_bEP[1]++;
						}
					}
					else {
						pm->fsm_errno = PM_ERROR_SENSOR_EABI_FAULT;
						pm->fsm_state = PM_STATE_HALT;
						pm->fsm_phase = 0;
						break;
					}

					pm->eabi_const_EP = range_bEP[1];

					pm->eabi_bEP = pm->fb_EP;
					pm->eabi_lEP = pm->fb_EP;
				}

				pm_quick_build(pm);

				lse_construct(ls, LSE_CASCADE_MAX, 2, 1);

				pm->lu_revol = 0;

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, 10.f * pm->tm_average_probe);

				pm->fsm_phase += 1;
			}
			break;

		case 2:
			if (pm->config_EABI_FRONTEND == PM_EABI_INCREMENTAL) {

				WRAP = 0x10000;

				relEP = pm->fb_EP - pm->eabi_bEP;
				relEP +=  unlikely(relEP > WRAP / 2 - 1) ? - WRAP
					: unlikely(relEP < - WRAP / 2) ? WRAP : 0;

				pm->eabi_bEP = pm->fb_EP;
				pm->eabi_lEP += relEP;
			}
			else if (pm->config_EABI_FRONTEND == PM_EABI_ABSOLUTE) {

				WRAP = pm->eabi_const_EP;

				pm->eabi_bEP = pm->eabi_lEP - (pm->eabi_lEP / WRAP) * WRAP;
				pm->eabi_bEP += (pm->eabi_bEP < 0) ? WRAP : 0;

				relEP = pm->fb_EP - pm->eabi_bEP;
				relEP +=  unlikely(relEP > WRAP / 2 - 1) ? - WRAP
					: unlikely(relEP < - WRAP / 2) ? WRAP : 0;

				pm->eabi_lEP += relEP;
			}

			v[0] = 1.f;
			v[1] = (float) pm->eabi_lEP * pm->quick_ZiEP;
			v[2] = m_atan2f(pm->lu_F[1], pm->lu_F[0])
				+ (float) pm->lu_revol * M_2_PI_F;

			lse_insert(ls, v);

			pm->tm_value++;

			if (pm->tm_value >= pm->tm_end) {

				pm->fsm_phase += 1;
			}
			break;

		case 3:
			lse_solve(ls);

			if (pm->config_EABI_FRONTEND == PM_EABI_INCREMENTAL) {

				if (		m_isfinitef(ls->sol.m[1]) != 0
						&& ls->sol.m[1] > M_EPSILON) {

					v[0] = pm->eabi_const_EP / ls->sol.m[1];

					pm->eabi_const_EP = (int) (v[0] + 0.5f);

					pm_quick_build(pm);
				}
				else {
					pm->fsm_errno = PM_ERROR_SENSOR_EABI_FAULT;
					pm->fsm_state = PM_STATE_HALT;
					pm->fsm_phase = 0;
				}
			}
			else if (pm->config_EABI_FRONTEND == PM_EABI_ABSOLUTE) {

				if (		m_isfinitef(ls->sol.m[1]) != 0
						&& ls->sol.m[1] > M_EPSILON) {

					v[0] = pm->eabi_const_Zs * ls->sol.m[1];

					if (pm->eabi_const_Zs < 0) {

						pm->eabi_const_Zs = (int) (v[0] - 0.5f);
					}
					else {
						pm->eabi_const_Zs = (int) (v[0] + 0.5f);
					}

					pm_quick_build(pm);

					if (m_isfinitef(ls->sol.m[0]) != 0) {

						pm->eabi_F0[0] = m_cosf(ls->sol.m[0]);
						pm->eabi_F0[1] = m_sinf(ls->sol.m[0]);

						pm->eabi_ADJUST = PM_ENABLED;
					}
				}
				else {
					pm->fsm_errno = PM_ERROR_SENSOR_EABI_FAULT;
					pm->fsm_state = PM_STATE_HALT;
					pm->fsm_phase = 0;
				}
			}

			pm->fsm_state = PM_STATE_IDLE;
			pm->fsm_phase = 0;
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

			pm_clearance(pm, 0, 0, 0);
			pm_clearance(pm, 0, 0, 0);

			if (pm->lu_MODE != PM_LU_DISABLED) {

				pm->tm_value = 0;
				pm->tm_end = PM_TSMS(pm, pm->tm_pause_halt);

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
		case PM_STATE_PROBE_CONST_RESISTANCE:
		case PM_STATE_PROBE_CONST_INDUCTANCE:
		case PM_STATE_LU_DETACHED:
		case PM_STATE_LU_STARTUP:

			if (		pm->fsm_state == PM_STATE_IDLE
					&& pm->lu_MODE == PM_LU_DISABLED) {

				pm_quick_build(pm);

				pm->fsm_state = pm->fsm_req;
				pm->fsm_phase = 0;
			}
			break;

		case PM_STATE_LU_SHUTDOWN:
		case PM_STATE_PROBE_CONST_FLUX_LINKAGE:
		case PM_STATE_PROBE_CONST_INERTIA:
		case PM_STATE_PROBE_NOISE_THRESHOLD:
		case PM_STATE_ADJUST_SENSOR_HALL:
		case PM_STATE_ADJUST_SENSOR_EABI:
		case PM_STATE_ADJUST_SENSOR_SINCOS:

			if (		pm->fsm_state == PM_STATE_IDLE
					&& pm->lu_MODE != PM_LU_DISABLED) {

				pm->fsm_state = pm->fsm_req;
				pm->fsm_phase = 0;
			}
			break;

		case PM_STATE_HALT:

			if (pm->fsm_state == PM_STATE_HALT)
				break;

			if (		pm->fsm_state != PM_STATE_IDLE
					|| pm->lu_MODE != PM_LU_DISABLED) {

				pm->fsm_state = pm->fsm_req;
				pm->fsm_phase = 0;
			}
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

		case PM_STATE_PROBE_CONST_RESISTANCE:
			pm_fsm_state_probe_const_resistance(pm);
			break;

		case PM_STATE_PROBE_CONST_INDUCTANCE:
			pm_fsm_state_probe_const_inductance(pm);
			break;

		case PM_STATE_LU_DETACHED:
			pm_fsm_state_lu_startup(pm, PM_ZONE_LOCKED_IN_DETACH);
			break;

		case PM_STATE_LU_STARTUP:
			pm_fsm_state_lu_startup(pm, PM_ZONE_NONE);
			break;

		case PM_STATE_LU_SHUTDOWN:
			pm_fsm_state_lu_shutdown(pm);
			break;

		case PM_STATE_PROBE_CONST_FLUX_LINKAGE:
			pm_fsm_state_probe_const_flux_linkage(pm);
			break;

		case PM_STATE_PROBE_CONST_INERTIA:
			pm_fsm_state_probe_const_inertia(pm);
			break;

		case PM_STATE_PROBE_NOISE_THRESHOLD:
			pm_fsm_state_probe_noise_threshold(pm);
			break;

		case PM_STATE_ADJUST_SENSOR_HALL:
			pm_fsm_state_adjust_sensor_hall(pm);
			break;

		case PM_STATE_ADJUST_SENSOR_EABI:
			pm_fsm_state_adjust_sensor_eabi(pm);
			break;

		case PM_STATE_LOOP_BOOST:
			pm_fsm_state_loop_boost(pm);
			break;

		case PM_STATE_HALT:
		default:
			pm_fsm_state_halt(pm);
	}
}

#define PM_SFI_CASE(errno)	case errno: sym = PM_SFI(errno); break

const char *pm_strerror(int fsm_errno)
{
	const char		*sym;

	switch (fsm_errno) {

		PM_SFI_CASE(PM_OK);

		PM_SFI_CASE(PM_ERROR_ZERO_DRIFT_FAULT);
		PM_SFI_CASE(PM_ERROR_NO_MOTOR_CONNECTED);
		PM_SFI_CASE(PM_ERROR_BOOTSTRAP_FAULT);
		PM_SFI_CASE(PM_ERROR_POWER_STAGE_BROKEN);
		PM_SFI_CASE(PM_ERROR_INSUFFICIENT_ACCURACY);
		PM_SFI_CASE(PM_ERROR_CURRENT_LOOP_FAULT);
		PM_SFI_CASE(PM_ERROR_INSTANT_OVERCURRENT);
		PM_SFI_CASE(PM_ERROR_DC_LINK_OVERVOLTAGE);
		PM_SFI_CASE(PM_ERROR_UNCERTAIN_RESULT);
		PM_SFI_CASE(PM_ERROR_INVALID_OPERATION);
		PM_SFI_CASE(PM_ERROR_SENSOR_HALL_FAULT);
		PM_SFI_CASE(PM_ERROR_SENSOR_EABI_FAULT);

		PM_SFI_CASE(PM_ERROR_TIMEOUT);
		PM_SFI_CASE(PM_ERROR_NO_FLUX_CAUGHT);
		PM_SFI_CASE(PM_ERROR_NO_SYNC_FAULT);
		PM_SFI_CASE(PM_ERROR_KNOB_CONTROL_FAULT);
		PM_SFI_CASE(PM_ERROR_SPI_DATA_FAULT);

		PM_SFI_CASE(PM_ERROR_HW_UNMANAGED_IRQ);
		PM_SFI_CASE(PM_ERROR_HW_OVERCURRENT);
		PM_SFI_CASE(PM_ERROR_HW_OVERTEMPERATURE);
		PM_SFI_CASE(PM_ERROR_HW_EMERGENCY_STOP);

		default: sym = "";
	}

	return sym;
}

