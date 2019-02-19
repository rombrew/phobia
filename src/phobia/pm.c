#include "libm.h"
#include "pm.h"

void pm_config_default(pmc_t *pm)
{
	pm->dc_minimal = 42;
	pm->dc_clearance = 420;

	pm->config_ABC = PM_ABC_THREE_PHASE;
	pm->config_LDQ = PM_LDQ_SATURATION_SALIENCY;
	pm->config_TVSE = PM_ENABLED;

	pm->config_HALL = PM_DISABLED;
	pm->config_HFI = PM_DISABLED;
	pm->config_LOOP = PM_LOOP_DRIVE_SPEED;

	pm->tm_transient_skip = .05f;
	pm->tm_voltage_hold = .05f;
	pm->tm_current_hold = .5f;
	pm->tm_instant_probe = .01f;
	pm->tm_average_probe = .2f;
	pm->tm_startup = .1f;

	pm->adjust_IA[0] = 0.f;
	pm->adjust_IA[1] = 1.f;
	pm->adjust_IB[0] = 0.f;
	pm->adjust_IB[1] = 1.f;
	pm->adjust_US[0] = 0.f;
	pm->adjust_US[1] = 1.f;
	pm->adjust_UA[0] = 0.f;
	pm->adjust_UA[1] = 1.f;
	pm->adjust_UB[0] = 0.f;
	pm->adjust_UB[1] = 1.f;
	pm->adjust_UC[0] = 0.f;
	pm->adjust_UC[1] = 1.f;

	pm->probe_current_hold = 20.f;
	pm->probe_current_hold_Q = 0.f;
	pm->probe_current_sine = 5.f;
	pm->probe_freq_sine_hz = pm->freq_hz / 16.f;
	pm->probe_speed_low = 700.f;
	pm->probe_speed_ramp = 1700.f;
	pm->probe_gain_P = 1E-2f;
	pm->probe_gain_I = 1E-3f;

	pm->fault_voltage_tolerance = 1.f;
	pm->fault_current_tolerance = 2.f;
	pm->fault_current_halt_level = 20.f;
	pm->fault_adjust_tolerance = 5E-2f;
	pm->fault_flux_residue_maximal = 90.f;

	pm->vsi_gain_LP = 1E-1f;
	pm->vsi_gain_LW = 5E-1f;

	pm->vsi_A.const_TAU = 22E-6f;
	pm->vsi_A.const_TOF = 0.809E-6f;
	pm->vsi_B.const_TAU = 22E-6f;
	pm->vsi_B.const_TOF = 1.523E-6f;
	pm->vsi_C.const_TAU = 22E-6f;
	pm->vsi_C.const_TOF = 1.523E-6f;

	pm->forced_hold_D = 10.f;
	pm->forced_accel = 3E+3f;

	pm->flux_gain_LP = 1E-1f;
	pm->flux_gain_DA = 5E-1f;
	pm->flux_gain_QA = 5E-1f;
	pm->flux_gain_DP = 5E-3f;
	pm->flux_gain_DS = 1E+1f;
	pm->flux_gain_QS = 1E+1f;
	pm->flux_gain_QZ = 5E-2f;
	pm->flux_bemf_low_unlock = .2f;
	pm->flux_bemf_low_lock = .5f;
	pm->flux_bemf_high = 1.f;

	pm->hfi_freq_hz = pm->freq_hz / 12.f;
	pm->hfi_swing_D = 2.f;
	pm->hfi_gain_P = 3E-1f;
	pm->hfi_gain_S = 7E+1f;
	pm->hfi_gain_F = 1E-3f;

	pm->const_gain_LP = 1E-1f;
	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Ld = 0.f;
	pm->const_Lq = 0.f;
	pm->const_Zp = 1;
	pm->const_J = 0.f;

	pm->i_maximal = pm->fb_current_clamp;
	pm->i_watt_consumption_maximal = 3500.f;
	pm->i_watt_regeneration_maximal = -50.f;
	pm->i_gain_PD = 5E-2f;
	pm->i_gain_ID = 5E-3f;
	pm->i_gain_PQ = 5E-2f;
	pm->i_gain_IQ = 5E-3f;

	pm->s_maximal = pm->freq_hz * (2.f * M_PI_F / 12.f);
	pm->s_accel = 5E+6f;
	pm->s_gain_P = 2E-2f;
	pm->s_gain_I = 1E-4f;

	pm->p_gain_P = 50.f;
	pm->p_gain_I = 0.f;
}

void pm_config_tune_current_loop(pmc_t *pm)
{
	pm->i_gain_PD = .5f * pm->const_Ld * pm->freq_hz - pm->const_R;
	pm->i_gain_ID = 2E-2f * pm->const_Ld * pm->freq_hz;
	pm->i_gain_PQ = .5f * pm->const_Lq * pm->freq_hz - pm->const_R;
	pm->i_gain_IQ = 2E-2f * pm->const_Lq * pm->freq_hz;
}

void pm_config_tune_flux_observer(pmc_t *pm)
{
}

static void
pm_equation_2(pmc_t *pm, float Y[2], const float X[5])
{
	float		uD, uQ, X4, X5, R1, E1, fluxD, fluxQ;

	uD = X[2] * pm->vsi_X + X[3] * pm->vsi_Y;
	uQ = X[2] * pm->vsi_Y - X[3] * pm->vsi_X;

	X4 = X[4];
	X5 = pm->flux_drift_Q;
	R1 = pm->const_R;
	E1 = pm->const_E;

	fluxD = pm->const_Ld * X[0] + E1;
	fluxQ = pm->const_Lq * X[1];

	Y[0] = (uD - R1 * X[0] + fluxQ * X4) / pm->const_Ld;
	Y[1] = (uQ - R1 * X[1] - fluxD * X4 + X5) / pm->const_Lq;
}

static void
pm_solve_2(pmc_t *pm, float X[5])
{
	float		Y1[2], Y2[2];

	/* Second-order ODE solver.
	 * */

	pm_equation_2(pm, Y1, X);

	X[0] += (Y1[0]) * pm->dT;
	X[1] += (Y1[1]) * pm->dT;

	m_rotf(X + 2, X[4] * pm->dT, X + 2);

	pm_equation_2(pm, Y2, X);

	X[0] += (Y2[0] - Y1[0]) * pm->dT * .5f;
	X[1] += (Y2[1] - Y1[1]) * pm->dT * .5f;
}

static void
pm_solve_1(pmc_t *pm, float X[5])
{
	float		uD, uQ;

	uD = X[2] * pm->vsi_residue_X + X[3] * pm->vsi_residue_Y;
	uQ = X[2] * pm->vsi_residue_Y - X[3] * pm->vsi_residue_X;

	X[0] += uD * pm->dT / pm->const_Ld;
	X[1] += uQ * pm->dT / pm->const_Lq;
}

static void
pm_forced_update(pmc_t *pm)
{
	float		*X = pm->forced_X;
	float		iX, iY, wSP, dS, wMAX;

	iX = pm->flux_X[2] * pm->flux_X[0] - pm->flux_X[3] * pm->flux_X[1];
	iY = pm->flux_X[3] * pm->flux_X[0] + pm->flux_X[2] * pm->flux_X[1];

	X[0] = X[2] * iX + X[3] * iY;
	X[1] = X[2] * iY - X[3] * iX;

	dS = pm->forced_accel * pm->dT;
	wSP = pm->forced_setpoint;

	if (pm->const_E != 0.f) {

		wMAX = pm->flux_bemf_high / pm->const_E;
		wSP = (wSP < - wMAX) ? - wMAX : (wSP > wMAX) ? wMAX : wSP;
	}

	X[4] = (X[4] < wSP - dS) ? X[4] + dS : (X[4] > wSP + dS) ? X[4] - dS : wSP;

	m_rotf(X + 2, X[4] * pm->dT, X + 2);
}

static void
pm_flux_residue(pmc_t *pm)
{
	float		*X = pm->flux_X;
	float		iA, iB, iX, iY;

	iX = X[2] * X[0] - X[3] * X[1];
	iY = X[3] * X[0] + X[2] * X[1];

	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		iA = iX;
		iB = - .5f * iX + .8660254f * iY;

		iA = (iA < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
			(iA > pm->fb_current_clamp) ? pm->fb_current_clamp : iA;
		iB = (iB < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
			(iB > pm->fb_current_clamp) ? pm->fb_current_clamp : iB;

		iA = pm->fb_current_A - iA;
		iB = pm->fb_current_B - iB;

		if ((pm->vsi_ZONE & 0x33UL) == 0) {

			iX = iA;
			iY = .57735027f * iA + 1.1547005f * iB;
		}
		else if ((pm->vsi_ZONE & 0x11UL) == 0) {

			iX = iA;
			iY = 0.f;
		}
		else if ((pm->vsi_ZONE & 0x22UL) == 0) {

			iX = - .5f * iB;
			iY = .8660254f * iB;
		}
		else {
			iX = 0.f;
			iY = 0.f;
		}
	}
	else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

		iX = (iX < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
			(iX > pm->fb_current_clamp) ? pm->fb_current_clamp : iX;
		iY = (iY < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
			(iY > pm->fb_current_clamp) ? pm->fb_current_clamp : iY;

		iA = pm->fb_current_A - iA;
		iB = pm->fb_current_B - iB;

		iX = ((pm->vsi_ZONE & 0x11UL) == 0) ? iA : 0.f;
		iY = ((pm->vsi_ZONE & 0x22UL) == 0) ? iB : 0.f;
	}

	pm->flux_residue_D = X[2] * iX + X[3] * iY;
	pm->flux_residue_Q = X[2] * iY - X[3] * iX;
}

static void
pm_flux_update(pmc_t *pm)
{
	float		*X = pm->flux_X;
	float		eD, eQ, eR, dR, qS;

	if (pm->lu_mode != PM_LU_DETACHED) {

		pm_flux_residue(pm);
	}
	else {
		pm->flux_residue_D = - X[0];
		pm->flux_residue_Q = - X[1];
	}

	eD = pm->flux_residue_D;
	eQ = pm->flux_residue_Q;

	pm->flux_residue_lpf += (eD * eD + eQ * eQ - pm->flux_residue_lpf)
		* pm->flux_gain_LP;

	X[0] += pm->flux_gain_DA * eD;
	X[1] += pm->flux_gain_QA * eQ;

	if (pm->const_E != 0.f) {

		qS = (pm->lu_mode == PM_LU_FORCED) ? pm->forced_X[4] : X[4];
		eR = (qS < 0.f) ? - eD : eD;

		dR = pm->flux_gain_DP * eR;
		dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
		m_rotf(X + 2, dR, X + 2);

		qS = m_fabsf(X[4] * pm->const_E) / (.57735027f * pm->const_lpf_U);
		qS = (qS > 1.f) ? 1.f : qS;

		dR = pm->flux_gain_DS * qS * eR - pm->flux_gain_QS * (1.f - qS) * eQ;
		X[4] += dR;

		if (m_fabsf(X[4] * pm->const_E) > pm->flux_bemf_high) {

			pm->flux_drift_Q += pm->flux_gain_QZ * (eR + eQ);
		}
		else {
			pm->flux_drift_Q = 0.f;
		}
	}
	else if (pm->lu_mode == PM_LU_FORCED) {

		X[2] = pm->forced_X[2];
		X[3] = pm->forced_X[3];
		X[4] = pm->forced_X[4];

		pm->flux_drift_Q += pm->flux_gain_QZ * eQ;
	}

	pm_solve_2(pm, X);
}

static void
pm_hfi_update(pmc_t *pm)
{
	float		*X = pm->hfi_X;
	float		eD, eQ, eR, dR, C2;

	pm_flux_residue(pm);

	eD = pm->flux_residue_D;
	eQ = pm->flux_residue_Q;

	X[0] += pm->flux_gain_DA * eD;
	X[1] += pm->flux_gain_QA * eQ;

	eR = pm->hfi_CS[1] * eQ;
	dR = pm->hfi_gain_P * eR;
	dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
	m_rotf(X + 2, dR, X + 2);

	X[4] += pm->hfi_gain_S * eR;

	if (1) {

		C2 = pm->hfi_CS[0] * pm->hfi_CS[0] - pm->hfi_CS[1] * pm->hfi_CS[1];
		pm->hfi_flux_polarity += eD * C2 * pm->hfi_gain_F;

		if (pm->hfi_flux_polarity > 1.f) {

			X[2] = - X[2];
			X[3] = - X[3];

			pm->hfi_flux_polarity = 0.f;
		}
		else if (pm->hfi_flux_polarity < 0.f) {

			pm->hfi_flux_polarity = 0.f;
		}
	}

	pm_solve_2(pm, X);
}

static void
pm_hall_update(pmc_t *pm)
{
	/* TODO */
}

static void
pm_instant_take(pmc_t *pm)
{
	float			uA, uB, uQ;

	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		uQ = (1.f / 3.f) * (pm->fb_voltage_A + pm->fb_voltage_B + pm->fb_voltage_C);
		uA = pm->fb_voltage_A - uQ;
		uB = pm->fb_voltage_B - uQ;

		pm->vsi_X = uA;
		pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;
	}
	else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

		uA = pm->fb_voltage_A - pm->fb_voltage_C;
		uB = pm->fb_voltage_B - pm->fb_voltage_C;

		pm->vsi_X = uA;
		pm->vsi_Y = uB;
	}
}

void pm_voltage_initial_prep(pmc_t *pm)
{
	pm->vsi_A.prep_EXP[0] = m_expf((pm->dT / 2.f + pm->vsi_A.const_TOF) / pm->vsi_A.const_TAU);
	pm->vsi_A.prep_EXP[1] = m_expf((pm->vsi_A.const_TOF - pm->dT / 2.f) / pm->vsi_A.const_TAU);

	pm->vsi_B.prep_EXP[0] = m_expf((pm->dT / 2.f + pm->vsi_B.const_TOF) / pm->vsi_B.const_TAU);
	pm->vsi_B.prep_EXP[1] = m_expf((pm->vsi_B.const_TOF - pm->dT / 2.f) / pm->vsi_B.const_TAU);

	pm->vsi_C.prep_EXP[0] = m_expf((pm->dT / 2.f + pm->vsi_C.const_TOF) / pm->vsi_C.const_TAU);
	pm->vsi_C.prep_EXP[1] = m_expf((pm->vsi_C.const_TOF - pm->dT / 2.f) / pm->vsi_C.const_TAU);
}

void pm_voltage_recovery(pmc_t *pm)
{
	float			uA, uB, uC, uX, uY, uQ;
	float			sinh, tON;

	if (pm->vsi_ZONE == 0) {

		sinh = (pm->fb_voltage_A * pm->vsi_A.prep_EXP[0] - pm->vsi_A.temp_u0
				* pm->vsi_A.prep_EXP[1]) / (2.f * pm->const_lpf_U);

		tON = 2.f * pm->vsi_A.const_TAU * m_logf(sinh + m_sqrtf(sinh * sinh + 1.f));
		uA = tON * pm->freq_hz * pm->const_lpf_U;

		sinh = (pm->fb_voltage_B * pm->vsi_B.prep_EXP[0] - pm->vsi_B.temp_u0
				* pm->vsi_B.prep_EXP[1]) / (2.f * pm->const_lpf_U);

		tON = 2.f * pm->vsi_B.const_TAU * m_logf(sinh + m_sqrtf(sinh * sinh + 1.f));
		uB = tON * pm->freq_hz * pm->const_lpf_U;

		sinh = (pm->fb_voltage_C * pm->vsi_C.prep_EXP[0] - pm->vsi_C.temp_u0
				* pm->vsi_C.prep_EXP[1]) / (2.f * pm->const_lpf_U);

		tON = 2.f * pm->vsi_C.const_TAU * m_logf(sinh + m_sqrtf(sinh * sinh + 1.f));
		uC = tON * pm->freq_hz * pm->const_lpf_U;

		if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

			uQ = (1.f / 3.f) * (uA + uB + uC);
			uA = uA - uQ;
			uB = uB - uQ;

			uX = uA;
			uY = .57735027f * uA + 1.1547005f * uB;
		}
		else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

			uA = uA - uC;
			uB = uB - uC;

			uX = uA;
			uY = uB;
		}

		pm->vsi_residue_X = uX - pm->vsi_queue_X;
		pm->vsi_residue_Y = uY - pm->vsi_queue_Y;
	}

	pm->vsi_A.temp_u0 = pm->fb_voltage_A;
	pm->vsi_B.temp_u0 = pm->fb_voltage_B;
	pm->vsi_C.temp_u0 = pm->fb_voltage_C;

	pm->vsi_queue_X = pm->vsi_X;
	pm->vsi_queue_Y = pm->vsi_Y;
}

static void
pm_lu_FSM(pmc_t *pm)
{
	float		*X = pm->lu_X;

	if (pm->lu_mode == PM_LU_DETACHED) {

		pm_instant_take(pm);
		pm_flux_update(pm);
	}
	else if (pm->lu_mode == PM_LU_FORCED) {

		if (PM_CONFIG_TVSE(pm) == PM_ENABLED) {

			pm_voltage_recovery(pm);
			pm_solve_1(pm, pm->flux_X);
		}

		pm_flux_update(pm);
		pm_forced_update(pm);

		X[0] = pm->forced_X[0];
		X[1] = pm->forced_X[1];
		X[2] = pm->forced_X[2];
		X[3] = pm->forced_X[3];
		X[4] = pm->forced_X[4];

		if (m_fabsf(X[4] * pm->const_E) > pm->flux_bemf_low_lock
				&& m_fabsf(pm->flux_X[4] * pm->const_E) > pm->flux_bemf_low_lock) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;
		}
	}
	else if (pm->lu_mode == PM_LU_ESTIMATE_FLUX) {

		if (PM_CONFIG_TVSE(pm) == PM_ENABLED) {

			pm_voltage_recovery(pm);
			pm_solve_1(pm, pm->flux_X);
		}

		pm_flux_update(pm);

		X[0] = pm->flux_X[0];
		X[1] = pm->flux_X[1];
		X[2] = pm->flux_X[2];
		X[3] = pm->flux_X[3];
		X[4] = pm->flux_X[4];

		if (m_fabsf(X[4] * pm->const_E) < pm->flux_bemf_low_unlock) {

			if (pm->config_HALL != PM_DISABLED) {

				pm->lu_mode = PM_LU_SENSORED_HALL;
			}
			else if (pm->config_HFI != PM_DISABLED) {

				pm->lu_mode = PM_LU_ESTIMATE_HFI;

				pm->hfi_X[0] = X[0];
				pm->hfi_X[1] = X[1];
				pm->hfi_X[2] = X[2];
				pm->hfi_X[3] = X[3];
				pm->hfi_X[4] = X[4];
			}
			else {
				pm->lu_mode = PM_LU_FORCED;

				pm->forced_X[0] = X[0];
				pm->forced_X[1] = X[1];
				pm->forced_X[2] = X[2];
				pm->forced_X[3] = X[3];
				pm->forced_X[4] = X[4];
			}
		}
	}
	else if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		pm_hfi_update(pm);

		X[0] = pm->hfi_X[0];
		X[1] = pm->hfi_X[1];
		X[2] = pm->hfi_X[2];
		X[3] = pm->hfi_X[3];
		X[4] = pm->hfi_X[4];

		if (m_fabsf(X[4] * pm->const_E) > pm->flux_bemf_low_lock) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;

			pm->flux_X[0] = X[0];
			pm->flux_X[1] = X[1];
			pm->flux_X[2] = X[2];
			pm->flux_X[3] = X[3];
			pm->flux_X[4] = X[4];
		}
	}
	else if (pm->lu_mode == PM_LU_SENSORED_HALL) {

		pm_hall_update(pm);
	}

	if (PM_CONFIG_TVSE(pm) == PM_ENABLED) {

		pm->vsi_A.temp_u0 = pm->fb_voltage_A;
		pm->vsi_B.temp_u0 = pm->fb_voltage_B;
		pm->vsi_C.temp_u0 = pm->fb_voltage_C;

		pm->vsi_queue_X = pm->vsi_X;
		pm->vsi_queue_Y = pm->vsi_Y;
	}
}

static void
pm_lu_validate(pmc_t *pm)
{
	float		*X = pm->lu_X;

	if (pm->flux_residue_lpf > pm->fault_flux_residue_maximal) {

		pm->fail_reason = PM_ERROR_RESIDUE_UNSTABLE;
		pm_fsm_req(pm, PM_STATE_HALT);
	}

	if (m_isfinitef(X[1]) == 0) {

		pm->fail_reason = PM_ERROR_INVALID_OPERATION;
		pm_fsm_req(pm, PM_STATE_HALT);
	}
}

void pm_voltage_control(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC;
	float		uMIN, uMAX, uQ;
	int		xA, xB, xC;
	int		xMIN, xMAX, ZONE;

	uX /= pm->const_lpf_U;
	uY /= pm->const_lpf_U;

	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		uA = uX;
		uB = - .5f * uX + .8660254f * uY;
		uC = - .5f * uX - .8660254f * uY;
	}
	else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

		uA = uX;
		uB = uY;
		uC = 0.f;
	}

	if (uA < uB) {

		uMIN = uA;
		uMAX = uB;
	}
	else {
		uMIN = uB;
		uMAX = uA;
	}

	if (uC < uMIN) {

		uMIN = uC;
	}
	else if (uMAX < uC) {

		uMAX = uC;
	}

	uQ = uMAX - uMIN;

	if (uQ > 1.f) {

		uQ = 1.f / uQ;

		uA *= uQ;
		uB *= uQ;
		uC *= uQ;

		uMIN *= uQ;
		uMAX *= uQ;
	}

	if (pm->vsi_clamp_to_GND != 0) {

		uQ = 0.f - uMIN;
	}
	else {
		uQ = .5f - (uMIN + uMAX) * .5f;
	}

	uA += uQ;
	uB += uQ;
	uC += uQ;

	xA = (int) (pm->dc_resolution * uA);
	xB = (int) (pm->dc_resolution * uB);
	xC = (int) (pm->dc_resolution * uC);

	xMIN = pm->dc_minimal;
	xMAX = pm->dc_resolution - pm->dc_minimal;

	xA = (xA < xMIN) ? 0 : (xA > xMAX) ? pm->dc_resolution : xA;
	xB = (xB < xMIN) ? 0 : (xB > xMAX) ? pm->dc_resolution : xB;
	xC = (xC < xMIN) ? 0 : (xC > xMAX) ? pm->dc_resolution : xC;

	pm->proc_set_DC(xA, xB, xC);

	xMAX = pm->dc_resolution;
	xMIN = pm->dc_resolution - pm->dc_clearance;

	ZONE = (pm->vsi_ZONE & 0xFUL) << 4;

	ZONE |= (xA > xMIN) ? (xA < xMAX) ? (1UL << 0) : (1UL << 3) : 0UL;
	ZONE |= (xB > xMIN) ? (xB < xMAX) ? (1UL << 1) : (1UL << 3) : 0UL;
	ZONE |= (xC > xMIN) ? (xC < xMAX) ? (1UL << 2) : (1UL << 3) : 0UL;

	pm->vsi_ZONE = ZONE;

	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		uQ = (1.f / 3.f) * (xA + xB + xC);
		uA = (xA - uQ) * pm->const_lpf_U / pm->dc_resolution;
		uB = (xB - uQ) * pm->const_lpf_U / pm->dc_resolution;

		pm->vsi_X = uA;
		pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;
	}
	else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

		uA = (xA - xC) * pm->const_lpf_U / pm->dc_resolution;
		uB = (xB - xC) * pm->const_lpf_U / pm->dc_resolution;

		pm->vsi_X = uA;
		pm->vsi_Y = uB;
	}
}

static void
pm_current_control(pmc_t *pm)
{
	float		*X = pm->lu_X, F[2];
	float		sD, sQ, eD, eQ;
	float		uD, uQ, uX, uY, wP;
	float		iMAX, uMAX, temp;

	if (pm->lu_mode == PM_LU_FORCED) {

		sD = pm->forced_hold_D;
		sQ = 0.f;
	}
	else {
		sD = pm->i_setpoint_D;
		sQ = pm->i_setpoint_Q;
	}

	iMAX = (pm->i_maximal < pm->i_derated) ? pm->i_maximal : pm->i_derated;

	sD = (sD > iMAX) ? iMAX : (sD < - iMAX) ? - iMAX : sD;
	sQ = (sQ > iMAX) ? iMAX : (sQ < - iMAX) ? - iMAX : sQ;

	m_rotf(F, - X[4] * pm->dT * .5f, X + 2);

	uD = F[0] * pm->vsi_X + F[1] * pm->vsi_Y;
	uQ = F[0] * pm->vsi_Y - F[1] * pm->vsi_X;

	wP = 1.5f * (X[0] * uD + X[1] * uQ);
	pm->vsi_lpf_watt += (wP - pm->vsi_lpf_watt) * pm->vsi_gain_LW;

	pm->vsi_lpf_D += (uD - pm->vsi_lpf_D) * pm->vsi_gain_LP;
	pm->vsi_lpf_Q += (uQ - pm->vsi_lpf_Q) * pm->vsi_gain_LP;

	wP = 1.5f * (sD * pm->vsi_lpf_D + sQ * pm->vsi_lpf_Q);

	if (wP > pm->i_watt_consumption_maximal) {

		temp = pm->i_watt_consumption_maximal / wP;
		sD *= temp;
		sQ *= temp;
	}
	else if (wP < pm->i_watt_regeneration_maximal) {

		temp = pm->i_watt_regeneration_maximal / wP;
		sD *= temp;
		sQ *= temp;
	}

	/* Obtain discrepancy.
	 * */
	eD = sD - X[0];
	eQ = sQ - X[1];

	if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		temp = 2.f * M_PI_F * pm->hfi_freq_hz;
		m_rotf(pm->hfi_CS, temp * pm->dT, pm->hfi_CS);

		eD += pm->hfi_CS[1] * pm->hfi_swing_D;
	}

	uD = pm->i_gain_PD * eD;
	uQ = pm->i_gain_PQ * eQ;

	uMAX = (2.f / 3.f) * pm->const_lpf_U;

	pm->i_integral_D += pm->i_gain_ID * eD;
	pm->i_integral_D = (pm->i_integral_D > uMAX) ? uMAX :
		(pm->i_integral_D < - uMAX) ? - uMAX : pm->i_integral_D;
	uD += pm->i_integral_D;

	pm->i_integral_Q += pm->i_gain_IQ * eQ;
	pm->i_integral_Q = (pm->i_integral_Q > uMAX) ? uMAX :
		(pm->i_integral_Q < - uMAX) ? - uMAX : pm->i_integral_Q;
	uQ += pm->i_integral_Q;

	if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		uD += pm->hfi_CS[0] * pm->hfi_swing_D * temp * pm->const_Ld;
	}

	uX = X[2] * uD - X[3] * uQ;
	uY = X[3] * uD + X[2] * uQ;

	pm_voltage_control(pm, uX, uY);
}

static void
pm_speed_control(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		iSP, wSP, D, dS;

	wSP = pm->s_setpoint;
	wSP = (wSP < - pm->s_maximal) ? - pm->s_maximal :
		(wSP > pm->s_maximal) ? pm->s_maximal : wSP;

	if (1) {

		dS = pm->s_accel * pm->dT;
		pm->s_track = (pm->s_track < wSP - dS) ? pm->s_track + dS
			: (pm->s_track > wSP + dS) ? pm->s_track - dS : wSP;

		if (pm->lu_mode == PM_LU_FORCED) {

			pm->forced_setpoint = pm->s_track;
		}
		else {
			/* Obtain discrepancy.
			 * */
			D = pm->s_track - X[4];

			iSP = pm->s_gain_P * D;

			pm->s_integral += (X[1] - pm->s_integral) * pm->s_gain_I;
			iSP += pm->s_integral;

			pm->i_setpoint_D = 0.f;
			pm->i_setpoint_Q = iSP;
		}
	}
}

static void
pm_position_control(pmc_t *pm)
{
/*	float		dX, dY, eP;

	m_rotf(pm->p_set_point, pm->p_set_point_s * pm->dT, pm->p_set_point);

	if (pm->lu_X[2] < 0.f) {

		if (pm->lu_X[3] < 0.f) {

			if (pm->lu_temp[0] >= 0.f)
				pm->lu_revol += 1;
		}
		else {
			if (pm->lu_temp[0] < 0.f)
				pm->lu_revol -= 1;
		}
	}

	if (pm->p_set_point[0] < 0.f) {

		if (pm->p_set_point[1] < 0.f) {

			if (pm->lu_temp[1] >= 0.f)
				pm->p_set_point_revol += 1;
		}
		else {
			if (pm->lu_temp[1] < 0.f)
				pm->p_set_point_revol -= 1;
		}
	}

	pm->lu_temp[0] = pm->lu_X[3];
	pm->lu_temp[1] = pm->p_set_point[1];
*/
	/* Obtain discrepancy.
	 * */
/*	dX = pm->p_set_point[0] * pm->lu_X[2] +
		pm->p_set_point[1] * pm->lu_X[3];
	dY = pm->p_set_point[1] * pm->lu_X[2] -
		pm->p_set_point[0] * pm->lu_X[3];

	eP = atan2f(dY, dX);

	if (dY < 0.f) {

		if (pm->lu_X[3] < 0.f && pm->p_set_point[1] >= 0.f)
			eP += 2.f * M_PI_F;
	}
	else {
		if (pm->lu_X[3] >= 0.f && pm->p_set_point[1] < 0.f)
			eP -= 2.f * M_PI_F;
	}

	eP += (pm->p_set_point_revol - pm->lu_revol) * 2.f * M_PI_F;

	pm->s_set_point = pm->p_gain_P * eP;*/
}

void pm_feedback(pmc_t *pm, pmfb_t *fb)
{
	float		A, B, U;

	A = pm->adjust_IA[1] * fb->current_A + pm->adjust_IA[0];
	B = pm->adjust_IB[1] * fb->current_B + pm->adjust_IB[0];

	A = (A < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
		(A > pm->fb_current_clamp) ? pm->fb_current_clamp : A;
	B = (B < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
		(B > pm->fb_current_clamp) ? pm->fb_current_clamp : B;

	pm->fb_current_A = A;
	pm->fb_current_B = B;

	U = pm->adjust_US[1] * fb->voltage_U + pm->adjust_US[0];
	pm->const_lpf_U += (U - pm->const_lpf_U) * pm->const_gain_LP;

	if (PM_CONFIG_TVSE(pm) == PM_ENABLED) {

		pm->fb_voltage_A = pm->adjust_UA[1] * fb->voltage_A + pm->adjust_UA[0];
		pm->fb_voltage_B = pm->adjust_UB[1] * fb->voltage_B + pm->adjust_UB[0];
		pm->fb_voltage_C = pm->adjust_UC[1] * fb->voltage_C + pm->adjust_UC[0];
	}

	pm_FSM(pm);

	if (pm->lu_mode != PM_LU_DISABLED) {

		pm_lu_FSM(pm);

		if (pm->lu_mode != PM_LU_DETACHED) {

			pm_current_control(pm);

			if (pm->config_LOOP == PM_LOOP_DRIVE_SPEED) {

				pm_speed_control(pm);
			}
		}

		pm_lu_validate(pm);
	}
}

