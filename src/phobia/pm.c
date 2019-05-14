#include "libm.h"
#include "pm.h"

void pm_default(pmc_t *pm)
{
	pm->dc_minimal = 21;
	pm->dc_clearance = 420;

	pm->config_ABC = PM_ABC_THREE_PHASE;
	pm->config_LDQ = PM_LDQ_SATURATION_SALIENCY;
	pm->config_VM = PM_ENABLED;
	pm->config_HALL = PM_DISABLED;
	pm->config_HFI = PM_DISABLED;
	pm->config_LOOP = PM_LOOP_DRIVE_SPEED;

	pm->tm_transient_slow = .05f;
	pm->tm_transient_fast = .002f;
	pm->tm_voltage_hold = .05f;
	pm->tm_current_hold = .5f;
	pm->tm_instant_probe = .01f;
	pm->tm_average_drift = .1f;
	pm->tm_average_probe = .5f;
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

	pm->probe_current_hold_X = 20.f;
	pm->probe_current_hold_Y = 0.f;
	pm->probe_current_sine = 5.f;
	pm->probe_freq_sine_hz = pm->freq_hz / 24.f;
	pm->probe_speed_low = 700.f;
	pm->probe_speed_ramp = 1700.f;
	pm->probe_gain_P = 1E-2f;
	pm->probe_gain_I = 1E-3f;

	pm->fault_voltage_tolerance = 2.f;
	pm->fault_current_tolerance = 2.f;
	pm->fault_voltage_halt_level = 59.f;
	pm->fault_current_halt_level = 50.f;
	pm->fault_adjust_tolerance = 1E-1f;
	pm->fault_flux_residue_maximal = 90.f;

	pm->vsi_clamp_to_GND = PM_ENABLED;

	pm->vm_maximal = .16f;
	pm->vm_FIR_A[0] = 0.f;
	pm->vm_FIR_A[1] = 0.f;
	pm->vm_FIR_A[2] = 0.f;
	pm->vm_FIR_B[0] = 0.f;
	pm->vm_FIR_B[1] = 0.f;
	pm->vm_FIR_B[2] = 0.f;
	pm->vm_FIR_C[0] = 0.f;
	pm->vm_FIR_C[1] = 0.f;
	pm->vm_FIR_C[2] = 0.f;

	pm->forced_hold_D = 10.f;
	pm->forced_maximal = 700.f;
	pm->forced_accel = 500.f;

	pm->flux_bemf_unlock = .2f;
	pm->flux_bemf_lock = .3f;
	pm->flux_bemf_belief = 1.f;
	pm->flux_bemf_reject = 11.f;
	pm->flux_gain_LP_E = 1E-1f;
	pm->flux_gain_DA = 5E-1f;
	pm->flux_gain_QA = 5E-1f;
	pm->flux_gain_DP = 5E-3f;
	pm->flux_gain_DS = 1E+1f;
	pm->flux_gain_QS = 1E+1f;
	pm->flux_gain_QZ = 1E-2f;

	pm->adapt_current_belief = 5.f;
	pm->adapt_R_lower = 0.f;
	pm->adapt_R_upper = 0.f;
	pm->adapt_gain_LP_T = 1E-3f;

	pm->hfi_freq_hz = pm->freq_hz / 6.f;
	pm->hfi_swing_D = 2.f;
	pm->hfi_derated = PM_INFINITY;
	pm->hfi_gain_P = 5E-2f;
	pm->hfi_gain_S = 7E+1f;
	pm->hfi_gain_F = 2E-3f;

	pm->const_gain_LP_U = 5E-1f;
	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Ld = 0.f;
	pm->const_Lq = 0.f;
	pm->const_Zp = 1;
	pm->const_J = 0.f;

	pm->i_maximal = pm->fb_current_clamp - 1.f;
	pm->i_gain_PD = 2E-1f;
	pm->i_gain_ID = 5E-3f;
	pm->i_gain_PQ = 2E-1f;
	pm->i_gain_IQ = 5E-3f;

	pm->watt_maximal = 2000.f;
	pm->watt_reverse = - 500.f;
	pm->watt_tolerance = 1.f;
	pm->watt_gain_LP_V = 1E-1f;
	pm->watt_gain_LP_P = 5E-2f;

	pm->lpfu_maximal = 52.f;
	pm->lpfu_gain_PU = 5E-1f;
	pm->lpfu_gain_LP_Q = 5E-2f;

	pm->s_maximal = pm->freq_hz * (2.f * M_PI_F / 12.f);
	pm->s_accel = 10000.f;
	pm->s_gain_P = 5E-2f;
	pm->s_gain_LP_I = 1E-3f;

	pm->x_near_distance = 5.f;
	pm->x_gain_P = 70.f;
	pm->x_gain_NP = 50.f;
}

static void
pm_equation_2(pmc_t *pm, float Y[2], const float X[5])
{
	float		uD, uQ, X4, X5, R1, E1, fluxD, fluxQ;

	uD = X[2] * pm->vsi_X + X[3] * pm->vsi_Y;
	uQ = X[2] * pm->vsi_Y - X[3] * pm->vsi_X;

	X4 = X[4];
	X5 = pm->flux_drift_Q;
	R1 = pm->const_R + pm->adapt_lpf_R;
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
	float		uX, uY, uD, uQ;

	/* First-order ODE solver.
	 * */

	uX = pm->vm_DX - pm->vsi_DX;
	uY = pm->vm_DY - pm->vsi_DY;

	uD = X[2] * uX + X[3] * uY;
	uQ = X[2] * uY - X[3] * uX;

	X[0] += uD * pm->dT / pm->const_Ld;
	X[1] += uQ * pm->dT / pm->const_Lq;
}

static void
pm_forced_update(pmc_t *pm)
{
	float		*X = pm->forced_X;
	float		iX, iY, wSP, dS, tR, wMAX;

	/* In the forced control we apply a current vector without feedback
	 * to force rotor turn.
	 * */

	iX = pm->flux_X[2] * pm->flux_X[0] - pm->flux_X[3] * pm->flux_X[1];
	iY = pm->flux_X[3] * pm->flux_X[0] + pm->flux_X[2] * pm->flux_X[1];

	/* Get current from FLUX observer.
	 * */
	X[0] = X[2] * iX + X[3] * iY;
	X[1] = X[2] * iY - X[3] * iX;

	/* Get the setpoint of speed.
	 * */
	if (pm->config_LOOP == PM_LOOP_DRIVE_CURRENT) {

		wSP = (pm->i_setpoint_Q < 0.f) ? - pm->forced_maximal : pm->forced_maximal;
	}
	else if (pm->config_LOOP == PM_LOOP_DRIVE_SPEED) {

		wSP = pm->s_track;
	}
	else if (pm->config_LOOP == PM_LOOP_DRIVE_SERVO) {

		wSP = pm->s_track;
	}
	else {
		/* The function should not be called in this case.
		 * */
		wSP = 0.f;
	}

	/* Maximal forced speed constraint.
	 * */
	wMAX = pm->forced_maximal;
	wSP = (wSP < - wMAX) ? - wMAX : (wSP > wMAX) ? wMAX : wSP;

	/* Update the actual speed with specified acceleration.
	 * */
	dS = pm->forced_accel * pm->dT;
	X[4] = (X[4] < wSP - dS) ? X[4] + dS : (X[4] > wSP + dS) ? X[4] - dS : wSP;

	/* Update DQ frame.
	 * */
	m_rotf(X + 2, X[4] * pm->dT, X + 2);

	/* Thermal drift resistance adaptation.
	 * */
	if (pm->forced_hold_D > pm->adapt_current_belief) {

		tR = pm->watt_lpf_D / pm->forced_hold_D - pm->const_R;
		pm->adapt_lpf_R += (tR - pm->adapt_lpf_R) * pm->adapt_gain_LP_T;
	}
}

static void
pm_flux_residue(pmc_t *pm, float X[5])
{
	float		iA, iB, iX, iY;
	int		vA, vB;

	/* Get our current estimate on XY.
	 * */
	iX = X[2] * X[0] - X[3] * X[1];
	iY = X[3] * X[0] + X[2] * X[1];

	/* Check for measurement is undistorted.
	 * */
	vA = (pm->vsi_current_ZONE & 0x11UL) ? 0 : 1;
	vB = (pm->vsi_current_ZONE & 0x22UL) ? 0 : 1;

	/* Check for measurement is in range.
	 * */
	vA = (pm->fb_current_A < - pm->fb_current_clamp) ? 0 :
		(pm->fb_current_A > pm->fb_current_clamp) ? 0 : vA;
	vB = (pm->fb_current_B < - pm->fb_current_clamp) ? 0 :
		(pm->fb_current_B > pm->fb_current_clamp) ? 0 : vB;

	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		iA = iX;
		iB = - .5f * iX + .8660254f * iY;

		vA = (iA < - pm->fb_current_clamp) ? 0 : (iA > pm->fb_current_clamp) ? 0 : vA;
		vB = (iB < - pm->fb_current_clamp) ? 0 : (iB > pm->fb_current_clamp) ? 0 : vB;

		/* Get residue.
		 * */
		iA = pm->fb_current_A - iA;
		iB = pm->fb_current_B - iB;

		if (vA && vB) {

			/* Current sensors A and B are valid.
			 * */

			iX = iA;
			iY = .57735027f * iA + 1.1547005f * iB;
		}
		else if (vA) {

			/* Only current sensor A is valid.
			 * */

			iX = iA;
			iY = 0.f;
		}
		else if (vB) {

			/* Only current sensor B is valid.
			 * */

			iX = - .5f * iB;
			iY = .8660254f * iB;
		}
		else {
			/* No valid current sensor.
			 * */

			iX = 0.f;
			iY = 0.f;
		}
	}
	else {
		vA = (iX < - pm->fb_current_clamp) ? 0 : (iX > pm->fb_current_clamp) ? 0 : vA;
		vB = (iY < - pm->fb_current_clamp) ? 0 : (iY > pm->fb_current_clamp) ? 0 : vB;

		iX = pm->fb_current_A - iX;
		iY = pm->fb_current_B - iY;

		iX = (vA) ? iX : 0.f;
		iY = (vB) ? iY : 0.f;
	}

	/* Get residue on DQ.
	 * */
	pm->flux_residue_D = X[2] * iX + X[3] * iY;
	pm->flux_residue_Q = X[2] * iY - X[3] * iX;
}

static void
pm_flux_update(pmc_t *pm)
{
	float		*X = pm->flux_X;
	float		eD, eQ, eR, dR, qS;

	if (pm->lu_mode != PM_LU_DETACHED) {

		if (PM_CONFIG_VM(pm) == PM_ENABLED) {

			/* Update voltage from previous cycle.
			 * */
			pm_solve_1(pm, pm->flux_X);
		}

		/* Get residue.
		 * */
		pm_flux_residue(pm, X);
	}
	else {
		/* We have exactly zero current in detached mode.
		 * */
		pm->flux_residue_D = 0.f - X[0];
		pm->flux_residue_Q = 0.f - X[1];
	}

	eD = pm->flux_residue_D;
	eQ = pm->flux_residue_Q;

	pm->flux_residue_lpf += (eD * eD + eQ * eQ - pm->flux_residue_lpf)
		* pm->flux_gain_LP_E;

	/* Update current estimate.
	 * */
	X[0] += pm->flux_gain_DA * eD;
	X[1] += pm->flux_gain_QA * eQ;

	if (pm->const_E != 0.f) {

		eR = (X[4] < 0.f) ? - eD : eD;

		/* Update DQ frame estimate with D residue.
		 * */
		dR = pm->flux_gain_DP * eR;
		dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
		m_rotf(X + 2, dR, X + 2);

		qS = m_fabsf(X[4] * pm->const_E) / pm->flux_bemf_reject;
		qS = (qS > 1.f) ? 1.f : qS;

		/* We prefer the Q residue at low speed as it gives a fast
		 * response. At high speed it may inject a ripple if BEMF is
		 * not pure sinusoidal so we suppress it and more and more go
		 * to D residue.
		 * */
		dR = pm->flux_gain_DS * qS * eR - pm->flux_gain_QS * (1.f - qS) * eQ;
		X[4] += dR;

		if (m_fabsf(X[4] * pm->const_E) > pm->flux_bemf_belief) {

			/* Estimate Q drift at high speed to get relaxed solution.
			 * */
			pm->flux_drift_Q += pm->flux_gain_QZ * (eR + eQ);
		}
		else {
			/* At low speed Q drift cannot be estimated.
			 * */
			pm->flux_drift_Q = 0.f;
		}
	}
	else if (pm->lu_mode == PM_LU_FORCED) {

		/* Get the forced state vector as we do not have own estimate.
		 * */
		X[2] = pm->forced_X[2];
		X[3] = pm->forced_X[3];
		X[4] = pm->forced_X[4];

		/* Estimate Q drift to be able to get E constant.
		 * */
		pm->flux_drift_Q += pm->flux_gain_QZ * eQ;
	}

	/* We must always be sure that adaptation resistance is within
	 * permissible range.
	 * */
	pm->adapt_lpf_R = (pm->adapt_lpf_R < pm->adapt_R_lower) ? pm->adapt_R_lower :
		(pm->adapt_lpf_R > pm->adapt_R_upper) ? pm->adapt_R_upper : pm->adapt_lpf_R;

	/* Time update to next cycle.
	 * */
	pm_solve_2(pm, X);
}

static void
pm_hall_update(pmc_t *pm)
{
	/* TODO */
}

static void
pm_hfi_update(pmc_t *pm)
{
	float		*X = pm->hfi_X;
	float		eD, eQ, eR, dR;

	pm_flux_residue(pm, X);

	eD = pm->flux_residue_D;
	eQ = pm->flux_residue_Q;

	X[0] += pm->flux_gain_DA * eD;
	X[1] += pm->flux_gain_QA * eQ;

	/* We demodulate the Q residue with carrier sine wave.
	 * */
	eR = eQ * pm->hfi_wave[1];
	dR = pm->hfi_gain_P * eR;
	dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
	m_rotf(X + 2, dR, X + 2);

	X[4] += pm->hfi_gain_S * eR * m_fabsf(eR);

	if (pm->config_LDQ == PM_LDQ_SATURATION_SALIENCY) {

		/* D axis responce has an asymmetry that we exctact with
		 * doubled frequency cosine.
		 * */
		dR = pm->hfi_wave[0] * pm->hfi_wave[0] - pm->hfi_wave[1] * pm->hfi_wave[1];
		pm->hfi_flux += pm->hfi_gain_F * eD * dR;

		if (pm->hfi_flux > 1.f) {

			/* Flip into the true position.
			 * */
			X[2] = - X[2];
			X[3] = - X[3];

			pm->hfi_flux = 0.f;
		}
		else if (pm->hfi_flux < 0.f) {

			pm->hfi_flux = 0.f;
		}
	}

	pm_solve_2(pm, X);
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
	else {
		uA = pm->fb_voltage_A - pm->fb_voltage_C;
		uB = pm->fb_voltage_B - pm->fb_voltage_C;

		pm->vsi_X = uA;
		pm->vsi_Y = uB;
	}
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

		pm_flux_update(pm);
		pm_forced_update(pm);

		X[0] = pm->forced_X[0];
		X[1] = pm->forced_X[1];
		X[2] = pm->forced_X[2];
		X[3] = pm->forced_X[3];
		X[4] = pm->forced_X[4];

		if ((pm->forced_hold_D == 0.f || m_fabsf(X[4] * pm->const_E) > pm->flux_bemf_lock)
				&& m_fabsf(pm->flux_X[4] * pm->const_E) > pm->flux_bemf_lock) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;
		}
	}
	else if (pm->lu_mode == PM_LU_ESTIMATE_FLUX) {

		pm_flux_update(pm);

		X[0] = pm->flux_X[0];
		X[1] = pm->flux_X[1];
		X[2] = pm->flux_X[2];
		X[3] = pm->flux_X[3];
		X[4] = pm->flux_X[4];

		if (m_fabsf(X[4] * pm->const_E) < pm->flux_bemf_unlock) {

			if (pm->config_LOOP == PM_LOOP_RECTIFIER_VOLTAGE) {

				/* Stay on FLUX observer.
				 * */
			}
			else if (pm->config_HALL == PM_ENABLED) {

				pm->lu_mode = PM_LU_SENSORED_HALL;
			}
			else if (pm->config_HFI == PM_ENABLED) {

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
	else if (pm->lu_mode == PM_LU_SENSORED_HALL) {

		pm_hall_update(pm);
	}
	else if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		pm_flux_update(pm);
		pm_hfi_update(pm);

		X[0] = pm->hfi_X[0];
		X[1] = pm->hfi_X[1];
		X[2] = pm->hfi_X[2];
		X[3] = pm->hfi_X[3];
		X[4] = pm->hfi_X[4];

		if (		m_fabsf(X[4] * pm->const_E) > pm->flux_bemf_lock
				&& m_fabsf(pm->flux_X[4] * pm->const_E) > pm->flux_bemf_lock) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;
		}
	}
}

static void
pm_lu_validate(pmc_t *pm)
{
	float		*X = pm->lu_X;

	if (pm->const_lpf_U > pm->fault_voltage_halt_level) {

		pm->fail_reason = PM_ERROR_DC_LINK_OVER_VOLTAGE;
		pm_fsm_req(pm, PM_STATE_HALT);
	}

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
	else {
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

	if (pm->vsi_precise_MODE == PM_ENABLED) {

		uQ = 0.f - uMIN;
	}
	else if (pm->vsi_clamp_to_GND == PM_ENABLED) {

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

	if (pm->vsi_precise_MODE == PM_ENABLED) {

		xA += pm->dc_minimal;
		xB += pm->dc_minimal;
		xC += pm->dc_minimal;
	}

	xMIN = pm->dc_minimal;
	xMAX = pm->dc_resolution - pm->dc_minimal;

	xA = (xA < xMIN) ? 0 : (xA > xMAX) ? pm->dc_resolution : xA;
	xB = (xB < xMIN) ? 0 : (xB > xMAX) ? pm->dc_resolution : xB;
	xC = (xC < xMIN) ? 0 : (xC > xMAX) ? pm->dc_resolution : xC;

	pm->proc_set_DC(xA, xB, xC);

	xMAX = pm->dc_resolution;
	xMIN = pm->dc_resolution - pm->dc_clearance;

	ZONE = (pm->vsi_current_ZONE & 0xFUL) << 4;

	/* We check if there are PWM edges within clearance zone. The CURRENT
	 * measurement will be used or rejected based on these flags.
	 * */
	ZONE |= (xA > xMIN) ? (xA < xMAX) ? (1UL << 0) : (1UL << 3) : 0UL;
	ZONE |= (xB > xMIN) ? (xB < xMAX) ? (1UL << 1) : (1UL << 3) : 0UL;
	ZONE |= (xC > xMIN) ? (xC < xMAX) ? (1UL << 2) : (1UL << 3) : 0UL;

	pm->vsi_current_ZONE = ZONE;

	if (PM_CONFIG_VM(pm) == PM_ENABLED) {

		xMAX = (int) (pm->dc_resolution * pm->vm_maximal);
		ZONE = (pm->vsi_voltage_ZONE & 0xFUL) << 4;

		/* We check if voltages are within acceptable zone. The VOLTAGE
		 * measurement will be used or rejected based on these flags.
		 * */
		ZONE |= (xA != 0) ? (xA < xMAX) ? (1UL << 0) : (1UL << 3) : 0UL;
		ZONE |= (xB != 0) ? (xB < xMAX) ? (1UL << 1) : (1UL << 3) : 0UL;
		ZONE |= (xC != 0) ? (xC < xMAX) ? (1UL << 2) : (1UL << 3) : 0UL;

		pm->vsi_voltage_ZONE = ZONE;
	}

	pm->vsi_DX = pm->vsi_X;
	pm->vsi_DY = pm->vsi_Y;

	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		uQ = (1.f / 3.f) * (xA + xB + xC);
		uA = (xA - uQ) * pm->const_lpf_U / pm->dc_resolution;
		uB = (xB - uQ) * pm->const_lpf_U / pm->dc_resolution;

		pm->vsi_X = uA;
		pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;
	}
	else {
		uA = (xA - xC) * pm->const_lpf_U / pm->dc_resolution;
		uB = (xB - xC) * pm->const_lpf_U / pm->dc_resolution;

		pm->vsi_X = uA;
		pm->vsi_Y = uB;
	}
}

static void
pm_current_control(pmc_t *pm)
{
	float		*X = pm->lu_X, mF[2];
	float		sD, sQ, eD, eQ, qE;
	float		uD, uQ, uX, uY, wP, wS;
	float		iMAX, uMAX, wMAX, wREV;

	if (pm->lu_mode == PM_LU_FORCED) {

		sD = pm->forced_hold_D;
		sQ = 0.f;
	}
	else {
		sD = pm->i_setpoint_D;
		sQ = pm->i_setpoint_Q;
	}

	/* Maximal current constraint.
	 * */
	iMAX = (pm->i_maximal < pm->i_derated) ? pm->i_maximal : pm->i_derated;

	if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		iMAX = (pm->hfi_derated < iMAX) ? pm->hfi_derated : iMAX;
	}

	sD = (sD > iMAX) ? iMAX : (sD < - iMAX) ? - iMAX : sD;
	sQ = (sQ > iMAX) ? iMAX : (sQ < - iMAX) ? - iMAX : sQ;

	/* Get the DQ frame in which a voltage has been obtained.
	 * */
	wS = - 1.5f * X[4] * pm->dT;
	m_rotf(mF, wS, X + 2);

	uD = mF[0] * pm->vm_DX + mF[1] * pm->vm_DY;
	uQ = mF[0] * pm->vm_DY - mF[1] * pm->vm_DX;

	pm->watt_lpf_D += (uD - pm->watt_lpf_D) * pm->watt_gain_LP_V;
	pm->watt_lpf_Q += (uQ - pm->watt_lpf_Q) * pm->watt_gain_LP_V;

	/* Operating POWER is a scalar product of voltage and current.
	 * */
	wP = PM_KWAT(pm) * (X[0] * pm->watt_lpf_D + X[1] * pm->watt_lpf_Q);
	pm->watt_lpf_VA += (wP - pm->watt_lpf_VA) * pm->watt_gain_LP_P;

	wP = PM_KWAT(pm) * (sD * pm->watt_lpf_D + sQ * pm->watt_lpf_Q);

	wMAX = (pm->watt_derated_1 < pm->watt_derated_2)
		? pm->watt_derated_1 : pm->watt_derated_2;
	wMAX = (pm->watt_maximal < wMAX) ? pm->watt_maximal : wMAX;
	wREV = pm->watt_reverse;

	if (pm->const_lpf_U > pm->lpfu_maximal) {

		/* Derate reverse limit to prevent DC link overvoltage.
		 * */
		qE = 1.f - (pm->const_lpf_U - pm->lpfu_maximal) * pm->lpfu_gain_PU;
		qE = (qE < 0.f) ? 0.f : qE;

		pm->lpfu_integral += (qE - pm->lpfu_integral) * pm->lpfu_gain_LP_Q;
		wREV *= pm->lpfu_integral * qE;
	}

	/* There is hysteresis near zero.
	 * */
	if (m_fabsf(wP) < pm->watt_tolerance) {

		wP = pm->watt_tolerance * pm->watt_locked_1;
	}
	else {
		pm->watt_locked_1 = (wP < 0.f) ? - 1 : 1;
	}

	/* Apply POWER constraints.
	 * */
	if (wP > wMAX) {

		wP = wMAX / wP;
		sD *= wP;
		sQ *= wP;
	}
	else if (wP < wREV) {

		wP = wREV / wP;
		sD *= wP;
		sQ *= wP;
	}

	/* Obtain discrepancy.
	 * */
	eD = sD - X[0];
	eQ = sQ - X[1];

	if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		wS = 2.f * M_PI_F * pm->hfi_freq_hz;
		m_rotf(pm->hfi_wave, wS * pm->dT, pm->hfi_wave);

		eD += pm->hfi_wave[1] * pm->hfi_swing_D;
	}

	uD = pm->i_gain_PD * eD;
	uQ = pm->i_gain_PQ * eQ;

	uMAX = PM_UMAX(pm) * pm->const_lpf_U;

	pm->i_integral_D += pm->i_gain_ID * eD;
	pm->i_integral_D = (pm->i_integral_D > uMAX) ? uMAX :
		(pm->i_integral_D < - uMAX) ? - uMAX : pm->i_integral_D;
	uD += pm->i_integral_D;

	pm->i_integral_Q += pm->i_gain_IQ * eQ;
	pm->i_integral_Q = (pm->i_integral_Q > uMAX) ? uMAX :
		(pm->i_integral_Q < - uMAX) ? - uMAX : pm->i_integral_Q;
	uQ += pm->i_integral_Q;

	if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		uD += pm->hfi_wave[0] * pm->hfi_swing_D * wS * pm->const_Ld;
	}

	uX = X[2] * uD - X[3] * uQ;
	uY = X[3] * uD + X[2] * uQ;

	pm_voltage_control(pm, uX, uY);
}

static void
pm_speed_control(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		iSP, wSP, eS, dS;

	/* Maximal speed constraint.
	 * */
	wSP = pm->s_setpoint;
	wSP = (wSP < - pm->s_maximal) ? - pm->s_maximal :
		(wSP > pm->s_maximal) ? pm->s_maximal : wSP;

	/* Maximal acceleration constraint.
	 * */
	dS = pm->s_accel * pm->dT;
	pm->s_track = (pm->s_track < wSP - dS) ? pm->s_track + dS
		: (pm->s_track > wSP + dS) ? pm->s_track - dS : wSP;

	if (pm->lu_mode == PM_LU_FORCED) {

		/* Do nothins in this case */
	}
	else {
		/* Obtain discrepancy.
		 * */
		eS = pm->s_track - X[4];

		/* Here is P+LP regulator.
		 * */
		iSP = pm->s_gain_P * eS;

		if (m_fabsf(iSP) < pm->i_maximal) {

			pm->s_integral += (X[1] - pm->s_integral) * pm->s_gain_LP_I;
			iSP += pm->s_integral;
		}
		else {
			pm->s_integral = 0.f;
		}

		/* Update current loop setpoint. It would be possible here to
		 * use MTPA or something else.
		 * */
		pm->i_setpoint_Q = iSP;
	}
}

static void
pm_servo_control(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		eX, eY, eP, eP_abs, eS, gP;

	/* Full revolution counter.
	 * */
	if (X[2] < 0.f) {

		if (X[3] < 0.f) {

			pm->x_lu_revol += (pm->x_lu_sine >= 0.f) ? 1 : 0;
		}
		else {
			pm->x_lu_revol += (pm->x_lu_sine < 0.f) ? - 1 : 0;
		}
	}

	pm->x_lu_sine = X[3];

	/* Obtain discrepancy.
	 * */
	eX = pm->x_setpoint_DQ[0] * X[2] + pm->x_setpoint_DQ[1] * X[3];
	eY = pm->x_setpoint_DQ[1] * X[2] - pm->x_setpoint_DQ[0] * X[3];

	eP = m_atan2f(eY, eX);

	if (eY < 0.f) {

		if (X[3] < 0.f && pm->x_setpoint_DQ[1] >= 0.f)
			eP += 2.f * M_PI_F;
	}
	else {
		if (X[3] >= 0.f && pm->x_setpoint_DQ[1] < 0.f)
			eP -= 2.f * M_PI_F;
	}

	eP += (pm->x_setpoint_revol - pm->x_lu_revol) * 2.f * M_PI_F;
	eP_abs = m_fabsf(eP);

	/* Servo is based on constant acceleration formula.
	 * */
	eS = (eP < 0.f) ? - m_sqrtf(eP_abs) : m_sqrtf(eP_abs);
	gP = (eP_abs < pm->x_near_distance) ? eP_abs / pm->x_near_distance : 1.f;
	gP = pm->x_gain_NP + (pm->x_gain_P - pm->x_gain_NP) * gP;

	pm->s_setpoint = gP * eS;
}

void pm_feedback(pmc_t *pm, pmfb_t *fb)
{
	float		vA, vB, vC, U, Q;

	/* Get inline currents.
	 * */
	pm->fb_current_A = pm->adjust_IA[1] * fb->current_A + pm->adjust_IA[0];
	pm->fb_current_B = pm->adjust_IB[1] * fb->current_B + pm->adjust_IB[0];

	/* Get DC link voltage.
	 * */
	U = pm->adjust_US[1] * fb->voltage_U + pm->adjust_US[0];
	pm->const_lpf_U += (U - pm->const_lpf_U) * pm->const_gain_LP_U;

	if (PM_CONFIG_VM(pm) == PM_ENABLED) {

		vA = pm->vm_FIR_A[1] * pm->fb_voltage_A;
		vB = pm->vm_FIR_B[1] * pm->fb_voltage_B;
		vC = pm->vm_FIR_C[1] * pm->fb_voltage_C;

		/* Get terminal voltages.
		 * */
		pm->fb_voltage_A = pm->adjust_UA[1] * fb->voltage_A + pm->adjust_UA[0];
		pm->fb_voltage_B = pm->adjust_UB[1] * fb->voltage_B + pm->adjust_UB[0];
		pm->fb_voltage_C = pm->adjust_UC[1] * fb->voltage_C + pm->adjust_UC[0];

		if (pm->vm_FIR_A[0] != 0.f && (pm->vsi_voltage_ZONE & 0x88UL) == 0) {

			/* We extract actual terminal voltages using FIR filter.
			 * */

			if (pm->vsi_voltage_ZONE & 0x10UL) {

				vA += pm->vm_FIR_A[0] * pm->fb_voltage_A + pm->vm_FIR_A[2];
			}
			else {
				vA = 0.f;
			}

			if (pm->vsi_voltage_ZONE & 0x20UL) {

				vB += pm->vm_FIR_B[0] * pm->fb_voltage_B + pm->vm_FIR_B[2];
			}
			else {
				vB = 0.f;
			}

			if (pm->vsi_voltage_ZONE & 0x40UL) {

				vC += pm->vm_FIR_C[0] * pm->fb_voltage_C + pm->vm_FIR_C[2];
			}
			else {
				vC = 0.f;
			}

			pm->vm_A = vA;
			pm->vm_B = vB;
			pm->vm_C = vC;

			if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

				Q = (1.f / 3.f) * (vA + vB + vC);
				vA = vA - Q;
				vB = vB - Q;

				pm->vm_DX = vA;
				pm->vm_DY = .57735027f * vA + 1.1547005f * vB;
			}
			else {
				vA = vA - vC;
				vB = vB - vC;

				pm->vm_DX = vA;
				pm->vm_DY = vB;
			}
		}
		else {
			pm->vm_DX = pm->vsi_DX;
			pm->vm_DY = pm->vsi_DY;
		}
	}
	else {
		pm->vm_DX = pm->vsi_DX;
		pm->vm_DY = pm->vsi_DY;
	}

	/* Main FSM is used to execute external commands.
	 * */
	pm_FSM(pm);

	if (pm->lu_mode != PM_LU_DISABLED) {

		/* The observer FSM.
		 * */
		pm_lu_FSM(pm);

		if (pm->lu_mode != PM_LU_DETACHED) {

			if (pm->config_LOOP == PM_LOOP_DRIVE_SPEED) {

				pm_speed_control(pm);
			}
			else if (pm->config_LOOP == PM_LOOP_DRIVE_SERVO) {

				pm_servo_control(pm);
				pm_speed_control(pm);
			}

			/* Current loop is always enabled.
			 * */
			pm_current_control(pm);
		}

		pm_lu_validate(pm);
	}
}

