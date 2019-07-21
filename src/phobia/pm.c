#include "libm.h"
#include "pm.h"

void pm_default(pmc_t *pm)
{
	pm->dc_minimal = 21;
	pm->dc_clearance = 420;

	pm->config_NOP = PM_NOP_THREE_PHASE;
	pm->config_TVM = PM_DISABLED;
	pm->config_SENSOR = PM_SENSOR_DISABLED;
	pm->config_HFI = PM_DISABLED;
	pm->config_LOOP = PM_LOOP_DRIVE_SPEED;
	pm->config_WEAK = PM_DISABLED;
	pm->config_BRAKE = PM_DISABLED;
	pm->config_STAT	= PM_ENABLED;

	pm->tm_transient_slow = .05f;
	pm->tm_transient_fast = .002f;
	pm->tm_voltage_hold = .05f;
	pm->tm_current_hold = .5f;
	pm->tm_instant_probe = .01f;
	pm->tm_average_drift = .1f;
	pm->tm_average_probe = .5f;
	pm->tm_startup = .1f;

	pm->ad_IA[0] = 0.f;
	pm->ad_IA[1] = 1.f;
	pm->ad_IB[0] = 0.f;
	pm->ad_IB[1] = 1.f;
	pm->ad_US[0] = 0.f;
	pm->ad_US[1] = 1.f;
	pm->ad_UA[0] = 0.f;
	pm->ad_UA[1] = 1.f;
	pm->ad_UB[0] = 0.f;
	pm->ad_UB[1] = 1.f;
	pm->ad_UC[0] = 0.f;
	pm->ad_UC[1] = 1.f;

	pm->probe_current_hold = 10.f;
	pm->probe_current_bias_Q = 0.f;
	pm->probe_current_sine = 2.f;
	pm->probe_freq_sine_hz = pm->freq_hz / 24.f;
	pm->probe_speed_low = 170.f;
	pm->probe_speed_ramp = 500.f;
	pm->probe_gain_P = 1E-2f;
	pm->probe_gain_I = 1E-3f;

	pm->fault_voltage_tol = 2.f;
	pm->fault_current_tol = 2.f;
	pm->fault_accuracy_tol = 1E-1f;
	pm->fault_current_halt = 50.f;
	pm->fault_voltage_halt = 59.f;
	pm->fault_flux_lpfe_halt = 1.f;

	pm->tvm_range = .16f;
	pm->tvm_FIR_A[0] = 0.f;
	pm->tvm_FIR_A[1] = 0.f;
	pm->tvm_FIR_A[2] = 0.f;
	pm->tvm_FIR_B[0] = 0.f;
	pm->tvm_FIR_B[1] = 0.f;
	pm->tvm_FIR_B[2] = 0.f;
	pm->tvm_FIR_C[0] = 0.f;
	pm->tvm_FIR_C[1] = 0.f;
	pm->tvm_FIR_C[2] = 0.f;

	pm->lu_lock_S = .5f;
	pm->lu_unlock_S = .4f;
	pm->lu_gain_LP_S = 1E-1f;

	pm->forced_hold_D = 10.f;
	pm->forced_maximal = 200.f;
	pm->forced_accel = 400.f;

	pm->flux_N = PM_FLUX_MAX;
	pm->flux_lower_R = - .1f;
	pm->flux_upper_R = .4f;
	pm->flux_transient_S = 5.f;
	pm->flux_gain_IN = 5E-4f;
	pm->flux_gain_LO = 2E-5f;
	pm->flux_gain_HI = 3E-4f;
	pm->flux_gain_LP_E = 2E-5f;
	pm->flux_gain_SF = 5E-2f;

	pm->inject_bias_U = 1.f;
	pm->inject_ratio_D = .5f;

	pm->hfi_freq_hz = pm->freq_hz / 6.f;
	pm->hfi_swing_D = 1.f;
	pm->hfi_derated = 10.f;
	pm->hfi_gain_EP = 1E-1f;
	pm->hfi_gain_SF = 5E-3f;
	pm->hfi_gain_FP = 0E-3f;

	pm->const_gain_LP_U = 5E-1f;
	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_L = 0.f;
	pm->const_Zp = 1;
	pm->const_J = 0.f;

	pm->watt_wp_maximal = PM_INFINITY;
	pm->watt_ib_maximal = 50.f;
	pm->watt_wp_reverse = PM_INFINITY;
	pm->watt_ib_reverse = 50.f;
	pm->watt_derate_HI_U = 54.f;
	pm->watt_derate_LO_U = 5.f;
	pm->watt_derate_HI_S = pm->freq_hz * (2.f * M_PI_F / 18.f);
	pm->watt_gain_LP_F = 5E-2f;
	pm->watt_gain_LP_P = 5E-2f;
	pm->watt_gain_DU = 5E-1f;
	pm->watt_gain_DS = 5E-2f;
	pm->watt_gain_LP_I = 5E-2f;

	pm->i_maximal = 50.f;
	pm->i_brake = PM_INFINITY;
	pm->i_gain_P = 2E-1f;
	pm->i_gain_I = 5E-3f;

	pm->weak_maximal_D = 10.f;
	pm->weak_bias_U = 2.f;

	pm->s_maximal = pm->freq_hz * (2.f * M_PI_F / 18.f);
	pm->s_accel = 5000.f;
	pm->s_advance = PM_INFINITY;
	pm->s_gain_P = 5E-2f;
	pm->s_gain_LP_I = 2E-3f;
	pm->s_gain_HF_S = 5E-1f;

	pm->x_near_EP = 5.f;
	pm->x_gain_P = 70.f;
	pm->x_gain_N = 50.f;
}

static void
pm_forced_update(pmc_t *pm)
{
	float		wSP, dS;

	/* Get the setpoint of forced speed.
	 * */
	if (pm->config_LOOP == PM_LOOP_DRIVE_CURRENT) {

		wSP = (pm->i_setpoint_Q < 0.f) ? - PM_INFINITY : PM_INFINITY;
	}
	else if (pm->config_LOOP == PM_LOOP_DRIVE_SPEED) {

		wSP = pm->s_track;
	}
	else if (pm->config_LOOP == PM_LOOP_DRIVE_SERVO) {

		wSP = pm->s_track;
	}
	else {
		wSP = 0.f;
	}

	/* Maximal forced speed constraint.
	 * */
	wSP = (wSP < - pm->forced_maximal) ? - pm->forced_maximal :
		(wSP > pm->forced_maximal) ? pm->forced_maximal : wSP;

	if (pm->config_BRAKE == PM_ENABLED) {

		/* Make NO reverse.
		 * */
		wSP = (pm->s_brake_DIR * wSP < 0.f) ? 0.f : wSP;
	}

	/* Update actual speed with specified acceleration.
	 * */
	dS = pm->forced_accel * pm->dT;
	pm->forced_wS = (pm->forced_wS < wSP - dS) ? pm->forced_wS + dS :
		(pm->forced_wS > wSP + dS) ? pm->forced_wS - dS : wSP;

	if (pm->forced_hold_D > M_EPS_F) {

		/* Update DQ frame.
		 * */
		m_rotf(pm->forced_F, pm->forced_wS * pm->dT, pm->forced_F);
	}
}

static void
pm_flux_update(pmc_t *pm)
{
	float		EX, EY, UX, UY, LX, LY, IE, IQ, DX, DY, E, F;
	int		N, H;

	/* Get the actual voltage.
	 * */
	EX = pm->vsi_X - pm->const_R * pm->lu_iX;
	EY = pm->vsi_Y - pm->const_R * pm->lu_iY;

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		EX += pm->tvm_DX - pm->vsi_DX;
		EY += pm->tvm_DY - pm->vsi_DY;
	}

	/* Stator FLUX linear model.
	 * */
	LX = pm->const_L * pm->lu_iX;
	LY = pm->const_L * pm->lu_iY;

	if (pm->const_E > M_EPS_F) {

		UX = EX * pm->dT;
		UY = EY * pm->dT;

		IE = 1.f / pm->const_E;
		IQ = IE * IE;

		EX = pm->const_R * pm->lu_iX * pm->dT;
		EY = pm->const_R * pm->lu_iY * pm->dT;

		E = (pm->flux_lower_R - pm->flux_upper_R) / pm->flux_N;
		F = 0.f - pm->flux_lower_R;

		DX = EX * E;
		DY = EY * E;

		UX += EX * F;
		UY += EY * F;

		E = m_fabsf(pm->flux_wS * pm->const_E) / pm->flux_transient_S;
		E = (E > 1.f) ? 1.f : 0.f;

		/* Adaptive observer GAIN.
		 * */
		F = (pm->flux_gain_LO + E * pm->flux_gain_HI) * IE;

		for (N = 0, H = 0; N < pm->flux_N; N++) {

			/* This is multi-hypothesis FLUX observer.
			 * */
			pm->flux[N].X += UX;
			pm->flux[N].Y += UY;

			EX = pm->flux[N].X - LX;
			EY = pm->flux[N].Y - LY;

			E = 1.f - (EX * EX + EY * EY) * IQ;

			pm->flux[N].X += EX * E * F;
			pm->flux[N].Y += EY * E * F;

			/* Get H hypothesis with lowest residual.
			 * */
			pm->flux[N].lpf_E += (E * E - pm->flux[N].lpf_E) * pm->flux_gain_LP_E;
			H = (pm->flux[N].lpf_E < pm->flux[H].lpf_E) ? N : H;

			UX += DX;
			UY += DY;
		}

		/* Speed estimation using phase locked loop.
		 * */
		EX = pm->flux[pm->flux_H].X - LX;
		EY = pm->flux[pm->flux_H].Y - LY;

		m_rotf(pm->flux_F, pm->flux_wS * pm->dT, pm->flux_F);

		DX = EX * pm->flux_F[0] + EY * pm->flux_F[1];
		DY = EX * pm->flux_F[1] - EY * pm->flux_F[0];

		if (DX > (M_EPS_F * M_EPS_F)) {

			E = DY / DX * pm->freq_hz;
			pm->flux_wS += - E * pm->flux_gain_SF;
		}
	}
	else {
		/* This is startup observer.
		 * */
		H = pm->flux_H;

		pm->flux[H].X += EX * pm->dT;
		pm->flux[H].Y += EY * pm->dT;

		EX = pm->flux[H].X - LX;
		EY = pm->flux[H].Y - LY;

		pm->flux[H].X += - EX * pm->flux_gain_IN;
		pm->flux[H].Y += - EY * pm->flux_gain_IN;

		pm->flux_wS = pm->forced_wS;
	}

	/* Extract rotor position.
	 * */
	EX = pm->flux[H].X - LX;
	EY = pm->flux[H].Y - LY;

	E = m_sqrtf(EX * EX + EY * EY);

	pm->flux_E = E;
	pm->flux_H = H;

	if (E > M_EPS_F) {

		pm->flux_F[0] = EX / E;
		pm->flux_F[1] = EY / E;
	}

	pm->lu_lpf_wS += (pm->flux_wS - pm->lu_lpf_wS) * pm->lu_gain_LP_S;
}

static void
pm_hall_abc_update(pmc_t *pm)
{
	/* TODO */
}

static void
pm_hfi_update(pmc_t *pm)
{
	float		iD, iQ, uD, uQ, dTL;
	float		eD, eQ, eR, dR, wD;

	iD = pm->hfi_F[0] * pm->lu_iX + pm->hfi_F[1] * pm->lu_iY;
	iQ = pm->hfi_F[0] * pm->lu_iY - pm->hfi_F[1] * pm->lu_iX;

	eD = iD - pm->hfi_iD;
	eQ = iQ - pm->hfi_iQ;

	/* Demodulate the Q residue with carrier sine wave.
	 * */
	eR = eQ * pm->hfi_wave[1];
	dR = pm->hfi_gain_EP * eR;
	dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;

	m_rotf(pm->hfi_F, dR, pm->hfi_F);

	pm->hfi_wS += (dR * pm->freq_hz - pm->hfi_wS) * pm->hfi_gain_SF;

	if (m_fabsf(pm->hfi_gain_FP) > M_EPS_F) {

		/* D axis responce has an asymmetry that we exctact with
		 * doubled frequency cosine.
		 * */
		wD = pm->hfi_wave[0] * pm->hfi_wave[0] - pm->hfi_wave[1] * pm->hfi_wave[1];
		pm->hfi_polarity += pm->hfi_gain_FP * wD * eD;

		if (pm->hfi_polarity > 1.f) {

			/* Flip into the true position.
			 * */
			pm->hfi_F[0] = - pm->hfi_F[0];
			pm->hfi_F[1] = - pm->hfi_F[1];
			pm->hfi_polarity = 0.f;
		}
		else if (pm->hfi_polarity < 0.f) {

			pm->hfi_polarity = 0.f;
		}
	}

	uD = pm->hfi_F[0] * pm->vsi_X + pm->hfi_F[1] * pm->vsi_Y;
	uQ = pm->hfi_F[0] * pm->vsi_Y - pm->hfi_F[1] * pm->vsi_X;

	dTL = pm->dT / pm->const_L;

	/* Rough forecast to the next cycle.
	 * */
	pm->hfi_iD = iD + (uD - pm->const_R * iD) * dTL;
	pm->hfi_iQ = iQ + (uQ - pm->const_R * iQ) * dTL;
}

static void
pm_instant_take(pmc_t *pm)
{
	float			uA, uB, uQ;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		uQ = (1.f / 3.f) * (pm->fb_uA + pm->fb_uB + pm->fb_uC);
		uA = pm->fb_uA - uQ;
		uB = pm->fb_uB - uQ;

		pm->vsi_X = uA;
		pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;
	}
	else {
		uA = pm->fb_uA - pm->fb_uC;
		uB = pm->fb_uB - pm->fb_uC;

		pm->vsi_X = uA;
		pm->vsi_Y = uB;
	}
}

static void
pm_lu_FSM(pmc_t *pm)
{
	if (pm->lu_mode == PM_LU_DETACHED) {

		pm->lu_iX = 0.f;
		pm->lu_iY = 0.f;

		pm_instant_take(pm);
		pm_flux_update(pm);
	}
	else if (pm->lu_mode == PM_LU_FORCED) {

		pm_flux_update(pm);
		pm_forced_update(pm);

		pm->lu_F[0] = pm->forced_F[0];
		pm->lu_F[1] = pm->forced_F[1];
		pm->lu_wS = pm->forced_wS;

		if (m_fabsf(pm->lu_lpf_wS * pm->const_E) > pm->lu_lock_S) {

			if (pm->forced_hold_D > M_EPS_F) {

				if (m_fabsf(pm->forced_wS * pm->const_E) > pm->lu_lock_S) {

					pm->lu_mode = PM_LU_ESTIMATE_FLUX;
				}
			}
			else {
				pm->lu_mode = PM_LU_ESTIMATE_FLUX;
			}
		}
	}
	else if (pm->lu_mode == PM_LU_ESTIMATE_FLUX) {

		pm_flux_update(pm);

		pm->lu_F[0] = pm->flux_F[0];
		pm->lu_F[1] = pm->flux_F[1];
		pm->lu_wS = pm->flux_wS;

		if (m_fabsf(pm->lu_lpf_wS * pm->const_E) < pm->lu_unlock_S) {

			if (pm->config_LOOP == PM_LOOP_RECTIFIER_VOLTAGE) {

			}
			else if (pm->config_SENSOR == PM_SENSOR_HALL_ABC) {

				/*pm->lu_mode = PM_LU_SENSORED_HALL;*/
			}
			else if (pm->config_HFI == PM_ENABLED) {

				pm->lu_mode = PM_LU_ESTIMATE_HFI;

				pm->hfi_iD = pm->lu_iD;
				pm->hfi_iQ = pm->lu_iQ;
				pm->hfi_F[0] = pm->lu_F[0];
				pm->hfi_F[1] = pm->lu_F[1];
				pm->hfi_wS = pm->lu_wS;
			}
			else {
				pm->lu_mode = PM_LU_FORCED;

				pm->forced_F[0] = pm->flux_F[0];
				pm->forced_F[1] = pm->flux_F[1];
				pm->forced_wS = pm->flux_wS;
			}
		}
	}
	else if (pm->lu_mode == PM_LU_SENSORED_HALL_ABC) {

		pm_flux_update(pm);
		pm_hall_abc_update(pm);
	}
	else if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		pm_flux_update(pm);
		pm_hfi_update(pm);

		pm->lu_F[0] = pm->hfi_F[0];
		pm->lu_F[1] = pm->hfi_F[1];
		pm->lu_wS = pm->hfi_wS;

		if (m_fabsf(pm->lu_lpf_wS * pm->const_E) > pm->lu_lock_S) {

			if (m_fabsf(pm->hfi_wS * pm->const_E) > pm->lu_lock_S) {

				pm->lu_mode = PM_LU_ESTIMATE_FLUX;
			}
		}
	}

	pm->lu_iD = pm->lu_F[0] * pm->lu_iX + pm->lu_F[1] * pm->lu_iY;
	pm->lu_iQ = pm->lu_F[0] * pm->lu_iY - pm->lu_F[1] * pm->lu_iX;
}

void pm_voltage_control(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC;
	float		uMIN, uMAX, uQ;
	int		xA, xB, xC;
	int		xMIN, xMAX;

	uX /= pm->const_lpf_U;
	uY /= pm->const_lpf_U;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

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
	}

	uQ = 0.f - uMIN;

	uA += uQ;
	uB += uQ;
	uC += uQ;

	xA = (int) (pm->dc_resolution * uA);
	xB = (int) (pm->dc_resolution * uB);
	xC = (int) (pm->dc_resolution * uC);

	if (pm->lu_mode != PM_LU_DISABLED) {

		xMAX = pm->dc_resolution - pm->dc_clearance;

		if (xA > xMAX || xB > xMAX) {

			xMAX = (xA < xB) ? xB : xA;
			xMAX = (xMAX < xC) ? xC : xMAX;
			xMIN = pm->dc_resolution - xMAX;

			xA += xMIN;
			xB += xMIN;
			xC += xMIN;
		}

		xMIN = pm->dc_minimal;
		xMAX = pm->dc_resolution - pm->dc_minimal;

		xA = (xA < xMIN) ? 0 : (xA > xMAX) ? pm->dc_resolution : xA;
		xB = (xB < xMIN) ? 0 : (xB > xMAX) ? pm->dc_resolution : xB;
		xC = (xC < xMIN) ? 0 : (xC > xMAX) ? pm->dc_resolution : xC;
	}
	else {
		xA += pm->dc_minimal;
		xB += pm->dc_minimal;
		xC += pm->dc_minimal;

		xMAX = pm->dc_resolution - pm->dc_clearance;

		xA = (xA > xMAX) ? pm->dc_resolution : xA;
		xB = (xB > xMAX) ? pm->dc_resolution : xB;
		xC = (xC > xMAX) ? pm->dc_resolution : xC;
	}

	pm->proc_set_DC(xA, xB, xC);

	xMAX = pm->dc_resolution;
	xMIN = pm->dc_resolution - pm->dc_clearance;

	/* Check if there are PWM edges within clearance zone. The CURRENT
	 * measurement will be used or rejected based on this flag.
	 * */
	pm->vsi_IF = (pm->vsi_IF & 1) ? 2 : 0;
	pm->vsi_IF |= (xA > xMIN && xA < xMAX) ? 1 : 0;
	pm->vsi_IF |= (xB > xMIN && xB < xMAX) ? 1 : 0;

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		xMAX = (int) (pm->dc_resolution * pm->tvm_range);

		/* Check if voltages are within acceptable zone. The VOLTAGE
		 * measurement will be used or rejected based on these flags.
		 * */
		pm->vsi_UF = (pm->vsi_UF & 1) ? 2 : 0;
		pm->vsi_UF |= (xA > xMAX || xB > xMAX || xC > xMAX) ? 1 : 0;

		/* Check if voltages are exactly zero to get more accuracy.
		 * */
		pm->vsi_AZ = (pm->vsi_AZ & 1) ? 2 : 0;
		pm->vsi_AZ |= (xA == 0) ? 1 : 0;
		pm->vsi_BZ = (pm->vsi_BZ & 1) ? 2 : 0;
		pm->vsi_BZ |= (xB == 0) ? 1 : 0;
		pm->vsi_CZ = (pm->vsi_CZ & 1) ? 2 : 0;
		pm->vsi_CZ |= (xC == 0) ? 1 : 0;
	}

	pm->vsi_DX = pm->vsi_X;
	pm->vsi_DY = pm->vsi_Y;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

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

static float
pm_brake_control(pmc_t *pm, float bSP)
{
	float		iSP, eS;

	/* Obtain discrepancy.
	 * */
	eS = 0.f - pm->lu_wS;

	if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		/* Slow down in case of HFI mode.
		 * */
		eS *= pm->s_gain_HF_S;
	}

	/* Here is P+LP regulator.
	 * */
	iSP = pm->s_gain_P * eS;

	if (m_fabsf(iSP) < pm->i_maximal) {

		pm->s_integral += (pm->lu_iQ - pm->s_integral) * pm->s_gain_LP_I;
		iSP += pm->s_integral;
	}
	else {
		pm->s_integral = 0.f;
	}

	/* Output clamp.
	 * */
	iSP = (iSP < - bSP) ? - bSP : (iSP > bSP) ? bSP : iSP;

	return iSP;
}

static void
pm_current_control(pmc_t *pm)
{
	float		sD, sQ, eD, eQ, uD, uQ, uX, uY, E, Q;
	float		wP, iMAX, uMAX, wMAX, wREV, wS_abs;

	if (pm->lu_mode == PM_LU_FORCED) {

		sD = pm->forced_hold_D;
		sQ = 0.f;
	}
	else {
		sD = pm->i_setpoint_D;
		sQ = pm->i_setpoint_Q;

		if (pm->inject_ratio_D > M_EPS_F) {

			E = m_fabsf(pm->lu_wS) * pm->const_E - pm->inject_bias_U;
			Q = m_fabsf(pm->lu_iQ) * pm->const_R * pm->flux_upper_R;

			if (E < Q) {

				/* Inject D current to increase observability.
				 * */
				sD = pm->inject_ratio_D * m_fabsf(pm->lu_iQ);
			}
		}

		if (		pm->config_LOOP == PM_LOOP_DRIVE_CURRENT
				&& pm->config_BRAKE == PM_ENABLED) {

			if (pm->s_brake_DIR * sQ < 0.f) {

				sQ = m_fabsf(sQ);
				sQ = (sQ > pm->i_brake) ? pm->i_brake : sQ;

				/* Enable brake function.
				 * */
				sQ = pm_brake_control(pm, sQ);
			}
		}

		if (pm->config_WEAK == PM_ENABLED) {

			E = pm->lu_iQ * pm->const_R + pm->lu_wS * pm->const_E;
			E = PM_EMAX(pm) * pm->const_lpf_U - m_fabsf(E) - pm->weak_bias_U;

			if (E < 0.f) {

				/* Flux weakening.
				 * */
				sD = E / m_fabsf(pm->lu_wS * pm->const_L);
				sD = (sD < - pm->weak_maximal_D) ? - pm->weak_maximal_D : sD;
			}
		}
	}

	/* Get VSI voltages on DQ-axes.
	 * */
	uD = pm->lu_F[0] * pm->tvm_DX + pm->lu_F[1] * pm->tvm_DY;
	uQ = pm->lu_F[0] * pm->tvm_DY - pm->lu_F[1] * pm->tvm_DX;

	pm->watt_lpf_D += (uD - pm->watt_lpf_D) * pm->watt_gain_LP_F;
	pm->watt_lpf_Q += (uQ - pm->watt_lpf_Q) * pm->watt_gain_LP_F;

	/* Operating POWER is a scalar product of voltage and current.
	 * */
	wP = PM_KWAT(pm) * (pm->lu_iD * pm->watt_lpf_D + pm->lu_iQ * pm->watt_lpf_Q);
	pm->watt_lpf_wP += (wP - pm->watt_lpf_wP) * pm->watt_gain_LP_P;

	/* Maximal CURRENT constraint.
	 * */
	iMAX = (pm->i_maximal < pm->i_derated_1) ? pm->i_maximal : pm->i_derated_1;
	iMAX = (sQ * pm->watt_lpf_Q < 0.f && pm->i_brake < iMAX) ? pm->i_brake : iMAX;
	iMAX = (pm->lu_mode == PM_LU_ESTIMATE_HFI && pm->hfi_derated < iMAX)
		? pm->hfi_derated : iMAX;

	sD = (sD > iMAX) ? iMAX : (sD < - iMAX) ? - iMAX : sD;
	sQ = (sQ > iMAX) ? iMAX : (sQ < - iMAX) ? - iMAX : sQ;

	/* Maximal POWER constraint.
	 * */
	wMAX = (pm->watt_wp_maximal < pm->watt_derated_1)
		? pm->watt_wp_maximal : pm->watt_derated_1;
	wREV = - pm->watt_wp_reverse;

	/* Maximal DC link CURRENT constraint.
	 * */
	wP = pm->watt_ib_maximal * pm->const_lpf_U;
	wMAX = (wP < wMAX) ? wP : wMAX;
	wP = - pm->watt_ib_reverse * pm->const_lpf_U;
	wREV = (wP > wREV) ? wP : wREV;

	if (pm->const_lpf_U > pm->watt_derate_HI_U) {

		/* Prevent DC link overvoltage.
		 * */
		E = 1.f - (pm->const_lpf_U - pm->watt_derate_HI_U) * pm->watt_gain_DU;
		E = (E < 0.f) ? 0.f : E;

		pm->watt_integral[0] += (E - pm->watt_integral[0]) * pm->watt_gain_LP_I;
		wREV *= pm->watt_integral[0] * E;
	}
	else if (pm->const_lpf_U < pm->watt_derate_LO_U) {

		/* Prevent power source overload.
		 * */
		E = 1.f - (pm->watt_derate_LO_U - pm->const_lpf_U) * pm->watt_gain_DU;
		E = (E < 0.f) ? 0.f : E;

		pm->watt_integral[1] += (E - pm->watt_integral[1]) * pm->watt_gain_LP_I;
		wMAX *= pm->watt_integral[1] * E;
	}

	wS_abs = m_fabsf(pm->lu_wS);

	if (wS_abs > pm->watt_derate_HI_S) {

		/* Prevent excessive SPEED.
		 * */
		E = 1.f - (wS_abs - pm->watt_derate_HI_S) * pm->watt_gain_DS;
		E = (E < 0.f) ? 0.f : E;

		pm->watt_integral[2] += (E - pm->watt_integral[2]) * pm->watt_gain_LP_I;
		wMAX *= pm->watt_integral[2] * E;
	}

	/* Apply POWER constraints (with D-axis priority).
	 * */
	wP = PM_KWAT(pm) * (sD * pm->watt_lpf_D + sQ * pm->watt_lpf_Q);

	if (wP > wMAX) {

		wP = PM_KWAT(pm) * sD * pm->watt_lpf_D;

		if (wP > wMAX) {

			sD *= wMAX / wP;
			sQ = 0.f;
		}
		else {
			wMAX -= wP;
			wP = PM_KWAT(pm) * sQ * pm->watt_lpf_Q;

			if (wP > M_EPS_F) {

				sQ *= wMAX / wP;
			}
		}
	}
	else if (wP < wREV) {

		wP = PM_KWAT(pm) * sD * pm->watt_lpf_D;

		if (wP < wREV) {

			sD *= wREV / wP;
			sQ = 0.f;
		}
		else {
			wREV -= wP;
			wP = PM_KWAT(pm) * sQ * pm->watt_lpf_Q;

			if (wP < - M_EPS_F) {

				sQ *= wREV / wP;
			}
		}
	}

	/* Obtain discrepancy.
	 * */
	eD = sD - pm->lu_iD;
	eQ = sQ - pm->lu_iQ;

	if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		wS_abs = 2.f * M_PI_F * pm->hfi_freq_hz;
		m_rotf(pm->hfi_wave, wS_abs * pm->dT, pm->hfi_wave);

		eD += pm->hfi_wave[1] * pm->hfi_swing_D;
	}

	uD = pm->i_gain_P * eD;
	uQ = pm->i_gain_P * eQ;

	uD += - pm->lu_wS * pm->const_L * sQ;
	uQ += pm->lu_wS * pm->const_L * sD;

	uMAX = PM_UMAX(pm) * pm->const_lpf_U;

	pm->i_integral_D += pm->i_gain_I * eD;
	pm->i_integral_D = (pm->i_integral_D > uMAX) ? uMAX :
		(pm->i_integral_D < - uMAX) ? - uMAX : pm->i_integral_D;
	uD += pm->i_integral_D;

	pm->i_integral_Q += pm->i_gain_I * eQ;
	pm->i_integral_Q = (pm->i_integral_Q > uMAX) ? uMAX :
		(pm->i_integral_Q < - uMAX) ? - uMAX : pm->i_integral_Q;
	uQ += pm->i_integral_Q;

	if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		uD += pm->hfi_wave[0] * pm->hfi_swing_D * wS_abs * pm->const_L;
	}

	uX = pm->lu_F[0] * uD - pm->lu_F[1] * uQ;
	uY = pm->lu_F[1] * uD + pm->lu_F[0] * uQ;

	pm_voltage_control(pm, uX, uY);
}

static void
pm_speed_control(pmc_t *pm)
{
	float		iSP, wSP, eS, dS;

	/* Maximal speed constraint.
	 * */
	wSP = pm->s_setpoint;
	wSP = (wSP < - pm->s_maximal) ? - pm->s_maximal :
		(wSP > pm->s_maximal) ? pm->s_maximal : wSP;

	if (pm->config_BRAKE == PM_ENABLED) {

		/* Make NO reverse.
		 * */
		wSP = (pm->s_brake_DIR * wSP < 0.f) ? 0.f : wSP;
	}

	/* Maximal acceleration constraint.
	 * */
	dS = pm->s_accel * pm->dT;
	pm->s_track = (pm->s_track < wSP - dS) ? pm->s_track + dS
		: (pm->s_track > wSP + dS) ? pm->s_track - dS : wSP;

	dS = pm->s_advance;
	pm->s_track = (pm->s_track < pm->lu_wS - dS) ? pm->lu_wS - dS
		: (pm->s_track > pm->lu_wS + dS) ? pm->lu_wS + dS  : pm->s_track;

	if (pm->lu_mode == PM_LU_FORCED) {

		/* Do nothins in this case */
	}
	else {
		/* Obtain discrepancy.
		 * */
		eS = pm->s_track - pm->lu_wS;

		if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

			/* Slow down in case of HFI mode.
			 * */
			eS *= pm->s_gain_HF_S;
		}

		/* Here is P+LP regulator.
		 * */
		iSP = pm->s_gain_P * eS;

		if (m_fabsf(iSP) < pm->i_maximal) {

			pm->s_integral += (pm->lu_iQ - pm->s_integral) * pm->s_gain_LP_I;
			iSP += pm->s_integral;
		}
		else {
			pm->s_integral = 0.f;
		}

		/* Output clamp.
		 * */
		iSP = (iSP < - pm->i_maximal) ? - pm->i_maximal :
			(iSP > pm->i_maximal) ? pm->i_maximal : iSP;

		/* Update current loop setpoint. It would be possible here to
		 * use MTPA or something else.
		 * */
		pm->i_setpoint_Q = iSP;
	}
}

static void
pm_servo_control(pmc_t *pm)
{
	float		EX, EY, eP, eP_abs, eS, gP;

	/* Full revolution counter.
	 * */
	if (pm->lu_F[0] < 0.f) {

		if (pm->lu_F[1] < 0.f) {

			pm->x_lu_revol += (pm->x_lu_F1 >= 0.f) ? 1 : 0;
		}
		else {
			pm->x_lu_revol += (pm->x_lu_F1 < 0.f) ? - 1 : 0;
		}
	}

	pm->x_lu_F1 = pm->lu_F[1];

	/* Obtain discrepancy.
	 * */
	EX = pm->x_setpoint_F[0] * pm->lu_F[0] + pm->x_setpoint_F[1] * pm->lu_F[1];
	EY = pm->x_setpoint_F[1] * pm->lu_F[0] - pm->x_setpoint_F[0] * pm->lu_F[1];

	eP = m_atan2f(EY, EX);

	if (EY < 0.f) {

		if (pm->lu_F[1] < 0.f && pm->x_setpoint_F[1] >= 0.f)
			eP += 2.f * M_PI_F;
	}
	else {
		if (pm->lu_F[1] >= 0.f && pm->x_setpoint_F[1] < 0.f)
			eP -= 2.f * M_PI_F;
	}

	eP += (pm->x_setpoint_revol - pm->x_lu_revol) * 2.f * M_PI_F;
	eP_abs = m_fabsf(eP);

	/* Servo is based on constant acceleration formula.
	 * */
	eS = (eP < 0.f) ? - m_sqrtf(eP_abs) : m_sqrtf(eP_abs);
	gP = (eP_abs < pm->x_near_EP) ? eP_abs / pm->x_near_EP : 1.f;
	gP = pm->x_gain_N + (pm->x_gain_P - pm->x_gain_N) * gP;

	pm->s_setpoint = gP * eS;
}

static void
pm_stat_calculate(pmc_t *pm)
{
	float		mN, dE, dI;

	/* Full revolution counter.
	 * */
	if (pm->lu_F[0] < 0.f) {

		if (pm->lu_F[1] < 0.f) {

			pm->stat_revol_qu += (pm->stat_lu_F1 >= 0.f) ? 1 : 0;
		}
		else {
			pm->stat_revol_qu += (pm->stat_lu_F1 < 0.f) ? - 1 : 0;
		}
	}

	pm->stat_lu_F1 = pm->lu_F[1];

	if (pm->stat_revol_qu < - 3) {

		pm->stat_revol_total += - pm->stat_revol_qu;
		pm->stat_revol_qu = 0;
	}
	else if (pm->stat_revol_qu > 3) {

		pm->stat_revol_total += pm->stat_revol_qu;
		pm->stat_revol_qu = 0;
	}

	/* Traveled distance.
	 * */
	mN = (float) pm->stat_revol_total / (float) pm->const_Zp;
	pm->stat_distance = mN * pm->const_dd_T * M_PI_F;

	/* Get WATT per HOUR.
	 * */
	dE = pm->watt_lpf_wP * pm->dT * (1.f / 3600.f);
	dI = dE / pm->const_lpf_U;

	if (dE > 0.f) {

		pm_ADD(&pm->stat_consumed_wh, &pm->stat_FIX[0], dE);
		pm_ADD(&pm->stat_consumed_ah, &pm->stat_FIX[1], dI);
	}
	else {
		pm_ADD(&pm->stat_reverted_wh, &pm->stat_FIX[2], - dE);
		pm_ADD(&pm->stat_reverted_ah, &pm->stat_FIX[3], - dI);
	}
}

void pm_feedback(pmc_t *pm, pmfb_t *fb)
{
	float		vA, vB, vC, U, Q;

	if ((pm->vsi_IF & 2) == 0) {

		/* Get inline currents.
		 * */
		pm->fb_iA = pm->ad_IA[1] * fb->current_A + pm->ad_IA[0];
		pm->fb_iB = pm->ad_IB[1] * fb->current_B + pm->ad_IB[0];

		if (		m_fabsf(pm->fb_iA) > pm->fault_current_halt
				|| m_fabsf(pm->fb_iB) > pm->fault_current_halt) {

			pm->fail_reason = PM_ERROR_INLINE_OVER_CURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}

		if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = .57735027f * pm->fb_iA + 1.1547005f * pm->fb_iB;
		}
		else {
			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = pm->fb_iB;
		}
	}
	else if (pm->lu_mode != PM_LU_DISABLED) {

		m_rotf(pm->lu_F, pm->lu_wS * pm->dT, pm->lu_F);

		pm->lu_iX = pm->lu_F[0] * pm->lu_iD - pm->lu_F[1] * pm->lu_iQ;
		pm->lu_iY = pm->lu_F[1] * pm->lu_iD + pm->lu_F[0] * pm->lu_iQ;
	}

	/* Get DC link voltage.
	 * */
	U = pm->ad_US[1] * fb->voltage_U + pm->ad_US[0];
	pm->const_lpf_U += (U - pm->const_lpf_U) * pm->const_gain_LP_U;

	if (pm->const_lpf_U > pm->fault_voltage_halt) {

		pm->fail_reason = PM_ERROR_DC_LINK_OVER_VOLTAGE;
		pm->fsm_req = PM_STATE_HALT;
	}

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		vA = pm->tvm_FIR_A[1] * pm->fb_uA;
		vB = pm->tvm_FIR_B[1] * pm->fb_uB;
		vC = pm->tvm_FIR_C[1] * pm->fb_uC;

		/* Get terminal voltages.
		 * */
		pm->fb_uA = pm->ad_UA[1] * fb->voltage_A + pm->ad_UA[0];
		pm->fb_uB = pm->ad_UB[1] * fb->voltage_B + pm->ad_UB[0];
		pm->fb_uC = pm->ad_UC[1] * fb->voltage_C + pm->ad_UC[0];

		if (pm->tvm_FIR_A[0] != 0.f && pm->vsi_UF == 0) {

			/* Extract actual terminal voltages using FIR filter.
			 * */

			if ((pm->vsi_AZ & 2) == 0) {

				vA += pm->tvm_FIR_A[0] * pm->fb_uA + pm->tvm_FIR_A[2];
			}
			else {
				vA = 0.f;
			}

			if ((pm->vsi_BZ & 2) == 0) {

				vB += pm->tvm_FIR_B[0] * pm->fb_uB + pm->tvm_FIR_B[2];
			}
			else {
				vB = 0.f;
			}

			if ((pm->vsi_CZ & 2) == 0) {

				vC += pm->tvm_FIR_C[0] * pm->fb_uC + pm->tvm_FIR_C[2];
			}
			else {
				vC = 0.f;
			}

			pm->tvm_A = vA;
			pm->tvm_B = vB;
			pm->tvm_C = vC;

			if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

				Q = (1.f / 3.f) * (vA + vB + vC);
				vA = vA - Q;
				vB = vB - Q;

				pm->tvm_DX = vA;
				pm->tvm_DY = .57735027f * vA + 1.1547005f * vB;
			}
			else {
				vA = vA - vC;
				vB = vB - vC;

				pm->tvm_DX = vA;
				pm->tvm_DY = vB;
			}
		}
		else {
			pm->tvm_DX = pm->vsi_DX;
			pm->tvm_DY = pm->vsi_DY;
		}
	}
	else {
		pm->tvm_DX = pm->vsi_DX;
		pm->tvm_DY = pm->vsi_DY;
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

		if (pm->config_STAT == PM_ENABLED) {

			pm_stat_calculate(pm);
		}

		if (pm->flux[pm->flux_H].lpf_E > pm->fault_flux_lpfe_halt) {

			pm->fail_reason = PM_ERROR_FLUX_UNSTABLE;
			pm->fsm_state = PM_STATE_HALT;
		}

		if (m_isfinitef(pm->lu_F[0]) == 0) {

			pm->fail_reason = PM_ERROR_INVALID_OPERATION;
			pm->fsm_state = PM_STATE_HALT;
		}
	}
}

