#include "libm.h"
#include "pm.h"

void pm_quick_build(pmc_t *pm)
{
	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		pm->k_UMAX = .66666667f;
		pm->k_EMAX = .57735027f;
		pm->k_KWAT = 1.5f;
	}
	else {
		pm->k_UMAX = 1.f;
		pm->k_EMAX = .70710678f;
		pm->k_KWAT = 1.f;
	}

	pm->ts_minimal = (int) (pm->dc_minimal * (1.f / 1000000.f)
			* pm->freq_hz * (float) pm->dc_resolution);
	pm->ts_clearance = (int) (pm->dc_clearance * (1.f / 1000000.f)
			* pm->freq_hz * (float) pm->dc_resolution);
	pm->ts_skip = (int) (pm->dc_skip * (1.f / 1000000.f)
			* pm->freq_hz * (float) pm->dc_resolution);
	pm->ts_bootstrap = PM_TSMS(pm, pm->dc_bootstrap);
	pm->ts_clamped = (int) (pm->freq_hz * pm->dc_clamped);
	pm->ts_inverted = 1.f / (float) pm->dc_resolution;

	pm->quick_iE = (pm->const_E > M_EPS_F) ? 1.f / pm->const_E : 0.f;
	pm->quick_iE2 = pm->quick_iE * pm->quick_iE;
	pm->quick_iL1 = (pm->const_im_L1 > M_EPS_F) ? 1.f / pm->const_im_L1 : 0.f;
	pm->quick_iL2 = (pm->const_im_L2 > M_EPS_F) ? 1.f / pm->const_im_L2 : 0.f;

	pm->quick_TiL1 = pm->dT * pm->quick_iL1;
	pm->quick_TiL2 = pm->dT * pm->quick_iL2;

	pm->quick_hfwS = 2.f * M_PI_F * pm->freq_hz / (float) pm->hfi_INJS;
	pm->quick_hfSC[0] = m_cosf(pm->quick_hfwS * pm->dT * .5f);
	pm->quick_hfSC[1] = m_sinf(pm->quick_hfwS * pm->dT * .5f);

	pm->quick_ZiEP = 2.f * M_PI_F * (float) (pm->const_Zp * pm->abi_gear_Zs)
		/ (float) (pm->abi_gear_Zq * pm->abi_EPPR);

	pm->quick_ZiSQ = (float) (pm->const_Zp * pm->sincos_gear_Zs)
		/ (float) pm->sincos_gear_Zq;

	if (		pm->config_LU_ESTIMATE_HFI == PM_HFI_ON_KALMAN
			&& pm->config_LU_ESTIMATE_FLUX != PM_FLUX_KALMAN) {

		pm->config_LU_ESTIMATE_HFI = PM_HFI_NONE;
	}

	if (		pm->config_LU_LOCATION == PM_LOCATION_ABI
			&& pm->config_LU_SENSOR == PM_SENSOR_ABI) {

		pm->config_LU_LOCATION = PM_LOCATION_INHERITED;
	}
}

static void
pm_auto_basic_default(pmc_t *pm)
{
	pm->dc_minimal = 0.2f;			/* (us) */
	pm->dc_clearance = 5.0f;		/* (us) */
	pm->dc_skip = 2.0f;			/* (us) */
	pm->dc_bootstrap = 100.f;		/* (ms) */
	pm->dc_clamped = 1.f;			/* (s)  */

	pm->config_NOP = PM_NOP_THREE_PHASE;
	pm->config_IFB = PM_IFB_ABC_INLINE;
	pm->config_TVM = PM_ENABLED;
	pm->config_DEBUG = PM_DISABLED;

	pm->fault_voltage_tol = 4.f;		/* (V) */
	pm->fault_current_tol = 4.f;		/* (A) */
	pm->fault_accuracy_tol = .10f;		/*     */
	pm->fault_terminal_tol = 0.090f;	/* (V) */
	pm->fault_current_halt = 156.f;		/* (A) */
	pm->fault_voltage_halt = 57.f;		/* (V) */

	pm->vsi_AF = 1;				/* NOTE: Disable all flags until  */
	pm->vsi_BF = 1;				/* clearance zone is calculated.  */
	pm->vsi_CF = 1;
	pm->vsi_SF = 1;
	pm->vsi_UF = 1;

	pm->watt_wP_maximal = 4000.f;		/* (Watt) */
	pm->watt_iDC_maximal = 80.f;		/* (A) */
	pm->watt_wP_reverse = 4000.f;		/* (Watt) */
	pm->watt_iDC_reverse = 80.f;		/* (A) */
	pm->watt_dclink_HI = 52.f;		/* (V) */
	pm->watt_dclink_LO = 7.f;		/* (V) */

	pm->i_maximal = 120.f;			/* (A) */
	pm->i_reverse = pm->i_maximal;
}

static void
pm_auto_config_default(pmc_t *pm)
{
	pm->config_VSI_CIRCULAR = PM_DISABLED;
	pm->config_VSI_PRECISE = PM_DISABLED;
	pm->config_LU_FORCED = PM_ENABLED;
	pm->config_LU_ESTIMATE_FLUX = PM_FLUX_ORTEGA;
	pm->config_LU_ESTIMATE_HFI = PM_HFI_NONE;
	pm->config_LU_SENSOR = PM_SENSOR_DISABLED;
	pm->config_LU_LOCATION = PM_LOCATION_INHERITED;
	pm->config_LU_DRIVE = PM_DRIVE_SPEED;
	pm->config_RELUCTANCE = PM_DISABLED;
	pm->config_WEAKENING = PM_DISABLED;
	pm->config_HFI_IMPEDANCE = PM_DISABLED;
	pm->config_HFI_POLARITY = PM_DISABLED;
	pm->config_HOLDING_BRAKE = PM_DISABLED;
	pm->config_SPEED_LIMITED = PM_ENABLED;
	pm->config_ABI_FRONTEND = PM_ABI_INCREMENTAL;
	pm->config_SINCOS_FRONTEND = PM_SINCOS_ANALOG;
	pm->config_MILEAGE_INFO	= PM_ENABLED;
	pm->config_BOOST_CHARGE = PM_DISABLED;

	pm->tm_transient_slow = 40.f;		/* (ms) */
	pm->tm_transient_fast = 2.f;		/* (ms) */
	pm->tm_voltage_hold = 100.f;		/* (ms) */
	pm->tm_current_hold = 300.f;		/* (ms) */
	pm->tm_current_ramp = 400.f;		/* (ms) */
	pm->tm_instant_probe = 2.f;		/* (ms) */
	pm->tm_average_probe = 200.f;		/* (ms) */
	pm->tm_average_drift = 100.f;		/* (ms) */
	pm->tm_average_inertia = 700.f;		/* (ms) */
	pm->tm_startup = 100.f;			/* (ms) */
	pm->tm_halt_pause = 1000.f;		/* (ms) */

	pm->ad_IA[0] = 0.f;
	pm->ad_IA[1] = 1.f;
	pm->ad_IB[0] = 0.f;
	pm->ad_IB[1] = 1.f;
	pm->ad_IC[0] = 0.f;
	pm->ad_IC[1] = 1.f;
	pm->ad_US[0] = 0.f;
	pm->ad_US[1] = 1.f;
	pm->ad_UA[0] = 0.f;
	pm->ad_UA[1] = 1.f;
	pm->ad_UB[0] = 0.f;
	pm->ad_UB[1] = 1.f;
	pm->ad_UC[0] = 0.f;
	pm->ad_UC[1] = 1.f;

	pm->noise_DI = 1.f;
	pm->noise_DS = 1.f;

	pm->probe_current_hold = 20.f;
	pm->probe_current_weak = 10.f;
	pm->probe_hold_angle = 0.f;
	pm->probe_current_sine = 5.f;
	pm->probe_current_bias = 0.f;
	pm->probe_freq_sine = 1100.f;
	pm->probe_speed_hold = 900.f;
	pm->probe_speed_detached = 50.f;
	pm->probe_gain_P = 1E-2f;
	pm->probe_gain_I = 1E-3f;

	pm->vsi_gain_LP = 5E-3f;
	pm->vsi_mask_XF = PM_MASK_NONE;

	pm->tvm_USEABLE = PM_DISABLED;
	pm->tvm_clean_zone = .10f;
	pm->tvm_FIR_A[0] = 0.f;
	pm->tvm_FIR_A[1] = 0.f;
	pm->tvm_FIR_A[2] = 0.f;
	pm->tvm_FIR_B[0] = 0.f;
	pm->tvm_FIR_B[1] = 0.f;
	pm->tvm_FIR_B[2] = 0.f;
	pm->tvm_FIR_C[0] = 0.f;
	pm->tvm_FIR_C[1] = 0.f;
	pm->tvm_FIR_C[2] = 0.f;

	pm->lu_transient = 800.f;
	pm->lu_gain_QF = 5E-3f;

	pm->forced_hold_D = 20.f;
	pm->forced_maximal = 900.f;
	pm->forced_reverse = pm->forced_maximal;
	pm->forced_accel = 400.f;
	pm->forced_slew_rate = 800.f;
	pm->forced_maximal_DC = 0.7f;

	pm->detach_threshold_BASE = 1.f;
	pm->detach_trip_AD = 2E-1f;
	pm->detach_gain_SF = 5E-2f;

	pm->flux_trip_AD = 2E-1f;
	pm->flux_gain_IN = 5E-4f;
	pm->flux_gain_LO = 2E-6f;
	pm->flux_gain_HI = 5E-5f;
	pm->flux_gain_SF = 5E-2f;
	pm->flux_gain_IF = .5f;

	pm->kalman_gain_Q[0] = 5E-5f;
	pm->kalman_gain_Q[1] = 5E-5f;
	pm->kalman_gain_Q[2] = 5E-4f;
	pm->kalman_gain_Q[3] = 5E+1f;
	pm->kalman_gain_Q[4] = 5E-5f;
	pm->kalman_gain_R = 5E-1f;

	pm->zone_threshold_NOISE = 50.f;
	pm->zone_threshold_BASE = 80.f;
	pm->zone_gain_TH = .7f;
	pm->zone_gain_LP = 5E-3f;

	pm->hfi_sine = 5.f;
	pm->hfi_maximal = 900.f;
	pm->hfi_INJS = 6;
	pm->hfi_SKIP = 1;
	pm->hfi_ESTI = 5;
	pm->hfi_gain_FP = 1E+1f;
	pm->hfi_gain_SF = 5E-3f;
	pm->hfi_gain_IF = 1.f;

	pm->hall_USEABLE = PM_DISABLED;
	pm->hall_time_prol = 100.f;		/* (ms) */
	pm->hall_gain_SF = 5E-3f;
	pm->hall_gain_PF = 1.f;
	pm->hall_gain_IF = 1.f;

	pm->abi_USEABLE = PM_DISABLED;
	pm->abi_EPPR = 2400;
	pm->abi_gear_Zs = 1;
	pm->abi_gear_Zq = 1;
	pm->abi_gain_SF = 5E-2f;
	pm->abi_gain_PF = 1.f;
	pm->abi_gain_IF = 1.f;

	pm->sincos_USEABLE = PM_DISABLED;
	pm->sincos_gear_Zs = 1;
	pm->sincos_gear_Zq = 1;

	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Zp = 1;
	pm->const_Ja = 0.f;
	pm->const_im_L1 = 0.f;
	pm->const_im_L2 = 0.f;
	pm->const_im_B = 0.f;
	pm->const_im_R = 0.f;

	pm->watt_gain_LP = 5E-2f;

	pm->i_derated_HFI = 30.f;
	pm->i_slew_rate = 7000.f;
	pm->i_tolerance = 0.f;
	pm->i_gain_P = 2E-1f;
	pm->i_gain_I = 5E-3f;

	pm->weak_maximal = 30.f;
	pm->weak_gain_EU = 5E-2f;

	pm->v_maximal = 90.f;
	pm->v_reverse = pm->v_maximal;

	pm->s_maximal = 15000.f;
	pm->s_reverse = pm->s_maximal;
	pm->s_accel = 7000.f;
	pm->s_linspan = 100.f;
	pm->s_tolerance = 0.f;
	pm->s_gain_P = 1E-1f;
	pm->s_gain_I = 1.f;

	pm->x_location_range[0] = -10.f;
	pm->x_location_range[1] = 10.f;
	pm->x_location_home = 0.f;

	pm->x_weak_zone = 1.f;
	pm->x_tolerance = 0.f;
	pm->x_gain_P = 35.f;
	pm->x_gain_N = 5.f;

	pm->boost_gain_P = 1E-1f;
	pm->boost_gain_I = 1E-3f;
}

static void
pm_auto_probe_default(pmc_t *pm)
{
	pm->probe_speed_hold = 900.f;

	pm->lu_gain_QF = 5E-3f;

	pm->forced_maximal = 900.f;
	pm->forced_reverse = pm->forced_maximal;
	pm->forced_accel = 400.f;

	pm->zone_threshold_NOISE = 50.f;
	pm->zone_threshold_BASE = 80.f;

	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Ja = 0.f;
	pm->const_im_L1 = 0.f;
	pm->const_im_L2 = 0.f;
	pm->const_im_B = 0.f;
	pm->const_im_R = 0.f;

	pm->i_slew_rate = 7000.f;
	pm->i_gain_P = 2E-1f;
	pm->i_gain_I = 5E-3f;

	pm->s_gain_P = 1E-1f;
	pm->s_gain_I = 1.f;
}

static void
pm_auto_maximal_current(pmc_t *pm)
{
	float			maximal_A, new_A;

	/* Get the maximal INLINE current.
	 * */
	maximal_A = pm->fault_current_halt * 0.8f;

	if (pm->const_R > M_EPS_F) {

		/* Based on DC link voltage.
		 * */
		new_A = pm->k_UMAX * pm->const_fb_U / pm->const_R;
		maximal_A = (new_A < maximal_A) ? new_A : maximal_A;

		/* Based on resistive LOSSES.
		 * */
		new_A = m_sqrtf(400.f / pm->const_R);
		maximal_A = (new_A < maximal_A) ? new_A : maximal_A;

		if (maximal_A < pm->i_maximal) {

			pm->i_maximal = (float) (int) maximal_A;
			pm->i_reverse = pm->i_maximal;
		}
	}
	else {
		pm->i_maximal = (float) (int) maximal_A;
		pm->i_reverse = pm->i_maximal;
	}
}

static void
pm_auto_probe_speed_hold(pmc_t *pm)
{
	float			probe_MAX, probe_MIN;

	if (pm->const_E > M_EPS_F) {

		probe_MAX = 0.7f * pm->k_EMAX * pm->const_fb_U / pm->const_E;
		probe_MIN = 1.4f * (pm->zone_threshold_BASE + pm->zone_threshold_NOISE);

		if (pm->probe_speed_hold > probe_MAX) {

			pm->probe_speed_hold = probe_MAX;
		}

		if (pm->probe_speed_hold < probe_MIN) {

			pm->probe_speed_hold = probe_MIN;
		}
	}
}

static void
pm_auto_zone_threshold(pmc_t *pm)
{
	float			threshold_MAX, byvoltage_MAX, threshold_MIN;
	float			threshold_IRU, threshold_DTU;

	if (		   pm->const_R > M_EPS_F
			&& pm->const_E > M_EPS_F) {

		/* Suppress the NOISE threshold if it is WRONGLY large.
		 * */
		threshold_MAX = 0.4f * pm->forced_maximal;
		byvoltage_MAX = 10.f / pm->const_E;

		threshold_MAX = (byvoltage_MAX < threshold_MAX)
			? byvoltage_MAX : threshold_MAX;

		if (pm->zone_threshold_NOISE > threshold_MAX) {

			pm->zone_threshold_NOISE = threshold_MAX;
		}

		threshold_MIN = 10.f;

		if (pm->zone_threshold_NOISE < threshold_MIN) {

			pm->zone_threshold_NOISE = threshold_MIN;
		}

		/* Based on uncertainty due to RESISTANCE thermal drift.
		 * */
		threshold_IRU = 0.2f * pm->i_maximal * pm->const_R;

		if (pm->tvm_USEABLE == PM_ENABLED) {

			threshold_DTU = 0.1f;
		}
		else {
			/* Based on voltage uncertainty on DT.
			 * */
			threshold_DTU = pm->dc_minimal * (1.f / 1000000.f)
				* pm->freq_hz * pm->const_fb_U;
		}

		/* The BASE threshold.
		 * */
		pm->zone_threshold_BASE = (threshold_IRU + threshold_DTU) / pm->const_E;

		threshold_MAX = 0.7f * pm->forced_maximal - pm->zone_threshold_NOISE;

		if (pm->zone_threshold_BASE > threshold_MAX) {

			pm->zone_threshold_BASE = threshold_MAX;
		}

		if (pm->zone_threshold_BASE < threshold_MIN) {

			pm->zone_threshold_BASE = threshold_MIN;
		}
	}
}

static void
pm_auto_forced_maximal(pmc_t *pm)
{
	float		forced_MAX, forced_MIN;

	forced_MAX = 0.7f * pm->k_EMAX * pm->const_fb_U / pm->const_E;
	forced_MIN = pm->probe_speed_hold;

	if (pm->forced_maximal > forced_MAX) {

		pm->forced_maximal = forced_MAX;
	}

	if (pm->forced_maximal < forced_MIN) {

		pm->forced_maximal = forced_MIN;
	}
}

static void
pm_auto_forced_accel(pmc_t *pm)
{
	float		hold_D, mQ;

	if (pm->const_Ja > M_EPS_F) {

		hold_D = pm->forced_hold_D;
		mQ = pm_torque_equation(pm, hold_D, hold_D);

		/* Tune forced control based on the motor constants.
		 * */
		pm->forced_accel = 0.1f * mQ / pm->const_Ja;
	}
}

static void
pm_auto_loop_current(pmc_t *pm)
{
	float		Lm, Di, Kp, Ki;

	if (		   pm->const_im_L1 > M_EPS_F
			&& pm->const_im_L1 > M_EPS_F) {

		Lm = (pm->const_im_L1 < pm->const_im_L2)
			? pm->const_im_L1 : pm->const_im_L2;

		Di = pm->noise_DI;

		/* Tune the current loop based on state-space model.
		 *
		 *          [1-R*T/L-Kp*T/L  -Ki*T/L]
		 * x(k+1) = [1                1     ] * x(k)
		 *
		 * */
		Kp = 0.5f * Lm * Di * pm->freq_hz - pm->const_R;
		Ki = 0.02f * Lm * Di * pm->freq_hz;

		pm->i_gain_P = (Kp > 0.f) ? Kp : 0.f;
		pm->i_gain_I = Ki;

		/* Get the current slew rate limit.
		 * */
		pm->i_slew_rate = 0.05f * pm->const_fb_U / Lm;
	}
}

static void
pm_auto_loop_speed(pmc_t *pm)
{
	float		Ds = pm->noise_DS;

	if (pm->zone_threshold_NOISE > M_EPS_F) {

		/* Tune load torque estimate.
		 * */
		if (		pm->const_E > M_EPS_F
				&& pm->const_Ja > M_EPS_F) {

			pm->lu_gain_QF = 10.f * Ds * pm->const_E * pm->dT
				/ pm->const_Ja / pm->zone_threshold_NOISE;
		}

		/* Tune speed loop based on threshold NOISE value.
		 * */
		pm->s_gain_P = 2.f * Ds / pm->zone_threshold_NOISE;
		pm->s_gain_I = 1.f;
	}
}

void pm_auto(pmc_t *pm, int req)
{
	switch (req) {

		case PM_AUTO_BASIC_DEFAULT:
			pm_auto_basic_default(pm);
			break;

		case PM_AUTO_CONFIG_DEFAULT:
			pm_auto_config_default(pm);
			break;

		case PM_AUTO_PROBE_DEFAULT:
			pm_auto_probe_default(pm);
			break;

		case PM_AUTO_MAXIMAL_CURRENT:
			pm_auto_maximal_current(pm);
			break;

		case PM_AUTO_PROBE_SPEED_HOLD:
			pm_auto_probe_speed_hold(pm);
			break;

		case PM_AUTO_ZONE_THRESHOLD:
			pm_auto_zone_threshold(pm);
			break;

		case PM_AUTO_FORCED_MAXIMAL:
			pm_auto_forced_maximal(pm);
			break;

		case PM_AUTO_FORCED_ACCEL:
			pm_auto_forced_accel(pm);
			break;

		case PM_AUTO_LOOP_CURRENT:
			pm_auto_loop_current(pm);
			break;

		case PM_AUTO_LOOP_SPEED:
			pm_auto_loop_speed(pm);
			break;

		default:
			break;
	}
}

float pm_torque_equation(pmc_t *pm, float iD, float iQ)
{
	float		mQ, E_rel = 0.f;

	if (pm->config_RELUCTANCE == PM_ENABLED) {

		E_rel = (pm->const_im_L1 - pm->const_im_L2) * iD;
	}

	mQ = pm->k_KWAT * (pm->const_E + E_rel) * iQ;

	return mQ;
}

static float
pm_torque_accel(pmc_t *pm, float iD, float iQ)
{
	float			mQ, accel = 0.f;

	if (pm->const_Ja > M_EPS_F) {

		mQ = pm_torque_equation(pm, pm->lu_iD, pm->lu_iQ);
		accel = (mQ - pm->lu_load_torque) / pm->const_Ja;
	}

	return accel;
}

static float
pm_torque_do_MTPA(pmc_t *pm, float iQ)
{
	float		E1, Q_rel, MTPA_sine = 0.f;

	if (m_fabsf(iQ) > M_EPS_F) {

		E1 = pm->const_E;
		Q_rel = 4.f * (pm->const_im_L2 - pm->const_im_L1) * iQ;

		/* Sine of MTPA angle.
		 * */
		MTPA_sine = (E1 - m_sqrtf(Q_rel * Q_rel * .5f + E1 * E1)) / Q_rel;
	}

	return MTPA_sine;
}

static float
pm_torque_approx(pmc_t *pm, float mQ)
{
	float		iQ = 0.f;

	if (pm->config_RELUCTANCE == PM_ENABLED) {

		/* TODO */
	}
	else {
		if (pm->const_E > M_EPS_F) {

			iQ = mQ / (pm->k_KWAT * pm->const_E);
		}
	}

	return iQ;
}

static void
pm_forced(pmc_t *pm)
{
	float		wSP, dS;

	/* Get the SETPOINT of forced speed.
	 * */
	if (pm->config_LU_DRIVE == PM_DRIVE_CURRENT) {

		wSP = (pm->i_setpoint_current < - M_EPS_F) ? - PM_MAX_F
			: (pm->i_setpoint_current > M_EPS_F) ? PM_MAX_F : 0.f;
	}
	else {
		wSP = pm->s_setpoint_speed;
	}

	/* Maximal forced speed constraint.
	 * */
	wSP = (wSP > pm->forced_maximal) ? pm->forced_maximal :
		(wSP < - pm->forced_reverse) ? - pm->forced_reverse : wSP;

	if (pm->vsi_lpf_DC > pm->forced_maximal_DC) {

		/* We are unable to keep such a high speed at
		 * this DC link voltage. Stop.
		 * */
		wSP = 0.f;
	}

	/* Update actual speed with specified acceleration.
	 * */
	dS = pm->forced_accel * pm->dT;
	pm->forced_wS = (pm->forced_wS < wSP - dS) ? pm->forced_wS + dS :
		(pm->forced_wS > wSP + dS) ? pm->forced_wS - dS : wSP;

	/* Update DQ-axes.
	 * */
	m_rotatef(pm->forced_F, pm->forced_wS * pm->dT);
}

static void
pm_detached_BEMF(pmc_t *pm)
{
	float		uA, uB, uC, uX, uY, U, EX, EY, E, S;

	/* Get BEMF voltage.
	 * */
	uA = pm->fb_uA;
	uB = pm->fb_uB;
	uC = pm->fb_uC;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		U = .33333333f * (uA + uB + uC);
		uA = uA - U;
		uB = uB - U;

		uX = uA;
		uY = .57735027f * uA + 1.1547005f * uB;
	}
	else {
		uX = uA - uC;
		uY = uB - uC;
	}

	pm->vsi_X = uX;
	pm->vsi_Y = uY;

	/* Absolute BEMF voltage.
	 * */
	U = m_sqrtf(uX * uX + uY * uY);

	if (U > pm->detach_threshold_BASE) {

		E = 1.f / U;

		uX *= E;
		uY *= E;

		if (pm->detach_TIM != 0) {

			/* Speed estimation (PLL).
			 * */
			m_rotatef(pm->flux_X, pm->flux_wS * pm->dT);

			EX = uX * pm->flux_X[0] + uY * pm->flux_X[1];
			EY = uY * pm->flux_X[0] - uX * pm->flux_X[1];

			if (EX > M_EPS_F) {

				E = EY * pm->freq_hz;

				S = U * pm->detach_trip_AD;
				S = (S > 1.f) ? 1.f : S;

				pm->flux_wS += E * S * pm->detach_gain_SF;
			}

			pm->flux_E = U / m_fabsf(pm->flux_wS);

			E = (pm->flux_wS < 0.f) ? - 1.f : 1.f;

			pm->flux_F[0] = uY * E;
			pm->flux_F[1] = - uX * E;
		}

		pm->detach_TIM++;

		pm->flux_X[0] = uX;
		pm->flux_X[1] = uY;
	}
	else {
		pm->detach_TIM = 0;

		pm->flux_wS = 0.f;
	}
}

static void
pm_flux_ORTEGA(pmc_t *pm)
{
	float		uX, uY, lX, lY, EX, EY, DX, DY, S, E, A;

	/* Get the actual voltage.
	 * */
	uX = pm->vsi_X - pm->const_R * pm->lu_iX;
	uY = pm->vsi_Y - pm->const_R * pm->lu_iY;

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		uX += pm->tvm_DX - pm->vsi_DX;
		uY += pm->tvm_DY - pm->vsi_DY;
	}

	/* Stator FLUX.
	 * */
	lX = pm->const_im_L2 * pm->lu_iX;
	lY = pm->const_im_L2 * pm->lu_iY;

	if (pm->const_E > M_EPS_F) {

		/* FLUX equations.
		 * */
		pm->flux_X[0] += uX * pm->dT;
		pm->flux_X[1] += uY * pm->dT;

		EX = pm->flux_X[0] - lX;
		EY = pm->flux_X[1] - lY;

		S = m_fabsf(pm->flux_wS * pm->const_E) * pm->flux_trip_AD;
		S = (S > 1.f) ? 1.f : S;

		/* Get the flux RESIDUE.
		 * */
		E = 1.f - (EX * EX + EY * EY) * pm->quick_iE2;

		/* Adaptive GAIN.
		 * */
		E *=	  pm->flux_gain_HI * S
			+ pm->flux_gain_LO * (1.f - S);

		pm->flux_X[0] += EX * E * pm->quick_iE;
		pm->flux_X[1] += EY * E * pm->quick_iE;
	}
	else {
		/* Startup estimation.
		 * */
		pm->flux_X[0] += uX * pm->dT;
		pm->flux_X[1] += uY * pm->dT;

		EX = pm->flux_X[0] - lX;
		EY = pm->flux_X[1] - lY;

		E = - pm->flux_gain_IN;

		pm->flux_X[0] += EX * E;
		pm->flux_X[1] += EY * E;
	}

	/* Extract the rotor FLUX linkage.
	 * */
	EX = pm->flux_X[0] - lX;
	EY = pm->flux_X[1] - lY;

	E = m_sqrtf(EX * EX + EY * EY);

	pm->flux_E = E;

	if (E > M_EPS_F) {

		E = 1.f / E;

		EX *= E;
		EY *= E;

		if (pm->const_E > M_EPS_F) {

			/* Speed estimation (PLL).
			 * */
			m_rotatef(pm->flux_F, pm->flux_wS * pm->dT);

			DX = EX * pm->flux_F[0] + EY * pm->flux_F[1];
			DY = EY * pm->flux_F[0] - EX * pm->flux_F[1];

			if (DX > M_EPS_F) {

				E = DY * pm->freq_hz;
				pm->flux_wS += E * pm->flux_gain_SF;
			}

			if (pm->flux_gain_IF > M_EPS_F) {

				A = pm_torque_accel(pm, pm->lu_iD, pm->lu_iQ);
				pm->flux_wS += A * pm->dT * pm->flux_gain_IF;
			}
		}
		else {
			/* Startup estimate borrowing.
			 * */
			pm->flux_wS = pm->lu_wS;
		}

		pm->flux_F[0] = EX;
		pm->flux_F[1] = EY;
	}
}

static void
pm_kalman_equation(pmc_t *pm, float Y[2], const float X[2], const float F[2])
{
        float           uD, uQ, R1, E1, flux_D, flux_Q;

        uD = F[0] * pm->vsi_X + F[1] * pm->vsi_Y;
        uQ = F[0] * pm->vsi_Y - F[1] * pm->vsi_X;

        R1 = pm->const_R;
        E1 = pm->const_E;

	uQ += pm->kalman_bias_Q;

        flux_D = pm->const_im_L1 * X[0] + E1;
        flux_Q = pm->const_im_L2 * X[1];

        Y[0] = (uD - R1 * X[0] + flux_Q * pm->flux_wS) * pm->quick_iL1;
        Y[1] = (uQ - R1 * X[1] - flux_D * pm->flux_wS) * pm->quick_iL2;
}

static void
pm_kalman_solve(pmc_t *pm, float X[2], float F[2], float wS)
{
        float           Y1[2], Y2[2];

        /* Second-order ODE solver.
         * */

        pm_kalman_equation(pm, Y1, X, F);

        X[0] += Y1[0] * pm->dT;
        X[1] += Y1[1] * pm->dT;

        m_rotatef(F, wS * pm->dT);

        pm_kalman_equation(pm, Y2, X, F);

        X[0] += (Y2[0] - Y1[0]) * pm->dT * .5f;
        X[1] += (Y2[1] - Y1[1]) * pm->dT * .5f;
}

static void
pm_kalman_solve_TVM(pmc_t *pm, float X[2], float F[2])
{
        float           uX, uY, uD, uQ;

        /* First-order ODE solver.
         * */

        uX = pm->tvm_DX - pm->vsi_DX;
        uY = pm->tvm_DY - pm->vsi_DY;

        uD = F[0] * uX + F[1] * uY;
        uQ = F[0] * uY - F[1] * uX;

        X[0] += uD * pm->dT * pm->quick_iL1;
        X[1] += uQ * pm->dT * pm->quick_iL2;
}

static void
pm_kalman_jacobian(pmc_t *pm, const float X[2], const float F[2], float wS)
{
	float		*A = pm->kalman_A;

	/*
	 *     [ A(0) A(1) A(2) A(3) 0    ]
	 *     [ A(4) A(5) A(6) A(7) A(8) ]
	 * A = [ 0    0    1    A(9) 0    ]
	 *     [ 0    0    0    1    0    ]
	 *     [ 0    0    0    0    1    ]
	 * */

	A[0] = 1.f - pm->const_R * pm->quick_TiL1;
	A[1] = wS * pm->const_im_L2 * pm->quick_TiL1;
	A[2] = (F[0] * pm->vsi_Y - F[1] * pm->vsi_X) * pm->quick_TiL1;
	A[3] = X[1] * pm->const_im_L2 * pm->quick_TiL1;

	A[4] = - wS * pm->const_im_L1 * pm->quick_TiL2;
	A[5] = 1.f - pm->const_R * pm->quick_TiL2;
	A[6] = (- F[0] * pm->vsi_X - F[1] * pm->vsi_Y) * pm->quick_TiL2;
	A[7] = (- pm->const_E - X[0] * pm->const_im_L1) * pm->quick_TiL2;
	A[8] = pm->quick_TiL2;
	A[9] = pm->dT;
}

static void
pm_kalman_predict(pmc_t *pm)
{
	float		*P = pm->kalman_P;
	const float	*A = pm->kalman_A;
	const float	*Q = pm->kalman_gain_Q;

	float		AP[16];

	/*
	 * Calculate predicted (a priori) covariance to the next cycle.
	 *
	 * P = A * P * A' + Q.
	 *
	 *     [ P(0)  P(1)  P(3)  P(6)  P(10) ]
	 *     [ P(1)  P(2)  P(4)  P(7)  P(11) ]
	 * P = [ P(3)  P(4)  P(5)  P(8)  P(12) ]
	 *     [ P(6)  P(7)  P(8)  P(9)  P(13) ]
	 *     [ P(10) P(11) P(12) P(13) P(14) ]
	 *
	 * */

	AP[0] = A[0] * P[0] + A[1] * P[1] + A[2] * P[3] + A[3] * P[6];
	AP[1] = A[0] * P[1] + A[1] * P[2] + A[2] * P[4] + A[3] * P[7];
	AP[2] = A[0] * P[3] + A[1] * P[4] + A[2] * P[5] + A[3] * P[8];
	AP[3] = A[0] * P[6] + A[1] * P[7] + A[2] * P[8] + A[3] * P[9];

	AP[4] = A[4] * P[0] + A[5] * P[1] + A[6] * P[3] + A[7] * P[6] + A[8] * P[10];
	AP[5] = A[4] * P[1] + A[5] * P[2] + A[6] * P[4] + A[7] * P[7] + A[8] * P[11];
	AP[6] = A[4] * P[3] + A[5] * P[4] + A[6] * P[5] + A[7] * P[8] + A[8] * P[12];
	AP[7] = A[4] * P[6] + A[5] * P[7] + A[6] * P[8] + A[7] * P[9] + A[8] * P[13];
	AP[8] = A[4] * P[10] + A[5] * P[11] + A[6] * P[12] + A[7] * P[13] + A[8] * P[14];

	AP[9] = P[3] + A[9] * P[6];
	AP[10] = P[4] + A[9] * P[7];
	AP[11] = P[5] + A[9] * P[8];
	AP[12] = P[8] + A[9] * P[9];
	AP[13] = P[12] + A[9] * P[13];

	AP[14] = P[6];
	AP[15] = P[10];

	P[0] = AP[0] * A[0] + AP[1] * A[1] + AP[2] * A[2] + AP[3] * A[3];
	P[1] = AP[4] * A[0] + AP[5] * A[1] + AP[6] * A[2] + AP[7] * A[3];
	P[2] = AP[4] * A[4] + AP[5] * A[5] + AP[6] * A[6] + AP[7] * A[7] + AP[8] * A[8];
	P[3] = AP[9] * A[0] + AP[10] * A[1] + AP[11] * A[2] + AP[12] * A[3];
	P[4] = AP[9] * A[4] + AP[10] * A[5] + AP[11] * A[6] + AP[12] * A[7] + AP[13] * A[8];
	P[5] = AP[11] + AP[12] * A[9];
	P[6] = AP[14] * A[0] + P[7] * A[1] + P[8] * A[2] + P[9] * A[3];
	P[7] = AP[14] * A[4] + P[7] * A[5] + P[8] * A[6] + P[9] * A[7] + P[13] * A[8];
	P[8] = P[8] + P[9] * A[9];

	P[0] += Q[0];
	P[2] += Q[1];
	P[5] += Q[2];
	P[9] += Q[3];

	if (pm->flux_ZONE == PM_ZONE_HIGH) {

		P[10] = AP[15] * A[0] + P[11] * A[1] + P[12] * A[2] + P[13] * A[3];
		P[11] = AP[15] * A[4] + P[11] * A[5] + P[12] * A[6] + P[13] * A[7]
			+ P[14] * A[8];
		P[12] = P[12] + P[13] * A[9];

		P[14] += Q[4];
	}
	else {
		P[10] = 0.f;
		P[11] = 0.f;
		P[12] = 0.f;
		P[13] = 0.f;
		P[14] = 0.f;
	}
}

static void
pm_kalman_update(pmc_t *pm, const float X[2])
{
	float		*P = pm->kalman_P;
	float		*K = pm->kalman_K;

	float		CP[5], SI;

	/*
	 * Calculate updated (a posteriori) covariance and Kalman gain.
	 *
	 * S = C * P * C' + R.
	 * K = P * C' / S.
	 * P = P - K * C * P.
	 *
	 * C(1) = [ 1  0  -X[1]  0  0 ]
	 * C(2) = [ 0  1   X[0]  0  0 ]
	 *
	 * */

	CP[0] = P[0] - X[1] * P[3];
	CP[1] = P[1] - X[1] * P[4];
	CP[2] = P[3] - X[1] * P[5];
	CP[3] = P[6] - X[1] * P[8];
	CP[4] = P[10] - X[1] * P[12];

	SI = 1.f / (CP[0] - CP[2] * X[1] + pm->kalman_gain_R);

	K[0] = CP[0] * SI;
	K[2] = CP[1] * SI;
	K[4] = CP[2] * SI;
	K[6] = CP[3] * SI;
	K[8] = CP[4] * SI;

	P[0] += - K[0] * CP[0];
	P[1] += - K[2] * CP[0];
	P[2] += - K[2] * CP[1];
	P[3] += - K[4] * CP[0];
	P[4] += - K[4] * CP[1];
	P[5] += - K[4] * CP[2];
	P[6] += - K[6] * CP[0];
	P[7] += - K[6] * CP[1];
	P[8] += - K[6] * CP[2];
	P[9] += - K[6] * CP[3];
	P[10] += - K[8] * CP[0];
	P[11] += - K[8] * CP[1];
	P[12] += - K[8] * CP[2];
	P[13] += - K[8] * CP[3];
	P[14] += - K[8] * CP[4];

	CP[0] = P[1] + X[0] * P[3];
	CP[1] = P[2] + X[0] * P[4];
	CP[2] = P[4] + X[0] * P[5];
	CP[3] = P[7] + X[0] * P[8];
	CP[4] = P[11] + X[0] * P[12];

	SI = 1.f / (CP[1] + CP[2] * X[0] + pm->kalman_gain_R);

	K[1] = CP[0] * SI;
	K[3] = CP[1] * SI;
	K[5] = CP[2] * SI;
	K[7] = CP[3] * SI;
	K[9] = CP[4] * SI;

	P[0] += - K[1] * CP[0];
	P[1] += - K[3] * CP[0];
	P[2] += - K[3] * CP[1];
	P[3] += - K[5] * CP[0];
	P[4] += - K[5] * CP[1];
	P[5] += - K[5] * CP[2];
	P[6] += - K[7] * CP[0];
	P[7] += - K[7] * CP[1];
	P[8] += - K[7] * CP[2];
	P[9] += - K[7] * CP[3];
	P[10] += - K[9] * CP[0];
	P[11] += - K[9] * CP[1];
	P[12] += - K[9] * CP[2];
	P[13] += - K[9] * CP[3];
	P[14] += - K[9] * CP[4];
}

static void
pm_kalman_lock_guard(pmc_t *pm, float A)
{
	float		threshold_wS;
	int		k_UNLOCK = PM_DISABLED;

	/* Bare speed estiamte (LPF).
	 * */
	pm->kalman_lpf_wS += (A * pm->freq_hz - pm->kalman_lpf_wS) * pm->zone_gain_LP;

	if (		   pm->flux_ZONE == PM_ZONE_NONE
			|| pm->flux_ZONE == PM_ZONE_UNCERTAIN) {

		threshold_wS = pm->zone_threshold_BASE * pm->zone_gain_TH;

		if (		pm->kalman_lpf_wS < - threshold_wS
				&& pm->flux_wS > threshold_wS) {

			k_UNLOCK = PM_ENABLED;
		}
		else if (	pm->kalman_lpf_wS > threshold_wS
				&& pm->flux_wS < - threshold_wS) {

			k_UNLOCK = PM_ENABLED;
		}

		if (k_UNLOCK == PM_ENABLED) {

			/* Restart Kalman and Flip DQ-frame.
			 * */
			pm->flux_TYPE = PM_FLUX_NONE;

			pm->flux_F[0] = - pm->flux_F[0];
			pm->flux_F[1] = - pm->flux_F[1];
			pm->flux_wS = pm->kalman_lpf_wS;

			pm->kalman_POSTPONED = PM_DISABLED;
		}
	}
}

static void
pm_flux_KALMAN(pmc_t *pm)
{
	const float		*K = pm->kalman_K;
	float			eD, eQ, A, bF[2];

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		pm_kalman_solve_TVM(pm, pm->flux_X, pm->flux_F);
	}

	/* DQ-axes position frame.
	 * */
	bF[0] = pm->flux_F[0];
	bF[1] = pm->flux_F[1];

	/* Get the current RESIDUE in DQ-axes.
	 * */
	eD = bF[0] * pm->lu_iX + bF[1] * pm->lu_iY - pm->flux_X[0];
	eQ = bF[0] * pm->lu_iY - bF[1] * pm->lu_iX - pm->flux_X[1];

	if (pm->const_E > M_EPS_F) {

		if (pm->vsi_IF == 0) {

			pm->flux_X[0] += K[0] * eD + K[1] * eQ;
			pm->flux_X[1] += K[2] * eD + K[3] * eQ;

			A = K[4] * eD + K[5] * eQ;
			A = (A < - 1.f) ? - 1.f : (A > 1.f) ? 1.f : A;

			m_rotatef(pm->flux_F, A);

			pm->flux_wS += K[6] * eD + K[7] * eQ;

			if (pm->flux_gain_IF > M_EPS_F) {

				A = pm_torque_accel(pm, pm->flux_X[0], pm->flux_X[1]);
				pm->flux_wS += A * pm->dT * pm->flux_gain_IF;
			}

			if (pm->flux_ZONE == PM_ZONE_HIGH) {

				pm->kalman_bias_Q += K[8] * eD + K[9] * eQ;
			}
			else {
				pm->kalman_bias_Q = 0.f;
			}
		}

		pm->kalman_POSTPONED = PM_ENABLED;

		/* Build sparse Jacobian and postpone the rest of work.
		 * */
		pm_kalman_jacobian(pm, pm->flux_X, pm->flux_F, pm->flux_wS);
	}
	else {
		/* Startup estimate borrowing.
		 * */
		pm->flux_X[0] = pm->lu_iD;
		pm->flux_X[1] = pm->lu_iQ;
		pm->flux_F[0] = pm->lu_F[0];
		pm->flux_F[1] = pm->lu_F[1];
		pm->flux_wS = pm->lu_wS;

		/* Startup estimate E constant.
		 * */
		pm->kalman_bias_Q += pm->flux_gain_IN * eQ;
	}

	/* Time update to the next cycle.
	 * */
	pm_kalman_solve(pm, pm->flux_X, pm->flux_F, pm->flux_wS);

	/* Guard from lock in an inverted position with reversed speed.
	 * */
	pm_kalman_lock_guard(pm, bF[0] * pm->flux_F[1] - bF[1] * pm->flux_F[0]);
}

static void
pm_flux_ZONE(pmc_t *pm)
{
	float			threshold_wS;
	int			lev_TIM;

	/* Get smooth speed passed through LPF.
	 * */
	pm->zone_lpf_wS += (pm->flux_wS - pm->zone_lpf_wS) * pm->zone_gain_LP;

	if (		   pm->flux_ZONE == PM_ZONE_NONE
			|| pm->flux_ZONE == PM_ZONE_UNCERTAIN) {

		threshold_wS = pm->zone_threshold_BASE + pm->zone_threshold_NOISE;

		if (pm->lu_MODE == PM_LU_DETACHED) {

			lev_TIM = PM_TSMS(pm, pm->tm_transient_slow);

			if (		m_fabsf(pm->zone_lpf_wS) > threshold_wS
					&& pm->detach_TIM > lev_TIM) {

				pm->flux_ZONE = PM_ZONE_HIGH;
			}
		}
		else {
			if (		pm->zone_lpf_wS > threshold_wS
					&& pm->lu_wS > threshold_wS) {

				pm->flux_ZONE = PM_ZONE_HIGH;
			}
			else if (	pm->zone_lpf_wS < - threshold_wS
					&& pm->lu_wS < - threshold_wS) {

				pm->flux_ZONE = PM_ZONE_HIGH;
			}
		}
	}
	else if (pm->flux_ZONE == PM_ZONE_HIGH) {

		threshold_wS = pm->zone_threshold_BASE * pm->zone_gain_TH;

		if (pm->lu_MODE == PM_LU_DETACHED) {

			if (		m_fabsf(pm->zone_lpf_wS) < threshold_wS
					|| pm->detach_TIM < 10) {

				pm->flux_ZONE = PM_ZONE_UNCERTAIN;
			}
		}
		else {
			if (m_fabsf(pm->zone_lpf_wS) < threshold_wS) {

				pm->flux_ZONE = PM_ZONE_UNCERTAIN;
			}
		}
	}
}

static void
pm_estimate_FLUX(pmc_t *pm)
{
	if (pm->config_LU_ESTIMATE_FLUX == PM_FLUX_ORTEGA) {

		if (pm->flux_TYPE != PM_FLUX_ORTEGA) {

			float			E1, L2;

			E1 = pm->const_E;
			L2 = pm->const_im_L2;

			pm->flux_X[0] = E1 * pm->flux_F[0] + L2 * pm->lu_iX;
			pm->flux_X[1] = E1 * pm->flux_F[1] + L2 * pm->lu_iY;

			pm->flux_TYPE = PM_FLUX_ORTEGA;
		}

		pm_flux_ORTEGA(pm);
	}
	else if (pm->config_LU_ESTIMATE_FLUX == PM_FLUX_KALMAN) {

		if (pm->flux_TYPE != PM_FLUX_KALMAN) {

			pm->flux_X[0] = pm->lu_iD;
			pm->flux_X[1] = pm->lu_iQ;

			pm->kalman_P[0] = 0.f;
			pm->kalman_P[1] = 0.f;
			pm->kalman_P[2] = 0.f;
			pm->kalman_P[3] = 0.f;
			pm->kalman_P[4] = 0.f;
			pm->kalman_P[5] = 1.f;
			pm->kalman_P[6] = 0.f;
			pm->kalman_P[7] = 0.f;
			pm->kalman_P[8] = 0.f;
			pm->kalman_P[9] = 0.f;
			pm->kalman_P[10] = 0.f;
			pm->kalman_P[11] = 0.f;
			pm->kalman_P[12] = 0.f;
			pm->kalman_P[13] = 0.f;
			pm->kalman_P[14] = 0.f;

			pm->kalman_K[0] = 0.f;
			pm->kalman_K[1] = 0.f;
			pm->kalman_K[2] = 0.f;
			pm->kalman_K[3] = 0.f;
			pm->kalman_K[4] = 0.f;
			pm->kalman_K[5] = 0.f;
			pm->kalman_K[6] = 0.f;
			pm->kalman_K[7] = 0.f;
			pm->kalman_K[8] = 0.f;
			pm->kalman_K[9] = 0.f;

			pm->kalman_bias_Q = 0.f;

			pm->flux_TYPE = PM_FLUX_KALMAN;
		}

		pm_flux_KALMAN(pm);
	}
	else {
		/* NOTE: No sensorless observer selected. It is ok when you
		 * only need a SENSORED drive */

		if (pm->flux_TYPE != PM_FLUX_NONE) {

			pm->flux_TYPE = PM_FLUX_NONE;
		}
	}

	if (pm->config_LU_ESTIMATE_FLUX != PM_FLUX_NONE) {

		pm_flux_ZONE(pm);
	}
}

void pm_hfi_DFT(pmc_t *pm, float la[5])
{
	lse_t		*ls = &pm->probe_LS[0];
	lse_float_t	v[5];

	float		*DFT = pm->hfi_DFT;
	float		Z[3], iW;

	/* The primary impedance equation is \Z * \I = \U,
	 *
	 * [R - jZ(0)      jZ(1)] * [IX] = [UX]
	 * [    jZ(1)  R - jZ(2)]   [IY]   [UY], where
	 *
	 * IX = [DFT(0) + jDFT(1)],  UX = [DFT(2) + jDFT(3)],
	 * IY = [DFT(4) + jDFT(5)],  UY = [DFT(6) + jDFT(7)].
	 *
	 * We rewrite it with respect to the impedance components.
	 *
	 * [DFT(0)  DFT(1) -DFT(5)  0     ]   [R   ]   [DFT(2)]
	 * [DFT(1) -DFT(0)  DFT(4)  0     ] * [Z(0)] = [DFT(3)]
	 * [DFT(4)  0      -DFT(1)  DFT(5)]   [Z(1)]   [DFT(6)].
	 * [DFT(5)  0       DFT(0) -DFT(4)]   [Z(2)]   [DFT(7)]
	 *
	 * */

	lse_construct(ls, LSE_CASCADE_MAX, 4, 1);

	v[0] = DFT[0];
	v[1] = DFT[1];
	v[2] = - DFT[5];
	v[3] = 0.f;
	v[4] = DFT[2];

	lse_reduce(ls, v);

	v[0] = DFT[1];
	v[1] = - DFT[0];
	v[2] = DFT[4];
	v[3] = 0.f;
	v[4] = DFT[3];

	lse_reduce(ls, v);

	v[0] = DFT[4];
	v[1] = 0.f;
	v[2] = - DFT[1];
	v[3] = DFT[5];
	v[4] = DFT[6];

	lse_reduce(ls, v);

	v[0] = DFT[5];
	v[1] = 0.f;
	v[2] = DFT[0];
	v[3] = - DFT[4];
	v[4] = DFT[7];

	lse_reduce(ls, v);
	lse_solve(ls);

	iW = 1.f / pm->quick_hfwS;

	la[4] = ls->sol.m[0];

	Z[0] = ls->sol.m[1] * iW;
	Z[1] = ls->sol.m[2] * iW;
	Z[2] = ls->sol.m[3] * iW;

	m_la_eigf(Z, la, (pm->config_HFI_IMPEDANCE == PM_ENABLED) ? 1 : 0);
}

static void
pm_hfi_polarity(pmc_t *pm, float iD, float F[2])
{
	float		pol_WAVE, pol_TH;

	if (pm->hfi_IN >= pm->hfi_INJS * pm->hfi_SKIP) {

		/* Get reference doubled frequency cosine wave.
		 * */
		pol_WAVE = pm->hfi_wave[0] * pm->hfi_wave[0]
			- pm->hfi_wave[1] * pm->hfi_wave[1];

		/* Extract D-axis polarity (based on the magnetic saturation).
		 * */
		m_rsumf(&pm->hfi_DFT[8], &pm->hfi_REM[8], iD * pol_WAVE);
	}
	else if (pm->hfi_IN == 0) {

		pol_TH = pm->hfi_gain_FP * pm->hfi_sine;

		if (pm->hfi_DFT[8] < - pol_TH) {

			/* Flip DQ-frame.
			 * */
			F[0] = - F[0];
			F[1] = - F[1];

			pm->hfi_DFT[8] = 0.f;
			pm->hfi_REM[8] = 0.f;
		}
		else if (pm->hfi_DFT[8] > pol_TH) {

			pm->hfi_DFT[8] = pol_TH;
			pm->hfi_REM[8] = 0.f;
		}
	}
}

static void
pm_hfi_EIGEN(pmc_t *pm)
{
	float		iD, iQ, uD, uQ;
	float		la[5], hF[2], E, A;

	iD = pm->hfi_F[0] * pm->hfi_REM[10] + pm->hfi_F[1] * pm->hfi_REM[11];
	iQ = pm->hfi_F[0] * pm->hfi_REM[11] - pm->hfi_F[1] * pm->hfi_REM[10];

	pm->hfi_REM[10] = pm->lu_iX;
	pm->hfi_REM[11] = pm->lu_iY;

	if (pm->hfi_IN >= pm->hfi_INJS * pm->hfi_SKIP) {

		/* Get corrected frame (half-period shift).
		 * */
		hF[0] = pm->hfi_F[0] * pm->quick_hfSC[0] - pm->hfi_F[1] * pm->quick_hfSC[1];
		hF[1] = pm->hfi_F[1] * pm->quick_hfSC[0] + pm->hfi_F[0] * pm->quick_hfSC[1];

		/* Get VSI voltages on DQ-axes.
		 * */
		uD = hF[0] * pm->tvm_DX + hF[1] * pm->tvm_DY;
		uQ = hF[0] * pm->tvm_DY - hF[1] * pm->tvm_DX;

		/* Discrete Fourier Transform (DFT).
		 * */
		m_rsumf(&pm->hfi_DFT[0], &pm->hfi_REM[0], iD * pm->hfi_wave[0]);
		m_rsumf(&pm->hfi_DFT[1], &pm->hfi_REM[1], iD * pm->hfi_wave[1]);
		m_rsumf(&pm->hfi_DFT[2], &pm->hfi_REM[2], uD * pm->hfi_wave[0]);
		m_rsumf(&pm->hfi_DFT[3], &pm->hfi_REM[3], uD * pm->hfi_wave[1]);
		m_rsumf(&pm->hfi_DFT[4], &pm->hfi_REM[4], iQ * pm->hfi_wave[0]);
		m_rsumf(&pm->hfi_DFT[5], &pm->hfi_REM[5], iQ * pm->hfi_wave[1]);
		m_rsumf(&pm->hfi_DFT[6], &pm->hfi_REM[6], uQ * pm->hfi_wave[0]);
		m_rsumf(&pm->hfi_DFT[7], &pm->hfi_REM[7], uQ * pm->hfi_wave[1]);
	}
	else if (pm->hfi_IN == 0) {

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
	}

	if (pm->config_HFI_POLARITY == PM_ENABLED) {

		pm_hfi_polarity(pm, iD, pm->hfi_F);
	}

	pm->hfi_IN += 1;

	if (pm->hfi_IN >= pm->hfi_INJS * (pm->hfi_SKIP + pm->hfi_ESTI)) {

		pm_hfi_DFT(pm, la);

		if (m_isfinitef(la[0]) != 0 && m_isfinitef(la[1]) != 0) {

			/* Speed estimation (PLL).
			 * */
			if (la[0] > M_EPS_F) {

				E = la[1] * pm->freq_hz;
				pm->hfi_wS += E * pm->hfi_gain_SF;
			}

			/* Get actual saliency frame.
			 * */
			hF[0] = pm->hfi_F[0] * la[0] - pm->hfi_F[1] * la[1];
			hF[1] = pm->hfi_F[1] * la[0] + pm->hfi_F[0] * la[1];

			A = (3.f - hF[0] * hF[0] - hF[1] * hF[1]) * .5f;

			pm->hfi_F[0] = hF[0] * A;
			pm->hfi_F[1] = hF[1] * A;

			pm->hfi_im_L1 = la[2];
			pm->hfi_im_L2 = la[3];
			pm->hfi_im_R  = la[4];
		}

		pm->hfi_IN = 0;
	}

	if (pm->hfi_gain_IF > M_EPS_F) {

		A = pm_torque_accel(pm, pm->lu_iD, pm->lu_iQ);
		pm->hfi_wS += A * pm->dT * pm->hfi_gain_IF;
	}

	/* Maximal HFI speed constraint.
	 * */
	pm->hfi_wS = (pm->hfi_wS > pm->hfi_maximal) ? pm->hfi_maximal :
		(pm->hfi_wS < - pm->hfi_maximal) ? - pm->hfi_maximal : pm->hfi_wS;

	/* Transfer the rotor position.
	 * */
	m_rotatef(pm->hfi_F, pm->hfi_wS * pm->dT);

	/* HF wave synthesis.
	 * */
	m_rotatef(pm->hfi_wave, pm->quick_hfwS * pm->dT);

	/* Enable to inject HF wave inside the current loop.
	 * */
	pm->hfi_INJECT = PM_ENABLED;
}

static void
pm_hfi_on_KALMAN(pmc_t *pm)
{
	if (pm->config_HFI_POLARITY == PM_ENABLED) {

		pm_hfi_polarity(pm, pm->hfi_REM[10], pm->flux_F);
	}

	pm->hfi_REM[10] = pm->flux_X[0];

	pm->hfi_IN += 1;

	if (pm->hfi_IN >= pm->hfi_INJS * (pm->hfi_SKIP + pm->hfi_ESTI)) {

		/* HFI cycle restart manually.
		 * */
		pm->hfi_IN = 0;
	}

	/* HF wave synthesis.
	 * */
	m_rotatef(pm->hfi_wave, pm->quick_hfwS * pm->dT);

	/* Enable to inject HF wave inside the current loop.
	 * */
	pm->hfi_INJECT = PM_ENABLED;
}

static void
pm_estimate_HFI(pmc_t *pm)
{
	if (pm->config_LU_ESTIMATE_HFI == PM_HFI_EIGEN) {

		if (pm->hfi_TYPE != PM_HFI_EIGEN) {

			pm->hold_TIM = 0;

			pm->hfi_F[0] = pm->lu_F[0];
			pm->hfi_F[1] = pm->lu_F[1];
			pm->hfi_wS = pm->lu_wS;
			pm->hfi_IN = 0;

			pm->hfi_TYPE = PM_HFI_EIGEN;
		}

		pm_hfi_EIGEN(pm);
	}
	else if (pm->config_LU_ESTIMATE_HFI == PM_HFI_ON_KALMAN) {

		if (pm->hfi_TYPE != PM_HFI_ON_KALMAN) {

			pm->hold_TIM = 0;
			pm->hfi_IN = 0;

			pm->hfi_TYPE = PM_HFI_ON_KALMAN;
		}

		pm_hfi_on_KALMAN(pm);
	}
	else {
		/* NOTE: You should not run the HFI in this case.
		 * */
		pm->hfi_TYPE = PM_HFI_NONE;
	}
}

static void
pm_sensor_HALL(pmc_t *pm)
{
	float		hF[2], A, B, relE, gain_SF;
	int		HS, lev_PROL;

	const float	avg_SPAN = 1.0472f;	/* ~60 degree */

	if (pm->hall_OPERATE != PM_ENABLED) {

		pm->hall_base_HS = pm->fb_HS;
		pm->hall_rel = 0;
		pm->hall_prol = 0.f;
		pm->hall_TIM = PM_TSMS(pm, pm->hall_time_prol);
		pm->hall_ERN = 0;

		pm->hall_F[0] = pm->lu_F[0];
		pm->hall_F[1] = pm->lu_F[1];
		pm->hall_wS = pm->lu_wS;

		pm->hall_OPERATE = PM_ENABLED;
	}

	HS = pm->fb_HS;

	if (HS >= 1 && HS <= 6) {

		pm->hall_TIM++;
		pm->hall_ERN = 0;

		if (HS == pm->hall_base_HS) {

			if (pm->hall_rel != 0) {

				relE = pm->hall_wS * pm->hall_gain_PF * pm->dT;
				B = avg_SPAN * pm->hall_gain_PF;

				if (		m_fabsf(relE) > M_EPS_F
						&& m_fabsf(pm->hall_prol) < B) {

					m_rotatef(pm->hall_F, relE);

					pm->hall_prol += relE;
				}
				else {
					relE = 0.f;
				}

				lev_PROL = PM_TSMS(pm, pm->hall_time_prol);

				if (pm->hall_TIM >= lev_PROL) {

					pm->hall_rel = 0;

					relE = 0.f;
				}
			}
			else {
				pm->hall_F[0] = pm->hall_ST[HS].X;
				pm->hall_F[1] = pm->hall_ST[HS].Y;

				relE = 0.f;
			}
		}
		else {
			hF[0] = pm->hall_ST[HS].X;
			hF[1] = pm->hall_ST[HS].Y;

			A = pm->hall_ST[pm->hall_base_HS].X;
			B = pm->hall_ST[pm->hall_base_HS].Y;

			relE = hF[1] * A - hF[0] * B;
			pm->hall_rel = (relE < 0.f) ? - 1 : 1;

			relE = - pm->hall_rel * (avg_SPAN / 2.f);

			m_rotatef(hF, relE * pm->hall_gain_PF);

			A = hF[0] * pm->hall_F[0] + hF[1] * pm->hall_F[1];
			B = hF[1] * pm->hall_F[0] - hF[0] * pm->hall_F[1];

			pm->hall_prol = 0.f;
			pm->hall_F[0] = hF[0];
			pm->hall_F[1] = hF[1];

			relE = m_atan2f(B, A);
		}

		gain_SF = 1.f / (float) pm->hall_TIM;
		gain_SF = (gain_SF < pm->hall_gain_SF) ? gain_SF : pm->hall_gain_SF;

		pm->hall_wS += (relE * pm->freq_hz - pm->hall_wS) * gain_SF;

		B = avg_SPAN * pm->hall_gain_PF;

		if (		pm->hall_gain_IF > M_EPS_F
				&& m_fabsf(pm->hall_prol) < B) {

			A = pm_torque_accel(pm, pm->lu_iD, pm->lu_iQ);
			pm->hall_wS += A * pm->dT * pm->hall_gain_IF;
		}

		if (HS != pm->hall_base_HS) {

			pm->hall_base_HS = HS;
			pm->hall_TIM = 0;
		}
	}
	else {
		pm->hall_ERN++;

		if (pm->hall_ERN >= 10) {

			pm->hall_USEABLE = PM_DISABLED;

			pm->fsm_errno = PM_ERROR_SENSOR_HALL_FAULT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}
}

static void
pm_sensor_ABI(pmc_t *pm)
{
	float			relAN, locAN, luF[2], gain_SF, A;
	int			relEP, uEP, WRAP;

	if (pm->abi_OPERATE != PM_ENABLED) {

		pm->abi_base_EP = pm->fb_EP;
		pm->abi_unwrap = 0;
		pm->abi_rel_EP = 0;
		pm->abi_loc_EP = 0;
		pm->abi_TIM = 0;
		pm->abi_prol = 0.f;

		if (pm->config_ABI_FRONTEND == PM_ABI_INCREMENTAL) {

			pm->abi_loc_F[0] = pm->lu_F[0];
			pm->abi_loc_F[1] = pm->lu_F[1];
		}

		pm->abi_F[0] = pm->lu_F[0];
		pm->abi_F[1] = pm->lu_F[1];
		pm->abi_wS = pm->lu_wS;

		pm->abi_OPERATE = PM_ENABLED;
	}

	if (pm->config_ABI_FRONTEND == PM_ABI_INCREMENTAL) {

		WRAP = 0x10000U;

		relEP = pm->fb_EP - pm->abi_base_EP;
		relEP +=  (relEP > (WRAP / 2 - 1)) ? - WRAP
			: (relEP < (1 - WRAP / 2 - 1)) ? WRAP : 0;

		pm->abi_base_EP = pm->fb_EP;
	}
	else if (pm->config_ABI_FRONTEND == PM_ABI_ABSOLUTE) {

		WRAP = pm->abi_EPPR;

		pm->abi_base_EP = (pm->abi_gear_Zq > 1)
			? pm->abi_loc_EP % WRAP : pm->abi_loc_EP;

		relEP = pm->fb_EP - pm->abi_base_EP;
		relEP +=  (relEP > (WRAP / 2 - 1)) ? - WRAP
			: (relEP < (1 - WRAP / 2 - 1)) ? WRAP : 0;
	}
	else {
		relEP = 0;
	}

	/* Debounce (around one EP).
	 * */
	uEP =  (relEP > 0 && pm->abi_rel_EP < 0) ? - 1 : 0;
	uEP += (relEP < 0 && pm->abi_rel_EP > 0) ?   1 : 0;

	pm->abi_rel_EP = (relEP != 0) ? relEP : pm->abi_rel_EP;

	relEP += uEP;

	pm->abi_TIM++;

	if (relEP == 0) {

		relAN = pm->abi_wS * pm->abi_gain_PF * pm->dT;
		relAN = (m_fabsf(pm->abi_prol) < pm->quick_ZiEP) ? relAN : 0.f;

		pm->abi_prol += relAN;
	}
	else {
		relAN = (float) relEP * pm->quick_ZiEP - pm->abi_prol;

		pm->abi_loc_EP += relEP;
		pm->abi_prol = 0.f;

		WRAP = pm->abi_EPPR * pm->abi_gear_Zq;

		if (pm->abi_loc_EP < - WRAP) {

			pm->abi_unwrap += - pm->abi_gear_Zq;
			pm->abi_loc_EP += WRAP;
		}
		else if (pm->abi_loc_EP > WRAP) {

			pm->abi_unwrap += pm->abi_gear_Zq;
			pm->abi_loc_EP += - WRAP;
		}
	}

	if (pm->config_LU_LOCATION == PM_LOCATION_ABI) {

		float		locEP;

		/* Take the electrical absolute LOCATION.
		 * */
		locEP = (float) pm->abi_unwrap * (float) pm->abi_EPPR
			+ (float) pm->abi_loc_EP;

		pm->abi_location = locEP * pm->quick_ZiEP + pm->abi_prol;
	}

	/* Take the electrical position.
	 * */
	locAN = (float) pm->abi_loc_EP * pm->quick_ZiEP + pm->abi_prol;
	locAN = m_wrapf(locAN);

	luF[0] = m_cosf(locAN);
	luF[1] = m_sinf(locAN);

	pm->abi_F[0] = luF[0] * pm->abi_loc_F[0] - luF[1] * pm->abi_loc_F[1];
	pm->abi_F[1] = luF[1] * pm->abi_loc_F[0] + luF[0] * pm->abi_loc_F[1];

	gain_SF = 1.f / (float) pm->abi_TIM;
	gain_SF = (gain_SF < pm->abi_gain_SF) ? gain_SF : pm->abi_gain_SF;

	/* Speed estimation.
	 * */
	pm->abi_wS += (relAN * pm->freq_hz - pm->abi_wS) * gain_SF;

	if (		pm->abi_gain_IF > M_EPS_F
			&& m_fabsf(pm->abi_prol) < pm->quick_ZiEP) {

		A = pm_torque_accel(pm, pm->lu_iD, pm->lu_iQ);
		pm->abi_wS += A * pm->dT * pm->abi_gain_IF;
	}

	if (relEP != 0) {

		pm->abi_TIM = 0;
	}
}

static void
pm_sensor_SINCOS(pmc_t *pm)
{
	float			*FIR = pm->sincos_FIR;
	float			scAN, locAN;
	int			WRAP;

	if (pm->sincos_OPERATE != PM_ENABLED) {

		pm->sincos_OPERATE = PM_ENABLED;
	}

	if (pm->config_SINCOS_FRONTEND == PM_SINCOS_ANALOG) {

		float		Q[9];

		Q[0] = pm->fb_COS;
		Q[1] = pm->fb_SIN;
		Q[2] = Q[0] * Q[1];
		Q[3] = Q[0] * Q[0];
		Q[4] = Q[1] * Q[1];
		Q[5] = Q[3] * Q[1];
		Q[6] = Q[4] * Q[0];
		Q[7] = Q[3] * Q[0];
		Q[8] = Q[4] * Q[1];

		pm->sincos_SC[0] = FIR[0] + FIR[2] * Q[0] + FIR[4]  * Q[1]
			+ FIR[6]  * Q[2] + FIR[8]  * Q[3] + FIR[10] * Q[4]
			+ FIR[12] * Q[5] + FIR[14] * Q[6] + FIR[16] * Q[7]
			+ FIR[18] * Q[8];

		pm->sincos_SC[0] = FIR[1] + FIR[3] * Q[0] + FIR[5]  * Q[1]
			+ FIR[7]  * Q[2] + FIR[9]  * Q[3] + FIR[11] * Q[4]
			+ FIR[13] * Q[5] + FIR[15] * Q[6] + FIR[17] * Q[7]
			+ FIR[19] * Q[8];
	}
	else if (pm->config_SINCOS_FRONTEND == PM_SINCOS_RESOLVER) {

		/* TODO */
	}

	/* Track the SIN/COS position.
	 * */
	if (pm->sincos_SC[0] < 0.f) {

		if (		   pm->sincos_SC[1] < 0.f
				&& pm->sincos_SC[2] >= 0.f) {

			pm->sincos_revol += 1;
		}
		else if (	   pm->sincos_SC[1] >= 0.f
				&& pm->sincos_SC[2] < 0.f) {

			pm->sincos_revol += - 1;
		}
	}

	pm->sincos_SC[2] = pm->sincos_SC[1];

	WRAP = pm->sincos_gear_Zq;

	if (pm->sincos_revol < - WRAP) {

		pm->sincos_unwrap += - WRAP;
		pm->sincos_revol += WRAP;
	}
	else if (pm->sincos_revol > WRAP) {

		pm->sincos_unwrap += WRAP;
		pm->sincos_revol += - WRAP;
	}

	scAN = m_atan2f(pm->sincos_SC[1], pm->sincos_SC[0])
		+ (float) pm->sincos_revol * (2.f * M_PI_F);

	if (pm->config_LU_LOCATION == PM_LOCATION_SINCOS) {

		float		scLOC;

		scLOC = scAN + (float) pm->sincos_unwrap * (2.f * M_PI_F);

		/* Take the electrical absolute LOCATION.
		 * */
		pm->sincos_location = scLOC * pm->quick_ZiSQ;
	}

	/* Take the electrical position.
	 * */
	locAN = m_wrapf(scAN * pm->quick_ZiSQ);

	pm->sincos_F[0] = m_cosf(locAN);
	pm->sincos_F[1] = m_sinf(locAN);

	/* TODO */
}

static void
pm_lu_FSM(pmc_t *pm)
{
	float			EX, EY, hTS, lu_F[2], hold_D, mQ;
	int			lev_SKIP, lev_HOLD;

	/* Get the current on DQ-axes.
	 * */
	pm->lu_iD = pm->lu_F[0] * pm->lu_iX + pm->lu_F[1] * pm->lu_iY;
	pm->lu_iQ = pm->lu_F[0] * pm->lu_iY - pm->lu_F[1] * pm->lu_iX;

	/* Transfer to the next apriori position.
	 * */
	m_rotatef(pm->lu_F, pm->lu_wS * pm->dT);

	if (pm->vsi_IF != 0) {

		/* The current prediction in XY-axes will be used in case of
		 * current samples are discarded.
		 * */
		pm->lu_iX = pm->lu_F[0] * pm->lu_iD - pm->lu_F[1] * pm->lu_iQ;
		pm->lu_iY = pm->lu_F[1] * pm->lu_iD + pm->lu_F[0] * pm->lu_iQ;
	}

	if (pm->lu_MODE == PM_LU_DETACHED) {

		if (pm->base_TIM >= 0) {

			pm_detached_BEMF(pm);
		}

		pm_flux_ZONE(pm);

		lu_F[0] = pm->flux_F[0];
		lu_F[1] = pm->flux_F[1];

		pm->lu_wS = pm->flux_wS;

		lev_SKIP = PM_TSMS(pm, pm->tm_startup);

		if (pm->flux_ZONE == PM_ZONE_LOCKED_IN_DETACH) {

			/* Lock in detached mode permanently.
			 * */
		}
		else if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE_FLUX;

			pm->proc_set_Z(PM_Z_NONE);
		}
		else if (pm->base_TIM < lev_SKIP) {

			/* Not enough time passed to go into low speed mode.
			 * */
			pm->base_TIM++;
		}
		else if (	pm->config_LU_SENSOR == PM_SENSOR_HALL
				&& pm->hall_USEABLE == PM_ENABLED) {

			pm->lu_MODE = PM_LU_SENSOR_HALL;
			pm->hall_OPERATE = PM_DISABLED;

			pm->proc_set_Z(PM_Z_NONE);
		}
		else if (	pm->config_LU_SENSOR == PM_SENSOR_ABI
				&& pm->config_ABI_FRONTEND == PM_ABI_ABSOLUTE
				&& pm->abi_USEABLE == PM_ENABLED) {

			pm->lu_MODE = PM_LU_SENSOR_ABI;
			pm->abi_OPERATE = PM_DISABLED;

			pm->proc_set_Z(PM_Z_NONE);
		}
		else if (	pm->config_LU_ESTIMATE_HFI != PM_HFI_NONE
				&& pm->config_HFI_POLARITY == PM_ENABLED) {

			pm->lu_MODE = PM_LU_ESTIMATE_HFI;
			pm->hfi_TYPE = PM_HFI_NONE;

			pm->proc_set_Z(PM_Z_NONE);
		}
		else if (pm->config_LU_FORCED == PM_ENABLED) {

			pm->lu_MODE = PM_LU_FORCED;

			pm->hold_TIM = 0;

			pm->forced_F[0] = pm->lu_F[0];
			pm->forced_F[1] = pm->lu_F[1];
			pm->forced_wS = pm->lu_wS;

			pm->proc_set_Z(PM_Z_NONE);
		}
	}
	else if (pm->lu_MODE == PM_LU_FORCED) {

		pm_estimate_FLUX(pm);
		pm_forced(pm);

		lu_F[0] = pm->forced_F[0];
		lu_F[1] = pm->forced_F[1];

		pm->lu_wS = pm->forced_wS;

		hold_D = (pm->forced_wS < 0.f)
			? - pm->forced_hold_D : pm->forced_hold_D;

		/* Assume maximal load torque.
		 * */
		pm->lu_load_torque = pm_torque_equation(pm, hold_D, hold_D);

		if (		pm->flux_ZONE == PM_ZONE_HIGH
				&& pm->const_E > M_EPS_F) {

			pm->lu_MODE = PM_LU_ESTIMATE_FLUX;
		}
		else {
			lev_HOLD = PM_TSMS(pm, pm->tm_current_hold);

			if (pm->hold_TIM < lev_HOLD) {

				pm->hold_TIM++;
			}
			else if (	pm->config_LU_SENSOR == PM_SENSOR_ABI
					&& pm->abi_USEABLE == PM_ENABLED) {

				pm->lu_MODE = PM_LU_SENSOR_ABI;
				pm->abi_OPERATE = PM_DISABLED;
			}
			else if (pm->config_LU_ESTIMATE_HFI != PM_HFI_NONE) {

				pm->lu_MODE = PM_LU_ESTIMATE_HFI;
				pm->hfi_TYPE = PM_HFI_NONE;
			}
		}
	}
	else if (pm->lu_MODE == PM_LU_ESTIMATE_FLUX) {

		pm_estimate_FLUX(pm);

		lu_F[0] = pm->flux_F[0];
		lu_F[1] = pm->flux_F[1];

		pm->lu_wS = pm->flux_wS;

		lev_SKIP = PM_TSMS(pm, pm->tm_startup);

		if (pm->base_TIM < lev_SKIP) {

			/* Not enough time passed to go into low speed mode.
			 * */
			pm->base_TIM++;
		}
		else if (	   pm->flux_ZONE == PM_ZONE_NONE
				|| pm->flux_ZONE == PM_ZONE_UNCERTAIN) {

			if (		pm->config_LU_SENSOR == PM_SENSOR_HALL
					&& pm->hall_USEABLE == PM_ENABLED) {

				pm->lu_MODE = PM_LU_SENSOR_HALL;
				pm->hall_OPERATE = PM_DISABLED;
			}
			else if (	pm->config_LU_SENSOR == PM_SENSOR_ABI
					&& pm->abi_USEABLE == PM_ENABLED
					&& pm->flux_ZONE != PM_ZONE_NONE) {

				pm->lu_MODE = PM_LU_SENSOR_ABI;
				pm->abi_OPERATE = PM_DISABLED;
			}
			else if (pm->config_LU_ESTIMATE_HFI != PM_HFI_NONE) {

				pm->lu_MODE = PM_LU_ESTIMATE_HFI;
				pm->hfi_TYPE = PM_HFI_NONE;
			}
			else if (pm->config_LU_FORCED == PM_ENABLED) {

				pm->lu_MODE = PM_LU_FORCED;

				pm->hold_TIM = 0;

				pm->forced_F[0] = pm->lu_F[0];
				pm->forced_F[1] = pm->lu_F[1];
				pm->forced_wS = pm->lu_wS;
			}
			else if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				pm->lu_MODE = PM_LU_DETACHED;

				pm->base_TIM = - PM_TSMS(pm, pm->tm_transient_fast);
				pm->detach_TIM = 0;

				pm->watt_lpf_D = 0.f;
				pm->watt_lpf_Q = 0.f;
				pm->watt_lpf_wP = 0.f;

				pm->proc_set_Z(PM_Z_ABC);
			}
		}
	}
	else if (pm->lu_MODE == PM_LU_ESTIMATE_HFI) {

		pm_estimate_FLUX(pm);
		pm_estimate_HFI(pm);

		if (pm->hfi_TYPE == PM_HFI_EIGEN) {

			lu_F[0] = pm->hfi_F[0];
			lu_F[1] = pm->hfi_F[1];

			pm->lu_wS = pm->hfi_wS;
		}
		else if (pm->hfi_TYPE == PM_HFI_ON_KALMAN) {

			lu_F[0] = pm->flux_F[0];
			lu_F[1] = pm->flux_F[1];

			pm->lu_wS = pm->flux_wS;
		}
		else {
			lu_F[0] = pm->lu_F[0];
			lu_F[1] = pm->lu_F[1];
		}

		if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE_FLUX;
		}
		else {
			lev_HOLD = PM_TSMS(pm, pm->tm_startup);

			if (pm->hold_TIM < lev_HOLD) {

				pm->hold_TIM++;
			}
			else if (	pm->config_LU_SENSOR == PM_SENSOR_ABI
					&& pm->abi_USEABLE == PM_ENABLED) {

				pm->lu_MODE = PM_LU_SENSOR_ABI;
				pm->abi_OPERATE = PM_DISABLED;
			}
		}
	}
	else if (pm->lu_MODE == PM_LU_SENSOR_HALL) {

		pm_estimate_FLUX(pm);
		pm_sensor_HALL(pm);

		lu_F[0] = pm->hall_F[0];
		lu_F[1] = pm->hall_F[1];

		pm->lu_wS = pm->hall_wS;

		if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE_FLUX;
		}
	}
	else if (pm->lu_MODE == PM_LU_SENSOR_ABI) {

		pm_estimate_FLUX(pm);
		pm_sensor_ABI(pm);

		if (PM_CONFIG_DEBUG(pm) == PM_ENABLED) {

			pm_estimate_HFI(pm);
		}

		lu_F[0] = pm->abi_F[0];
		lu_F[1] = pm->abi_F[1];

		pm->lu_wS = pm->abi_wS;

		if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE_FLUX;
		}
	}
	else if (pm->lu_MODE == PM_LU_SENSOR_SINCOS) {

		/* TODO */
	}

	/* Take the LU position estimate with TRANSIENT rate limited.
	 * */
	hTS = pm->lu_transient * pm->dT;

	EX = lu_F[0] * pm->lu_F[0] + lu_F[1] * pm->lu_F[1];
	EY = lu_F[1] * pm->lu_F[0] - lu_F[0] * pm->lu_F[1];

	if (EX > M_EPS_F && EY < - hTS) {

		m_rotatef(pm->lu_F, - hTS);
	}
	else if (EX > M_EPS_F && EY > hTS) {

		m_rotatef(pm->lu_F, hTS);
	}
	else {
		pm->lu_F[0] = lu_F[0];
		pm->lu_F[1] = lu_F[1];

		if (EX < - M_EPS_F) {

			/* Reset integrals in case of position FLIP.
			 * */
			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;
		}
	}

	/* Track the position to get full number of revolutions.
	 * */
	if (pm->lu_F[0] < 0.f) {

		if (		   pm->lu_F[1] < 0.f
				&& pm->lu_F[2] >= 0.f) {

			pm->lu_revol += 1;
		}
		else if (	   pm->lu_F[1] >= 0.f
				&& pm->lu_F[2] < 0.f) {

			pm->lu_revol += - 1;
		}
	}

	pm->lu_F[2] = pm->lu_F[1];

	if (pm->lu_revol - pm->lu_revob < - 4) {

		pm->lu_total_revol += pm->lu_revob - pm->lu_revol;
		pm->lu_revob = pm->lu_revol;
	}
	else if (pm->lu_revol - pm->lu_revob > 4) {

		pm->lu_total_revol += pm->lu_revol - pm->lu_revob;
		pm->lu_revob = pm->lu_revol;
	}

	/* Take the LOCATION according to the configuration.
	 * */
	if (pm->config_LU_LOCATION == PM_LOCATION_INHERITED) {

		pm->lu_location = m_atan2f(pm->lu_F[1], pm->lu_F[0])
			+ (float) pm->lu_revol * (2.f * M_PI_F);
	}
	else if (pm->config_LU_LOCATION == PM_LOCATION_ABI) {

		if (pm->lu_MODE != PM_LU_SENSOR_ABI) {

			pm_sensor_ABI(pm);
		}

		pm->lu_wS = pm->abi_wS;
		pm->lu_location = pm->abi_location;
	}
	else if (pm->config_LU_LOCATION == PM_LOCATION_SINCOS) {

		/* TODO */
	}

	if (		pm->flux_TYPE == PM_FLUX_KALMAN
			&& pm->lu_MODE == PM_LU_ESTIMATE_FLUX) {

		/* Replace the current on DQ-axes with predicted one.
		 * */
		pm->lu_iD = pm->flux_X[0];
		pm->lu_iQ = pm->flux_X[1];
	}

	if (pm->lu_MODE != PM_LU_FORCED) {

		/* Get an external mechanical LOAD torque estimate.
		 * */
		mQ = pm_torque_equation(pm, pm->lu_iD, pm->lu_iQ)
			- (pm->lu_wS - pm->lu_base_wS) * pm->freq_hz * pm->const_Ja;

		/* Pass through LPF to suppress the noise.
		 * */
		pm->lu_load_torque += (mQ - pm->lu_load_torque) * pm->lu_gain_QF;
	}

	pm->lu_base_wS = pm->lu_wS;
}

void pm_clearance(pmc_t *pm, int xA, int xB, int xC)
{
	int		xMIN;

	xA = pm->dc_resolution - xA;
	xB = pm->dc_resolution - xB;
	xC = pm->dc_resolution - xC;

	/* Check if there are PWM edges within clearance zone. The CURRENT
	 * measurements will be used or rejected based on this flags.
	 *
	 * NOTE: In case of INLINE current measurement it is possible to get
	 * sample at the TOP.
	 *
	 * NOTE: To get best result you should have a current sensor with a
	 * fast transient that allows you to specify narrow clearance zone.
	 *
	 *                   1 - sqrt(3) / 2
	 * 	clearance < -----------------
	 *                      pwm_freq
	 *
	 * */
	if (PM_CONFIG_IFB(pm) == PM_IFB_AB_INLINE) {

		pm->vsi_AF = ((pm->vsi_AG >= pm->ts_clearance && xA > pm->ts_skip)
				|| (pm->vsi_AG == 0 && xA == 0)) ? 0 : 1;
		pm->vsi_BF = ((pm->vsi_BG >= pm->ts_clearance && xB > pm->ts_skip)
				|| (pm->vsi_BG == 0 && xB == 0)) ? 0 : 1;
		pm->vsi_CF = 1;
	}
	else if (PM_CONFIG_IFB(pm) == PM_IFB_AB_GND) {

		pm->vsi_AF = (pm->vsi_AG >= pm->ts_clearance && xA > pm->ts_skip) ? 0 : 1;
		pm->vsi_BF = (pm->vsi_BG >= pm->ts_clearance && xB > pm->ts_skip) ? 0 : 1;
		pm->vsi_CF = 1;
	}
	else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_INLINE) {

		pm->vsi_AF = ((pm->vsi_AG >= pm->ts_clearance && xA > pm->ts_skip)
				|| (pm->vsi_AG == 0 && xA == 0)) ? 0 : 1;
		pm->vsi_BF = ((pm->vsi_BG >= pm->ts_clearance && xB > pm->ts_skip)
				|| (pm->vsi_BG == 0 && xB == 0)) ? 0 : 1;
		pm->vsi_CF = ((pm->vsi_CG >= pm->ts_clearance && xC > pm->ts_skip)
				|| (pm->vsi_CG == 0 && xC == 0)) ? 0 : 1;
	}
	else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_GND) {

		pm->vsi_AF = (pm->vsi_AG >= pm->ts_clearance && xA > pm->ts_skip) ? 0 : 1;
		pm->vsi_BF = (pm->vsi_BG >= pm->ts_clearance && xB > pm->ts_skip) ? 0 : 1;
		pm->vsi_CF = (pm->vsi_CG >= pm->ts_clearance && xC > pm->ts_skip) ? 0 : 1;
	}

	/* You can mask a specific channel for some reasons.
	 * */
	pm->vsi_AF = (pm->vsi_mask_XF == PM_MASK_A) ? 1 : pm->vsi_AF;
	pm->vsi_BF = (pm->vsi_mask_XF == PM_MASK_B) ? 1 : pm->vsi_BF;
	pm->vsi_CF = (pm->vsi_mask_XF == PM_MASK_C) ? 1 : pm->vsi_CF;

	/* Chech if at least TWO samples are clean so they can be used in
	 * control loops.
	 * */
	pm->vsi_IF = (pm->vsi_AF + pm->vsi_BF + pm->vsi_CF < 2) ? 0 : 1;

	/* Check if there are PWM edges within clearance zone. The DC link
	 * voltage measurement will be used or rejected based on this flag.
	 * */
	pm->vsi_SF = (	   ((pm->vsi_AG > pm->ts_skip && xA > pm->ts_skip)
				|| (pm->vsi_AG == 0 && xA == 0))
			&& ((pm->vsi_BG > pm->ts_skip && xB > pm->ts_skip)
				|| (pm->vsi_BG == 0 && xB == 0))
			&& ((pm->vsi_CG > pm->ts_skip && xC > pm->ts_skip)
				|| (pm->vsi_CG == 0 && xC == 0))) ? 0 : 1;

	if (		PM_CONFIG_TVM(pm) == PM_ENABLED
			&& pm->tvm_USEABLE == PM_ENABLED) {

		xMIN = (int) (pm->dc_resolution * (1.f - pm->tvm_clean_zone));

		/* Check if terminal voltages were sampled within acceptable
		 * zone. The VOLTAGE measurement will be used or rejected based
		 * on these flags.
		 * */
		pm->vsi_UF = (	   pm->vsi_AG > xMIN
				&& pm->vsi_BG > xMIN
				&& pm->vsi_CG > xMIN
				&& xA > xMIN
				&& xB > xMIN
				&& xC > xMIN) ? 0 : 1;

		/* Check if terminal voltages are exactly ZERO to get more
		 * accuracy.
		 * */
		pm->vsi_AZ = (pm->vsi_AG == pm->dc_resolution) ? 0 : 1;
		pm->vsi_BZ = (pm->vsi_BG == pm->dc_resolution) ? 0 : 1;
		pm->vsi_CZ = (pm->vsi_CG == pm->dc_resolution) ? 0 : 1;
	}
	else {
		pm->vsi_UF = 1;
	}

	pm->vsi_AG = xA;
	pm->vsi_BG = xB;
	pm->vsi_CG = xC;
}

void pm_voltage(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC, uMIN, uMAX, uDC;
	int		xA, xB, xC, xMIN, xMAX;

	uX *= pm->quick_iUdc;
	uY *= pm->quick_iUdc;

	uDC = m_sqrtf(uX * uX + uY * uY);

	pm->vsi_DC = uDC / pm->k_EMAX;
	pm->vsi_lpf_DC += (pm->vsi_DC - pm->vsi_lpf_DC) * pm->vsi_gain_LP;

	if (		pm->config_VSI_CIRCULAR == PM_ENABLED
			&& uDC > pm->k_EMAX) {

		uDC = pm->k_EMAX / uDC;

		uX *= uDC;
		uY *= uDC;
	}

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

	uDC = uMAX - uMIN;

	if (uDC > 1.f) {

		uDC = 1.f / uDC;

		uA *= uDC;
		uB *= uDC;
		uC *= uDC;

		uMIN *= uDC;
		uMAX *= uDC;
	}

	if (pm->config_VSI_PRECISE == PM_ENABLED) {

		uDC = .5f - (uMAX + uMIN) * .5f;
	}
	else {
		uDC = 0.f - uMIN;
	}

	uA += uDC;
	uB += uDC;
	uC += uDC;

	xA = (int) (pm->dc_resolution * uA);
	xB = (int) (pm->dc_resolution * uB);
	xC = (int) (pm->dc_resolution * uC);

	if (pm->lu_MODE != PM_LU_DISABLED) {

		if (PM_CONFIG_IFB(pm) == PM_IFB_AB_INLINE) {

			xMAX = pm->dc_resolution - pm->ts_clearance;

			if (xA > xMAX || xB > xMAX) {

				xMAX = (xA > xB) ? xA : xB;
				xMAX = (xC > xMAX) ? xC : xMAX;
				xMIN = pm->dc_resolution - xMAX;

				xA += xMIN;
				xB += xMIN;
				xC += xMIN;
			}

			xMAX = pm->dc_resolution - pm->ts_minimal;

			xA = (xA < pm->ts_minimal) ? 0 : (xA > xMAX) ? pm->dc_resolution : xA;
			xB = (xB < pm->ts_minimal) ? 0 : (xB > xMAX) ? pm->dc_resolution : xB;
			xC = (xC < pm->ts_minimal) ? 0 : (xC > xMAX) ? pm->dc_resolution : xC;
		}
		else if (PM_CONFIG_IFB(pm) == PM_IFB_AB_GND) {

			xMAX = pm->dc_resolution - pm->ts_clearance;
			xMIN = pm->dc_resolution - pm->ts_minimal;

			xA = (xA < pm->ts_minimal) ? 0 : (xA > xMAX) ? xMAX : xA;
			xB = (xB < pm->ts_minimal) ? 0 : (xB > xMAX) ? xMAX : xB;
			xC = (xC < pm->ts_minimal) ? 0 : (xC > xMAX) ? xMAX : xC;
		}
		else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_INLINE) {

			xMAX = pm->dc_resolution - pm->ts_clearance;

			if (xA > xMAX || xB > xMAX || xC > xMAX) {

				xMAX = (xA > xB) ? xA : xB;
				xMAX = (xC > xMAX) ? xC : xMAX;
				xMIN = pm->dc_resolution - xMAX;

				xA += xMIN;
				xB += xMIN;
				xC += xMIN;
			}

			xMAX = pm->dc_resolution - pm->ts_minimal;

			xA = (xA < pm->ts_minimal) ? 0 : (xA > xMAX) ? pm->dc_resolution : xA;
			xB = (xB < pm->ts_minimal) ? 0 : (xB > xMAX) ? pm->dc_resolution : xB;
			xC = (xC < pm->ts_minimal) ? 0 : (xC > xMAX) ? pm->dc_resolution : xC;
		}
		else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_GND) {

			xA = (xA < pm->ts_minimal) ? 0 : xA;
                        xB = (xB < pm->ts_minimal) ? 0 : xB;
			xC = (xC < pm->ts_minimal) ? 0 : xC;

			xMAX = pm->dc_resolution - pm->ts_clearance;
			xMIN = pm->dc_resolution - pm->ts_minimal;

			if (xA > xMAX && xB > xMAX) {

				xC = (xC > xMIN) ? pm->dc_resolution : xC;

				if (xA < xB) {

					xA = (xA > xMAX) ? xMAX : xA;
					xB = (xB > xMIN) ? pm->dc_resolution : xB;
				}
				else {
					xA = (xA > xMIN) ? pm->dc_resolution : xA;
					xB = (xB > xMAX) ? xMAX : xB;
				}
			}
			else if (xB > xMAX && xC > xMAX) {

				xA = (xA > xMIN) ? pm->dc_resolution : xA;

				if (xB < xC) {

					xB = (xB > xMAX) ? xMAX : xB;
					xC = (xC > xMIN) ? pm->dc_resolution : xC;
				}
				else {
					xB = (xB > xMIN) ? pm->dc_resolution : xB;
					xC = (xC > xMAX) ? xMAX : xC;
				}
			}
			else if (xA > xMAX && xC > xMAX) {

				xB = (xB > xMIN) ? pm->dc_resolution : xB;

				if (xA < xC) {

					xA = (xA > xMAX) ? xMAX : xA;
					xC = (xC > xMIN) ? pm->dc_resolution : xC;
				}
				else {
					xA = (xA > xMIN) ? pm->dc_resolution : xA;
					xC = (xC > xMAX) ? xMAX : xC;
				}
			}
		}
	}
	else {
		xA += pm->ts_minimal;
		xB += pm->ts_minimal;
		xC += pm->ts_minimal;

		xMAX = pm->dc_resolution - pm->ts_clearance;

		xA = (xA > xMAX) ? xMAX : xA;
		xB = (xB > xMAX) ? xMAX : xB;
		xC = (xC > xMAX) ? xMAX : xC;
	}

	if (pm->ts_bootstrap != 0) {

		pm->vsi_SA = (xA == pm->dc_resolution) ? pm->vsi_SA + 1 : 0;
		pm->vsi_SB = (xB == pm->dc_resolution) ? pm->vsi_SB + 1 : 0;
		pm->vsi_SC = (xC == pm->dc_resolution) ? pm->vsi_SC + 1 : 0;

		if (		   pm->vsi_SA > pm->ts_bootstrap
				|| pm->vsi_SB > pm->ts_bootstrap
				|| pm->vsi_SC > pm->ts_bootstrap) {

			/* Clamp the output DC to a safe level if bootstrap
			 * retention time is running out.
			 * */
			pm->vsi_TIM = 1;
		}

		if (pm->vsi_TIM >= 1) {

			xMAX = pm->dc_resolution - pm->ts_clearance;

			xA = (xA > xMAX) ? xMAX : xA;
			xB = (xB > xMAX) ? xMAX : xB;
			xC = (xC > xMAX) ? xMAX : xC;

			pm->vsi_TIM++;

			if (pm->vsi_TIM >= pm->ts_clamped) {

				pm->vsi_TIM = 0;
			}
		}
	}

	/* Output DC values to PWM.
	 * */
	pm->proc_set_DC(xA, xB, xC);

	pm->vsi_DX = pm->vsi_X;
	pm->vsi_DY = pm->vsi_Y;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		uDC = .33333333f * (xA + xB + xC);

		uA = (xA - uDC) * pm->const_fb_U * pm->ts_inverted;
		uB = (xB - uDC) * pm->const_fb_U * pm->ts_inverted;

		pm->vsi_X = uA;
		pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;
	}
	else {
		uA = (xA - xC) * pm->const_fb_U * pm->ts_inverted;
		uB = (xB - xC) * pm->const_fb_U * pm->ts_inverted;

		pm->vsi_X = uA;
		pm->vsi_Y = uB;
	}

	/* Update the measurement clearance flags according to the DC values.
	 * */
	pm_clearance(pm, xA, xB, xC);
}

static float
pm_form_iSP(pmc_t *pm, float eS)
{
	float		iSP;

	/* There is a tolerance.
	 * */
	eS = (m_fabsf(eS) > pm->s_tolerance) ? eS : 0.f;

	/* Basic proportional regulator.
	 * */
	iSP = pm->s_gain_P * eS;

	/* The speed regulator uses an load torque estimate as feed forward
	 * component that replaces the integral component.
	 * */
	iSP += pm->s_gain_I * pm_torque_approx(pm, pm->lu_load_torque);

	return iSP;
}

static void
pm_loop_current(pmc_t *pm)
{
	float		track_D, track_Q, eD, eQ, uD, uQ, uX, uY, wP;
	float		iMAX, iREV, uMAX, uREV, wMAX, wREV, dS;

	if (		pm->lu_MODE == PM_LU_FORCED
			|| pm->forced_track_D > M_EPS_F) {

		track_D = (pm->lu_MODE == PM_LU_FORCED)
			? pm->forced_hold_D : 0.f;

		/* Forced current slew rate limited tracking.
		 * */
		dS = pm->forced_slew_rate * pm->dT;
		pm->forced_track_D = (pm->forced_track_D < track_D - dS)
			? pm->forced_track_D + dS : (pm->forced_track_D > track_D + dS)
			? pm->forced_track_D - dS : track_D;
	}

	track_D = pm->forced_track_D;
	track_Q = 0.f;

	if (pm->lu_MODE == PM_LU_FORCED) {

		/* Only forced current on D-axis.
		 * */
	}
	else if (	pm->lu_MODE == PM_LU_ESTIMATE_FLUX
			&& pm->flux_ZONE != PM_ZONE_HIGH) {

		/* TODO: Add some current in case of MTPA control */
	}
	else {
		/* Generic torque control case.
		 * */
		track_Q = pm->i_setpoint_current;

		if (pm->config_LU_DRIVE == PM_DRIVE_CURRENT) {

			if (		pm->config_HOLDING_BRAKE == PM_ENABLED
					&& pm->i_setpoint_current < 0.f) {

				track_Q = pm_form_iSP(pm, 0.f - pm->lu_wS);

				iMAX = m_fabsf(pm->i_setpoint_current);

				track_Q = (track_Q > iMAX) ? iMAX
					: (track_Q < - iMAX) ? - iMAX : track_Q;
			}

			if (pm->config_SPEED_LIMITED == PM_ENABLED) {

				float		lerp, E;

				/* Maximal speed constraint.
				 * */
				if (pm->lu_wS < - pm->s_reverse) {

					E = - pm->s_reverse - pm->lu_wS;

					lerp = E / pm->s_linspan;
					lerp = (lerp > 1.f) ? 1.f : lerp;

					track_Q += (pm_form_iSP(pm, E) - track_Q) * lerp;
				}
				else if (pm->lu_wS > pm->s_maximal) {

					E = pm->s_maximal - pm->lu_wS;

					lerp = - E / pm->s_linspan;
					lerp = (lerp > 1.f) ? 1.f : lerp;

					track_Q += (pm_form_iSP(pm, E) - track_Q) * lerp;
				}

				/* Maximal acceleration constraint.
				 * */
				dS = pm->s_accel * pm->dT;
				pm->s_track = (pm->s_track < pm->lu_wS - dS)
					? pm->s_track + dS : (pm->s_track > pm->lu_wS + dS)
					? pm->s_track - dS : pm->lu_wS;

				E = pm->s_track - pm->lu_wS;

				lerp = m_fabsf(E) / pm->s_linspan;
				lerp = (lerp > 1.f) ? 1.f : lerp;

				track_Q += (pm_form_iSP(pm, E) - track_Q) * lerp;
			}
		}

		if (pm->config_RELUCTANCE == PM_ENABLED) {

			float		MTPA_sine;

			MTPA_sine = pm_torque_do_MTPA(pm, track_Q);

			track_D += MTPA_sine * track_Q;
			track_Q *= m_sqrtf(1.f - MTPA_sine * MTPA_sine);
		}

		if (pm->config_WEAKENING == PM_ENABLED) {

			float		E;

			E = (1.f - pm->vsi_DC) * pm->const_fb_U;

			if (pm->const_fb_U > pm->watt_dclink_HI) {

				/* Prevent the lack of weakening in case of overvoltage.
				 * */
				E = (E > 0.f) ? 0.f : E;
			}

			pm->weak_D += E * pm->weak_gain_EU;
			pm->weak_D = (pm->weak_D < - pm->weak_maximal) ? - pm->weak_maximal
				: (pm->weak_D > 0.f) ? 0.f : pm->weak_D;

			if (pm->weak_D < - M_EPS_F) {

				E = pm->k_EMAX * pm->const_fb_U;

				pm->i_derated_WEAK = E / (pm->lu_wS * pm->const_im_L2);

				/* Flux weakening control.
				 * */
				track_D += pm->weak_D;
			}
			else {
				pm->i_derated_WEAK = PM_MAX_F;
			}
		}
	}

	/* Get VSI voltages on DQ-axes.
	 * */
	uD = pm->lu_F[0] * pm->tvm_DX + pm->lu_F[1] * pm->tvm_DY;
	uQ = pm->lu_F[0] * pm->tvm_DY - pm->lu_F[1] * pm->tvm_DX;

	/* LPF is necessary to ensure the stability of POWER constraints loop.
	 * */
	pm->watt_lpf_D += (uD - pm->watt_lpf_D) * pm->watt_gain_LP;
	pm->watt_lpf_Q += (uQ - pm->watt_lpf_Q) * pm->watt_gain_LP;

	/* Operating POWER is a scalar product of voltage and current.
	 * */
	wP = pm->k_KWAT * (pm->lu_iD * pm->watt_lpf_D + pm->lu_iQ * pm->watt_lpf_Q);
	pm->watt_lpf_wP += (wP - pm->watt_lpf_wP) * pm->watt_gain_LP;

	/* Maximal CURRENT constraints.
	 * */
	iMAX = pm->i_maximal;
	iREV = - pm->i_reverse;

	if (		pm->config_WEAKENING == PM_ENABLED
			&& pm->weak_D < - M_EPS_F) {

		/* Flux weakening constraints.
		 * */
		track_Q = (track_Q > pm->i_derated_WEAK) ? pm->i_derated_WEAK
			: (track_Q < - pm->i_derated_WEAK) ? - pm->i_derated_WEAK : track_Q;
	}
	else {
		iMAX = (iMAX < pm->i_derated_PCB) ? iMAX : pm->i_derated_PCB;
		iREV = (iREV > - pm->i_derated_PCB) ? iREV : - pm->i_derated_PCB;
	}

	if (pm->hfi_INJECT == PM_ENABLED) {

		/* HFI constraints.
		 * */
		iMAX = (iMAX < pm->i_derated_HFI) ? iMAX : pm->i_derated_HFI;
		iREV = (iREV > - pm->i_derated_HFI) ? iREV : - pm->i_derated_HFI;
	}

	track_D = (track_D > iMAX) ? iMAX : (track_D < - iMAX) ? - iMAX : track_D;
	track_Q = (track_Q > iMAX) ? iMAX : (track_Q < iREV) ? iREV : track_Q;

	/* Prevent DC link OVERVOLTAGE.
	 * */
	track_Q = (pm->const_fb_U > pm->watt_dclink_HI) ? 0.f : track_Q;
	track_Q = (pm->const_fb_U < pm->watt_dclink_LO) ? 0.f : track_Q;

	/* Maximal POWER constraint.
	 * */
	wMAX = pm->watt_wP_maximal;
	wREV = - pm->watt_wP_reverse;

	/* Maximal DC link CURRENT constraint.
	 * */
	wP = pm->watt_iDC_maximal * pm->const_fb_U;
	wMAX = (wP < wMAX) ? wP : wMAX;
	wP = - pm->watt_iDC_reverse * pm->const_fb_U;
	wREV = (wP > wREV) ? wP : wREV;

	/* Apply POWER constraints (with D-axis priority).
	 * */
	wP = pm->k_KWAT * (track_D * pm->watt_lpf_D + track_Q * pm->watt_lpf_Q);

	if (wP > wMAX) {

		wP = pm->k_KWAT * track_D * pm->watt_lpf_D;

		if (wP > wMAX) {

			track_D *= wMAX / wP;
			track_Q = 0.f;
		}
		else {
			wMAX -= wP;
			wP = pm->k_KWAT * track_Q * pm->watt_lpf_Q;

			if (wP > M_EPS_F) {

				track_Q *= wMAX / wP;
			}
		}
	}
	else if (wP < wREV) {

		wP = pm->k_KWAT * track_D * pm->watt_lpf_D;

		if (wP < wREV) {

			track_D *= wREV / wP;
			track_Q = 0.f;
		}
		else {
			wREV -= wP;
			wP = pm->k_KWAT * track_Q * pm->watt_lpf_Q;

			if (wP < - M_EPS_F) {

				track_Q *= wREV / wP;
			}
		}
	}

	dS = pm->i_slew_rate * pm->dT;

	/* Slew rate limited current tracking.
	 * */
	pm->i_track_D = (pm->i_track_D < track_D - dS) ? pm->i_track_D + dS
		: (pm->i_track_D > track_D + dS) ? pm->i_track_D - dS : track_D;
	pm->i_track_Q = (pm->i_track_Q < track_Q - dS) ? pm->i_track_Q + dS
		: (pm->i_track_Q > track_Q + dS) ? pm->i_track_Q - dS : track_Q;

	/* Obtain discrepancy in DQ-axes.
	 * */
	eD = pm->i_track_D - pm->lu_iD;
	eQ = pm->i_track_Q - pm->lu_iQ;

	/* There is a tolerance.
	 * */
	eD = (m_fabsf(eD) > pm->i_tolerance) ? eD : 0.f;
	eQ = (m_fabsf(eQ) > pm->i_tolerance) ? eQ : 0.f;

	uD = pm->i_gain_P * eD;
	uQ = pm->i_gain_P * eQ;

	/* Feed forward compensation (R).
	 * */
	uD += pm->const_R * pm->i_track_D;
	uQ += pm->const_R * pm->i_track_Q;

	/* Feed forward compensation (L).
	 * */
	uD += - pm->lu_wS * pm->const_im_L2 * pm->i_track_Q;
	uQ += pm->lu_wS * (pm->const_im_L1 * pm->i_track_D + pm->const_E);

	uMAX = pm->k_UMAX * pm->const_fb_U;

	pm->i_integral_D += pm->i_gain_I * eD;
	pm->i_integral_D = (pm->i_integral_D > uMAX) ? uMAX :
		(pm->i_integral_D < - uMAX) ? - uMAX : pm->i_integral_D;
	uD += pm->i_integral_D;

	pm->i_integral_Q += pm->i_gain_I * eQ;
	pm->i_integral_Q = (pm->i_integral_Q > uMAX) ? uMAX :
		(pm->i_integral_Q < - uMAX) ? - uMAX : pm->i_integral_Q;
	uQ += pm->i_integral_Q;

	/* Output voltage CLAMP.
	 * */
	uD = (uD > uMAX) ? uMAX : (uD < - uMAX) ? - uMAX : uD;
	uQ = (uQ > uMAX) ? uMAX : (uQ < - uMAX) ? - uMAX : uQ;

	uMAX = pm->k_EMAX * pm->v_maximal;
	uREV = - pm->k_EMAX * pm->v_reverse;

	/* Output voltage (Q) specified constraint.
	 * */
	uQ = (uQ > uMAX) ? uMAX : (uQ < uREV) ? uREV : uQ;

	if (pm->hfi_INJECT == PM_ENABLED) {

		float		uHF;

		uHF = pm->hfi_sine * pm->quick_hfwS * pm->const_im_L1;

		/* HF voltage injection.
		 * */
		uD += pm->hfi_wave[0] * uHF;
		uQ += pm->hfi_wave[1] * uHF;

		pm->hfi_INJECT = PM_DISABLED;
	}

	/* Go to XY-axes.
	 * */
	uX = pm->lu_F[0] * uD - pm->lu_F[1] * uQ;
	uY = pm->lu_F[1] * uD + pm->lu_F[0] * uQ;

	pm_voltage(pm, uX, uY);
}

static void
pm_loop_speed(pmc_t *pm)
{
	float		wSP, eS, dS;

	wSP = pm->s_setpoint_speed;

	/* Maximal speed constraint.
	 * */
	wSP = (wSP > pm->s_maximal) ? pm->s_maximal :
		(wSP < - pm->s_reverse) ? - pm->s_reverse : wSP;

	if (pm->config_LU_DRIVE == PM_DRIVE_SPEED) {

		/* Maximal acceleration constraint.
		 * */
		dS = pm->s_accel * pm->dT;
		pm->s_track = (pm->s_track < wSP - dS) ? pm->s_track + dS
			: (pm->s_track > wSP + dS) ? pm->s_track - dS : wSP;
	}
	else {
		/* No constraint required.
		 * */
		pm->s_track = wSP;
	}

	if (pm->lu_MODE == PM_LU_FORCED) {

		pm->s_track = pm->forced_wS;
	}
	else {
		/* Get speed discrepancy.
		 * */
		eS = pm->s_track - pm->lu_wS;

		/* Update current loop SETPOINT.
		 * */
		pm->i_setpoint_current = pm_form_iSP(pm, eS);
	}
}

static void
pm_loop_servo(pmc_t *pm)
{
	float		xSP, xER, xER_abs, wSP, lerp, gain_S;

	/* Move setpoint in accordance with speed setpoint.
	 * */
	pm->x_setpoint_location += pm->x_setpoint_speed * pm->dT;

	/* Get setpoint under limits.
	 * */
	xSP = pm->x_setpoint_location;
	xSP = (xSP < pm->x_location_range[0]) ? pm->x_location_range[0]
		: (xSP > pm->x_location_range[1]) ? pm->x_location_range[1] : xSP;

	/* Get location discrepancy.
	 * */
	xER = xSP - pm->lu_location;
	pm->x_discrepancy = xER;

	/* Servo is based on constant acceleration formula.
	 * */
	xER_abs = m_fabsf(xER);
	xER = (xER < 0.f) ? - m_sqrtf(xER_abs) : m_sqrtf(xER_abs);

	/* There is a tolerance.
	 * */
	xER = (xER_abs > pm->x_tolerance) ? xER : 0.f;

	/* Slow down in NEAR zone.
	 * */
	lerp = (xER_abs < pm->x_weak_zone) ? xER_abs / pm->x_weak_zone : 1.f;
	gain_S = pm->x_gain_P * lerp + pm->x_gain_N * (1.f - lerp);

	wSP = xER * gain_S + pm->x_setpoint_speed;

	/* Update speed loop SETPOINT.
	 * */
	pm->s_setpoint_speed = wSP;
}

static void
pm_mileage_info(pmc_t *pm)
{
	float		Wh, Ah, fuel;

	/* Traveled distance (m).
	 * */
	pm->im_distance = (float) pm->lu_total_revol
		* pm->const_ld_S / (float) pm->const_Zp;

	/* Get WATT per HOUR.
	 * */
	Wh = pm->watt_lpf_wP * pm->dT * 0.00027777778f;
	Ah = Wh * pm->quick_iUdc;

	if (Wh > 0.f) {

		m_rsumf(&pm->im_consumed_Wh, &pm->im_REM[0], Wh);
		m_rsumf(&pm->im_consumed_Ah, &pm->im_REM[1], Ah);
	}
	else {
		m_rsumf(&pm->im_reverted_Wh, &pm->im_REM[2], - Wh);
		m_rsumf(&pm->im_reverted_Ah, &pm->im_REM[3], - Ah);
	}

	/* Fuel gauge.
	 * */
	if (pm->im_capacity_Ah > M_EPS_F) {

		fuel = (pm->im_capacity_Ah - pm->im_consumed_Ah
				+ pm->im_reverted_Ah) / pm->im_capacity_Ah;

		pm->im_fuel_pc = 100.f * fuel;
	}
}

void pm_feedback(pmc_t *pm, pmfb_t *fb)
{
	float		vA, vB, vC, Q;

	if (pm->vsi_AF == 0) {

		/* Get inline current A.
		 * */
		pm->fb_iA = pm->ad_IA[1] * fb->current_A + pm->ad_IA[0];

		if (m_fabsf(pm->fb_iA) > pm->fault_current_halt) {

			pm->fsm_errno = PM_ERROR_INSTANT_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (pm->vsi_BF == 0) {

		/* Get inline current B.
		 * */
		pm->fb_iB = pm->ad_IB[1] * fb->current_B + pm->ad_IB[0];

		if (m_fabsf(pm->fb_iB) > pm->fault_current_halt) {

			pm->fsm_errno = PM_ERROR_INSTANT_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (pm->vsi_CF == 0) {

		/* Get inline current C.
		 * */
		pm->fb_iC = pm->ad_IC[1] * fb->current_C + pm->ad_IC[0];

		if (m_fabsf(pm->fb_iC) > pm->fault_current_halt) {

			pm->fsm_errno = PM_ERROR_INSTANT_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		if (		   pm->vsi_AF == 0
				&& pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			Q = .33333333f * (pm->fb_iA + pm->fb_iB + pm->fb_iC);
			vA = pm->fb_iA - Q;
			vB = pm->fb_iB - Q;

			pm->lu_iX = vA;
			pm->lu_iY = .57735027f * vA + 1.1547005f * vB;
		}
		else if (	   pm->vsi_AF == 0
				&& pm->vsi_BF == 0) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = .57735027f * pm->fb_iA + 1.1547005f * pm->fb_iB;
		}
		else if (	   pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			pm->lu_iX = - pm->fb_iB - pm->fb_iC;
			pm->lu_iY = .57735027f * pm->fb_iB - .57735027f * pm->fb_iC;
		}
		else if (	   pm->vsi_AF == 0
				&& pm->vsi_CF == 0) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = - .57735027f * pm->fb_iA - 1.1547005f * pm->fb_iC;
		}
	}
	else {
		if (		   pm->vsi_AF == 0
				&& pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			Q = .33333333f * (pm->fb_iA + pm->fb_iB + pm->fb_iC);
			vA = pm->fb_iA - Q;
			vB = pm->fb_iB - Q;

			pm->lu_iX = vA;
			pm->lu_iY = vB;
		}
		else if (	   pm->vsi_AF == 0
				&& pm->vsi_BF == 0) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = pm->fb_iB;
		}
		else if (	   pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			pm->lu_iX = - pm->fb_iB - pm->fb_iC;
			pm->lu_iY = pm->fb_iB;

		}
		else if (	   pm->vsi_AF == 0
				&& pm->vsi_CF == 0) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = - pm->fb_iA - pm->fb_iC;
		}
	}

	if (pm->vsi_SF == 0) {

		/* Get DC link voltage.
		 * */
		pm->const_fb_U = pm->ad_US[1] * fb->voltage_U + pm->ad_US[0];
		pm->quick_iUdc = 1.f / pm->const_fb_U;

		if (		pm->const_fb_U > pm->fault_voltage_halt
				&& pm->weak_D > - M_EPS_F) {

			pm->fsm_errno = PM_ERROR_DC_LINK_OVERVOLTAGE;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	/* This fallback makes possible the use of TVM voltages in case of
	 * no TVM capable hardware or TVM is not enabled.
	 * */
	pm->tvm_DX = pm->vsi_DX;
	pm->tvm_DY = pm->vsi_DY;

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		/* Save previous cycle voltages.
		 * */
		vA = pm->fb_uA;
		vB = pm->fb_uB;
		vC = pm->fb_uC;

		/* Get instant terminal voltages.
		 * */
		pm->fb_uA = pm->ad_UA[1] * fb->voltage_A + pm->ad_UA[0];
		pm->fb_uB = pm->ad_UB[1] * fb->voltage_B + pm->ad_UB[0];
		pm->fb_uC = pm->ad_UC[1] * fb->voltage_C + pm->ad_UC[0];

		if (		pm->lu_MODE != PM_LU_DETACHED
				&& pm->vsi_UF == 0) {

			/* Extract the average terminal voltages using FIR.
			 * */

			vA *= pm->tvm_FIR_A[1];
			vB *= pm->tvm_FIR_B[1];
			vC *= pm->tvm_FIR_C[1];

			if (pm->vsi_AZ != 0) {

				vA += pm->tvm_FIR_A[0] * pm->fb_uA + pm->tvm_FIR_A[2];
			}
			else {
				vA = 0.f;
			}

			if (pm->vsi_BZ != 0) {

				vB += pm->tvm_FIR_B[0] * pm->fb_uB + pm->tvm_FIR_B[2];
			}
			else {
				vB = 0.f;
			}

			if (pm->vsi_CZ != 0) {

				vC += pm->tvm_FIR_C[0] * pm->fb_uC + pm->tvm_FIR_C[2];
			}
			else {
				vC = 0.f;
			}

			pm->tvm_A = vA;
			pm->tvm_B = vB;
			pm->tvm_C = vC;

			if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

				Q = .33333333f * (vA + vB + vC);
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
	}

	/* Get SENSOR values.
	 * */
	pm->fb_SIN = fb->analog_SIN;
	pm->fb_COS = fb->analog_COS;
	pm->fb_HS = fb->pulse_HS;
	pm->fb_EP = fb->pulse_EP;

	/* Main FSM is used to execute external commands.
	 * */
	pm_FSM(pm);

	if (pm->lu_MODE != PM_LU_DISABLED) {

		/* The observer FSM.
		 * */
		pm_lu_FSM(pm);

		if (pm->lu_MODE == PM_LU_DETACHED) {

			pm_voltage(pm, pm->vsi_X, pm->vsi_Y);
		}
		else {
			if (pm->config_LU_DRIVE == PM_DRIVE_SPEED) {

				pm_loop_speed(pm);
			}
			else if (pm->config_LU_DRIVE == PM_DRIVE_SERVO) {

				pm_loop_servo(pm);
				pm_loop_speed(pm);
			}

			/* Current loop is always enabled.
			 * */
			pm_loop_current(pm);

			if (pm->kalman_POSTPONED == PM_ENABLED) {

				/* We have to do most expensive work after DC
				 * values are output to the PWM. This allows
				 * efficient use of CPU.
				 * */
				pm_kalman_predict(pm);

				if (pm->vsi_IF == 0) {

					pm_kalman_update(pm, pm->flux_X);
				}

				pm->kalman_POSTPONED = PM_DISABLED;
			}
		}

		if (pm->config_MILEAGE_INFO == PM_ENABLED) {

			/* To collect mileage information.
			 * */
			pm_mileage_info(pm);
		}

		if (m_isfinitef(pm->lu_F[0]) == 0) {

			pm->fsm_errno = PM_ERROR_INVALID_OPERATION;
			pm->fsm_state = PM_STATE_HALT;
		}
	}
}

