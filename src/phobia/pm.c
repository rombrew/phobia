#include "libm.h"
#include "pm.h"

void pm_quick_build(pmc_t *pm)
{
	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		pm->k_UMAX = 0.66666667f;	/* 2 / NOP */
		pm->k_EMAX = 0.57735027f;	/* 1 / sqrt(NOP) */
		pm->k_KWAT = 1.5f;		/* NOP / 2 */
	}
	else {
		pm->k_UMAX = 1.f;		/* 2 / NOP */
		pm->k_EMAX = 0.70710678f;	/* 1 / sqrt(NOP) */
		pm->k_KWAT = 1.f;		/* NOP / 2 */
	}

	pm->ts_minimal = (int) (pm->dc_minimal * (1.f / 1000000.f)
			* pm->m_freq * (float) pm->dc_resolution);
	pm->ts_clearance = (int) (pm->dc_clearance * (1.f / 1000000.f)
			* pm->m_freq * (float) pm->dc_resolution);
	pm->ts_skip = (int) (pm->dc_skip * (1.f / 1000000.f)
			* pm->m_freq * (float) pm->dc_resolution);
	pm->ts_bootstrap = PM_TSMS(pm, pm->dc_bootstrap);
	pm->ts_inverted = 1.f / (float) pm->dc_resolution;

	if (pm->const_lambda > M_EPSILON) {

		pm->quick_iWb = 1.f / pm->const_lambda;
		pm->quick_iWb2 = pm->quick_iWb * pm->quick_iWb;

		pm->flux_LINKAGE = PM_ENABLED;
	}

	if (		   pm->const_im_Ld > M_EPSILON
			&& pm->const_im_Lq > M_EPSILON) {

		float		Ld = pm->const_im_Ld;
		float		Lq = pm->const_im_Lq;

		pm->quick_iLd = 1.f / Ld;
		pm->quick_iLq = 1.f / Lq;

		pm->quick_Lrel = Ld - Lq;
		pm->quick_iL4rel = 0.25f / pm->quick_Lrel;

		pm->quick_TiLd = pm->m_dT * pm->quick_iLd;
		pm->quick_TiLq = pm->m_dT * pm->quick_iLq;

		pm->quick_TiLu[0] = pm->m_dT * (pm->quick_iLd - pm->quick_iLq);
		pm->quick_TiLu[1] = pm->m_dT * (Ld * pm->quick_iLq - Lq * pm->quick_iLd);
		pm->quick_TiLu[2] = pm->m_dT * (Ld * pm->quick_iLq - 1.f);
		pm->quick_TiLu[3] = pm->m_dT * (1.f - Lq * pm->quick_iLd);
	}

	if (pm->config_HFI_WAVETYPE != PM_HFI_NONE) {

		pm->quick_HFwS = M_2_PI_F * pm->hfi_freq;

		pm->quick_HF[0] = m_cosf(pm->quick_HFwS * pm->m_dT);
		pm->quick_HF[1] = m_sinf(pm->quick_HFwS * pm->m_dT);
	}

	if (		   pm->eabi_const_Zq != 0
			&& pm->eabi_const_EP != 0) {

		float		Zf = (float) (pm->const_Zp * pm->eabi_const_Zs);
		float		Zq = (float) (pm->eabi_const_Zq * pm->eabi_const_EP);

		pm->quick_ZiEP = M_2_PI_F * Zf / Zq;
	}

	if (pm->sincos_const_Zq != 0) {

		float		Zf = (float) (pm->const_Zp * pm->sincos_const_Zs);
		float		Zq = (float) pm->sincos_const_Zq;

		pm->quick_ZiSQ = Zf / Zq;
	}
}

static void
pm_auto_basic_default(pmc_t *pm)
{
	pm->dc_minimal = 0.2f;			/* (us) */
	pm->dc_clearance = 5.0f;		/* (us) */
	pm->dc_skip = 2.0f;			/* (us) */
	pm->dc_bootstrap = 100.f;		/* (ms) */

	pm->config_NOP = PM_NOP_THREE_PHASE;
	pm->config_IFB = PM_IFB_ABC_INLINE;
	pm->config_TVM = PM_ENABLED;
	pm->config_DBG = PM_DISABLED;

	pm->fault_voltage_tol = 5.f;		/* (V) */
	pm->fault_current_tol = 5.f;		/* (A) */
	pm->fault_accuracy_tol = 0.10f;		/*     */
	pm->fault_terminal_tol = 0.10f;		/* (V) */
	pm->fault_current_halt = 156.f;		/* (A) */
	pm->fault_voltage_halt = 57.f;		/* (V) */

	pm->vsi_AF = 1;				/* NOTE: Disable all flags until  */
	pm->vsi_BF = 1;				/* clearance zone is calculated.  */
	pm->vsi_CF = 1;
	pm->vsi_UF = 1;

	pm->watt_wP_maximal = 4000.f;		/* (Watt) */
	pm->watt_wA_maximal = 80.f;		/* (A) */
	pm->watt_wP_reverse = 4000.f;		/* (Watt) */
	pm->watt_wA_reverse = 80.f;		/* (A) */
	pm->watt_uDC_maximal = 52.f;		/* (V) */
	pm->watt_uDC_minimal = 7.f;		/* (V) */

	pm->i_maximal = 120.f;			/* (A) */
	pm->i_reverse = pm->i_maximal;

	m_lf_randseed(&pm->lfseed, 80);		/* NOTE: lfg initial random seed. */
}

static void
pm_auto_config_default(pmc_t *pm)
{
	pm->config_VSI_ZERO = PM_VSI_GND;
	pm->config_VSI_CLAMP = PM_DISABLED;
	pm->config_DCU_VOLTAGE = PM_ENABLED;
	pm->config_LU_FORCED = PM_ENABLED;
	pm->config_LU_FREEWHEEL = PM_ENABLED;
	pm->config_LU_ESTIMATE = PM_FLUX_ORTEGA;
	pm->config_LU_SENSOR = PM_SENSOR_NONE;
	pm->config_LU_LOCATION = PM_LOCATION_NONE;
	pm->config_LU_DRIVE = PM_DRIVE_SPEED;
	pm->config_HFI_WAVETYPE = PM_HFI_NONE;
	pm->config_HFI_PERMANENT = PM_DISABLED;
	pm->config_EXCITATION = PM_EXCITATION_CONST;
	pm->config_SALIENCY = PM_SALIENCY_NEGATIVE;
	pm->config_RELUCTANCE = PM_DISABLED;
	pm->config_WEAKENING = PM_DISABLED;
	pm->config_CC_BRAKE_STOP = PM_ENABLED;
	pm->config_CC_SPEED_TRACK = PM_ENABLED;
	pm->config_EABI_FRONTEND = PM_EABI_INCREMENTAL;
	pm->config_SINCOS_FRONTEND = PM_SINCOS_ANALOG;

	pm->tm_transient_slow = 50.f;		/* (ms) */
	pm->tm_transient_fast = 2.f;		/* (ms) */
	pm->tm_voltage_hold = 100.f;		/* (ms) */
	pm->tm_current_hold = 200.f;		/* (ms) */
	pm->tm_current_ramp = 900.f;		/* (ms) */
	pm->tm_instant_probe = 2.f;		/* (ms) */
	pm->tm_average_probe = 200.f;		/* (ms) */
	pm->tm_average_drift = 100.f;		/* (ms) */
	pm->tm_average_inertia = 900.f;		/* (ms) */
	pm->tm_average_outside = 2000.f;	/* (ms) */
	pm->tm_pause_startup = 100.f;		/* (ms) */
	pm->tm_pause_forced = 1000.f;		/* (ms) */
	pm->tm_pause_halt = 2000.f;		/* (ms) */

	pm->scale_iA[0] = 0.f;
	pm->scale_iA[1] = 1.f;
	pm->scale_iB[0] = 0.f;
	pm->scale_iB[1] = 1.f;
	pm->scale_iC[0] = 0.f;
	pm->scale_iC[1] = 1.f;
	pm->scale_uS[0] = 0.f;
	pm->scale_uS[1] = 1.f;
	pm->scale_uA[0] = 0.f;
	pm->scale_uA[1] = 1.f;
	pm->scale_uB[0] = 0.f;
	pm->scale_uB[1] = 1.f;
	pm->scale_uC[0] = 0.f;
	pm->scale_uC[1] = 1.f;

	pm->probe_current_hold = 20.f;		/* (A) */
	pm->probe_weak_level = 0.5f;
	pm->probe_hold_angle = 0.f;
	pm->probe_current_sine = 10.f;		/* (A) */
	pm->probe_current_bias = 0.f;		/* (A) */
	pm->probe_freq_sine = 1100.f;		/* (Hz) */
	pm->probe_speed_hold = 900.f;		/* (rad/s) */
	pm->probe_speed_tol = 50.f;		/* (rad/s) */
	pm->probe_location_tol = 0.10f;		/* (rad) */
	pm->probe_loss_maximal = 400.f;		/* (Watt) */
	pm->probe_gain_P = 5.E-2f;
	pm->probe_gain_I = 5.E-4f;

	pm->vsi_gain_LP = 5.E-3f;

	pm->dcu_deadband = 100.f;		/* (ns) */
	pm->dcu_tol = 5.f;			/* (A) */

	pm->lu_transient = 400.f;		/* (rad/s) */
	pm->lu_gain_mq_LP = 5.E-4f;

	pm->forced_hold_D = 20.f;		/* (A) */
	pm->forced_weak_D = 0.f;		/* (A) */
	pm->forced_maximal = 280.f;		/* (rad/s) */
	pm->forced_reverse = pm->forced_maximal;
	pm->forced_accel = 400.f;		/* (rad/s2) */
	pm->forced_slew_rate = 50.f;		/* (A/s) */
	pm->forced_fall_rate = 5000.f;		/* (A/s) */
	pm->forced_stop_DC = 0.7f;

	pm->detach_threshold = 1.f;		/* (V) */
	pm->detach_trip_tol = 5.f;		/* (V) */
	pm->detach_gain_SF = 5.E-2f;

	pm->flux_trip_tol = 5.f;		/* (V) */
	pm->flux_gain_IN = 5.E-3f;
	pm->flux_gain_LO = 2.E-6f;
	pm->flux_gain_HI = 5.E-5f;
	pm->flux_gain_SF = 5.E-2f;
	pm->flux_gain_IF = 0.1f;

	pm->kalman_gain_Q[0] = 5.E-1f;
	pm->kalman_gain_Q[1] = 2.E+0f;
	pm->kalman_gain_Q[2] = 2.E+6f;
	pm->kalman_gain_Q[3] = 2.E-1f;
	pm->kalman_gain_R = 5.E-1f;

	pm->zone_noise = 50.f;			/* (rad/s) */
	pm->zone_threshold = 90.f;		/* (rad/s) */
	pm->zone_gain_TH = 0.7f;
	pm->zone_gain_LP = 5.E-3f;

	pm->hfi_freq = 4761.f;			/* (Hz) */
	pm->hfi_amplitude = 5.f;		/* (A) */

	pm->hall_trip_tol = 200.f;		/* (rad/s) */
	pm->hall_gain_LO = 5.E-4f;
	pm->hall_gain_SF = 7.E-3f;
	pm->hall_gain_IF = 0.1f;

	pm->eabi_const_EP = 2400;
	pm->eabi_const_Zs = 1;
	pm->eabi_const_Zq = 1;
	pm->eabi_trip_tol = 20.f;		/* (rad/s) */
	pm->eabi_gain_LO = 5.E-3f;
	pm->eabi_gain_SF = 5.E-2f;
	pm->eabi_gain_IF = 0.1f;

	pm->sincos_CONST[0] = 0.f;
	pm->sincos_CONST[1] = 0.f;
	pm->sincos_CONST[2] = 0.f;
	pm->sincos_CONST[3] = 0.f;
	pm->sincos_CONST[4] = 0.f;
	pm->sincos_CONST[5] = 0.f;
	pm->sincos_CONST[6] = 0.f;
	pm->sincos_CONST[7] = 0.f;
	pm->sincos_CONST[8] = 0.f;
	pm->sincos_CONST[9] = 0.f;
	pm->sincos_CONST[10] = 0.f;
	pm->sincos_CONST[11] = 0.f;
	pm->sincos_CONST[12] = 0.f;
	pm->sincos_CONST[13] = 0.f;
	pm->sincos_CONST[14] = 0.f;
	pm->sincos_CONST[15] = 0.f;

	pm->sincos_const_Zs = 1;
	pm->sincos_const_Zq = 1;
	pm->sincos_gain_PF = 5.E-1f;
	pm->sincos_gain_SF = 5.E-3f;
	pm->sincos_gain_IF = 0.1f;

	pm->const_lambda = 0.f;			/* (Wb) */
	pm->const_Rs = 0.f;			/* (Ohm) */
	pm->const_Zp = 1;
	pm->const_Ja = 0.f;
	pm->const_im_Ld = 0.f;			/* (H) */
	pm->const_im_Lq = 0.f;			/* (H) */
	pm->const_im_A = 0.f;
	pm->const_im_Rz = 0.f;

	pm->watt_uDC_tol = 4.f;			/* (V) */
	pm->watt_gain_P = 5.E+1f;
	pm->watt_gain_I = 5.E-1f;
	pm->watt_gain_LP = 5.E-3f;
	pm->watt_gain_WF = 5.E-2f;

	pm->i_maximal_on_HFI = 20.f;		/* (A) */
	pm->i_slew_rate = 10000.f;		/* (A/s) */
	pm->i_damping = 1.f;
	pm->i_gain_P = 2.E-1f;
	pm->i_gain_I = 5.E-3f;

	pm->mtpa_tol = 50.f;			/* (A) */
	pm->mtpa_gain_LP = 5.E-2f;

	pm->weak_maximal = 50.f;		/* (A) */
	pm->weak_gain_EU = 5.E-3f;

	pm->v_maximal = 90.f;			/* (V) */
	pm->v_reverse = pm->v_maximal;		/* (V) */

	pm->s_maximal = 15000.f;		/* (rad/s) */
	pm->s_reverse = pm->s_maximal;		/* (rad/s) */
	pm->s_accel_forward = 10000.f;		/* (rad/s2) */
	pm->s_accel_reverse = pm->s_accel_forward;
	pm->s_damping = 1.f;
	pm->s_gain_P = 4.E-2f;
	pm->s_gain_I = 0.f;
	pm->s_gain_D = 2.E-4f;

	pm->l_track_tol = 50.f;			/* (rad/s) */
	pm->l_gain_LP = 5.E-3f;

	pm->x_maximal = 100.f;			/* (rad) */
	pm->x_minimal = - pm->x_maximal;	/* (rad) */
	pm->x_boost_tol= 0.10f;			/* (rad) */
	pm->x_track_tol = 0.f;			/* (rad) */
	pm->x_gain_P = 35.f;
	pm->x_gain_D = 10.f;
}

static void
pm_auto_machine_default(pmc_t *pm)
{
	pm->probe_speed_hold = 900.f;

	pm->lu_gain_mq_LP = 5.E-4f;

	pm->forced_maximal = 280.f;
	pm->forced_reverse = pm->forced_maximal;
	pm->forced_accel = 400.f;

	pm->zone_noise = 50.f;
	pm->zone_threshold = 90.f;

	pm->const_lambda = 0.f;
	pm->const_Rs = 0.f;
	pm->const_Ja = 0.f;
	pm->const_im_Ld = 0.f;
	pm->const_im_Lq = 0.f;
	pm->const_im_A = 0.f;
	pm->const_im_Rz = 0.f;

	pm->i_slew_rate = 10000.f;
	pm->i_gain_P = 2.E-1f;
	pm->i_gain_I = 5.E-3f;

	pm->s_gain_P = 4.E-2f;
	pm->s_gain_I = 0.f;
	pm->s_gain_D = 2.E-4f;
}

static void
pm_auto_scale_default(pmc_t *pm)
{
	pm->scale_iA[0] = 0.f;
	pm->scale_iA[1] = 1.f;
	pm->scale_iB[0] = 0.f;
	pm->scale_iB[1] = 1.f;
	pm->scale_iC[0] = 0.f;
	pm->scale_iC[1] = 1.f;
	pm->scale_uS[0] = 0.f;
	pm->scale_uS[1] = 1.f;
	pm->scale_uA[0] = 0.f;
	pm->scale_uA[1] = 1.f;
	pm->scale_uB[0] = 0.f;
	pm->scale_uB[1] = 1.f;
	pm->scale_uC[0] = 0.f;
	pm->scale_uC[1] = 1.f;
}

static void
pm_auto_maximal_current(pmc_t *pm)
{
	float			maximal_A, thld_A;

	/* Get the maximal inline current.
	 * */
	maximal_A = pm->fault_current_halt * 0.8f;

	if (pm->const_Rs > M_EPSILON) {

		/* Based on DC link voltage.
		 * */
		thld_A = pm->k_UMAX * pm->const_fb_U / pm->const_Rs;
		maximal_A = (thld_A < maximal_A) ? thld_A : maximal_A;

		if (pm->probe_loss_maximal > M_EPSILON) {

			/* Based on resistive LOSSES.
			 * */
			thld_A = m_sqrtf(pm->k_UMAX / pm->const_Rs
					* pm->probe_loss_maximal);

			maximal_A = (thld_A < maximal_A) ? thld_A : maximal_A;
		}

		if (maximal_A < pm->i_maximal) {

			pm->i_maximal = (float) (int) maximal_A;
		}

		if (maximal_A < pm->i_reverse) {

			pm->i_reverse = (float) (int) maximal_A;
		}
	}
	else {
		pm->i_maximal = (float) (int) maximal_A;
		pm->i_reverse = pm->i_maximal;
	}

	/* Hold current based on maximal machine current.
	 * */
	maximal_A = pm->i_maximal * 0.8f;

	if (pm->probe_current_hold > maximal_A) {

		pm->probe_current_hold = maximal_A;
	}

	if (pm->forced_hold_D > maximal_A) {

		pm->forced_hold_D = maximal_A;
	}

	/* Sine current based on maximal machine current.
	 * */
	maximal_A = pm->i_maximal * 0.2f;

	if (pm->probe_current_sine > maximal_A) {

		pm->probe_current_sine = maximal_A;
	}
}

static void
pm_auto_probe_speed_hold(pmc_t *pm)
{
	float			probe_MAX, probe_MIN;

	probe_MIN = 1.5f * (pm->zone_threshold + pm->zone_noise);

	if (pm->probe_speed_hold < probe_MIN) {

		pm->probe_speed_hold = probe_MIN;
	}

	if (pm->const_lambda > M_EPSILON) {

		probe_MAX = 0.7f * pm->k_EMAX * pm->const_fb_U / pm->const_lambda;

		if (pm->probe_speed_hold > probe_MAX) {

			pm->probe_speed_hold = probe_MAX;
		}
	}
}

static void
pm_auto_forced_maximal(pmc_t *pm)
{
	float		forced_MAX;

	pm->forced_maximal = 1.5f * (pm->zone_threshold + pm->zone_noise);
	pm->forced_reverse = pm->forced_maximal;

	if (pm->const_lambda > M_EPSILON) {

		forced_MAX = 0.7f * pm->k_EMAX * pm->const_fb_U / pm->const_lambda;

		if (pm->forced_maximal > forced_MAX) {

			pm->forced_maximal = forced_MAX;
			pm->forced_reverse = pm->forced_maximal;
		}
	}
}

static void
pm_auto_forced_accel(pmc_t *pm)
{
	float		mQ;

	if (pm->const_Ja > 0.f) {

		mQ = pm_torque_maximal(pm, pm->forced_hold_D);

		/* Tune forced control based on the motor constants.
		 * */
		pm->forced_accel = 0.1f * mQ / pm->const_Ja;
	}
}

static void
pm_auto_zone_threshold(pmc_t *pm)
{
	float			thld_MAX, thld_MIN, thld_IRU;

	if (pm->const_Rs > M_EPSILON) {

		/* Allowable range of the noise threshold.
		 * */
		thld_MAX = 400.f;			/* (rad/s) */
		thld_MIN = 10.f;			/* (rad/s) */

		if (pm->zone_noise > thld_MAX) {

			pm->zone_noise = thld_MAX;
		}

		if (pm->zone_noise < thld_MIN) {

			pm->zone_noise = thld_MIN;
		}

		if (pm->const_lambda > M_EPSILON) {

			thld_MAX = 10.f / pm->const_lambda;

			if (pm->zone_noise > thld_MAX) {

				pm->zone_noise = thld_MAX;
			}

			thld_MIN = pm->fault_terminal_tol / pm->const_lambda;

			if (pm->zone_noise < thld_MIN) {

				pm->zone_noise = thld_MIN;
			}
		}

		/* Based on uncertainty due to resistance thermal drift.
		 * */
		thld_IRU = 0.2f * pm->const_Rs * pm->i_maximal;

		if (pm->config_DCU_VOLTAGE == PM_ENABLED) {

			/* Based on DT compensation accuracy.
			 * */
			thld_IRU += pm->fault_terminal_tol;
		}
		else {
			/* Based on voltage uncertainty on DT.
			 * */
			thld_IRU += PM_DTNS(pm, pm->dcu_deadband) * pm->const_fb_U;
		}

		if (pm->const_lambda > M_EPSILON) {

			/* Total ZONE threshold.
			 * */
			pm->zone_threshold = thld_IRU / pm->const_lambda;
		}

		if (pm->zone_threshold > thld_MAX) {

			pm->zone_threshold = thld_MAX;
		}

		if (pm->zone_threshold < thld_MIN) {

			pm->zone_threshold = thld_MIN;
		}
	}
}

static void
pm_auto_loop_current(pmc_t *pm)
{
	float		Lmin, Df, Kp, Ki;

	if (		   pm->const_im_Ld > M_EPSILON
			&& pm->const_im_Lq > M_EPSILON) {

		Lmin = (pm->const_im_Ld < pm->const_im_Lq)
			? pm->const_im_Ld : pm->const_im_Lq;

		Df = pm->i_damping;

		/* We tune the current loop based on state-space model.
		 *
		 *          [ 1-R*T/L 0 ]          [ T/L ]
		 * x(k+1) = [ T       1 ] * x(k) + [ 0   ] * u(k),
		 *
		 * u(k)   = [ Kp  Ki ] * (x(k) - x(0)).
		 *
		 * */
		Kp = 0.5f  * Lmin * Df * pm->m_freq - pm->const_Rs;
		Ki = 0.02f * Lmin * Df * pm->m_freq;

		pm->i_gain_P = (Kp > 0.f) ? Kp : 0.f;
		pm->i_gain_I = Ki;

		pm->i_slew_rate = 0.2f * Df * pm->const_fb_U / Lmin;
	}
}

static void
pm_auto_loop_speed(pmc_t *pm)
{
	float		Df, Nf, mq_LP, Kp, Kd;

	if (		pm->zone_noise > M_EPSILON
			&& pm->const_Ja > 0.f) {

		Df = pm->s_damping;
		Nf = 1.f / pm->zone_noise;

		mq_LP = 0.02f * Nf * pm->m_dT / pm->const_Ja;
		mq_LP =   (mq_LP > 5.E-3f) ? 5.E-3f
			: (mq_LP < 5.E-4f) ? 5.E-4f : mq_LP;

		/* We tune the speed loop based on noise damping.
		 * */
		Kp = 2.0f  * Df * Nf;
		Kd = 0.01f * Df * Nf;

		pm->lu_gain_mq_LP = mq_LP * Df;

		pm->s_gain_P = Kp;
		pm->s_gain_I = 0.f;
		pm->s_gain_D = Kd;
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

		case PM_AUTO_MACHINE_DEFAULT:
			pm_auto_machine_default(pm);
			break;

		case PM_AUTO_SCALE_DEFAULT:
			pm_auto_scale_default(pm);
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
	float		mQ, rel;

	if (pm->config_RELUCTANCE == PM_ENABLED) {

		rel = pm->quick_Lrel * iD;
	}
	else {
		rel = 0.f;
	}

	mQ = pm->k_KWAT * (pm->const_lambda + rel) * iQ;

	return mQ;
}

static float
pm_torque_MTPA(pmc_t *pm, float iQ)
{
	float		iD, bQ, bW;

	bQ = iQ * iQ * pm->quick_Lrel * pm->quick_Lrel;
	bW = pm->const_lambda * pm->const_lambda;

	iD = (m_sqrtf(16.f * bQ - 4.f * pm->const_lambda * m_sqrtf(4.f * bQ + bW)
				+ 5.f * bW) - pm->const_lambda) * pm->quick_iL4rel;

	return iD;
}

float pm_torque_maximal(pmc_t *pm, float iQ)
{
	float		mQ;

	if (pm->config_RELUCTANCE == PM_ENABLED) {

		mQ = pm_torque_equation(pm, pm_torque_MTPA(pm, iQ), iQ);
	}
	else {
		mQ = pm_torque_equation(pm, 0.f, iQ);
	}

	return mQ;
}

static float
pm_lu_current(pmc_t *pm, float mSP, float *Q)
{
	float		iQ, mQ, iQd, mQd;

	if (pm->config_RELUCTANCE == PM_ENABLED) {

		iQ = *Q;
		mQ = pm_torque_equation(pm, pm_torque_MTPA(pm, iQ), iQ);

		iQd = (mSP < mQ) ? iQ - pm->mtpa_tol : iQ + pm->mtpa_tol;
		mQd = pm_torque_equation(pm, pm_torque_MTPA(pm, iQd), iQd);

		iQ += (mSP - mQ) * (iQd - iQ) * m_fast_recipf(mQd - mQ);

		iQ =      (iQ > pm->i_maximal) ? pm->i_maximal
			: (iQ < - pm->i_reverse) ? - pm->i_reverse : iQ;

		*Q = iQ;
	}
	else {
		if (pm->const_lambda > M_EPSILON) {

			iQ = mSP / (pm->k_KWAT * pm->const_lambda);
		}
		else {
			iQ = 0.f;
		}
	}

	return iQ;
}

static float
pm_lu_accel(pmc_t *pm)
{
	float			mQ, tA = 0.f;

	if (pm->const_Ja > 0.f) {

		mQ = pm->lu_mq_produce - pm->lu_mq_load;
		tA = mQ * m_fast_recipf(pm->const_Ja);
	}

	return tA;
}

static void
pm_forced(pmc_t *pm)
{
	float		wSP, dSA, xRF;

	wSP = pm->s_setpoint_speed;

	if (pm->flux_LINKAGE == PM_ENABLED) {

		/* Maximal forced speed constraint.
		 * */
		wSP = (wSP > pm->forced_maximal) ? pm->forced_maximal :
			(wSP < - pm->forced_reverse) ? - pm->forced_reverse : wSP;
	}

	/* Reduce the acceleration in case of current lack.
	 * */
	xRF = m_fabsf(pm->forced_track_D * m_fast_recipf(pm->forced_hold_D));
	dSA = pm->forced_accel * xRF * pm->m_dT;

	if (		pm->vsi_lpf_DC < pm->forced_stop_DC
			|| m_fabsf(wSP) < m_fabsf(pm->forced_wS)) {

		/* Update actual speed with allowed acceleration.
		 * */
		pm->forced_wS = (pm->forced_wS < wSP - dSA) ? pm->forced_wS + dSA :
			(pm->forced_wS > wSP + dSA) ? pm->forced_wS - dSA : wSP;
	}

	/* Update DQ-axes.
	 * */
	m_rotatef(pm->forced_F, pm->forced_wS * pm->m_dT);
	m_normalizef(pm->forced_F);
}

static void
pm_flux_detached(pmc_t *pm)
{
	float		uA, uB, uC, uX, uY, U, A, B, blend;

	/* Get back EMF voltage.
	 * */
	uA = pm->fb_uA;
	uB = pm->fb_uB;
	uC = pm->fb_uC;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		U = 0.33333333f * (uA + uB + uC);

		uA = uA - U;
		uB = uB - U;

		uX = uA;
		uY = 0.57735027f * uA + 1.1547005f * uB;
	}
	else {
		uX = uA - uC;
		uY = uB - uC;
	}

	pm->vsi_X = uX;
	pm->vsi_Y = uY;

	/* Absolute back EMF voltage.
	 * */
	U = m_hypotf(uX, uY);

	if (U > pm->detach_threshold) {

		A = 1.f / U;

		uX *= A;
		uY *= A;

		if (pm->detach_TIM != 0) {

			/* Speed estimation in phase-locked loop.
			 * */
			m_rotatef(pm->flux_X, pm->flux_wS * pm->m_dT);
			m_normalizef(pm->flux_X);

			A = uX * pm->flux_X[0] + uY * pm->flux_X[1];
			B = uY * pm->flux_X[0] - uX * pm->flux_X[1];

			if (A > M_EPSILON) {

				blend = U * m_fast_recipf(pm->detach_trip_tol);
				blend = (blend > 1.f) ? 1.f : blend;

				A = pm->detach_gain_SF * blend;

				pm->flux_wS += B * pm->m_freq * A;
			}

			pm->flux_lambda = U * m_fast_recipf(m_fabsf(pm->flux_wS));

			A = (pm->flux_wS < 0.f) ? - 1.f : 1.f;

			pm->flux_F[0] = uY * A;
			pm->flux_F[1] = - uX * A;
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
pm_flux_ortega(pmc_t *pm)
{
	float		uX, uY, lX, lY, fX, fY, E, A, B, blend;

	/* Get the actual voltage.
	 * */
	uX = pm->dcu_X - pm->const_Rs * pm->lu_iX;
	uY = pm->dcu_Y - pm->const_Rs * pm->lu_iY;

	/* Stator FLUX.
	 * */
	lX = pm->const_im_Lq * pm->lu_iX;
	lY = pm->const_im_Lq * pm->lu_iY;

	if (pm->flux_LINKAGE == PM_ENABLED) {

		/* Total FLUX equations.
		 * */
		pm->flux_X[0] += uX * pm->m_dT;
		pm->flux_X[1] += uY * pm->m_dT;

		fX = pm->flux_X[0] - lX;
		fY = pm->flux_X[1] - lY;

		blend = m_fabsf(pm->flux_wS * pm->const_lambda)
			* m_fast_recipf(pm->flux_trip_tol);
		blend = (blend > 1.f) ? 1.f : blend;

		/* Get the flux RESIDUE.
		 * */
		E = 1.f - (fX * fX + fY * fY) * pm->quick_iWb2;

		/* Adaptive GAIN.
		 * */
		E *=	  pm->flux_gain_HI * blend
			+ pm->flux_gain_LO * (1.f - blend);

		pm->flux_X[0] += fX * E * pm->quick_iWb;
		pm->flux_X[1] += fY * E * pm->quick_iWb;
	}
	else {
		/* Startup estimation.
		 * */
		pm->flux_X[0] += uX * pm->m_dT;
		pm->flux_X[1] += uY * pm->m_dT;

		fX = pm->flux_X[0] - lX;
		fY = pm->flux_X[1] - lY;

		E = - pm->flux_gain_IN;

		pm->flux_X[0] += fX * E;
		pm->flux_X[1] += fY * E;
	}

	/* Extract the rotor flux linkage.
	 * */
	fX = pm->flux_X[0] - lX;
	fY = pm->flux_X[1] - lY;

	E = m_hypotf(fX, fY);

	pm->flux_lambda = E;

	if (E > M_EPSILON) {

		A = 1.f / E;

		fX *= A;
		fY *= A;

		if (pm->flux_LINKAGE == PM_ENABLED) {

			/* Speed estimation in phase-locked loop.
			 * */
			m_rotatef(pm->flux_F, pm->flux_wS * pm->m_dT);
			m_normalizef(pm->flux_F);

			A = fX * pm->flux_F[0] + fY * pm->flux_F[1];
			B = fY * pm->flux_F[0] - fX * pm->flux_F[1];

			if (A > M_EPSILON) {

				pm->flux_wS += B * pm->m_freq * pm->flux_gain_SF;
			}

			if (pm->flux_gain_IF > M_EPSILON) {

				A = pm_lu_accel(pm) * pm->m_dT;
				pm->flux_wS += A * pm->flux_gain_IF;
			}
		}
		else {
			/* Borrow the speed estimate.
			 * */
			pm->flux_wS = pm->lu_wS;
		}

		pm->flux_F[0] = fX;
		pm->flux_F[1] = fY;
	}
}

static void
pm_kalman_equation(pmc_t *pm, float D[2])
{
	float		uD, uQ, R1, E1, fD, fQ;

	uD = pm->flux_F[0] * pm->dcu_X + pm->flux_F[1] * pm->dcu_Y;
	uQ = pm->flux_F[0] * pm->dcu_Y - pm->flux_F[1] * pm->dcu_X;

	R1 = pm->const_Rs;
	E1 = pm->const_lambda;

	uQ += pm->kalman_bias_Q;

	fD = pm->const_im_Ld * pm->flux_X[0] + E1;
	fQ = pm->const_im_Lq * pm->flux_X[1];

	D[0] = (uD - R1 * pm->flux_X[0] + fQ * pm->flux_wS) * pm->quick_iLd;
	D[1] = (uQ - R1 * pm->flux_X[1] - fD * pm->flux_wS) * pm->quick_iLq;
}

static void
pm_kalman_solve(pmc_t *pm)
{
	float		D0[2], D1[2];

	/* Second-order ODE solver.
	 * */

	pm_kalman_equation(pm, D0);

	pm->flux_X[0] += D0[0] * pm->m_dT;
	pm->flux_X[1] += D0[1] * pm->m_dT;

	m_rotatef(pm->flux_F, pm->flux_wS * pm->m_dT);
	m_normalizef(pm->flux_F);

	pm_kalman_equation(pm, D1);

	pm->flux_X[0] += (D1[0] - D0[0]) * pm->m_dT * 0.5f;
	pm->flux_X[1] += (D1[1] - D0[1]) * pm->m_dT * 0.5f;
}

static void
pm_kalman_forecast(pmc_t *pm)
{
	float		*P = pm->kalman_P;
	const float	*A = pm->kalman_A;
	const float	*Q = pm->kalman_gain_Q;

	const float	iX = A[0];
	const float	iY = A[1];
	const float	uX = A[2];
	const float	uY = A[3];
	const float	fC = A[4];
	const float	fS = A[5];
	const float	wS = A[6];
	const float	bQ = A[7];

	float		u[17], F[10], R1, E1;

	/*
	 * Calculate predicted (a priori) covariance to the next cycle.
	 *
	 * P = F * P * F' + Q.
	 *
	 *     [ F(0) F(1) F(2) F(3) F(4) ]
	 *     [ F(5) F(6) F(7) F(8) F(9) ]
	 * F = [ 0    0    1    dT   0    ]
	 *     [ 0    0    0    1    0    ]
	 *     [ 0    0    0    0    1    ]
	 *
	 *     [ P(0)  P(1)  P(3)  P(6)  P(10) ]
	 *     [ P(1)  P(2)  P(4)  P(7)  P(11) ]
	 * P = [ P(3)  P(4)  P(5)  P(8)  P(12) ]
	 *     [ P(6)  P(7)  P(8)  P(9)  P(13) ]
	 *     [ P(10) P(11) P(12) P(13) P(14) ]
	 *
	 * */

	u[0] = fC * fC;
	u[1] = fS * fS;
	u[2] = fC * fS;
	u[3] = 2.0f * u[2];	/* sin(2*th) */
	u[4] = u[0] - u[1];	/* cos(2*th) */

	R1 = pm->const_Rs;
	E1 = pm->const_lambda;

	u[5] = (uX - R1 * iX) * pm->quick_TiLu[0];
	u[6] = (uY - R1 * iY) * pm->quick_TiLu[0];
	u[7] = (wS * E1 - bQ) * pm->quick_TiLq;

	u[8] = iX * u[4] + iY * u[3];
	u[9] = iY * u[4] - iX * u[3];

	F[0] = 1.f - R1 * (pm->quick_TiLd - u[1] * pm->quick_TiLu[0]);
	F[6] = 1.f - R1 * (pm->quick_TiLq + u[1] * pm->quick_TiLu[0]);

	F[1] = - R1 * u[2] * pm->quick_TiLu[0];
	F[5] = F[1];

	F[2] = u[6] * u[4] - u[5] * u[3] + u[7] * fC;
	F[7] = u[5] * u[4] + u[6] * u[3] + u[7] * fS;

	F[3] =   E1 * fS * pm->quick_TiLq;
	F[8] = - E1 * fC * pm->quick_TiLq;

	F[4] = - fS * pm->quick_TiLq;
	F[9] =   fC * pm->quick_TiLq;

	F[0] +=   wS * u[2] * pm->quick_TiLu[1];
	F[6] += - wS * u[2] * pm->quick_TiLu[1];

	F[1] += wS * (pm->quick_TiLu[2] - u[0] * pm->quick_TiLu[1]);
	F[5] += wS * (pm->quick_TiLu[3] - u[0] * pm->quick_TiLu[1]);

	F[2] +=   wS * u[8] * pm->quick_TiLu[1];
	F[7] += - wS * u[9] * pm->quick_TiLu[1];

	u[5] = pm->const_im_Ld - pm->const_im_Lq;
	u[6] = pm->const_im_Ld + pm->const_im_Lq;

	F[3] += - 0.5f * (iY * u[5] + u[9] * u[6]) * pm->quick_TiLu[0];
	F[8] +=   0.5f * (iX * u[5] + u[8] * u[6]) * pm->quick_TiLu[0];

	u[0] = F[0] * P[0]  + F[1] * P[1]  + F[2] * P[3]  + F[3] * P[6]  + F[4] * P[10];
	u[1] = F[0] * P[1]  + F[1] * P[2]  + F[2] * P[4]  + F[3] * P[7]  + F[4] * P[11];
	u[2] = F[0] * P[3]  + F[1] * P[4]  + F[2] * P[5]  + F[3] * P[8]  + F[4] * P[12];
	u[3] = F[0] * P[6]  + F[1] * P[7]  + F[2] * P[8]  + F[3] * P[9]  + F[4] * P[13];
	u[4] = F[0] * P[10] + F[1] * P[11] + F[2] * P[12] + F[3] * P[13] + F[4] * P[14];

	u[5] = F[5] * P[0]  + F[6] * P[1]  + F[7] * P[3]  + F[8] * P[6]  + F[9] * P[10];
	u[6] = F[5] * P[1]  + F[6] * P[2]  + F[7] * P[4]  + F[8] * P[7]  + F[9] * P[11];
	u[7] = F[5] * P[3]  + F[6] * P[4]  + F[7] * P[5]  + F[8] * P[8]  + F[9] * P[12];
	u[8] = F[5] * P[6]  + F[6] * P[7]  + F[7] * P[8]  + F[8] * P[9]  + F[9] * P[13];
	u[9] = F[5] * P[10] + F[6] * P[11] + F[7] * P[12] + F[8] * P[13] + F[9] * P[14];

	u[10] = P[3]  + pm->m_dT * P[6];
	u[11] = P[4]  + pm->m_dT * P[7];
	u[12] = P[5]  + pm->m_dT * P[8];
	u[13] = P[8]  + pm->m_dT * P[9];
	u[14] = P[12] + pm->m_dT * P[13];

	u[15] = P[6];
	u[16] = P[10];

	P[0]  = u[0] * F[0] + u[1] * F[1] + u[2] * F[2] + u[3] * F[3] + u[4] * F[4];
	P[1]  = u[5] * F[0] + u[6] * F[1] + u[7] * F[2] + u[8] * F[3] + u[9] * F[4];
	P[2]  = u[5] * F[5] + u[6] * F[6] + u[7] * F[7] + u[8] * F[8] + u[9] * F[9];
	P[3]  = u[10] * F[0] + u[11] * F[1] + u[12] * F[2] + u[13] * F[3] + u[14] * F[4];
	P[4]  = u[10] * F[5] + u[11] * F[6] + u[12] * F[7] + u[13] * F[8] + u[14] * F[9];
	P[5]  = u[12] + u[13] * pm->m_dT;
	P[6]  = u[15] * F[0] + P[7] * F[1] + P[8] * F[2] + P[9] * F[3] + P[13] * F[4];
	P[7]  = u[15] * F[5] + P[7] * F[6] + P[8] * F[7] + P[9] * F[8] + P[13] * F[9];
	P[10] = u[16] * F[0] + P[11] * F[1] + P[12] * F[2] + P[13] * F[3] + P[14] * F[4];
	P[11] = u[16] * F[5] + P[11] * F[6] + P[12] * F[7] + P[13] * F[8] + P[14] * F[9];

	P[8]  += P[9]  * pm->m_dT;
	P[12] += P[13] * pm->m_dT;

	P[0]  += Q[0] * pm->quick_TiLq;
	P[2]  += Q[0] * pm->quick_TiLq;
	P[5]  += Q[1] * pm->m_dT;
	P[9]  += Q[2] * pm->m_dT;
	P[14] += Q[3] * pm->m_dT;
}

static void
pm_kalman_update(pmc_t *pm)
{
	float		*P = pm->kalman_P;
	float		*K = pm->kalman_K;
	float		iR = pm->kalman_gain_R;

	float		HP[5], u;

	/*
	 * Calculate updated (a posteriori) covariance and Kalman gain.
	 *
	 * S = H * P * H' + R.
	 * K = P * H' / S.
	 * P = P - K * H * P.
	 *
	 * H(1) = [ 1  0  0  0  0 ]
	 * H(2) = [ 0  1  0  0  0 ]
	 *
	 * */

	HP[0] = P[0];
	HP[1] = P[1];
	HP[2] = P[3];
	HP[3] = P[6];
	HP[4] = P[10];

	u = 1.f / (HP[0] + iR);

	K[0] = HP[0] * u;
	K[2] = HP[1] * u;
	K[4] = HP[2] * u;
	K[6] = HP[3] * u;
	K[8] = HP[4] * u;

	P[0]  += - K[0] * HP[0];
	P[1]  += - K[2] * HP[0];
	P[2]  += - K[2] * HP[1];
	P[3]  += - K[4] * HP[0];
	P[4]  += - K[4] * HP[1];
	P[5]  += - K[4] * HP[2];
	P[6]  += - K[6] * HP[0];
	P[7]  += - K[6] * HP[1];
	P[8]  += - K[6] * HP[2];
	P[9]  += - K[6] * HP[3];
	P[10] += - K[8] * HP[0];
	P[11] += - K[8] * HP[1];
	P[12] += - K[8] * HP[2];
	P[13] += - K[8] * HP[3];
	P[14] += - K[8] * HP[4];

	HP[0] = P[1];
	HP[1] = P[2];
	HP[2] = P[4];
	HP[3] = P[7];
	HP[4] = P[11];

	u = 1.f / (HP[1] + iR);

	K[1] = HP[0] * u;
	K[3] = HP[1] * u;
	K[5] = HP[2] * u;
	K[7] = HP[3] * u;
	K[9] = HP[4] * u;

	P[0]  += - K[1] * HP[0];
	P[1]  += - K[3] * HP[0];
	P[2]  += - K[3] * HP[1];
	P[3]  += - K[5] * HP[0];
	P[4]  += - K[5] * HP[1];
	P[5]  += - K[5] * HP[2];
	P[6]  += - K[7] * HP[0];
	P[7]  += - K[7] * HP[1];
	P[8]  += - K[7] * HP[2];
	P[9]  += - K[7] * HP[3];
	P[10] += - K[9] * HP[0];
	P[11] += - K[9] * HP[1];
	P[12] += - K[9] * HP[2];
	P[13] += - K[9] * HP[3];
	P[14] += - K[9] * HP[4];

	u = - K[2];

	K[0] += K[1] * u;
	K[2] += K[3] * u;
	K[4] += K[5] * u;
	K[6] += K[7] * u;
	K[8] += K[9] * u;
}

static void
pm_kalman_lockout_guard(pmc_t *pm, float dA)
{
	float		thld_wS;

	/* Get speed LPF of actual DQ-axes.
	 * */
	pm->kalman_lpf_wS += (dA * pm->m_freq - pm->kalman_lpf_wS) * pm->zone_gain_LP;

	if (		   pm->flux_ZONE == PM_ZONE_NONE
			|| pm->flux_ZONE == PM_ZONE_UNCERTAIN) {

		thld_wS = pm->zone_threshold * pm->zone_gain_TH;

		if (m_fabsf(pm->kalman_lpf_wS) > thld_wS) {

			/* Restart Kalman and flip DQ-axes.
			 * */
			pm->flux_TYPE = PM_FLUX_NONE;

			pm->flux_F[0] = - pm->flux_F[0];
			pm->flux_F[1] = - pm->flux_F[1];
			pm->flux_wS += pm->kalman_lpf_wS;

			pm->kalman_POSTPONED = PM_DISABLED;
			pm->kalman_lpf_wS = 0.f;
		}
	}
}

static void
pm_flux_kalman(pmc_t *pm)
{
	const float		*K = pm->kalman_K;
	float			*A = pm->kalman_A;

	float			E[2], tA, dA = 0.f;

	/* Get the current estimate in XY-axes.
	 * */
	A[0] = pm->flux_F[0] * pm->flux_X[0] - pm->flux_F[1] * pm->flux_X[1];
	A[1] = pm->flux_F[1] * pm->flux_X[0] + pm->flux_F[0] * pm->flux_X[1];

	/* Get the current residuals.
	 * */
	E[0] = pm->lu_iX - A[0];
	E[1] = pm->lu_iY - A[1];

	if (likely(pm->vsi_IF == 0)) {

		A[0] += K[0] * E[0] + K[1] * E[1];
		A[1] += K[2] * E[0] + K[3] * E[1];

		dA = K[4] * E[0] + K[5] * E[1];
		dA = (dA < - 1.f) ? - 1.f : (dA > 1.f) ? 1.f : dA;

		m_rotatef(pm->flux_F, dA);

		pm->flux_X[0] = pm->flux_F[0] * A[0] + pm->flux_F[1] * A[1];
		pm->flux_X[1] = pm->flux_F[0] * A[1] - pm->flux_F[1] * A[0];

		if (pm->flux_LINKAGE == PM_ENABLED) {

			pm->flux_wS += K[6] * E[0] + K[7] * E[1];
			pm->flux_wS = (pm->flux_wS < - pm->m_freq) ? - pm->m_freq
				: (pm->flux_wS > pm->m_freq) ? pm->m_freq : pm->flux_wS;

			if (pm->flux_ZONE == PM_ZONE_HIGH) {

				pm->kalman_bias_Q += K[8] * E[0] + K[9] * E[1];
			}
			else {
				pm->kalman_bias_Q = 0.f;
			}
		}
		else {
			/* Borrow the speed estimate.
			 * */
			pm->flux_wS = pm->lu_wS;

			pm->kalman_bias_Q += K[8] * E[0] + K[9] * E[1];

			if (		pm->kalman_bias_Q < 0.f
					&& pm->flux_wS < 0.f) {

				pm->kalman_bias_Q = 0.f;
			}
			else if (	pm->kalman_bias_Q > 0.f
					&& pm->flux_wS > 0.f) {

				pm->kalman_bias_Q = 0.f;
			}
		}

		if (PM_CONFIG_DBG(pm) == PM_ENABLED) {

			pm->kalman_rsu_D = pm->flux_F[0] * E[0] + pm->flux_F[1] * E[1];
			pm->kalman_rsu_Q = pm->flux_F[0] * E[1] - pm->flux_F[1] * E[0];
		}
	}

	if (		pm->flux_LINKAGE == PM_ENABLED
			&& pm->flux_gain_IF > M_EPSILON) {

		tA = pm_lu_accel(pm) * pm->m_dT;
		pm->flux_wS += tA * pm->flux_gain_IF;
	}

	/* Set aside the variables to calculate covariance at the end of cycle.
	 * */
	A[2] = pm->vsi_X;
	A[3] = pm->vsi_Y;
	A[4] = pm->flux_F[0];
	A[5] = pm->flux_F[1];
	A[6] = pm->flux_wS;
	A[7] = pm->kalman_bias_Q;

	pm->kalman_POSTPONED = PM_ENABLED;

	/* We propagate the state estimates to the next cycle.
	 * */
	pm_kalman_solve(pm);

	/* Guard against lockout of DQ-axes in reverse position.
	 * */
	pm_kalman_lockout_guard(pm, dA);
}

static void
pm_flux_zone(pmc_t *pm)
{
	float			thld_wS;

	/* Get FLUX speed LPF to switch ZONE.
	 * */
	pm->zone_lpf_wS += (pm->flux_wS - pm->zone_lpf_wS) * pm->zone_gain_LP;

	if (		   pm->flux_ZONE == PM_ZONE_NONE
			|| pm->flux_ZONE == PM_ZONE_UNCERTAIN) {

		thld_wS = pm->zone_threshold + pm->zone_noise;

		if (pm->lu_MODE == PM_LU_DETACHED) {

			int	lev_TIM = PM_TSMS(pm, pm->tm_transient_slow);

			if (		m_fabsf(pm->zone_lpf_wS) > thld_wS
					&& pm->detach_TIM > lev_TIM) {

				pm->flux_ZONE = PM_ZONE_HIGH;
			}
		}
		else {
			if (		pm->zone_lpf_wS > thld_wS
					&& pm->lu_wS > thld_wS) {

				pm->flux_ZONE = PM_ZONE_HIGH;
			}
			else if (	pm->zone_lpf_wS < - thld_wS
					&& pm->lu_wS < - thld_wS) {

				pm->flux_ZONE = PM_ZONE_HIGH;
			}
		}
	}
	else if (pm->flux_ZONE == PM_ZONE_HIGH) {

		thld_wS = pm->zone_threshold * pm->zone_gain_TH;

		if (pm->lu_MODE == PM_LU_DETACHED) {

			if (		m_fabsf(pm->zone_lpf_wS) < thld_wS
					|| pm->detach_TIM < 10) {

				pm->flux_ZONE = PM_ZONE_UNCERTAIN;
			}
		}
		else {
			if (m_fabsf(pm->zone_lpf_wS) < thld_wS) {

				pm->flux_ZONE = PM_ZONE_UNCERTAIN;
			}
		}
	}
}

static void
pm_estimate(pmc_t *pm)
{
	if (pm->config_LU_ESTIMATE == PM_FLUX_ORTEGA) {

		if (pm->flux_TYPE != PM_FLUX_ORTEGA) {

			float			E1, Lq;

			E1 = pm->const_lambda;
			Lq = pm->const_im_Lq;

			pm->flux_X[0] = E1 * pm->flux_F[0] + Lq * pm->lu_iX;
			pm->flux_X[1] = E1 * pm->flux_F[1] + Lq * pm->lu_iY;

			pm->flux_TYPE = PM_FLUX_ORTEGA;
		}

		pm_flux_ortega(pm);
		pm_flux_zone(pm);
	}
	else if (pm->config_LU_ESTIMATE == PM_FLUX_KALMAN) {

		if (pm->flux_TYPE != PM_FLUX_KALMAN) {

			pm->flux_X[0] = pm->lu_iD;
			pm->flux_X[1] = pm->lu_iQ;

			pm->flux_F[0] = pm->lu_F[0];
			pm->flux_F[1] = pm->lu_F[1];

			pm->kalman_P[0] = 0.f;
			pm->kalman_P[1] = 0.f;
			pm->kalman_P[2] = 0.f;
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

		pm_flux_kalman(pm);
		pm_flux_zone(pm);
	}
	else {
		/* NOTE: No sensorless observer selected. It is ok when you
		 * only need a SENSORED drive */

		if (pm->flux_TYPE != PM_FLUX_NONE) {

			pm->flux_TYPE = PM_FLUX_NONE;
			pm->flux_ZONE = PM_ZONE_NONE;
		}
	}
}

static float
pm_hfi_wave(pmc_t *pm)
{
	float		uHF, hCOS, hSIN;

	if (pm->config_HFI_WAVETYPE == PM_HFI_SINE) {

		const float	*HF = pm->quick_HF;

		/* HF sine wavetype.
		 * */
		hCOS = HF[0] * pm->hfi_wave[0] - HF[1] * pm->hfi_wave[1];
		hSIN = HF[1] * pm->hfi_wave[0] + HF[0] * pm->hfi_wave[1];

		pm->hfi_wave[0] = hCOS;
		pm->hfi_wave[1] = hSIN;

		m_normalizef(pm->hfi_wave);

		uHF = pm->hfi_wave[0] * pm->hfi_amplitude
			* pm->quick_HFwS * pm->const_im_Ld;
	}
	else if (pm->config_HFI_WAVETYPE == PM_HFI_SQUARE) {

		/* HF square wavetype.
		 * */
		pm->hfi_wave[0] = (pm->hfi_wave[0] < 0.f) ? 2.1f : - 2.1f;

		uHF = pm->hfi_wave[0] * pm->hfi_amplitude
			* pm->m_freq * pm->const_im_Ld;
	}
	else if (pm->config_HFI_WAVETYPE == PM_HFI_RANDOM) {

		/* HF random sequence.
		 * */
		if (pm->hfi_wave[1] > M_PI_F) {

			pm->hfi_wave[0] = m_lf_gaussf(&pm->lfseed) * 0.2f;
			pm->hfi_wave[1] += - M_PI_F;
		}

		pm->hfi_wave[1] += pm->quick_HFwS * pm->m_dT;

		uHF = pm->hfi_wave[0] * pm->hfi_amplitude
			* pm->m_freq * pm->const_im_Ld;
	}
	else {
		/* No HF wave.
		 * */
		pm->hfi_wave[0] = 0.f;
		pm->hfi_wave[1] = 1.f;
	}

	return uHF;
}

static void
pm_sensor_hall(pmc_t *pm)
{
	float		F[2], A, B, blend, rel;
	int		HS;

	const float	tol = 0.6f;		/* ~34 degrees */

	HS = pm->fb_HS;

	if (likely(HS >= 1 && HS <= 6)) {

		pm->hall_ERN = 0;

		F[0] = pm->hall_ST[HS].X;
		F[1] = pm->hall_ST[HS].Y;

		A = F[0] * pm->hall_F[0] + F[1] * pm->hall_F[1];
		B = F[1] * pm->hall_F[0] - F[0] * pm->hall_F[1];

		rel = m_atan2f(B, A);

		if (m_fabsf(rel) > tol) {

			rel += (rel < 0.f) ? tol : - tol;

			m_rotatef(pm->hall_F, rel);

			blend = m_fabsf(pm->hall_wS)
				* m_fast_recipf(pm->hall_trip_tol);
			blend = (blend > 1.f) ? 1.f : blend;

			A =	  pm->hall_gain_SF * blend
				+ pm->hall_gain_LO * (1.f - blend);

			pm->hall_wS += rel * pm->m_freq * A;
		}

		if (pm->hall_gain_IF > M_EPSILON) {

			A = pm_lu_accel(pm) * pm->m_dT;
			pm->hall_wS += A * pm->hall_gain_IF;
		}

		m_rotatef(pm->hall_F, pm->hall_wS * pm->m_dT);
		m_normalizef(pm->hall_F);
	}
	else {
		pm->hall_ERN++;

		if (unlikely(pm->hall_ERN >= 10)) {

			pm->fsm_errno = PM_ERROR_SENSOR_HALL_FAULT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}
}

static void
pm_sensor_eabi(pmc_t *pm)
{
	float		F[2], A, blend, ANG, rel;
	int		relEP, WRAP;

	const float	tol = m_fabsf(pm->quick_ZiEP) * 0.6f;

	if (pm->eabi_RECENT != PM_ENABLED) {

		pm->eabi_bEP = pm->fb_EP;
		pm->eabi_unwrap = 0;
		pm->eabi_interp = 0.f;

		if (pm->config_EABI_FRONTEND == PM_EABI_INCREMENTAL) {

			pm->eabi_lEP = 0;
		}
		else if (pm->config_EABI_FRONTEND == PM_EABI_ABSOLUTE) {

			pm->eabi_lEP = pm->fb_EP;
		}

		if (pm->config_LU_SENSOR == PM_SENSOR_EABI) {

			pm->eabi_F[0] = pm->lu_F[0];
			pm->eabi_F[1] = pm->lu_F[1];
			pm->eabi_wS = pm->lu_wS;
		}

		pm->eabi_RECENT = PM_ENABLED;
	}

	if (pm->config_EABI_FRONTEND == PM_EABI_INCREMENTAL) {

		WRAP = 0x10000;

		relEP = pm->fb_EP - pm->eabi_bEP;
		relEP +=  unlikely(relEP > WRAP / 2 - 1) ? - WRAP
			: unlikely(relEP < - WRAP / 2) ? WRAP : 0;

		pm->eabi_bEP = pm->fb_EP;
	}
	else if (pm->config_EABI_FRONTEND == PM_EABI_ABSOLUTE) {

		WRAP = pm->eabi_const_EP;

		pm->eabi_bEP = pm->eabi_lEP - (pm->eabi_lEP / WRAP) * WRAP;
		pm->eabi_bEP += (pm->eabi_bEP < 0) ? WRAP : 0;

		relEP = pm->fb_EP - pm->eabi_bEP;
		relEP +=  unlikely(relEP > WRAP / 2 - 1) ? - WRAP
			: unlikely(relEP < - WRAP / 2) ? WRAP : 0;
	}
	else {
		relEP = 0;
	}

	if (relEP != 0) {

		pm->eabi_lEP += relEP;
		pm->eabi_interp += - (float) relEP * pm->quick_ZiEP;

		WRAP = pm->eabi_const_EP * pm->eabi_const_Zq;

		if (pm->eabi_lEP < - WRAP) {

			pm->eabi_unwrap += - pm->eabi_const_Zq;
			pm->eabi_lEP += WRAP;
		}
		else if (pm->eabi_lEP > WRAP) {

			pm->eabi_unwrap += pm->eabi_const_Zq;
			pm->eabi_lEP += - WRAP;
		}
	}

	/* We get residual when interp goes out of tolerance.
	 * */
	rel = (pm->eabi_interp > tol) ? tol - pm->eabi_interp
		: (pm->eabi_interp < - tol) ? - tol - pm->eabi_interp : 0.f;

	pm->eabi_interp += rel;

	blend = m_fabsf(pm->eabi_wS) * m_fast_recipf(pm->eabi_trip_tol);
	blend = (blend > 1.f) ? 1.f : blend;

	A =	  pm->eabi_gain_SF * blend
		+ pm->eabi_gain_LO * (1.f - blend);

	pm->eabi_wS += rel * pm->m_freq * A;

	if (pm->eabi_gain_IF > M_EPSILON) {

		A = pm_lu_accel(pm) * pm->m_dT;
		pm->eabi_wS += A * pm->eabi_gain_IF;
	}

	pm->eabi_interp += pm->eabi_wS * pm->m_dT;

	if (pm->config_LU_SENSOR == PM_SENSOR_EABI) {

		/* Take the electrical position DQ-axes.
		 * */
		ANG = (float) pm->eabi_lEP * pm->quick_ZiEP + pm->eabi_interp;

		F[0] = m_cosf(ANG);
		F[1] = m_sinf(ANG);

		if (pm->eabi_ADJUST != PM_ENABLED) {

			pm->eabi_F0[0] = F[0] * pm->lu_F[0] + F[1] * pm->lu_F[1];
			pm->eabi_F0[1] = F[0] * pm->lu_F[1] - F[1] * pm->lu_F[0];

			pm->eabi_ADJUST = PM_ENABLED;
		}

		pm->eabi_F[0] = F[0] * pm->eabi_F0[0] - F[1] * pm->eabi_F0[1];
		pm->eabi_F[1] = F[1] * pm->eabi_F0[0] + F[0] * pm->eabi_F0[1];
	}

	if (pm->config_LU_LOCATION == PM_LOCATION_EABI) {

		/* Take the electrical absolute LOCATION.
		 * */
		ANG = (float) pm->eabi_lEP + (float) pm->eabi_const_EP * pm->eabi_unwrap;

		pm->eabi_location = ANG * pm->quick_ZiEP + pm->eabi_interp;
	}
}

static void
pm_sensor_sincos(pmc_t *pm)
{
	float		*CONST = pm->sincos_CONST;

	float		F[2], A, B, ANG, locAN;
	int		WRAP;

	if (pm->sincos_RECENT != PM_ENABLED) {

		pm->sincos_SC[2] = 0.f;

		pm->sincos_revol = 0;
		pm->sincos_unwrap = 0;

		if (pm->config_LU_SENSOR == PM_SENSOR_SINCOS) {

			pm->sincos_F[0] = pm->lu_F[0];
			pm->sincos_F[1] = pm->lu_F[1];
			pm->sincos_wS = pm->lu_wS;
		}

		pm->sincos_RECENT = PM_ENABLED;
	}

	if (pm->config_SINCOS_FRONTEND == PM_SINCOS_ANALOG) {

		float		Q[7];

		Q[0] = pm->fb_COS;
		Q[1] = pm->fb_SIN;
		Q[2] = pm->lu_iX;
		Q[3] = pm->lu_iY;
		Q[4] = Q[0] * Q[1];
		Q[5] = Q[0] * Q[0];
		Q[6] = Q[1] * Q[1];

		pm->sincos_SC[0] = CONST[0] + CONST[1] * Q[0] + CONST[2] * Q[1]
			+ CONST[3] * Q[2]   + CONST[4] * Q[3] + CONST[5] * Q[4]
			+ CONST[6] * Q[5]   + CONST[7] * Q[6];

		pm->sincos_SC[1] = CONST[8] + CONST[9] * Q[0]  + CONST[10] * Q[1]
			+ CONST[11] * Q[2]  + CONST[12] * Q[3] + CONST[13] * Q[4]
			+ CONST[14] * Q[5]  + CONST[15] * Q[6];
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

	WRAP = pm->sincos_const_Zq;

	if (pm->sincos_revol < - WRAP) {

		pm->sincos_unwrap += - WRAP;
		pm->sincos_revol += WRAP;
	}
	else if (pm->sincos_revol > WRAP) {

		pm->sincos_unwrap += WRAP;
		pm->sincos_revol += - WRAP;
	}

	ANG = m_atan2f(pm->sincos_SC[1], pm->sincos_SC[0])
		+ (float) pm->sincos_revol * M_2_PI_F;

	/* Take the electrical position DQ-axes.
	 * */
	locAN = ANG * pm->quick_ZiSQ;

	F[0] = m_cosf(locAN);
	F[1] = m_sinf(locAN);

	A = F[0] * pm->sincos_F[0] + F[1] * pm->sincos_F[1];
	B = F[1] * pm->sincos_F[0] - F[0] * pm->sincos_F[1];

	if (A > M_EPSILON) {

		m_rotatef(pm->sincos_F, B * pm->sincos_gain_PF);

		pm->sincos_wS += B * pm->m_freq * pm->sincos_gain_SF;
	}
	else {
		pm->sincos_F[0] = F[0];
		pm->sincos_F[1] = F[1];
	}

	if (pm->sincos_gain_IF > M_EPSILON) {

		A = pm_lu_accel(pm) * pm->m_dT;
		pm->sincos_wS += A * pm->sincos_gain_IF;
	}

	m_rotatef(pm->sincos_F, pm->sincos_wS * pm->m_dT);
	m_normalizef(pm->sincos_F);

	if (pm->config_LU_LOCATION == PM_LOCATION_SINCOS) {

		locAN = ANG + (float) pm->sincos_unwrap * M_2_PI_F;

		/* Take the electrical absolute LOCATION.
		 * */
		pm->sincos_location = locAN * pm->quick_ZiSQ;
	}
}

static void
pm_lu_FSM(pmc_t *pm)
{
	float			lu_F[2], hS, A, B;

	int			lu_EABI		= PM_DISABLED;
	int			lu_SINCOS	= PM_DISABLED;

	/* Get the current on DQ-axes.
	 * */
	pm->lu_iD = pm->lu_F[0] * pm->lu_iX + pm->lu_F[1] * pm->lu_iY;
	pm->lu_iQ = pm->lu_F[0] * pm->lu_iY - pm->lu_F[1] * pm->lu_iX;

	/* Get voltage on DQ-axes on the middle of cycle.
	 * */
	m_rotatef(pm->lu_F, pm->lu_wS * pm->m_dT * 0.5f);

	pm->lu_uD = pm->lu_F[0] * pm->dcu_X + pm->lu_F[1] * pm->dcu_Y;
	pm->lu_uQ = pm->lu_F[0] * pm->dcu_Y - pm->lu_F[1] * pm->dcu_X;

	/* Transform DQ-axes to the future cycle position.
	 * */
	m_rotatef(pm->lu_F, pm->lu_wS * pm->m_dT * 0.5f);
	m_normalizef(pm->lu_F);

	if (unlikely(pm->vsi_IF != 0)) {

		/* We transform DQ-axes current back to XY-axes throught future
		 * DQ-axes if there are no clean measurements available.
		 * */
		pm->lu_iX = pm->lu_F[0] * pm->lu_iD - pm->lu_F[1] * pm->lu_iQ;
		pm->lu_iY = pm->lu_F[1] * pm->lu_iD + pm->lu_F[0] * pm->lu_iQ;
	}

	if (pm->lu_MODE != PM_LU_DETACHED) {

		/* Update torque production estimate.
		 * */
		pm->lu_mq_produce = pm_torque_equation(pm, pm->lu_iD, pm->lu_iQ);
	}
	else {
		pm->lu_mq_produce = 0.f;
	}

	if (		pm->config_LU_FORCED == PM_ENABLED
			&& (	   pm->config_LU_DRIVE == PM_DRIVE_CURRENT
				|| pm->config_LU_DRIVE == PM_DRIVE_TORQUE)) {

		float		wSP, iQ;

		iQ = (pm->config_LU_DRIVE == PM_DRIVE_CURRENT)
			? pm->i_setpoint_current : pm->i_setpoint_torque;

		/* Derive the speed SETPOINT in case of current control.
		 * */
		wSP = (iQ < - M_EPSILON) ? - pm->forced_reverse
			: (iQ > M_EPSILON) ? pm->forced_maximal : 0.f;

		pm->s_setpoint_speed = wSP;
	}

	if (pm->lu_MODE == PM_LU_DETACHED) {

		if (pm->flux_DETACH != PM_ENABLED) {

			pm->base_TIM = - PM_TSMS(pm, pm->tm_transient_fast);
			pm->detach_TIM = 0;

			pm->watt_DC_MAX = PM_DISABLED;
			pm->watt_DC_MIN = PM_DISABLED;

			pm->watt_lpf_D = 0.f;
			pm->watt_lpf_Q = 0.f;
			pm->watt_drain_wP = 0.f;
			pm->watt_drain_wA = 0.f;
			pm->watt_integral = 0.f;

			pm->i_track_D = 0.f;
			pm->i_track_Q = 0.f;
			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			pm->flux_DETACH = PM_ENABLED;
		}

		if (pm->base_TIM >= 0) {

			pm_flux_detached(pm);
			pm_flux_zone(pm);
		}

		lu_F[0] = pm->flux_F[0];
		lu_F[1] = pm->flux_F[1];

		pm->lu_wS = pm->flux_wS;

		pm->s_track = pm->lu_wS;
		pm->l_track = pm->lu_wS;

		if (pm->flux_ZONE == PM_ZONE_LOCKED_IN_DETACH) {

			/* Lock in DETACHED mode permanently.
			 * */
		}
		else if (pm->base_TIM < PM_TSMS(pm, pm->tm_pause_startup)) {

			/* Not enough time passed to go into control loop.
			 * */
			pm->base_TIM++;
		}
		else if (	pm->config_LU_ESTIMATE != PM_FLUX_NONE
				&& pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE;

			pm->proc_set_Z(PM_Z_NONE);
		}
		else if (pm->config_LU_SENSOR == PM_SENSOR_HALL) {

			pm->lu_MODE = PM_LU_SENSOR_HALL;

			pm->hall_ERN = 0;
			pm->hall_F[0] = pm->lu_F[0];
			pm->hall_F[1] = pm->lu_F[1];
			pm->hall_wS = pm->lu_wS;

			pm->proc_set_Z(PM_Z_NONE);
		}
		else if (	pm->config_LU_SENSOR == PM_SENSOR_EABI
				&& (	pm->eabi_ADJUST  == PM_ENABLED
					|| pm->flux_ZONE == PM_ZONE_HIGH)) {

			pm->lu_MODE = PM_LU_SENSOR_EABI;

			pm->proc_set_Z(PM_Z_NONE);
		}
		else if (pm->config_LU_SENSOR == PM_SENSOR_SINCOS) {

			pm->lu_MODE = PM_LU_SENSOR_SINCOS;

			pm->proc_set_Z(PM_Z_NONE);
		}
		else if (pm->config_LU_FORCED == PM_ENABLED) {

			if (		pm->config_LU_FREEWHEEL == PM_ENABLED
					&& m_fabsf(pm->s_setpoint_speed) < M_EPSILON) {

				/* Keep in freewheeling until non-zero setpoint is given.
				 * */
			}
			else {
				pm->lu_MODE = PM_LU_FORCED;

				pm->hold_TIM = 0;

				pm->forced_F[0] = pm->lu_F[0];
				pm->forced_F[1] = pm->lu_F[1];
				pm->forced_wS = pm->lu_wS;

				pm->proc_set_Z(PM_Z_NONE);
			}
		}
		else if (       pm->config_LU_ESTIMATE == PM_FLUX_KALMAN
				&& pm->config_HFI_WAVETYPE != PM_HFI_NONE) {

			pm->lu_MODE = PM_LU_ON_HFI;

			pm->hold_TIM = 0;

			pm->proc_set_Z(PM_Z_NONE);
		}
	}
	else if (pm->lu_MODE == PM_LU_FORCED) {

		pm_estimate(pm);
		pm_forced(pm);

		lu_F[0] = pm->forced_F[0];
		lu_F[1] = pm->forced_F[1];

		pm->lu_wS = pm->forced_wS;

		if (		pm->flux_TYPE != PM_FLUX_NONE
				&& pm->flux_LINKAGE != PM_ENABLED) {

			/* Hold on until flux linkage is estimated.
			 * */
		}
		else if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE;
		}
		else {
			if (pm->hold_TIM < PM_TSMS(pm, pm->tm_pause_forced)) {

				pm->hold_TIM++;
			}
			else if (pm->config_LU_SENSOR == PM_SENSOR_EABI) {

				pm->lu_MODE = PM_LU_SENSOR_EABI;
			}
			else if (	pm->config_LU_ESTIMATE == PM_FLUX_KALMAN
					&& pm->config_HFI_WAVETYPE != PM_HFI_NONE) {

				pm->lu_MODE = PM_LU_ON_HFI;
			}
			else if (	PM_CONFIG_TVM(pm) == PM_ENABLED
					&& pm->config_LU_FREEWHEEL == PM_ENABLED
					&& pm->forced_track_D < M_EPSILON) {

				pm->lu_MODE = PM_LU_DETACHED;

				pm->flux_DETACH = PM_DISABLED;
				pm->flux_TYPE = PM_FLUX_NONE;

				pm->proc_set_Z(PM_Z_ABC);
			}
		}
	}
	else if (pm->lu_MODE == PM_LU_ESTIMATE) {

		pm_estimate(pm);

		lu_F[0] = pm->flux_F[0];
		lu_F[1] = pm->flux_F[1];

		pm->lu_wS = pm->flux_wS;

		if (pm->base_TIM < PM_TSMS(pm, pm->tm_pause_startup)) {

			/* Not enough time passed to go into low speed mode.
			 * */
			pm->base_TIM++;
		}
		else if (	   pm->flux_ZONE == PM_ZONE_NONE
				|| pm->flux_ZONE == PM_ZONE_UNCERTAIN) {

			if (pm->config_LU_SENSOR == PM_SENSOR_HALL) {

				pm->lu_MODE = PM_LU_SENSOR_HALL;

				pm->hall_ERN = 0;
				pm->hall_F[0] = pm->lu_F[0];
				pm->hall_F[1] = pm->lu_F[1];
				pm->hall_wS = pm->lu_wS;
			}
			else if (	pm->config_LU_SENSOR == PM_SENSOR_EABI
					&& (	pm->eabi_ADJUST  == PM_ENABLED
						|| pm->flux_ZONE == PM_ZONE_UNCERTAIN)) {

				pm->lu_MODE = PM_LU_SENSOR_EABI;
			}
			else if (pm->config_LU_SENSOR == PM_SENSOR_SINCOS) {

				pm->lu_MODE = PM_LU_SENSOR_SINCOS;
			}
			else if (	pm->config_LU_ESTIMATE == PM_FLUX_KALMAN
					&& pm->config_HFI_WAVETYPE != PM_HFI_NONE) {

				pm->lu_MODE = PM_LU_ON_HFI;

				pm->hold_TIM = 0;
			}
			else if (pm->config_LU_FORCED == PM_ENABLED) {

				if (		pm->config_LU_FREEWHEEL == PM_ENABLED
					&& m_fabsf(pm->s_setpoint_speed) < M_EPSILON) {

					if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

						pm->lu_MODE = PM_LU_DETACHED;

						pm->flux_DETACH = PM_DISABLED;
						pm->flux_TYPE = PM_FLUX_NONE;

						pm->proc_set_Z(PM_Z_ABC);
					}
				}
				else {
					pm->lu_MODE = PM_LU_FORCED;

					pm->hold_TIM = 0;

					pm->forced_F[0] = pm->lu_F[0];
					pm->forced_F[1] = pm->lu_F[1];
					pm->forced_wS = pm->lu_wS;
				}
			}
			else if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				pm->lu_MODE = PM_LU_DETACHED;

				pm->flux_DETACH = PM_DISABLED;
				pm->flux_TYPE = PM_FLUX_NONE;

				pm->proc_set_Z(PM_Z_ABC);
			}
		}
	}
	else if (pm->lu_MODE == PM_LU_ON_HFI) {

		pm_estimate(pm);

		lu_F[0] = pm->flux_F[0];
		lu_F[1] = pm->flux_F[1];

		pm->lu_wS = pm->flux_wS;

		if (		pm->flux_ZONE == PM_ZONE_HIGH
				|| pm->config_HFI_WAVETYPE == PM_HFI_NONE) {

			pm->lu_MODE = PM_LU_ESTIMATE;
		}
		else {
			if (pm->hold_TIM < PM_TSMS(pm, pm->tm_pause_startup)) {

				pm->hold_TIM++;
			}
			else if (pm->config_LU_SENSOR == PM_SENSOR_EABI) {

				pm->lu_MODE = PM_LU_SENSOR_EABI;
			}
		}
	}
	else if (pm->lu_MODE == PM_LU_SENSOR_HALL) {

		pm_estimate(pm);
		pm_sensor_hall(pm);

		lu_F[0] = pm->hall_F[0];
		lu_F[1] = pm->hall_F[1];

		pm->lu_wS = pm->hall_wS;

		if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE;
		}
	}
	else if (pm->lu_MODE == PM_LU_SENSOR_EABI) {

		pm_estimate(pm);
		pm_sensor_eabi(pm);

		lu_F[0] = pm->eabi_F[0];
		lu_F[1] = pm->eabi_F[1];

		lu_EABI = PM_ENABLED;

		pm->lu_wS = pm->eabi_wS;

		if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE;
		}
	}
	else if (pm->lu_MODE == PM_LU_SENSOR_SINCOS) {

		pm_estimate(pm);
		pm_sensor_sincos(pm);

		lu_F[0] = pm->sincos_F[0];
		lu_F[1] = pm->sincos_F[1];

		lu_SINCOS = PM_ENABLED;

		pm->lu_wS = pm->sincos_wS;

		if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE;
		}
	}
	else {
		lu_F[0] = pm->lu_F[0];
		lu_F[1] = pm->lu_F[1];
	}

	/* Take the LU position estimate with TRANSIENT rate limited.
	 * */
	hS = pm->lu_transient * pm->m_dT;

	A = lu_F[0] * pm->lu_F[0] + lu_F[1] * pm->lu_F[1];
	B = lu_F[1] * pm->lu_F[0] - lu_F[0] * pm->lu_F[1];

	if (A > M_EPSILON && B < - hS) {

		m_rotatef(pm->lu_F, - hS);
	}
	else if (A > M_EPSILON && B > hS) {

		m_rotatef(pm->lu_F, hS);
	}
	else {
		pm->lu_F[0] = lu_F[0];
		pm->lu_F[1] = lu_F[1];

		if (A < M_EPSILON) {

			/* NOTE: We reset current loop integrals in
			 * case of position flip.
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

	if (pm->lu_revol - pm->lu_revob < - pm->const_Zp) {

		pm->lu_total_revol += pm->lu_revob - pm->lu_revol;
		pm->lu_revob = pm->lu_revol;
	}
	else if (pm->lu_revol - pm->lu_revob > pm->const_Zp) {

		pm->lu_total_revol += pm->lu_revol - pm->lu_revob;
		pm->lu_revob = pm->lu_revol;
	}

	/* Take the LOCATION according to the configuration.
	 * */
	if (pm->config_LU_LOCATION == PM_LOCATION_INHERITED) {

		pm->lu_location = m_atan2f(pm->lu_F[1], pm->lu_F[0])
			+ (float) pm->lu_revol * M_2_PI_F;
	}
	else if (pm->config_LU_LOCATION == PM_LOCATION_EABI) {

		if (lu_EABI != PM_ENABLED) {

			pm_sensor_eabi(pm);

			lu_EABI = PM_ENABLED;
		}

		pm->lu_wS = pm->eabi_wS;
		pm->lu_location = pm->eabi_location;
	}
	else if (pm->config_LU_LOCATION == PM_LOCATION_SINCOS) {

		if (lu_SINCOS != PM_ENABLED) {

			pm_sensor_sincos(pm);

			lu_SINCOS = PM_ENABLED;
		}

		pm->lu_wS = pm->sincos_wS;
		pm->lu_location = pm->sincos_location;
	}

	if (		pm->eabi_RECENT == PM_ENABLED
			&& lu_EABI != PM_ENABLED) {

		pm->eabi_RECENT = PM_DISABLED;

		if (pm->config_EABI_FRONTEND == PM_EABI_INCREMENTAL) {

			/* We need to adjust the position again
			 * after loss of tracking.
			 * */
			pm->eabi_ADJUST = PM_DISABLED;
		}
	}

	if (		pm->flux_TYPE == PM_FLUX_KALMAN
			&& pm->lu_MODE == PM_LU_ESTIMATE) {

		/* Replace the current on DQ-axes with predicted one.
		 * */
		pm->lu_iD = pm->flux_X[0];
		pm->lu_iQ = pm->flux_X[1];
	}

	if (pm->lu_MODE == PM_LU_FORCED) {

		pm->lu_mq_load = 0.f;
	}
	else {
		float		mQ_load, wS_accel;

		/* Get an external mechanical LOAD torque estimate.
		 * */
		wS_accel = (pm->lu_wS - pm->lu_wS_prev) * pm->m_freq;
		mQ_load = pm->lu_mq_produce - wS_accel * pm->const_Ja;

		pm->lu_mq_load += (mQ_load - pm->lu_mq_load) * pm->lu_gain_mq_LP;
	}

	pm->lu_wS_prev = pm->lu_wS;
}

void pm_clearance(pmc_t *pm, int xA, int xB, int xC)
{
	int		xZONE, xSKIP, xTOP;

	xZONE = pm->dc_resolution - pm->ts_clearance;
	xSKIP = pm->dc_resolution - pm->ts_skip;

	xTOP = pm->dc_resolution;

	/* Check if there are PWM edges within clearance zone. The CURRENT
	 * measurements will be used or rejected based on this flags.
	 *
	 * NOTE: In case of inline current sensors placement we can sometimes
	 * clamp voltage to the TOP level to get more valid samples.
	 *
	 * NOTE: To get the best result you should have a current sensor with a
	 * fast transient that allows you to specify narrow clearance zone.
	 *
	 * */
	if (PM_CONFIG_IFB(pm) == PM_IFB_AB_INLINE) {

		pm->vsi_AF = (pm->vsi_A0 < xZONE || pm->vsi_A0 == xTOP) ? 0 : 1;
		pm->vsi_BF = (pm->vsi_B0 < xZONE || pm->vsi_B0 == xTOP) ? 0 : 1;
		pm->vsi_CF = 1;
	}
	else if (PM_CONFIG_IFB(pm) == PM_IFB_AB_GND) {

		pm->vsi_AF = (pm->vsi_A0 < xZONE) ? 0 : 1;
		pm->vsi_BF = (pm->vsi_B0 < xZONE) ? 0 : 1;
		pm->vsi_CF = 1;
	}
	else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_INLINE) {

		pm->vsi_AF = (pm->vsi_A0 < xZONE || pm->vsi_A0 == xTOP) ? 0 : 1;
		pm->vsi_BF = (pm->vsi_B0 < xZONE || pm->vsi_B0 == xTOP) ? 0 : 1;
		pm->vsi_CF = (pm->vsi_C0 < xZONE || pm->vsi_C0 == xTOP) ? 0 : 1;
	}
	else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_GND) {

		pm->vsi_AF = (pm->vsi_A0 < xZONE) ? 0 : 1;
		pm->vsi_BF = (pm->vsi_B0 < xZONE) ? 0 : 1;
		pm->vsi_CF = (pm->vsi_C0 < xZONE) ? 0 : 1;
	}

	/* Chech if at least TWO samples are clean so the current can be used
	 * in control loops.
	 * */
	pm->vsi_IF = likely(pm->vsi_AF + pm->vsi_BF + pm->vsi_CF < 2) ? 0 : 1;

	/* Check if there are PWM edges within clearance zone. The DC link
	 * voltage measurement will be used or rejected based on this flag.
	 * */
	pm->vsi_UF = (	   ((pm->vsi_A0 < xSKIP && xA < xSKIP)
				|| (pm->vsi_A0 == xTOP && xA == xTOP))
			&& ((pm->vsi_B0 < xSKIP && xB < xSKIP)
				|| (pm->vsi_B0 == xTOP && xB == xTOP))
			&& ((pm->vsi_C0 < xSKIP && xC < xSKIP)
				|| (pm->vsi_C0 == xTOP && xC == xTOP))) ? 0 : 1;

	pm->vsi_A0 = xA;
	pm->vsi_B0 = xB;
	pm->vsi_C0 = xC;
}

void pm_voltage(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC, uMIN, uMAX, uDC;
	int		xA, xB, xC, xMIN, xMAX, nZONE;

	uX *= pm->quick_iU;
	uY *= pm->quick_iU;

	uDC = m_hypotf(uX, uY);

	pm->vsi_DC = uDC / pm->k_EMAX;
	pm->vsi_lpf_DC += (pm->vsi_DC - pm->vsi_lpf_DC) * pm->vsi_gain_LP;

	if (		pm->config_VSI_CLAMP == PM_ENABLED
			&& uDC > pm->k_EMAX) {

		/* CLAMP voltage along an inscribed circle.
		 * */
		uDC = pm->k_EMAX / uDC;

		uX *= uDC;
		uY *= uDC;
	}

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		uA = uX;
		uB = - 0.5f * uX + 0.8660254f * uY;
		uC = - 0.5f * uX - 0.8660254f * uY;
	}
	else {
		uA = uX;
		uB = uY;
		uC = 0.f;
	}

	if (uA < uB) {

		uMIN = (uC < uA) ? uC : uA;
		uMAX = (uB > uC) ? uB : uC;
	}
	else {
		uMIN = (uC < uB) ? uC : uB;
		uMAX = (uA > uC) ? uA : uC;
	}

	uDC = uMAX - uMIN;

	if (uDC > 1.f) {

		/* CLAMP voltage along an hexagon sides.
		 * */
		uDC = 1.f / uDC;

		uA *= uDC;
		uB *= uDC;
		uC *= uDC;

		uMIN *= uDC;
		uMAX *= uDC;
	}

	if (pm->config_VSI_ZERO == PM_VSI_GND) {

		uDC = 0.f - uMIN;
	}
	else if (pm->config_VSI_ZERO == PM_VSI_CENTER) {

		uDC = 0.5f - (uMAX + uMIN) * 0.5f;
	}
	else if (pm->config_VSI_ZERO == PM_VSI_EXTREME) {

		float	bA, bB, bC, bMIN, bMAX;

		if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

			bA = m_fabsf(pm->lu_iX);
			bB = m_fabsf(- 0.5f * pm->lu_iX + 0.8660254f * pm->lu_iY);
			bC = m_fabsf(- 0.5f * pm->lu_iX - 0.8660254f * pm->lu_iY);
		}
		else {
			bA = m_fabsf(pm->lu_iX);
			bB = m_fabsf(pm->lu_iY);
			bC = m_fabsf(- pm->lu_iX - pm->lu_iY);
		}

		if (uA < uB) {

			bMIN = (uC < uA) ? bC : bA;
			bMAX = (uB > uC) ? bB : bC;
		}
		else {
			bMIN = (uC < uB) ? bC : bB;
			bMAX = (uA > uC) ? bA : bC;
		}

		bA = pm->fault_current_tol;

		bA = (		   pm->vsi_A0 < pm->dc_resolution
				&& pm->vsi_B0 < pm->dc_resolution
				&& pm->vsi_C0 < pm->dc_resolution) ? bA : 0.f;

		uDC = (bMIN + bA < bMAX) ? 1.f - uMAX : 0.f - uMIN;
	}
	else {
		uDC = 0.f;
	}

	uA += uDC;
	uB += uDC;
	uC += uDC;

	xA = (int) (pm->dc_resolution * uA);
	xB = (int) (pm->dc_resolution * uB);
	xC = (int) (pm->dc_resolution * uC);

	if (likely(pm->lu_MODE != PM_LU_DISABLED)) {

		if (PM_CONFIG_IFB(pm) == PM_IFB_AB_INLINE) {

			xMAX = pm->dc_resolution - pm->ts_minimal;
			xMIN = pm->dc_resolution - pm->ts_clearance;

			nZONE  =  ((xA < xMIN || xA > xMAX) ? 1 : 0)
				+ ((xB < xMIN || xB > xMAX) ? 1 : 0);

			if (nZONE < 2) {

				xMIN = (xA < xB) ? (xC < xA) ? xC : xA
					: (xC < xB) ? xC : xB;

				if (xMIN > 0) {

					/* Forced clamp to GND.
					 * */
					xA -= xMIN;
					xB -= xMIN;
					xC -= xMIN;
				}
			}

			xMAX = pm->dc_resolution - pm->ts_minimal;
			xMIN = pm->dc_resolution - pm->ts_clearance;

			nZONE  =  ((xA < xMIN || xA > xMAX) ? 1 : 0)
				+ ((xB < xMIN || xB > xMAX) ? 1 : 0);

			if (nZONE < 2) {

				xMAX = (xA > xB) ? (xC > xA) ? xC : xA
					: (xC > xB) ? xC : xB;

				if (xMAX < pm->dc_resolution) {

					/* Forced clamp to TOP.
					 * */
					xA += pm->dc_resolution - xMAX;
					xB += pm->dc_resolution - xMAX;
					xC += pm->dc_resolution - xMAX;
				}
			}

			xMAX = pm->dc_resolution - pm->ts_minimal;
			xMIN = pm->ts_minimal;

			xA = (xA < xMIN) ? 0 : (xA > xMAX) ? pm->dc_resolution : xA;
			xB = (xB < xMIN) ? 0 : (xB > xMAX) ? pm->dc_resolution : xB;
			xC = (xC < xMIN) ? 0 : (xC > xMAX) ? pm->dc_resolution : xC;
		}
		else if (PM_CONFIG_IFB(pm) == PM_IFB_AB_GND) {

			xMAX = pm->dc_resolution - pm->ts_clearance;

			if (xA >= xMAX || xB >= xMAX) {

				xMIN = (xA < xB) ? (xC < xA) ? xC : xA
					: (xC < xB) ? xC : xB;

				if (xMIN > 0) {

					/* Forced clamp to GND.
					 * */
					xA -= xMIN;
					xB -= xMIN;
					xC -= xMIN;
				}
			}

			xMAX = pm->dc_resolution - (pm->ts_clearance + 1);
			xMIN = pm->ts_minimal;

			xA = (xA < xMIN) ? 0 : (xA > xMAX) ? xMAX : xA;
			xB = (xB < xMIN) ? 0 : (xB > xMAX) ? xMAX : xB;
			xC = (xC < xMIN) ? 0 : (xC > xMAX) ? xMAX : xC;
		}
		else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_INLINE) {

			xMAX = pm->dc_resolution - pm->ts_minimal;
			xMIN = pm->dc_resolution - pm->ts_clearance;

			nZONE  =  ((xA < xMIN || xA > xMAX) ? 1 : 0)
				+ ((xB < xMIN || xB > xMAX) ? 1 : 0)
				+ ((xC < xMIN || xC > xMAX) ? 1 : 0);

			if (nZONE < 2) {

				xMIN = (xA < xB) ? (xC < xA) ? xC : xA
					: (xC < xB) ? xC : xB;

				if (xMIN > 0) {

					/* Forced clamp to GND.
					 * */
					xA -= xMIN;
					xB -= xMIN;
					xC -= xMIN;
				}
			}

			xMAX = pm->dc_resolution - pm->ts_minimal;
			xMIN = pm->dc_resolution - pm->ts_clearance;

			nZONE  =  ((xA < xMIN || xA > xMAX) ? 1 : 0)
				+ ((xB < xMIN || xB > xMAX) ? 1 : 0)
				+ ((xC < xMIN || xC > xMAX) ? 1 : 0);

			if (nZONE < 2) {

				xMAX = (xA > xB) ? (xC > xA) ? xC : xA
					: (xC > xB) ? xC : xB;

				if (xMAX < pm->dc_resolution) {

					/* Forced clamp to TOP.
					 * */
					xA += pm->dc_resolution - xMAX;
					xB += pm->dc_resolution - xMAX;
					xC += pm->dc_resolution - xMAX;
				}
			}

			xMAX = pm->dc_resolution - pm->ts_minimal;
			xMIN = pm->ts_minimal;

			xA = (xA < xMIN) ? 0 : (xA > xMAX) ? pm->dc_resolution : xA;
			xB = (xB < xMIN) ? 0 : (xB > xMAX) ? pm->dc_resolution : xB;
			xC = (xC < xMIN) ? 0 : (xC > xMAX) ? pm->dc_resolution : xC;
		}
		else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_GND) {

			xMAX = pm->dc_resolution - pm->ts_clearance;

			nZONE  =  ((xA < xMAX) ? 1 : 0)
				+ ((xB < xMAX) ? 1 : 0)
				+ ((xC < xMAX) ? 1 : 0);

			if (nZONE < 2) {

				xMIN = (xA < xB) ? (xC < xA) ? xC : xA
					: (xC < xB) ? xC : xB;

				if (xMIN > 0) {

					/* Forced clamp to GND.
					 * */
					xA -= xMIN;
					xB -= xMIN;
					xC -= xMIN;
				}
			}

			xMAX = pm->dc_resolution - pm->ts_minimal;
			xMIN = pm->ts_minimal;

			xA = (xA < xMIN) ? 0 : (xA > xMAX) ? pm->dc_resolution : xA;
			xB = (xB < xMIN) ? 0 : (xB > xMAX) ? pm->dc_resolution : xB;
			xC = (xC < xMIN) ? 0 : (xC > xMAX) ? pm->dc_resolution : xC;
		}
	}
	else {
		xMIN = (xA < xB) ? (xC < xA) ? xC : xA : (xC < xB) ? xC : xB;
		xMAX = (xA > xB) ? (xC > xA) ? xC : xA : (xC > xB) ? xC : xB;

		xMAX = (pm->dc_resolution - (pm->ts_clearance + 1)
				- (xMAX + xMIN) + pm->ts_minimal) / 2;

		/* Clamp to middle.
		 * */
		xA += xMAX;
		xB += xMAX;
		xC += xMAX;

		xMAX = pm->dc_resolution - (pm->ts_clearance + 1);
		xMIN = pm->ts_minimal;

		xA = (xA < xMIN) ? xMIN : (xA > xMAX) ? xMAX : xA;
		xB = (xB < xMIN) ? xMIN : (xB > xMAX) ? xMAX : xB;
		xC = (xC < xMIN) ? xMIN : (xC > xMAX) ? xMAX : xC;
	}

	if (pm->ts_bootstrap != 0) {

		pm->vsi_AT = (xA == pm->dc_resolution) ? pm->vsi_AT + 1 : 0;
		pm->vsi_BT = (xB == pm->dc_resolution) ? pm->vsi_BT + 1 : 0;
		pm->vsi_CT = (xC == pm->dc_resolution) ? pm->vsi_CT + 1 : 0;

		if (unlikely(		   pm->vsi_AT > pm->ts_bootstrap
					|| pm->vsi_BT > pm->ts_bootstrap
					|| pm->vsi_CT > pm->ts_bootstrap)) {

			pm->fsm_errno = PM_ERROR_BOOTSTRAP_FAULT;
			pm->fsm_state = PM_STATE_HALT;
		}
	}

	/* Output DC values to PWM.
	 * */
	pm->proc_set_DC(xA, xB, xC);

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		uDC = 0.33333333f * (xA + xB + xC);

		uA = (xA - uDC) * pm->const_fb_U * pm->ts_inverted;
		uB = (xB - uDC) * pm->const_fb_U * pm->ts_inverted;

		pm->vsi_X = uA;
		pm->vsi_Y = 0.57735027f * uA + 1.1547005f * uB;
	}
	else {
		uA = (xA - xC) * pm->const_fb_U * pm->ts_inverted;
		uB = (xB - xC) * pm->const_fb_U * pm->ts_inverted;

		pm->vsi_X = uA;
		pm->vsi_Y = uB;
	}

	/* Update the clearance flags according to the new DC values.
	 * */
	pm_clearance(pm, xA, xB, xC);
}

static float
pm_form_SP(pmc_t *pm, float eSP)
{
	float		iSP;

	/* Basic proportional-integral regulator.
	 * */
	iSP = pm->s_gain_P * eSP + pm->s_integral;

	/* Add an load torque estimate as feed forward term.
	 * */
	iSP += pm_lu_current(pm, pm->lu_mq_load, &pm->mtpa_load_Q);

	/* Add derivative term based on the estimated acceleration.
	 * */
	iSP += - pm->s_gain_D * pm_lu_accel(pm);

	if (		(iSP < pm->i_maximal || eSP < 0.f)
			&& (iSP > - pm->i_reverse || eSP > 0.f)) {

		pm->s_integral += pm->s_gain_I * eSP;
	}

	/* Clamp the output in accordance with CURRENT constraints.
	 * */
	iSP =     (iSP > pm->i_maximal) ? pm->i_maximal
		: (iSP < - pm->i_reverse) ? - pm->i_reverse : iSP;

	return iSP;
}

static void
pm_wattage(pmc_t *pm)
{
	float		wP, TiH, Wh, Ah;

	/* Actual operating WATTAGE is a scalar product of voltage and current.
	 * */
	wP = pm->k_KWAT * (pm->lu_iD * pm->lu_uD + pm->lu_iQ * pm->lu_uQ);

	pm->watt_drain_wP += (wP - pm->watt_drain_wP) * pm->watt_gain_WF;
	pm->watt_drain_wA = pm->watt_drain_wP * pm->quick_iU;

	/* Traveled distance.
	 * */
	pm->watt_traveled = (float) pm->lu_total_revol
		* pm->const_ld_Sm / (float) pm->const_Zp;

	if (likely(m_isfinitef(pm->watt_drain_wA) != 0)) {

		TiH = pm->m_dT * 0.00027777778f;

		/* Get WATT per HOUR.
		 * */
		Wh = pm->watt_drain_wP * TiH;
		Ah = pm->watt_drain_wA * TiH;

		if (likely(Wh > 0.f)) {

			m_rsumf(&pm->watt_consumed_Wh, &pm->watt_rem[0], Wh);
			m_rsumf(&pm->watt_consumed_Ah, &pm->watt_rem[1], Ah);
		}
		else {
			m_rsumf(&pm->watt_reverted_Wh, &pm->watt_rem[2], - Wh);
			m_rsumf(&pm->watt_reverted_Ah, &pm->watt_rem[3], - Ah);
		}
	}
	else {
		pm->fsm_errno = PM_ERROR_NAN_OPERATION;
		pm->fsm_state = PM_STATE_HALT;
	}

	/* Fuel gauge.
	 * */
	if (pm->watt_capacity_Ah > M_EPSILON) {

		Ah = pm->watt_consumed_Ah - pm->watt_reverted_Ah;

		pm->watt_fuel_gauge = 100.f * Ah / pm->watt_capacity_Ah;
	}
}

static void
pm_loop_current(pmc_t *pm)
{
	float		track_D, track_Q, eD, eQ, uD, uQ, uX, uY, wP;
	float		iMAX, iREV, uMAX, uREV, wMAX, wREV, dSA, dFA;

	if (pm->lu_MODE == PM_LU_FORCED) {

		if (pm->config_LU_FREEWHEEL != PM_ENABLED) {

			track_D = pm->forced_hold_D;
		}
		else if (	m_fabsf(pm->s_setpoint_speed) > M_EPSILON
				|| m_fabsf(pm->forced_wS) > M_EPSILON) {

			track_D = pm->forced_hold_D;
		}
		else {
			track_D = pm->forced_weak_D;
		}
	}
	else {
		track_D = 0.f;
	}

	if (		pm->forced_track_D > M_EPSILON
			|| track_D > M_EPSILON) {

		dSA = pm->forced_slew_rate * pm->m_dT;
		dFA = pm->forced_fall_rate * pm->m_dT;

		/* Forced current slew rate limited tracking.
		 * */
		pm->forced_track_D = (pm->forced_track_D < track_D - dSA)
			? pm->forced_track_D + dSA : (pm->forced_track_D > track_D + dFA)
			? pm->forced_track_D - dFA : track_D;
	}

	track_D = pm->forced_track_D;
	track_Q = 0.f;

	if (		pm->lu_MODE == PM_LU_FORCED
			|| (	pm->lu_MODE == PM_LU_ESTIMATE
				&& pm->flux_ZONE != PM_ZONE_HIGH)) {

		if (		pm->config_CC_SPEED_TRACK == PM_ENABLED
				&& (	   pm->config_LU_DRIVE == PM_DRIVE_CURRENT
					|| pm->config_LU_DRIVE == PM_DRIVE_TORQUE)) {

			pm->l_track = pm->lu_wS;
		}
	}
	else {
		track_Q = pm->i_setpoint_current;

		if (		   pm->config_LU_DRIVE == PM_DRIVE_CURRENT
				|| pm->config_LU_DRIVE == PM_DRIVE_TORQUE) {

			if (pm->config_LU_DRIVE == PM_DRIVE_TORQUE) {

				/* Torque control.
				 * */
				track_Q += pm_lu_current(pm,
						pm->i_setpoint_torque,
						&pm->mtpa_setpoint_Q);
			}

			if (pm->config_CC_BRAKE_STOP == PM_ENABLED) {

				if (track_Q < - M_EPSILON) {

					iMAX = m_fabsf(track_Q);

					/* Replace current setpoint by speed regulation.
					 * */
					track_Q = pm_form_SP(pm, 0.f - pm->lu_wS);
					track_Q = (track_Q > iMAX) ? iMAX
						: (track_Q < - iMAX) ? - iMAX : track_Q;
				}
			}

			if (pm->config_CC_SPEED_TRACK == PM_ENABLED) {

				float		wSP, eSP, blend;

				wSP = pm->lu_wS;
				wSP = (wSP > pm->s_maximal) ? pm->s_maximal
					: (wSP < - pm->s_reverse) ? - pm->s_reverse : wSP;

				dSA = pm->s_accel_forward * pm->m_dT;
				dFA = pm->s_accel_reverse * pm->m_dT;

				pm->l_track = (pm->l_track < wSP - dSA) ? pm->l_track + dSA
					: (pm->l_track > wSP + dFA) ? pm->l_track - dFA : wSP;

				/* Obtain the speed tracking discrepancy.
				 * */
				eSP = pm->l_track - pm->lu_wS;

				blend = m_fabsf(eSP) * m_fast_recipf(pm->l_track_tol);
				blend = (blend > 1.f) ? 1.f : blend;

				/* Blend current setpoint with speed regulation.
				 * */
				pm->l_blend += (blend - pm->l_blend) * pm->l_gain_LP;
				track_Q += (pm_form_SP(pm, eSP) - track_Q) * pm->l_blend;
			}
		}

		if (pm->config_RELUCTANCE == PM_ENABLED) {

			float		iD;

			iD = pm_torque_MTPA(pm, pm->lu_iQ);
			pm->mtpa_D += (iD - pm->mtpa_D) * pm->mtpa_gain_LP;

			/* Maximum Torque Per Ampere (MTPA) control.
			 * */
			track_D += pm->mtpa_D;
		}

		if (pm->config_WEAKENING == PM_ENABLED) {

			float		eDC, wLS;

			uMAX = pm->watt_uDC_maximal + pm->watt_uDC_tol;
			eDC = (1.f - pm->vsi_DC) * pm->const_fb_U;

			if (		pm->const_fb_U > uMAX
					&& pm->weak_D < - M_EPSILON) {

				eDC = (eDC < 0.f) ? eDC : 0.f;
			}

			pm->weak_D += eDC * pm->weak_gain_EU;
			pm->weak_D = (pm->weak_D < - pm->weak_maximal) ? - pm->weak_maximal
				: (pm->weak_D > 0.f) ? 0.f : pm->weak_D;

			if (pm->weak_D < - M_EPSILON) {

				eDC = pm->k_EMAX * pm->const_fb_U;
				wLS = pm->lu_wS * pm->const_im_Lq;

				iMAX = eDC * m_fast_recipf(m_fabsf(wLS));

				track_Q = (track_Q > iMAX) ? iMAX
					: (track_Q < - iMAX) ? - iMAX : track_Q;

				/* Flux weakening control.
				 * */
				track_D += pm->weak_D;
			}
		}
	}

	/* Update LPF voltages to use them in WATTAGE constraints.
	 * */
	pm->watt_lpf_D += (pm->lu_uD - pm->watt_lpf_D) * pm->watt_gain_LP;
	pm->watt_lpf_Q += (pm->lu_uQ - pm->watt_lpf_Q) * pm->watt_gain_LP;

	/* Maximal CURRENT constraints.
	 * */
	iMAX = pm->i_maximal;
	iREV = - pm->i_reverse;

	track_D = (track_D > iMAX) ? iMAX : (track_D < - iMAX) ? - iMAX : track_D;
	track_Q = (track_Q > iMAX) ? iMAX : (track_Q < iREV) ? iREV : track_Q;

	if (pm->lu_MODE == PM_LU_ON_HFI) {

		iMAX = pm->i_maximal_on_HFI;

		/* Add current constraint from HFI.
		 * */
		track_D = (track_D > iMAX) ? iMAX : (track_D < - iMAX) ? - iMAX : track_D;
		track_Q = (track_Q > iMAX) ? iMAX : (track_Q < - iMAX) ? - iMAX : track_Q;
	}

	/* Add current constraint from PCB.
	 * */
	iMAX = pm->i_maximal_on_PCB;

	track_Q = (track_Q > iMAX) ? iMAX : (track_Q < - iMAX) ? - iMAX : track_Q;

	if (pm->weak_D > - M_EPSILON) {

		/* In case of no flux weakening also constraint D-axis current.
		 * */
		track_D = (track_D > iMAX) ? iMAX : (track_D < - iMAX) ? - iMAX : track_D;
	}

	/* Maximal WATTAGE constraints.
	 * */
	wMAX = pm->watt_wP_maximal;
	wREV = - pm->watt_wP_reverse;

	/* Maximal DC link current constraint.
	 * */
	wP = pm->watt_wA_maximal * pm->const_fb_U;
	wMAX = (wP < wMAX) ? wP : wMAX;
	wP = - pm->watt_wA_reverse * pm->const_fb_U;
	wREV = (wP > wREV) ? wP : wREV;

	uMAX = (pm->watt_DC_MAX != PM_ENABLED)
		? pm->watt_uDC_maximal
		: pm->watt_uDC_maximal - pm->watt_uDC_tol;

	/* Prevent DC link OVERVOLTAGE.
	 * */
	if (unlikely(pm->const_fb_U > uMAX)) {

		float		eDC, bSP;

		eDC = pm->const_fb_U - pm->watt_uDC_maximal;

		if (pm->watt_DC_MAX != PM_ENABLED) {

			pm->watt_DC_MAX = PM_ENABLED;
			pm->watt_integral = pm->watt_drain_wP;
		}

		bSP = pm->watt_gain_P * eDC + pm->watt_integral;
		bSP = (bSP < 0.f) ? bSP : 0.f;

		if (		(bSP < 0.f || eDC < 0.f)
				&& (bSP > wREV || eDC > 0.f)) {

			pm->watt_integral += pm->watt_gain_I * eDC;
		}

		wREV = bSP;
	}
	else if (pm->watt_DC_MAX == PM_ENABLED) {

		pm->watt_DC_MAX = PM_DISABLED;
	}

	uREV = (pm->watt_DC_MIN != PM_ENABLED)
		? pm->watt_uDC_minimal
		: pm->watt_uDC_minimal + pm->watt_uDC_tol;

	/* Prevent DC link UNDERVOLTAGE.
	 * */
	if (unlikely(pm->const_fb_U < uREV)) {

		float		eDC, bSP;

		eDC = pm->const_fb_U - pm->watt_uDC_minimal;

		if (pm->watt_DC_MIN != PM_ENABLED) {

			pm->watt_DC_MIN = PM_ENABLED;
			pm->watt_integral = pm->watt_drain_wP;
		}

		bSP = pm->watt_gain_P * eDC + pm->watt_integral;
		bSP = (bSP > 0.f) ? bSP : 0.f;

		if (		(bSP < wMAX || eDC < 0.f)
				&& (bSP > 0.f || eDC > 0.f)) {

			pm->watt_integral += pm->watt_gain_I * eDC;
		}

		wMAX = bSP;
	}
	else if (pm->watt_DC_MIN == PM_ENABLED) {

		pm->watt_DC_MIN = PM_DISABLED;
	}

	wP = pm->k_KWAT * (track_D * pm->watt_lpf_D + track_Q * pm->watt_lpf_Q);

	/* Apply WATTAGE regeneration constraint.
	 * */
	if (unlikely(wP < wREV)) {

		if (pm->weak_D > - M_EPSILON) {

			wREV /= wP;

			track_D *= wREV;
			track_Q *= wREV;
		}
		else {
			wP = pm->k_KWAT * track_Q * pm->watt_lpf_Q;

			if (wP < wREV) {

				track_Q *= wREV / wP;
			}
		}
	}
	else if (pm->watt_DC_MAX == PM_ENABLED) {

		pm->watt_DC_MAX = PM_DISABLED;
	}

	/* Apply WATTAGE consumption constraint.
	 * */
	if (unlikely(wP > wMAX)) {

		if (pm->weak_D > - M_EPSILON) {

			wMAX /= wP;

			track_D *= wMAX;
			track_Q *= wMAX;
		}
		else {
			wP = pm->k_KWAT * track_Q * pm->watt_lpf_Q;

			if (wP > wMAX) {

				track_Q *= wMAX / wP;
			}
		}
	}
	else if (pm->watt_DC_MIN == PM_ENABLED) {

		pm->watt_DC_MIN = PM_DISABLED;
	}

	dSA = pm->i_slew_rate * pm->m_dT;

	/* Slew rate limited current tracking.
	 * */
	pm->i_track_D = (pm->i_track_D < track_D - dSA) ? pm->i_track_D + dSA
		: (pm->i_track_D > track_D + dSA) ? pm->i_track_D - dSA : track_D;
	pm->i_track_Q = (pm->i_track_Q < track_Q - dSA) ? pm->i_track_Q + dSA
		: (pm->i_track_Q > track_Q + dSA) ? pm->i_track_Q - dSA : track_Q;

	/* Obtain the discrepancy in DQ-axes.
	 * */
	eD = pm->i_track_D - pm->lu_iD;
	eQ = pm->i_track_Q - pm->lu_iQ;

	/* Basic proportional-integral regulator.
	 * */
	uD = pm->i_gain_P * eD + pm->i_integral_D;
	uQ = pm->i_gain_P * eQ + pm->i_integral_Q;

	/* Feed forward compensation (R).
	 * */
	uD += pm->const_Rs * pm->i_track_D;
	uQ += pm->const_Rs * pm->i_track_Q;

	/* Feed forward compensation (L).
	 * */
	uD += - pm->lu_wS * pm->const_im_Lq * pm->i_track_Q;
	uQ += pm->lu_wS * (pm->const_im_Ld * pm->i_track_D + pm->const_lambda);

	uMAX = pm->k_UMAX * pm->const_fb_U;

	if (		(uD < uMAX || eD < 0.f)
			&& (uD > - uMAX || eD > 0.f)) {

		pm->i_integral_D += pm->i_gain_I * eD;
	}

	if (		(uQ < uMAX || eQ < 0.f)
			&& (uQ > - uMAX || eQ > 0.f)) {

		pm->i_integral_Q += pm->i_gain_I * eQ;
	}

	/* Output voltage CLAMP.
	 * */
	uD = (uD > uMAX) ? uMAX : (uD < - uMAX) ? - uMAX : uD;
	uQ = (uQ > uMAX) ? uMAX : (uQ < - uMAX) ? - uMAX : uQ;

	uMAX = pm->k_EMAX * pm->v_maximal;
	uREV = - pm->k_EMAX * pm->v_reverse;

	/* Output voltage (Q) specified constraint.
	 * */
	uQ = (uQ > uMAX) ? uMAX : (uQ < uREV) ? uREV : uQ;

	if (		pm->config_HFI_PERMANENT == PM_ENABLED
			|| pm->lu_MODE == PM_LU_ON_HFI) {

		/* HF voltage injection.
		 * */
		uD += pm_hfi_wave(pm);
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
	float		wSP, eSP, dSA, dFA;

	wSP = pm->s_setpoint_speed;

	/* Maximal SPEED constraint.
	 * */
	wSP = (wSP > pm->s_maximal) ? pm->s_maximal :
		(wSP < - pm->s_reverse) ? - pm->s_reverse : wSP;

	if (pm->lu_MODE == PM_LU_FORCED) {

		pm->s_track = pm->forced_wS;
	}
	else {
		if (pm->config_LU_DRIVE == PM_DRIVE_SPEED) {

			dSA = pm->s_accel_forward * pm->m_dT;
			dFA = pm->s_accel_reverse * pm->m_dT;

			/* Apply acceleration constraints.
			 * */
			pm->s_track = (pm->s_track < wSP - dSA) ? pm->s_track + dSA
				: (pm->s_track > wSP + dFA) ? pm->s_track - dFA : wSP;
		}
		else {
			pm->s_track = wSP;
		}

		if (		pm->config_LU_FREEWHEEL == PM_ENABLED
				&& m_fabsf(pm->s_track) < M_EPSILON) {

			pm->i_setpoint_current = 0.f;
		}
		else {
			/* Obtain the speed discrepancy.
			 * */
			eSP = pm->s_track - pm->lu_wS;

			/* Update current loop SETPOINT.
			 * */
			pm->i_setpoint_current = pm_form_SP(pm, eSP);
		}
	}
}

static void
pm_loop_location(pmc_t *pm)
{
	float		xSP, wSP, eSP, eDS, weak, gain;

	xSP = pm->x_setpoint_location;
	wSP = pm->x_setpoint_speed;

	/* Move location setpoint in accordance with speed setpoint.
	 * */
	xSP += wSP * pm->m_dT;

	/* Allowed location range constraints.
	 * */
	if (xSP > pm->x_maximal) {

		xSP = pm->x_maximal;
		wSP = 0.f;
	}
	else if (xSP < pm->x_minimal) {

		xSP = pm->x_minimal;
		wSP = 0.f;
	}

	pm->x_setpoint_location = xSP;

	/* Obtain the location discrepancy.
	 * */
	eSP = pm->x_setpoint_location - pm->lu_location;

	/* Absolute distance to the location setpoint.
	 * */
	eDS = m_fabsf(eSP);

	/* There is a residual tolerance.
	 * */
	eSP = (eDS > pm->x_track_tol) ? eSP : 0.f;

	/* Damping inside BOOST zone.
	 * */
	weak = (eDS < pm->x_boost_tol) ? eDS * m_fast_recipf(pm->x_boost_tol) : 1.f;
	gain = pm->x_gain_P * weak + pm->x_gain_D * (1.f - weak);

	/* Based on constant acceleration formula.
	 * */
	wSP += gain * eSP * m_rough_rsqrtf(eDS);

	/* Update speed loop SETPOINT.
	 * */
	pm->s_setpoint_speed = wSP;
}

static void
pm_dcu_voltage(pmc_t *pm)
{
	float		iA, iB, iC, uA, uB, uC, DTu;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		iA = pm->lu_iX;
		iB = - 0.5f * pm->lu_iX + 0.8660254f * pm->lu_iY;
		iC = - 0.5f * pm->lu_iX - 0.8660254f * pm->lu_iY;
	}
	else {
		iA = pm->lu_iX;
		iB = pm->lu_iY;
		iC = - pm->lu_iX - pm->lu_iY;
	}

	DTu = PM_DTNS(pm, pm->dcu_deadband) * pm->const_fb_U;

	if (likely(		pm->vsi_A0 != pm->dc_resolution
				&& pm->vsi_A0 != 0)) {

		if (likely(pm->lu_MODE != PM_LU_DISABLED)) {

			uA = iA * m_fast_recipf(m_fabsf(iA) + pm->dcu_tol) * DTu;
		}
		else {
			uA = (iA < 0.f) ? - DTu : DTu;
		}
	}
	else {
		uA = 0.f;
	}

	if (likely(		pm->vsi_B0 != pm->dc_resolution
				&& pm->vsi_B0 != 0)) {

		if (likely(pm->lu_MODE != PM_LU_DISABLED)) {

			uB = iB * m_fast_recipf(m_fabsf(iB) + pm->dcu_tol) * DTu;
		}
		else {
			uB = (iB < 0.f) ? - DTu : DTu;
		}
	}
	else {
		uB = 0.f;
	}

	if (likely(		pm->vsi_C0 != pm->dc_resolution
				&& pm->vsi_C0 != 0)) {

		if (likely(pm->lu_MODE != PM_LU_DISABLED)) {

			uC = iC * m_fast_recipf(m_fabsf(iC) + pm->dcu_tol) * DTu;
		}
		else {
			uC = (iC < 0.f) ? - DTu : DTu;
		}
	}
	else {
		uC = 0.f;
	}

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		uC = 0.33333333f * (uA + uB + uC);

		uA = uA - uC;
		uB = uB - uC;

		pm->dcu_DX = uA;
		pm->dcu_DY = 0.57735027f * uA + 1.1547005f * uB;
	}
	else {
		uA = uA - uC;
		uB = uB - uC;

		pm->dcu_DX = uA;
		pm->dcu_DY = uB;
	}
}

void pm_feedback(pmc_t *pm, pmfb_t *fb)
{
	float		iA, iB, Q;

	if (likely(pm->vsi_AF == 0)) {

		/* Get inline current A.
		 * */
		pm->fb_iA = pm->scale_iA[1] * fb->current_A + pm->scale_iA[0];

		if (unlikely(m_fabsf(pm->fb_iA) > pm->fault_current_halt)) {

			pm->fsm_errno = PM_ERROR_INSTANT_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (likely(pm->vsi_BF == 0)) {

		/* Get inline current B.
		 * */
		pm->fb_iB = pm->scale_iB[1] * fb->current_B + pm->scale_iB[0];

		if (unlikely(m_fabsf(pm->fb_iB) > pm->fault_current_halt)) {

			pm->fsm_errno = PM_ERROR_INSTANT_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (likely(pm->vsi_CF == 0)) {

		/* Get inline current C.
		 * */
		pm->fb_iC = pm->scale_iC[1] * fb->current_C + pm->scale_iC[0];

		if (unlikely(m_fabsf(pm->fb_iC) > pm->fault_current_halt)) {

			pm->fsm_errno = PM_ERROR_INSTANT_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		if (		   pm->vsi_AF == 0
				&& pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			Q = 0.33333333f * (pm->fb_iA + pm->fb_iB + pm->fb_iC);

			iA = pm->fb_iA - Q;
			iB = pm->fb_iB - Q;

			pm->lu_iX = iA;
			pm->lu_iY = 0.57735027f * iA + 1.1547005f * iB;
		}
		else if (	   pm->vsi_AF == 0
				&& pm->vsi_BF == 0) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = 0.57735027f * pm->fb_iA + 1.1547005f * pm->fb_iB;
		}
		else if (	   pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			pm->lu_iX = - pm->fb_iB - pm->fb_iC;
			pm->lu_iY = 0.57735027f * pm->fb_iB - 0.57735027f * pm->fb_iC;
		}
		else if (	   pm->vsi_AF == 0
				&& pm->vsi_CF == 0) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = - 0.57735027f * pm->fb_iA - 1.1547005f * pm->fb_iC;
		}
	}
	else {
		if (		   pm->vsi_AF == 0
				&& pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			Q = 0.33333333f * (pm->fb_iA + pm->fb_iB + pm->fb_iC);

			iA = pm->fb_iA - Q;
			iB = pm->fb_iB - Q;

			pm->lu_iX = iA;
			pm->lu_iY = iB;
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

	if (likely(pm->vsi_UF == 0)) {

		/* Get DC link voltage.
		 * */
		pm->const_fb_U = pm->scale_uS[1] * fb->voltage_U + pm->scale_uS[0];
		pm->quick_iU = 1.f / pm->const_fb_U;

		if (unlikely(		pm->const_fb_U > pm->fault_voltage_halt
					&& pm->weak_D > - M_EPSILON)) {

			pm->fsm_errno = PM_ERROR_DC_LINK_OVERVOLTAGE;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		/* Get instant terminal voltages.
		 * */
		pm->fb_uA = pm->scale_uA[1] * fb->voltage_A + pm->scale_uA[0];
		pm->fb_uB = pm->scale_uB[1] * fb->voltage_B + pm->scale_uB[0];
		pm->fb_uC = pm->scale_uC[1] * fb->voltage_C + pm->scale_uC[0];
	}

	pm->fb_SIN = fb->analog_SIN;
	pm->fb_COS = fb->analog_COS;
	pm->fb_HS = fb->pulse_HS;
	pm->fb_EP = fb->pulse_EP;

	if (		pm->config_DCU_VOLTAGE == PM_ENABLED
			&& pm->lu_MODE != PM_LU_DETACHED) {

		pm_dcu_voltage(pm);
	}
	else {
		pm->dcu_DX = 0.f;
		pm->dcu_DY = 0.f;
	}

	pm->dcu_X = pm->vsi_X - pm->dcu_DX;
	pm->dcu_Y = pm->vsi_Y - pm->dcu_DY;

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
			else if (pm->config_LU_DRIVE == PM_DRIVE_LOCATION) {

				pm_loop_location(pm);
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
				pm_kalman_forecast(pm);

				if (likely(pm->vsi_IF == 0)) {

					pm_kalman_update(pm);
				}

				pm->kalman_POSTPONED = PM_DISABLED;
			}

			/* Wattage information.
			 * */
			pm_wattage(pm);
		}

		if (PM_CONFIG_DBG(pm) == PM_ENABLED) {

			float		A, B;

			A = pm->lu_F[0] * pm->flux_F[0] + pm->lu_F[1] * pm->flux_F[1];
			B = pm->lu_F[1] * pm->flux_F[0] - pm->lu_F[0] * pm->flux_F[1];

			pm->dbg_flux_rsu = m_atan2f(B, A) * (180.f / M_PI_F);
		}
	}

	/* The FSM is used to execute assistive routines.
	 * */
	pm_FSM(pm);
}

