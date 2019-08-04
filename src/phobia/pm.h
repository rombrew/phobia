#ifndef _H_PM_
#define _H_PM_

#define PM_CONFIG_NOP(pm)		(pm)->config_NOP
#define PM_CONFIG_TVM(pm)		(pm)->config_TVM

#define PM_UMAX(pm)			((PM_CONFIG_NOP(pm) == 0) ? 2.f / 3.f : 1.f)
#define PM_EMAX(pm)			((PM_CONFIG_NOP(pm) == 0) ? .57735027f : .70710678f)
#define PM_KWAT(pm)			((PM_CONFIG_NOP(pm) == 0) ? 1.5f : 1.f)

#define PM_FLUX_MAX			25
#define PM_INFINITY			1E+33f
#define PM_SFI(s)			#s

enum {
	PM_NOP_THREE_PHASE			= 0,
	PM_NOP_TWO_PHASE,
};

enum {
	PM_SENSOR_DISABLED			= 0,
	PM_SENSOR_HALL_ABC,
	PM_SENSOR_QEP_AB,
};

enum {
	PM_DISABLED				= 0,
	PM_ENABLED
};

enum {
	PM_LOOP_DRIVE_CURRENT			= 0,
	PM_LOOP_DRIVE_SPEED,
	PM_LOOP_DRIVE_SERVO,
	PM_LOOP_RECTIFIER_VOLTAGE,
};

enum {
	PM_LU_DISABLED				= 0,
	PM_LU_DETACHED,
	PM_LU_FORCED,
	PM_LU_ESTIMATE_FLUX,
	PM_LU_SENSORED_HALL_ABC,
	PM_LU_ESTIMATE_HFI,
};

enum {
	PM_STATE_IDLE				= 0,
	PM_STATE_ZERO_DRIFT,
	PM_STATE_SELF_TEST_POWER_STAGE,
	PM_STATE_SELF_TEST_CLEARANCE,
	PM_STATE_STD_VOLTAGE,
	PM_STATE_STD_CURRENT,
	PM_STATE_ADJUST_VOLTAGE,
	PM_STATE_ADJUST_CURRENT,
	PM_STATE_PROBE_CONST_R,
	PM_STATE_PROBE_CONST_L,
	PM_STATE_LU_STARTUP,
	PM_STATE_LU_SHUTDOWN,
	PM_STATE_PROBE_CONST_E,
	PM_STATE_PROBE_CONST_J,
	PM_STATE_ADJUST_HS,
	PM_STATE_HALT,
};

enum {
	PM_OK					= 0,
	PM_ERROR_ZERO_DRIFT_FAULT,
	PM_ERROR_NO_MOTOR_CONNECTED,
	PM_ERORR_POWER_STAGE_FAULT,
	PM_ERROR_ACCURACY_FAULT,
	PM_ERROR_CURRENT_LOOP_FAULT,
	PM_ERROR_INLINE_OVER_CURRENT,
	PM_ERROR_DC_LINK_OVER_VOLTAGE,
	PM_ERROR_FLUX_UNSTABLE,
	PM_ERROR_INVALID_OPERATION,
};

typedef struct {

	float		current_A;
	float		current_B;
	float		voltage_U;

	float		voltage_A;
	float		voltage_B;
	float		voltage_C;

	int		pulse_HS;
	int		pulse_QEP;
}
pmfb_t;

typedef struct {

	float		freq_hz;
	float		dT;

	int		dc_resolution;
	int		dc_minimal;
	int		dc_clearance;

	int		fail_reason;
	int		self_BM[8];
	float		self_RMS[2];

	int		config_NOP;
	int		config_TVM;
	int		config_SENSOR;
	int		config_HFI;
	int		config_LOOP;
	int		config_WEAK;
	int		config_BRAKE;
	int		config_STAT;

	int		fsm_req;
	int		fsm_state;
	int		fsm_phase;
	int		fsm_phase_2;

	int		tm_value;
	int		tm_end;

	float		tm_transient_slow;
	float		tm_transient_fast;
	float		tm_voltage_hold;
	float		tm_current_hold;
	float		tm_instant_probe;
	float		tm_average_drift;
	float		tm_average_probe;
	float		tm_startup;

	float		ad_IA[2];
	float		ad_IB[2];
	float		ad_US[2];
	float		ad_UA[2];
	float		ad_UB[2];
	float		ad_UC[2];

	float		fb_iA;
	float		fb_iB;
	float		fb_uA;
	float		fb_uB;
	float		fb_uC;
	int		fb_HS;

	float		probe_current_hold;
	float		probe_current_bias_Q;
	float		probe_current_sine;
	float		probe_freq_sine_hz;
	float		probe_speed_low;
	float		probe_speed_ramp;
	float		probe_gain_P;
	float		probe_gain_I;
	float		probe_DFT[8];
	float		probe_LSQ_A[9];
	float		probe_LSQ_B[9];
	float		probe_LSQ_C[9];

	float		FIX[27];

	float		fault_voltage_tol;
	float		fault_current_tol;
	float		fault_accuracy_tol;
	float		fault_current_halt;
	float		fault_voltage_halt;
	float		fault_flux_lpfe_halt;

	float		vsi_X;
	float		vsi_Y;
	float		vsi_DX;
	float		vsi_DY;
	int		vsi_IF;
	int		vsi_UF;
	int		vsi_AZ;
	int		vsi_BZ;
	int		vsi_CZ;

	float		tvm_range;
	float		tvm_A;
	float		tvm_B;
	float		tvm_C;
	float		tvm_FIR_A[3];
	float		tvm_FIR_B[3];
	float		tvm_FIR_C[3];
	float		tvm_DX;
	float		tvm_DY;

	float		lu_iX;
	float		lu_iY;
	float		lu_iD;
	float		lu_iQ;
	float		lu_F[2];
	float		lu_wS;
	float		lu_lock_S;
	float		lu_unlock_S;
	float		lu_lpf_wS;
	float		lu_gain_LP_S;
	int		lu_mode;

	float		forced_F[2];
	float		forced_wS;
	float		forced_hold_D;
	float		forced_maximal;
	float		forced_accel;

	struct {

		float	X;
		float	Y;
		float	lpf_E;
	}
	flux[PM_FLUX_MAX];

	int		flux_N;
	float		flux_lower_R;
	float		flux_upper_R;
	float		flux_transient_S;
	float		flux_E;
	int		flux_H;
	float		flux_F[2];
	float		flux_wS;
	float		flux_gain_IN;
	float		flux_gain_LO;
	float		flux_gain_HI;
	float		flux_gain_LP_E;
	float		flux_gain_SF;

	float		inject_bias_U;
	float		inject_ratio_D;

	/*
	float		hall_X[5];
	float		hall_range;
	*/

	float		hfi_freq_hz;
	float		hfi_swing_D;
	float		hfi_derated;
	float		hfi_iD;
	float		hfi_iQ;
	float		hfi_F[2];
	float		hfi_wS;
	float		hfi_wave[2];
	float		hfi_polarity;
	float		hfi_gain_EP;
	float		hfi_gain_SF;
	float		hfi_gain_FP;

	float		const_lpf_U;
	float		const_gain_LP_U;
	float		const_E;
	float		const_R;
	float		const_L;
	int		const_Zp;
	float		const_J;
	float		const_im_LD;
	float		const_im_LQ;
	float		const_im_B;
	float		const_im_R;
	float		const_dd_T;

	float		watt_wp_maximal;
	float		watt_ib_maximal;
	float		watt_derated_1;
	float		watt_wp_reverse;
	float		watt_ib_reverse;
	float		watt_derate_HI_U;
	float		watt_derate_LO_U;
	float		watt_derate_HI_S;
	float		watt_integral[3];
	float		watt_lpf_D;
	float		watt_lpf_Q;
	float		watt_lpf_wP;
	float		watt_gain_LP_F;
	float		watt_gain_LP_P;
	float		watt_gain_DU;
	float		watt_gain_DS;
	float		watt_gain_LP_I;

	float		i_maximal;
	float		i_derated_1;
	float		i_brake;
	float		i_setpoint_D;
	float		i_setpoint_Q;
	float		i_integral_D;
	float		i_integral_Q;
	float		i_gain_P;
	float		i_gain_I;

	float		weak_maximal_D;
	float		weak_bias_U;

	float		s_maximal;
	float		s_setpoint;
	int		s_brake_DIR;
	float		s_accel;
	float		s_interval;
	float		s_track;
	float		s_integral;
	float		s_gain_P;
	float		s_gain_LP_I;
	float		s_gain_HF_S;

	float		x_setpoint_F[2];
	int		x_setpoint_revol;
	float		x_lu_F1;
	int		x_lu_revol;
	float		x_near_EP;
	float		x_gain_P;
	float		x_gain_N;

	float		stat_lu_F1;
	int		stat_revol_qu;
	int		stat_revol_total;
	float		stat_distance;
	float		stat_consumed_wh;
	float		stat_consumed_ah;
	float		stat_reverted_wh;
	float		stat_reverted_ah;
	float		stat_FIX[4];

	/*int		bt_mode;
	float		bt_*/

	void 		(* proc_set_DC) (int, int, int);
	void 		(* proc_set_Z) (int);
}
pmc_t;

void pm_default(pmc_t *pm);

void pm_voltage_control(pmc_t *pm, float uX, float uY);
void pm_feedback(pmc_t *pm, pmfb_t *fb);

void pm_ADD(float *S, float *C, float X);
void pm_FSM(pmc_t *pm);

const char *pm_strerror(int n);

#endif /* _H_PM_ */

