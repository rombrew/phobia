#ifndef _H_PM_
#define _H_PM_

#define PM_CONFIG_NOP(pm)		(pm)->config_NOP
#define PM_CONFIG_TVM(pm)		(pm)->config_TVM
#define PM_CONFIG_VSI(pm)		PM_VSI_AB_INLINE

#define PM_UMAX(pm)			((PM_CONFIG_NOP(pm) == 0) ? 2.f / 3.f : 1.f)
#define PM_EMAX(pm)			((PM_CONFIG_NOP(pm) == 0) ? .57735027f : .70710678f)
#define PM_KWAT(pm)			((PM_CONFIG_NOP(pm) == 0) ? 1.5f : 1.f)

#define PM_FLUX_MAX			25
#define PM_HALL_SPAN			1.05f
#define PM_MAX_F			7E+27f
#define PM_MAX_I			16777216l
#define PM_SFI(s)			#s

enum {
	PM_NOP_THREE_PHASE			= 0,
	PM_NOP_TWO_PHASE,
};

enum {
	PM_VSI_AB_INLINE			= 0,
	PM_VSI_AB_LOW,
	PM_VSI_FULL_LOW
};

enum {
	PM_SENSOR_DISABLED			= 0,
	PM_SENSOR_HALL,
	PM_SENSOR_QENC,
};

enum {
	PM_DISABLED				= 0,
	PM_ENABLED
};

enum {
	PM_DRIVE_CURRENT			= 0,
	PM_DRIVE_SPEED,
};

enum {
	PM_LU_UNCERTAIN				= 0,
	PM_LU_CAUGHT,
	PM_LU_LOCKED
};

enum {
	PM_LU_DISABLED				= 0,
	PM_LU_DETACHED,
	PM_LU_FORCED,
	PM_LU_ESTIMATE_FLUX,
	PM_LU_ESTIMATE_HFI,
	PM_LU_SENSOR_HALL,
	PM_LU_SENSOR_QENC,
};

enum {
	PM_STATE_IDLE				= 0,
	PM_STATE_ZERO_DRIFT,
	PM_STATE_SELF_TEST_BOOTSTRAP,
	PM_STATE_SELF_TEST_POWER_STAGE,
	PM_STATE_SELF_TEST_CLEARANCE,
	PM_STATE_ADJUST_VOLTAGE,
	PM_STATE_ADJUST_CURRENT,
	PM_STATE_PROBE_CONST_R,
	PM_STATE_PROBE_CONST_L,
	PM_STATE_LU_DETACHED,
	PM_STATE_LU_STARTUP,
	PM_STATE_LU_SHUTDOWN,
	PM_STATE_PROBE_CONST_E,
	PM_STATE_PROBE_CONST_J,
	PM_STATE_PROBE_LU_MPPE,
	PM_STATE_ADJUST_HALL,
	PM_STATE_ADJUST_QENC,
	PM_STATE_HALT,
};

enum {
	PM_OK					= 0,

	/* Internal.
	 * */
	PM_ERROR_ZERO_DRIFT_FAULT,
	PM_ERROR_NO_MOTOR_CONNECTED,
	PM_ERROR_POWER_STAGE_FAULT,
	PM_ERROR_ACCURACY_FAULT,
	PM_ERROR_CURRENT_LOOP_FAULT,
	PM_ERROR_INLINE_OVER_CURRENT,
	PM_ERROR_DC_LINK_OVER_VOLTAGE,
	PM_ERROR_FLUX_UNSTABLE,
	PM_ERROR_INVALID_OPERATION,
	PM_ERROR_SENSOR_HALL_FAULT,
	PM_ERROR_SENSOR_QENC_FAULT,

	/* External.
	 * */
	PM_ERROR_TIMEOUT,
};

typedef struct {

	float		current_A;
	float		current_B;
	float		voltage_U;

	float		voltage_A;
	float		voltage_B;
	float		voltage_C;

	int		pulse_HS;
	int		pulse_EP;
}
pmfb_t;

typedef struct {

	float		freq_hz;
	float		dT;

	int		dc_resolution;
	float		dc_minimal;
	float		dc_clearance;
	float		dc_bootstrap;

	int		ts_minimal;
	int		ts_clearance;
	int		ts_bootstrap;
	float		ts_inverted;

	int		fail_reason;
	float		self_BST[3];
	int		self_BM[8];
	float		self_RMSi[3];
	float		self_RMSu[3];

	int		config_NOP;
	int		config_TVM;
	int		config_FORCED;
	int		config_HFI;
	int		config_SENSOR;
	int		config_WEAK;
	int		config_DRIVE;
	int		config_SERVO;
	int		config_INFO;

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
	int		fb_EP;

	float		probe_current_hold;
	float		probe_current_bias_Q;
	float		probe_current_sine;
	float		probe_freq_sine_hz;
	float		probe_speed_hold;
	float		probe_speed_detached;
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

	float		vsi_EU;
	float		vsi_X;
	float		vsi_Y;
	float		vsi_DX;
	float		vsi_DY;
	int		vsi_SA;
	int		vsi_SB;
	int		vsi_SC;
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
	float		lu_lpf_wS;
	float		lu_MPPE;
	float		lu_gain_LP_S;
	float		lu_gain_TAKE;
	float		lu_gain_GIVE;
	float		lu_gain_LEVE;
	int		lu_caught;
	int		lu_mode;

	float		forced_F[2];
	float		forced_wS;
	float		forced_hold_D;
	float		forced_maximal;
	float		forced_reverse;
	float		forced_accel;

	float		detach_take_U;
	float		detach_X;
	float		detach_Y;
	float		detach_V[2];
	int		detach_TIM;
	int		detach_SKIP;
	float		detach_gain_AD;
	float		detach_gain_SF;

	struct {

		float	X;
		float	Y;
		float	lpf_E;
	}
	flux[PM_FLUX_MAX];

	int		flux_N;
	float		flux_lower_R;
	float		flux_upper_R;
	float		flux_E;
	int		flux_H;
	float		flux_F[2];
	float		flux_wS;
	float		flux_gain_IN;
	float		flux_gain_LO;
	float		flux_gain_HI;
	float		flux_gain_AD;
	float		flux_gain_LP_E;
	float		flux_gain_SF;

	float		inject_bias_U;
	float		inject_ratio_D;

	float		hfi_freq_hz;
	float		hfi_swing_D;
	float		hfi_iD;
	float		hfi_iQ;
	float		hfi_F[2];
	float		hfi_wS;
	float		hfi_wave[2];
	float		hfi_polarity;
	float		hfi_gain_EP;
	float		hfi_gain_SF;
	float		hfi_gain_FP;

	struct {

		float	X;
		float	Y;
	}
	hall_ST[8];

	int		hall_IF;
	int		hall_HS;
	int		hall_DIRF;
	float		hall_prolS;
	int		hall_prolTIM;
	float		hall_F[2];
	float		hall_wS;
	float		hall_lpf_wS;
	float		hall_prol_T;
	float		hall_gain_PF;
	float		hall_gain_SF;
	float		hall_gain_LP;

	int		qenc_baseEP;
	float		qenc_baseF[2];
	int		qenc_lastEP;
	int		qenc_rotEP;
	float		qenc_prolS;
	int		qenc_R;
	float		qenc_Zq;
	float		qenc_F[2];
	float		qenc_wS;
	float		qenc_lpf_wS;
	float		qenc_gain_PF;
	float		qenc_gain_SF;
	float		qenc_gain_LP;

	float		const_fb_U;
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

	float		temp_const_ifbU;
	float		temp_iE;
	float		temp_loRupRiN;
	float		temp_dTiL;
	float		temp_2PZiR;
	float		temp_JiEdTZp;

	float		watt_wP_maximal;
	float		watt_iB_maximal;
	float		watt_wP_reverse;
	float		watt_iB_reverse;
	float		watt_dclink_HI;
	float		watt_dclink_LO;
	float		watt_lpf_D;
	float		watt_lpf_Q;
	float		watt_lpf_wP;
	float		watt_gain_LP_F;
	float		watt_gain_LP_P;

	float		i_maximal;
	float		i_reverse;
	float		i_derated_1;
	float		i_derated_HFI;
	float		i_setpoint_D;
	float		i_setpoint_Q;
	float		i_integral_D;
	float		i_integral_Q;
	float		i_gain_P;
	float		i_gain_I;

	float		weak_maximal;
	float		weak_bias_U;
	float		weak_D;
	float		weak_gain_EU;

	float		v_maximal;
	float		v_reverse;

	float		s_maximal;
	float		s_reverse;
	float		s_setpoint;
	float		s_accel;
	float		s_track;
	float		s_integral;
	float		s_last_wS;
	float		s_gain_P;
	float		s_gain_I;
	float		s_gain_S;

	float		x_setpoint_F[2];
	int		x_setpoint_revol;
	float		x_lu_F1;
	int		x_lu_revol;
	float		x_near_tol;
	float		x_gain_P;
	float		x_gain_N;

	float		im_lu_F1;
	int		im_revol_1;
	int		im_revol_total;
	float		im_distance;
	float		im_consumed_Wh;
	float		im_consumed_Ah;
	float		im_reverted_Wh;
	float		im_reverted_Ah;
	float		im_capacity_Ah;
	float		im_fuel_pc;
	float		im_FIX[4];

	/*int		bt_mode;
	float		bt_*/

	void 		(* proc_set_DC) (int, int, int);
	void 		(* proc_set_Z) (int);
}
pmc_t;

void pm_default(pmc_t *pm);
void pm_build(pmc_t *pm);

void pm_voltage(pmc_t *pm, float uX, float uY);
void pm_feedback(pmc_t *pm, pmfb_t *fb);

void pm_ADD(float *S, float *C, float X);
void pm_FSM(pmc_t *pm);

const char *pm_strerror(int n);

#endif /* _H_PM_ */

