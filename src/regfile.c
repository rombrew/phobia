#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#ifdef HW_HAVE_NETWORK_EPCAN
#include "epcan.h"
#endif /* HW_HAVE_NETWORK_EPCAN */
#include "regfile.h"
#include "shell.h"
#include "tlm.h"

#define REG_DEF(l, e, i, u, f, m, p, t)	{ #l #e "\0" u, f, m, (void *) &l i, (void *) p, (void *) t}
#define REG_MAX				(sizeof(regfile) / sizeof(reg_t) - 1U)

static int		null;

static void
reg_proc_PWM(const reg_t *reg, float *lval, const float *rval)
{
	int			irq;

	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (pm.lu_MODE == PM_LU_DISABLED) {

			reg->link->f = *rval;

			irq = hal_lock_irq();

			PWM_configure();

			pm.freq_hz = hal.PWM_frequency;
			pm.dT = 1.f / pm.freq_hz;
			pm.dc_resolution = hal.PWM_resolution;

			hal_unlock_irq(irq);
		}
	}
}

static void
reg_proc_ADC(const reg_t *reg, float *lval, const float *rval)
{
	int			irq;

	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (pm.lu_MODE == PM_LU_DISABLED) {

			reg->link->f = *rval;

			irq = hal_lock_irq();

			ADC_const_build();

			reg_SET_F(ID_PM_FAULT_CURRENT_HALT, 0.f);

			hal_unlock_irq(irq);
		}
	}
}

#ifdef HW_HAVE_NETWORK_EPCAN
static void
reg_proc_net_TIM(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = hal.PWM_frequency / (float) reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = (int) (hal.PWM_frequency / *rval + .5f);
	}
}

static void
reg_proc_net_IDs(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = *rval;

		EPCAN_filter_ID();
	}
}

static void
reg_proc_net_LOST_ms(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = 1000.f / hal.PWM_frequency * (float) reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = (int) ((*rval) * hal.PWM_frequency / 1000.f + .5f);
	}
}
#endif /* HW_HAVE_NETWORK_EPCAN */

static void
reg_proc_PPM(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = *rval;

		PPM_configure();
	}
}

static void
reg_proc_DPS(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = *rval;

		DPS_configure();
	}
}

#ifdef HW_HAVE_PART_DRV_XX
static void
reg_proc_DRV_part(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}
	else if (rval != NULL) {

		if (pm.lu_MODE == PM_LU_DISABLED) {

			reg->link->i = *rval;

			DRV_halt();
			DRV_startup();
		}
	}
}

static void
reg_proc_DRV(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}
	else if (rval != NULL) {

		if (pm.lu_MODE == PM_LU_DISABLED) {

			reg->link->i = *rval;

			DRV_configure();
		}
	}
}
#endif /* HW_HAVE_PART_DRV_XX */

static void
reg_proc_OPT(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}
	else if (rval != NULL) {

		if (pm.lu_MODE == PM_LU_DISABLED) {

			reg->link->i = *rval;

			OPT_startup();
		}
	}
}

static void
reg_proc_PPM_get_PERIOD(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = PPM_get_PERIOD();
	}
}

static void
reg_proc_PPM_get_PULSE(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = PPM_get_PULSE();
	}
}


#ifdef HW_HAVE_ANALOG_KNOB
static void
reg_proc_ADC_get_analog_ANG(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = ADC_get_knob_ANG();
	}
}

static void
reg_proc_ADC_get_analog_BRK(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = ADC_get_knob_BRK();
	}
}
#endif /* HW_HAVE_ANALOG_KNOB */

static void
reg_proc_rpm(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * (30.f / M_PI_F) / (float) pm.const_Zp;
	}
	else if (rval != NULL) {

		reg->link->f = (*rval) * (M_PI_F / 30.f) * (float) pm.const_Zp;
	}
}

static void
reg_proc_mmps(const reg_t *reg, float *lval, const float *rval)
{
	float			rpm;

	if (lval != NULL) {

		reg_proc_rpm(reg, &rpm, NULL);

		*lval = rpm * pm.const_ld_S * (1000.f / 60.f);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			rpm = (*rval) / pm.const_ld_S * (60.f / 1000.f);

			reg_proc_rpm(reg, NULL, &rpm);
		}
	}
}

static void
reg_proc_epps(const reg_t *reg, float *lval, const float *rval)
{
	float			rpm;

	if (lval != NULL) {

		reg_proc_rpm(reg, &rpm, NULL);

		*lval = rpm * pm.abi_EPPR * (1.f / 60.f);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			rpm = (*rval) / pm.abi_EPPR * (60.f / 1.f);

			reg_proc_rpm(reg, NULL, &rpm);
		}
	}
}

static void
reg_proc_kmh(const reg_t *reg, float *lval, const float *rval)
{
	float			rpm;

	if (lval != NULL) {

		reg_proc_rpm(reg, &rpm, NULL);

		*lval = rpm * pm.const_ld_S * (3.6f / 60.f);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			rpm = (*rval) / pm.const_ld_S * (60.f / 3.6f);

			reg_proc_rpm(reg, NULL, &rpm);
		}
	}
}

static void
reg_proc_rpm_pc(const reg_t *reg, float *lval, const float *rval)
{
	float			kPC = pm.k_EMAX / 100.f;

	if (lval != NULL) {

		*lval = reg->link->f * pm.const_E / (kPC * pm.const_fb_U);
	}
	else if (rval != NULL) {

		if (pm.const_E > M_EPS_F) {

			reg->link->f = (*rval) * kPC * pm.const_fb_U / pm.const_E;
		}
	}
}

static void
reg_proc_bemf(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * pm.const_E;
	}
	else if (rval != NULL) {

		if (pm.const_E > M_EPS_F) {

			reg->link->f = (*rval) / pm.const_E;
		}
	}
}

static void
reg_proc_current_pc(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * 100.f / pm.i_maximal;
	}
	else if (rval != NULL) {

		reg->link->f = (*rval) * pm.i_maximal / 100.f;
	}
}

static void
reg_proc_E_kv(const reg_t *reg, float *lval, const float *rval)
{
        if (lval != NULL) {

                *lval = 5.513289f / (reg->link->f * (float) pm.const_Zp);
        }
        else if (rval != NULL) {

                reg->link->f = 5.513289f / ((*rval) * (float) pm.const_Zp);
        }
}

static void
reg_proc_E_nm(const reg_t *reg, float *lval, const float *rval)
{
	float			iD, iQ, mQ;

	if (		lval != NULL
			&& pm.const_E > M_EPS_F) {

		iD = 0.f;
		iQ = pm.i_maximal;

		if (pm.config_RELUCTANCE == PM_ENABLED) {

			/* TODO */
		}

		mQ = pm_torque_equation(&pm, iD, iQ);

                *lval = mQ * (float) pm.const_Zp;
        }
}

static void
reg_proc_kgm2(const reg_t *reg, float *lval, const float *rval)
{
	const float             Zp2 = (float) (pm.const_Zp * pm.const_Zp);

	if (lval != NULL) {

		*lval = reg->link->f * Zp2;
	}
	else if (rval != NULL) {

		if (pm.const_E > M_EPS_F) {

			reg->link->f = (*rval) / Zp2;
		}
	}
}

static void
reg_proc_kg(const reg_t *reg, float *lval, const float *rval)
{
	const float             Zp2 = (float) (pm.const_Zp * pm.const_Zp);
	const float		ld_R = pm.const_ld_S / (2.f * M_PI_F);

	if (lval != NULL) {

		if (ld_R > M_EPS_F) {

			*lval = reg->link->f * Zp2 / (ld_R * ld_R);
		}
		else {
			*lval = 0.f;
		}
	}
	else if (rval != NULL) {

		reg->link->f = (*rval) * (ld_R * ld_R) / Zp2;
	}
}

static void
reg_proc_load_nm(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * (float) pm.const_Zp;
	}
	else if (rval != NULL) {

		if (pm.const_E > M_EPS_F) {

			reg->link->f = (*rval) / (float) pm.const_Zp;
		}
	}
}

static void
reg_proc_auto_probe_speed_hold(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_PROBE_SPEED_HOLD);
		}
		else {
			reg->link->f = *rval;
		}
	}
}

static void
reg_proc_auto_threshold(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);
		}
		else {
			reg->link->f = *rval;
		}
	}
}

static void
reg_proc_auto_forced_accel(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_FORCED_ACCEL);
		}
		else {
			reg->link->f = *rval;
		}
	}
}

static void
reg_proc_auto_maximal_current(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_MAXIMAL_CURRENT);
		}
		else {
			reg->link->f = *rval;
		}
	}
}

static void
reg_proc_auto_loop_current(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_LOOP_CURRENT);
		}
		else {
			reg->link->f = *rval;
		}
	}
}

static void
reg_proc_auto_loop_speed(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_LOOP_SPEED);
		}
		else {
			reg->link->f = *rval;
		}
	}
}

static void
reg_proc_halt(const reg_t *reg, float *lval, const float *rval)
{
	float			halt, adjust;

	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			halt = m_fabsf(hal.const_ADC.GA * ADC_RESOLUTION / 2.f);

			adjust = (pm.ad_IA[1] < pm.ad_IB[1])
				? pm.ad_IA[1] : pm.ad_IB[1];

			halt *= adjust;

			reg->link->f = (float) (int) (halt * .95f);
		}
		else {
			reg->link->f = *rval;
		}
	}
}

static void
reg_proc_watt(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			reg->link->f = 1.f;
		}
		else {
			reg->link->f = *rval;
		}
	}
}

static void
reg_proc_INUSE(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}
	else if (rval != NULL) {

		if (*rval != PM_ENABLED) {

			reg->link->i = *rval;
		}
	}
}

static void
reg_proc_Fg(const reg_t *reg, float *lval, const float *rval)
{
	float			*F = (void *) reg->link;
	float			f_cosine, f_sine;
	int			irq;

	if (lval != NULL) {

		irq = hal_lock_irq();

		f_cosine = F[0];
		f_sine   = F[1];

		hal_unlock_irq(irq);

		*lval = m_atan2f(f_sine, f_cosine) * (180.f / M_PI_F);
	}
}

static void
reg_proc_Fg_nolock(const reg_t *reg, float *lval, const float *rval)
{
	float			*F = (void *) reg->link;

	if (lval != NULL) {

		*lval = m_atan2f(F[1], F[0]) * (180.f / M_PI_F);
	}
}

static void
reg_proc_debug_flux_EFg(const reg_t *reg, float *lval, const float *rval)
{
	float		A, B;
	int		irq;

	if (lval != NULL) {

		irq = hal_lock_irq();

		A = pm.flux_F[0] * pm.lu_F[0] + pm.flux_F[1] * pm.lu_F[1];
		B = pm.flux_F[0] * pm.lu_F[1] - pm.flux_F[1] * pm.lu_F[0];

		hal_unlock_irq(irq);

		*lval = m_atan2f(B, A) * (180.f / M_PI_F);
	}
}

static void
reg_proc_debug_hfi_EFg(const reg_t *reg, float *lval, const float *rval)
{
	float		A, B;
	int		irq;

	if (lval != NULL) {

		irq = hal_lock_irq();

		A = pm.hfi_F[0] * pm.lu_F[0] + pm.hfi_F[1] * pm.lu_F[1];
		B = pm.hfi_F[0] * pm.lu_F[1] - pm.hfi_F[1] * pm.lu_F[0];

		hal_unlock_irq(irq);

		*lval = m_atan2f(B, A) * (180.f / M_PI_F);
	}
}

static void
reg_proc_location_g(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * (180.f / M_PI_F) / (float) pm.const_Zp;
	}
	else if (rval != NULL) {

		reg->link->f = (*rval) * (M_PI_F / 180.f) * (float) pm.const_Zp;
	}
}

static void
reg_proc_location_mm(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * pm.const_ld_S * 1000.f
			/ (2.f * M_PI_F * (float) pm.const_Zp);
	}
	else if (rval != NULL) {

		reg->link->f = (*rval) * (2.f * M_PI_F) * (float) pm.const_Zp
			/ (pm.const_ld_S * 1000.f);
	}
}

static void
reg_proc_km(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f / 1000.f;
	}
	else if (rval != NULL) {

		reg->link->f = (*rval) * 1000.f;
	}
}

static void
reg_proc_mm(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * 1000.f;
	}
	else if (rval != NULL) {

		reg->link->f = (*rval) / 1000.f;
	}
}

static void
reg_proc_gain_accel(const reg_t *reg, float *lval, const float *rval)
{
        if (lval != NULL) {

                *lval = reg->link->f * reg->link->f / 2.f;
        }
        else if (rval != NULL) {

                reg->link->f = m_sqrtf((*rval) * 2.f);
        }
}

static void
reg_proc_gain_accel_mm(const reg_t *reg, float *lval, const float *rval)
{
	float			rads;

	if (lval != NULL) {

		reg_proc_gain_accel(reg, &rads, NULL);

		*lval = rads * pm.const_ld_S * 1000.f
			/ (2.f * M_PI_F * (float) pm.const_Zp);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			rads = (*rval) * (2.f * M_PI_F * (float) pm.const_Zp)
				/ (pm.const_ld_S * 1000.f);

			reg_proc_gain_accel(reg, NULL, &rads);
		}
	}
}

static void
reg_proc_tvm_FIR_tau(const reg_t *reg, float *lval, const float *rval)
{
	float		*FIR = (void *) reg->link;
	float		tau;

	if (lval != NULL) {

		tau = FIR[0] / - FIR[1];
		tau = (tau > M_EPS_F) ? pm.dT * 1000000.f / m_logf(tau) : 0.f;

		*lval = tau;
	}
}

static void
reg_proc_im_fuel(const reg_t *reg, float *lval, const float *rval)
{
	int			irq;

	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			irq = hal_lock_irq();

			pm.im_consumed_Wh = 0.f;
			pm.im_consumed_Ah = 0.f;
			pm.im_reverted_Wh = 0.f;
			pm.im_reverted_Ah = 0.f;

			pm.im_REM[0] = 0.f;
			pm.im_REM[1] = 0.f;
			pm.im_REM[2] = 0.f;
			pm.im_REM[3] = 0.f;

			hal_unlock_irq(irq);

			vTaskDelay((TickType_t) 1);
		}
	}
}

static void
reg_format_self_BST(const reg_t *reg)
{
	int		*BST = (void *) reg->link;

	printf("%1f %1f %1f (ms)", &BST[0], &BST[1], &BST[2]);
}

static void
reg_format_self_BM(const reg_t *reg)
{
	int		*BM = (void *) reg->link;

	printf("%2x %2x %2x %2x %2x %2x %2x", BM[0], BM[1], BM[2], BM[3], BM[4], BM[5], BM[6]);
}

static void
reg_format_self_RMSi(const reg_t *reg)
{
	float		*RMS = (void *) reg->link;

	printf("%3f %3f %3f (A)", &RMS[0], &RMS[1], &RMS[2]);
}

static void
reg_format_self_RMSu(const reg_t *reg)
{
	float		*RMS = (void *) reg->link;

	printf("%4f %4f %4f %4f (V)", &RMS[0], &RMS[1], &RMS[2], &RMS[3]);
}

static void
reg_format_E_nm(const reg_t *reg)
{
	printf(reg->fmt, &reg->link->f);
	printf(" (Nm at %1.f A)", &pm.i_maximal);
}

static void
reg_format_referenced(const reg_t *reg, int reg_ID)
{
	const char	*fmt, *su;

	reg_ID = reg_GET_I(reg_ID);

	fmt = regfile[reg_ID].fmt;

	if (		   fmt[1] != 'i'
			&& fmt[2] != 'x') {

		printf(fmt, &reg->link->f);

		su = regfile[reg_ID].sym;
		su += strlen(su) + 1;

		if (*su != 0) {

			printf(" (%s)", su);
		}
	}
}

static void
reg_format_referenced_ppm(const reg_t *reg)
{
	reg_format_referenced(reg, ID_AP_PPM_REG_ID);
}

#ifdef HW_HAVE_ANALOG_KNOB
static void
reg_format_referenced_knob(const reg_t *reg)
{
	reg_format_referenced(reg, ID_AP_KNOB_REG_ID);
}
#endif /* HW_HAVE_ANALOG_KNOB */

#define PM_SFI_CASE(value)	case value: printf("(%s)", PM_SFI(value)); break

static void
reg_format_enum(const reg_t *reg)
{
	int			reg_ID, val;

	reg_ID = (int) (reg - regfile);
	val = reg->link->i;

	printf("%i ", val);

	switch (reg_ID) {

		case ID_HAL_DPS_MODE:

			switch (val) {

				PM_SFI_CASE(DPS_DISABLED);
				PM_SFI_CASE(DPS_DRIVE_HALL);
				PM_SFI_CASE(DPS_DRIVE_ABI);
				PM_SFI_CASE(DPS_DRIVE_SOFTWARE);

				default: break;
			}
			break;

		case ID_HAL_PPM_MODE:

			switch (val) {

				PM_SFI_CASE(PPM_DISABLED);
				PM_SFI_CASE(PPM_PULSE_WIDTH);
				PM_SFI_CASE(PPM_OUTPULSE);
				PM_SFI_CASE(PPM_STEP_DIR);
				PM_SFI_CASE(PPM_BACKUP_ABI);

				default: break;
			}
			break;

#ifdef HW_HAVE_PART_DRV_XX
		case ID_HAL_DRV_PART:

			switch (val) {

				PM_SFI_CASE(DRV_NONE);
				PM_SFI_CASE(DRV_PART_DRV8303);
				PM_SFI_CASE(DRV_PART_DRV8305);

				default: break;
			}
			break;
#endif /* HW_HAVE_PART_DRV_XX */

#ifdef HW_HAVE_NETWORK_EPCAN
		case ID_NET_LOG_MODE:

			switch (val) {

				PM_SFI_CASE(EPCAN_LOG_DISABLED);
				PM_SFI_CASE(EPCAN_LOG_FILTERED);
				PM_SFI_CASE(EPCAN_LOG_PROMISCUOUS);

				default: break;
			}
			break;

		case ID_NET_EP0_MODE:
		case ID_NET_EP1_MODE:
		case ID_NET_EP2_MODE:
		case ID_NET_EP3_MODE:

			switch (val) {

				PM_SFI_CASE(EPCAN_PIPE_DISABLED);
				PM_SFI_CASE(EPCAN_PIPE_INCOMING);
				PM_SFI_CASE(EPCAN_PIPE_OUTGOING_REGULAR);
				PM_SFI_CASE(EPCAN_PIPE_OUTGOING_INJECTED);

				default: break;
			}
			break;

		case ID_NET_EP0_STARTUP:
		case ID_NET_EP1_STARTUP:
		case ID_NET_EP2_STARTUP:
		case ID_NET_EP3_STARTUP:

			switch (val) {

				PM_SFI_CASE(PM_DISABLED);
				PM_SFI_CASE(PM_ENABLED);

				default: break;
			}
			break;

		case ID_NET_EP0_PAYLOAD:
		case ID_NET_EP1_PAYLOAD:
		case ID_NET_EP2_PAYLOAD:
		case ID_NET_EP3_PAYLOAD:

			switch (val) {

				PM_SFI_CASE(EPCAN_PAYLOAD_FLOAT);
				PM_SFI_CASE(EPCAN_PAYLOAD_INT_16);
				PM_SFI_CASE(EPCAN_PAYLOAD_PACKED_INT_16_0);
				PM_SFI_CASE(EPCAN_PAYLOAD_PACKED_INT_16_1);
				PM_SFI_CASE(EPCAN_PAYLOAD_PACKED_INT_16_2);
				PM_SFI_CASE(EPCAN_PAYLOAD_PACKED_INT_16_3);

				default: break;
			}
			break;
#endif /* HW_HAVE_NETWORK_EPCAN */

		case ID_AP_PPM_STARTUP:
		case ID_AP_STEP_STARTUP:

#ifdef HW_HAVE_ANALOG_KNOB
		case ID_AP_KNOB_ENABLED:
		case ID_AP_KNOB_STARTUP:
#endif /* HW_HAVE_ANALOG_KNOB */

			switch (val) {

				PM_SFI_CASE(PM_DISABLED);
				PM_SFI_CASE(PM_ENABLED);

				default: break;
			}
			break;

		case ID_AP_NTC_PCB_TYPE:

#ifdef HW_HAVE_NTC_MOTOR
		case ID_AP_NTC_EXT_TYPE:
#endif /* HW_HAVE_NTC_MOTOR */

			switch (val) {

				PM_SFI_CASE(NTC_NONE);
				PM_SFI_CASE(NTC_GND);
				PM_SFI_CASE(NTC_VCC);
				PM_SFI_CASE(NTC_LINEAR);

				default: break;
			}
			break;

		case ID_AP_AUTO_APP0:
		case ID_AP_AUTO_APP1:

			printf("(%s)", app_name_by_ID(val));
			break;

		case ID_PM_CONFIG_NOP:

			switch (val) {

				PM_SFI_CASE(PM_NOP_THREE_PHASE);
				PM_SFI_CASE(PM_NOP_TWO_PHASE);

				default: break;
			}
			break;

		case ID_PM_CONFIG_IFB:

			switch (val) {

				PM_SFI_CASE(PM_IFB_AB_INLINE);
				PM_SFI_CASE(PM_IFB_AB_GND);
				PM_SFI_CASE(PM_IFB_ABC_INLINE);
				PM_SFI_CASE(PM_IFB_ABC_GND);

				default: break;
			}
			break;

		case ID_PM_CONFIG_TVM:
		case ID_PM_CONFIG_DEBUG:
		case ID_PM_CONFIG_VSI_CIRCULAR:
		case ID_PM_CONFIG_VSI_PRECISE:
		case ID_PM_CONFIG_LU_FORCED:
		case ID_PM_CONFIG_LU_ESTIMATE_HFI:
		case ID_PM_CONFIG_RELUCTANCE:
		case ID_PM_CONFIG_WEAKENING:
		case ID_PM_CONFIG_HOLDING_BRAKE:
		case ID_PM_CONFIG_SPEED_LIMITED:
		case ID_PM_CONFIG_IMPEDANCE_MAJOR:
		case ID_PM_CONFIG_ABI_ABSOLUTE:
		case ID_PM_CONFIG_MILEAGE_INFO:

			switch (val) {

				PM_SFI_CASE(PM_DISABLED);
				PM_SFI_CASE(PM_ENABLED);

				default: break;
			}
			break;

		case ID_PM_CONFIG_LU_ESTIMATE_FLUX:

			switch (val) {

				PM_SFI_CASE(PM_ESTIMATE_NONE);
				PM_SFI_CASE(PM_ESTIMATE_ORTEGA);
				PM_SFI_CASE(PM_ESTIMATE_KALMAN);

				default: break;
			}
			break;

		case ID_PM_CONFIG_LU_SENSOR:

			switch (val) {

				PM_SFI_CASE(PM_SENSOR_DISABLED);
				PM_SFI_CASE(PM_SENSOR_HALL);
				PM_SFI_CASE(PM_SENSOR_ABI);
				PM_SFI_CASE(PM_SENSOR_SINCOS);

				default: break;
			}
			break;

		case ID_PM_CONFIG_LU_LOCATION:

			switch (val) {

				PM_SFI_CASE(PM_LOCATION_INHERITED);
				PM_SFI_CASE(PM_LOCATION_ABI);
				PM_SFI_CASE(PM_LOCATION_SINCOS);

				default: break;
			}
			break;

		case ID_PM_CONFIG_LU_DRIVE:

			switch (val) {

				PM_SFI_CASE(PM_DRIVE_CURRENT);
				PM_SFI_CASE(PM_DRIVE_SPEED);
				PM_SFI_CASE(PM_DRIVE_SERVO);

				default: break;
			}
			break;

		case ID_PM_CONFIG_SINCOS_FRONTEND:

			switch (val) {

				PM_SFI_CASE(PM_SINCOS_ANALOG);
				PM_SFI_CASE(PM_SINCOS_RESOLVER);

				default: break;
			}
			break;


		case ID_PM_FSM_ERRNO:

			printf("(%s)", pm_strerror(val));
			break;

		case ID_PM_LU_MODE:

			switch (val) {

				PM_SFI_CASE(PM_LU_DISABLED);
				PM_SFI_CASE(PM_LU_DETACHED);
				PM_SFI_CASE(PM_LU_FORCED);
				PM_SFI_CASE(PM_LU_ESTIMATE_FLUX);
				PM_SFI_CASE(PM_LU_ESTIMATE_HFI);
				PM_SFI_CASE(PM_LU_SENSOR_HALL);
				PM_SFI_CASE(PM_LU_SENSOR_ABI);
				PM_SFI_CASE(PM_LU_SENSOR_SINCOS);

				default: break;
			}
			break;

		case ID_PM_FLUX_ZONE:

			switch (val) {

				PM_SFI_CASE(PM_ZONE_NONE);
				PM_SFI_CASE(PM_ZONE_UNCERTAIN);
				PM_SFI_CASE(PM_ZONE_HIGH);
				PM_SFI_CASE(PM_ZONE_LOCKED_IN_DETACH);

				default: break;
			}
			break;

		case ID_PM_TVM_INUSE:
		case ID_PM_HALL_INUSE:
		case ID_PM_ABI_INUSE:

			switch (val) {

				PM_SFI_CASE(PM_DISABLED);
				PM_SFI_CASE(PM_ENABLED);

				default: break;
			}
			break;

		default: break;
	}
}

const reg_t		regfile[] = {

	REG_DEF(null,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(iodef_ECHO,,,		"",	"%i",	0, 0, NULL),
	REG_DEF(iodef_PRETTY,,,		"",	"%i",	0, 0, NULL),

	REG_DEF(hal.USART_baud_rate,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(hal.PWM_frequency,,,		"Hz",	"%1f",	REG_CONFIG, &reg_proc_PWM, NULL),
	REG_DEF(hal.PWM_deadtime,,,		"ns",	"%1f",	REG_CONFIG, &reg_proc_PWM, NULL),
	REG_DEF(hal.ADC_reference_voltage,,,	"V",	"%3f",	REG_CONFIG, &reg_proc_ADC, NULL),
	REG_DEF(hal.ADC_shunt_resistance,,,	"Ohm",	"%4g",	REG_CONFIG, &reg_proc_ADC, NULL),
	REG_DEF(hal.ADC_amplifier_gain,,,	"",	"%1f",	REG_CONFIG, &reg_proc_ADC, NULL),
	REG_DEF(hal.ADC_voltage_ratio,,,	"",	"%4e",	REG_CONFIG, &reg_proc_ADC, NULL),
	REG_DEF(hal.ADC_terminal_ratio,,,	"",	"%4e",	REG_CONFIG, &reg_proc_ADC, NULL),
	REG_DEF(hal.ADC_terminal_bias,,,	"",	"%4e",	REG_CONFIG, &reg_proc_ADC, NULL),

#ifdef HW_HAVE_ANALOG_KNOB
	REG_DEF(hal.ADC_knob_ratio,,,		"",	"%4e",	REG_CONFIG, &reg_proc_ADC, NULL),
#endif /* HW_HAVE_ANALOG_KNOB */

	REG_DEF(hal.DPS_mode,,,		"",	"%i", REG_CONFIG, &reg_proc_DPS, &reg_format_enum),
	REG_DEF(hal.PPM_mode,,,		"",	"%i", REG_CONFIG, &reg_proc_PPM, &reg_format_enum),
	REG_DEF(hal.PPM_timebase,,,	"Hz",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(hal.PPM_caught,,,	"",	"%i",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(hal, .PPM_get_PERIOD,, "us", "%2f", REG_READ_ONLY, &reg_proc_PPM_get_PERIOD, NULL),
	REG_DEF(hal, .PPM_get_PULSE,,  "us", "%2f", REG_READ_ONLY, &reg_proc_PPM_get_PULSE, NULL),

#ifdef HW_HAVE_ANALOG_KNOB
	REG_DEF(hal, .ADC_get_knob_ANG,, "V", "%3f", REG_READ_ONLY, &reg_proc_ADC_get_analog_ANG, NULL),
	REG_DEF(hal, .ADC_get_knob_BRK,, "V", "%3f", REG_READ_ONLY, &reg_proc_ADC_get_analog_BRK, NULL),
#endif /* HW_HAVE_ANALOG_KNOB */

#ifdef HW_HAVE_PART_DRV_XX
	REG_DEF(hal.DRV.part,,,		"",	"%i",	REG_CONFIG, &reg_proc_DRV_part, &reg_format_enum),
	REG_DEF(hal.DRV.auto_RESET,,,	"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(hal.DRV.status_raw,,,	"",	"%4x",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(hal.DRV.gate_current,,,	"",	"%i",	REG_CONFIG, &reg_proc_DRV, NULL),
	REG_DEF(hal.DRV.ocp_level,,,	"",	"%i",	REG_CONFIG, &reg_proc_DRV, NULL),
#endif /* HW_HAVE_PART_DRV_XX */

	REG_DEF(hal.OPT,,,		"",	"%4x",	REG_CONFIG, &reg_proc_OPT, NULL),

#ifdef HW_HAVE_NETWORK_EPCAN
	REG_DEF(net.node_ID,,,		"",	"%i",	REG_CONFIG, &reg_proc_net_IDs, NULL),
	REG_DEF(net.log_MODE,,,		"",	"%i",	REG_CONFIG, &reg_proc_net_IDs, &reg_format_enum),
	REG_DEF(net.startup_LOST, _ms,,	"",	"%1f",	REG_CONFIG, &reg_proc_net_LOST_ms, NULL),

	REG_DEF(net.ep, 0_ID, [0].ID,"",		"%i",	REG_CONFIG, &reg_proc_net_IDs, NULL),
	REG_DEF(net.ep, 0_reg_DATA, [0].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(net.ep, 0_reg_ID, [0].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(net.ep, 0_MODE, [0].MODE,"",		"%i",	REG_CONFIG, &reg_proc_net_IDs, &reg_format_enum),
	REG_DEF(net.ep, 0_STARTUP, [0].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 0_TIM_hz, [0].TIM,	"Hz",	"%1f",	REG_CONFIG, &reg_proc_net_TIM, NULL),
	REG_DEF(net.ep, 0_inject_ID, [0].inject_ID,"",	"%i",	REG_CONFIG, &reg_proc_net_IDs, NULL),
	REG_DEF(net.ep, 0_PAYLOAD, [0].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 0_range0, [0].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(net.ep, 0_range1, [0].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(net.ep, 1_ID, [1].ID,"",		"%i",	REG_CONFIG, &reg_proc_net_IDs, NULL),
	REG_DEF(net.ep, 1_reg_DATA, [1].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(net.ep, 1_reg_ID, [1].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(net.ep, 1_MODE, [1].MODE,"",		"%i",	REG_CONFIG, &reg_proc_net_IDs, &reg_format_enum),
	REG_DEF(net.ep, 1_STARTUP, [1].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 1_TIM_hz, [1].TIM,	"Hz",	"%1f",	REG_CONFIG, &reg_proc_net_TIM, NULL),
	REG_DEF(net.ep, 1_inject_ID, [1].inject_ID,"",	"%i",	REG_CONFIG, &reg_proc_net_IDs, NULL),
	REG_DEF(net.ep, 1_PAYLOAD, [1].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 1_range0, [1].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(net.ep, 1_range1, [1].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(net.ep, 2_ID, [2].ID,"",		"%i",	REG_CONFIG, &reg_proc_net_IDs, NULL),
	REG_DEF(net.ep, 2_reg_DATA, [2].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(net.ep, 2_reg_ID, [2].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(net.ep, 2_MODE, [2].MODE,"",		"%i",	REG_CONFIG, &reg_proc_net_IDs, &reg_format_enum),
	REG_DEF(net.ep, 2_STARTUP, [2].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 2_TIM_hz, [2].TIM,	"Hz",	"%1f",	REG_CONFIG, &reg_proc_net_TIM, NULL),
	REG_DEF(net.ep, 2_inject_ID, [2].inject_ID,"",	"%i",	REG_CONFIG, &reg_proc_net_IDs, NULL),
	REG_DEF(net.ep, 2_PAYLOAD, [2].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 2_range0, [2].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(net.ep, 2_range1, [2].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(net.ep, 3_ID, [3].ID,"",		"%i",	REG_CONFIG, &reg_proc_net_IDs, NULL),
	REG_DEF(net.ep, 3_reg_DATA, [3].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(net.ep, 3_reg_ID, [3].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(net.ep, 3_MODE, [3].MODE,"",		"%i",	REG_CONFIG, &reg_proc_net_IDs, &reg_format_enum),
	REG_DEF(net.ep, 3_STARTUP, [3].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 3_TIM_hz, [3].TIM,	"Hz",	"%1f",	REG_CONFIG, &reg_proc_net_TIM, NULL),
	REG_DEF(net.ep, 3_inject_ID, [3].inject_ID,"",	"%i",	REG_CONFIG, &reg_proc_net_IDs, NULL),
	REG_DEF(net.ep, 3_PAYLOAD, [3].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 3_range0, [3].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(net.ep, 3_range1, [3].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),
#endif /* HW_HAVE_NETWORK_EPCAN */

	REG_DEF(ap.ppm_reg_ID,,,		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(ap.ppm_STARTUP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.ppm_in_range, 0, [0],	"us",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_in_range, 1, [1],	"us",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_in_range, 2, [2],	"us",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_control_range, 0, [0],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_ppm),
	REG_DEF(ap.ppm_control_range, 1, [1],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_ppm),
	REG_DEF(ap.ppm_control_range, 2, [2],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_ppm),

	REG_DEF(ap.step_reg_ID,,,		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(ap.step_STARTUP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.step_accuEP,,,		"",	"%i",	0, NULL, NULL),
	REG_DEF(ap.step_const_ld_EP,,,		"mm",	"%3f",	REG_CONFIG, NULL, NULL),

#ifdef HW_HAVE_ANALOG_KNOB
	REG_DEF(ap.knob_ENABLED,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.knob_reg_ID,,,		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(ap.knob_STARTUP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.knob_in_ANG, 0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_in_ANG, 1, [1],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_in_ANG, 2, [2],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_in_BRK, 0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_in_BRK, 1, [1],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_in_lost, 0, [0],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_in_lost, 1, [1],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_control_ANG, 0, [0],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_knob),
	REG_DEF(ap.knob_control_ANG, 1, [1],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_knob),
	REG_DEF(ap.knob_control_ANG, 2, [2],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_knob),
	REG_DEF(ap.knob_control_BRK,,,		"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_knob),
#endif /* HW_HAVE_ANALOG_KNOB */

	REG_DEF(ap.idle_TIME_s,,,		"s",	"%1f",	REG_CONFIG, NULL, NULL),

#ifdef HW_HAVE_NTC_ON_PCB
	REG_DEF(ap.ntc_PCB.type,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.ntc_PCB.balance,,,		"Ohm",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_PCB.ntc0,,,		"Ohm",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_PCB.ta0,,,		"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_PCB.betta,,,		"",	"%1f",	REG_CONFIG, NULL, NULL),
#endif /* HW_HAVE_NTC_ON_PCB */

#ifdef HW_HAVE_NTC_MOTOR
	REG_DEF(ap.ntc_EXT.type,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.ntc_EXT.balance,,,		"Ohm",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_EXT.ntc0,,,		"Ohm",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_EXT.ta0,,,		"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_EXT.betta,,,		"",	"%1f",	REG_CONFIG, NULL, NULL),
#endif /* HW_HAVE_NTC_MOTOR */

	REG_DEF(ap.temp_PCB,,,			"C",	"%1f",	REG_READ_ONLY, NULL, NULL),
#ifdef HW_HAVE_NTC_MOTOR
	REG_DEF(ap.temp_EXT,,,			"C",	"%1f",	REG_READ_ONLY, NULL, NULL),
#endif /* HW_HAVE_NTC_MOTOR */
	REG_DEF(ap.temp_INT,,,			"C",	"%1f",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(ap.tpro_PCB_temp_halt,,,	"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_PCB_temp_derate,,,	"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_PCB_temp_FAN,,,		"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_EXT_temp_derate,,,	"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_derated_PCB,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_derated_EXT,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_temp_recovery,,,	"C",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.servo_SPAN_mm, 0, [0],	"mm",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.servo_SPAN_mm, 1, [1],	"mm",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.servo_UNIFORM_mmps,,,	"mm/s",	"%2f",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.auto_APP, 0, [0],		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.auto_APP, 1, [1],		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),

	REG_DEF(ap.adc_load_kg,,,		"kg",	"%4f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(ap.adc_load_scale, 0, [0],	"kg",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.adc_load_scale, 1, [1],	"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.dc_resolution,,,		"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.dc_minimal,,,		"us",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_clearance,,,		"us",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_skip,,,			"us",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_bootstrap,,,		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_clamped,,,		"s",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.self_BST,,,		"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_BST),
	REG_DEF(pm.self_BM,,,		"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_BM),
	REG_DEF(pm.self_RMSi,,,		"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_RMSi),
	REG_DEF(pm.self_RMSu,,,		"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_RMSu),

	REG_DEF(pm.config_NOP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_IFB,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_TVM,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_DEBUG,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),

	REG_DEF(pm.config_VSI_CIRCULAR,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_VSI_PRECISE,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_FORCED,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_ESTIMATE_FLUX,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_ESTIMATE_HFI,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_SENSOR,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_LOCATION,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_DRIVE,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_RELUCTANCE,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_WEAKENING,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_HOLDING_BRAKE,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_SPEED_LIMITED,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_IMPEDANCE_MAJOR,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_ABI_ABSOLUTE,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_SINCOS_FRONTEND,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_MILEAGE_INFO,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_BOOST_CHARGE,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),

	REG_DEF(pm.fsm_req,,,			"",	"%i",	0, NULL, &reg_format_enum),
	REG_DEF(pm.fsm_state,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_enum),
	REG_DEF(pm.fsm_errno,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_enum),

	REG_DEF(pm.tm_transient_slow,,,		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_transient_fast,,,		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_voltage_hold,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_current_hold,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_current_ramp,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_instant_probe,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_average_probe,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_average_drift,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_average_inertia,,, 	"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_startup,,,		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_halt_pause,,,		"ms",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.ad_IA, 0, [0],		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_IA, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_IB, 0, [0],		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_IB, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_IC, 0, [0],		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_IC, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_US, 0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_US, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UA, 0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UA, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UB, 0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UB, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UC, 0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UC, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.fb_iA,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_iB,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_iC,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_uA,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_uB,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_uC,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_HS,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_EP,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_SIN,,,			"",	"%4f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_COS,,,			"",	"%4f",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(pm.probe_current_hold,,,	"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_current_weak,,,	"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_hold_angle,,,		"g",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_current_sine,,,	"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_current_bias,,,	"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_freq_sine_hz,,,	"Hz",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_speed_hold,,,		"rad/s","%2f",	REG_CONFIG, &reg_proc_auto_probe_speed_hold, NULL),
	REG_DEF(pm.probe_speed_hold, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.probe_speed_detached,,,	"rad/s","%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_speed_detached, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.probe_gain_P,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_gain_I,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.fault_voltage_tol,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.fault_current_tol,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.fault_accuracy_tol,,,	"",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.fault_terminal_tol,,,	"V",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.fault_current_halt,,,	"A",	"%3f",	REG_CONFIG, &reg_proc_halt, NULL),
	REG_DEF(pm.fault_voltage_halt,,,	"V",	"%3f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.vsi_DC,,,			"",	"%4f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_X,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_Y,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_DX,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_DY,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_AF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_BF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_CF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_SF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_UF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(pm.vsi_lpf_DC,,,		"",	"%4f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_gain_LP_F,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.tvm_INUSE,,,			"",	"%i",	REG_CONFIG, &reg_proc_INUSE, &reg_format_enum),
	REG_DEF(pm.tvm_range_DC,,,		"",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tvm_A,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_B,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_C,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_A, 0, [0],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_A, 1, [1],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_A, 2, [2],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_A, _tau,,	"us",	"%3f",	REG_READ_ONLY, &reg_proc_tvm_FIR_tau, NULL),
	REG_DEF(pm.tvm_FIR_B, 0, [0],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_B, 1, [1],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_B, 2, [2],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_B, _tau,,	"us",	"%3f",	REG_READ_ONLY, &reg_proc_tvm_FIR_tau, NULL),
	REG_DEF(pm.tvm_FIR_C, 0, [0],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_C, 1, [1],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_C, 2, [2],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_C, _tau,,	"us",	"%3f",	REG_READ_ONLY, &reg_proc_tvm_FIR_tau, NULL),
	REG_DEF(pm.tvm_DX,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_DY,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(pm.lu_MODE,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_enum),
	REG_DEF(pm.lu_iX,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_iY,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_iD,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_iQ,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg, NULL),
	REG_DEF(pm.lu_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.lu_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.lu_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.lu_location,,,		"rad",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_location, _g,,		"g",	"%2f",	REG_READ_ONLY, &reg_proc_location_g, NULL),
	REG_DEF(pm.lu_location, _mm,,		"mm",	"%3f",	REG_READ_ONLY, &reg_proc_location_mm, NULL),
	REG_DEF(pm.lu_total_revol,,,		"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_transient,,,	"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.lu_load_torque,,,		"Nm",	"%3f",	REG_READ_ONLY, &reg_proc_load_nm, NULL),
	REG_DEF(pm.lu_gain_TQ,,,		"",	"%2e",	REG_CONFIG, &reg_proc_auto_loop_speed, NULL),

	REG_DEF(pm.forced_hold_D,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_maximal,,,	"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_maximal, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.forced_reverse,,,	"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_reverse, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.forced_accel,,,	"rad/s2",	"%1f",	REG_CONFIG, &reg_proc_auto_forced_accel, NULL),
	REG_DEF(pm.forced_accel, _rpm,,	"rpm/s",	"%1f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.forced_accel, _mmps,,"mm/s2",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.forced_slew_rate,,,		"A/s",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_maximal_DC,,,		"",	"%2f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.detach_level_U,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.detach_trip_AD,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.detach_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.flux_ZONE,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_enum),
	REG_DEF(pm.flux_E,,,			"Wb",	"%4g",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.flux_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg, NULL),
	REG_DEF(pm.flux_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.flux_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.flux_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.flux_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.flux_trip_AD,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_IN,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_LO,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_HI,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_IF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.kalman_bias_Q,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.kalman_gain_Q, 0, [0],	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.kalman_gain_Q, 1, [1],	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.kalman_gain_Q, 2, [2],	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.kalman_gain_Q, 3, [3],	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.kalman_gain_Q, 4, [4],	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.kalman_gain_R,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.zone_lpf_wS,,,	"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.zone_threshold_NOISE,,, "rad/s", 	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.zone_threshold_NOISE, _bemf,, "V",	"%3f",	REG_CONFIG, &reg_proc_bemf, NULL),
	REG_DEF(pm.zone_threshold_BASE,,, "rad/s",	"%2f",	REG_CONFIG, &reg_proc_auto_threshold, NULL),
	REG_DEF(pm.zone_threshold_BASE, _bemf,, "V",	"%3f",	REG_CONFIG, &reg_proc_bemf, NULL),
	REG_DEF(pm.zone_gain_HY,,,		"",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.zone_gain_LP_S,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.hfi_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg, NULL),
	REG_DEF(pm.hfi_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hfi_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.hfi_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.hfi_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.hfi_inject_sine,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_maximal,,,	"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_maximal, _rpm,,		"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.hfi_im_L1,,,			"H",	"%4g",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hfi_im_L2,,,			"H",	"%4g",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hfi_im_R,,,			"Ohm",	"%4g",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hfi_INJS,,,			"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_SKIP,,,			"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_ESTI,,,			"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_POLA,,,			"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_gain_IF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.hall_ST, 1_X, [1].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 1_Y, [1].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 1, [1],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg_nolock, NULL),
	REG_DEF(pm.hall_ST, 2_X, [2].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 2_Y, [2].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 2, [2],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg_nolock, NULL),
	REG_DEF(pm.hall_ST, 3_X, [3].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 3_Y, [3].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 3, [3],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg_nolock, NULL),
	REG_DEF(pm.hall_ST, 4_X, [4].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 4_Y, [4].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 4, [4],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg_nolock, NULL),
	REG_DEF(pm.hall_ST, 5_X, [5].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 5_Y, [5].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 5, [5],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg_nolock, NULL),
	REG_DEF(pm.hall_ST, 6_X, [6].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 6_Y, [6].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 6, [6],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg_nolock, NULL),

	REG_DEF(pm.hall_INUSE,,,		"",	"%i",	REG_CONFIG, &reg_proc_INUSE, &reg_format_enum),
	REG_DEF(pm.hall_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg, NULL),
	REG_DEF(pm.hall_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.hall_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.hall_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.hall_prol_END,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hall_gain_PF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hall_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hall_gain_IF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.abi_INUSE,,,			"",	"%i",	REG_CONFIG, &reg_proc_INUSE, &reg_format_enum),
	REG_DEF(pm.abi_EPPR,,,			"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.abi_gear_ZS,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.abi_gear_ZQ,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.abi_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_Fg, NULL),
	REG_DEF(pm.abi_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.abi_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.abi_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.abi_gain_PF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.abi_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.abi_gain_IF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm, .debug_flux_EF_g,, "g", "%2f", REG_READ_ONLY, &reg_proc_debug_flux_EFg, NULL),
	REG_DEF(pm, .debug_hfi_EF_g,,  "g", "%2f", REG_READ_ONLY, &reg_proc_debug_hfi_EFg, NULL),

	REG_DEF(pm.const_fb_U,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.const_E,,,			"Wb",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_E, _kv,,	"rpm/v",	"%2f",	0, &reg_proc_E_kv, NULL),
	REG_DEF(pm.const_E, _nm,,		"Nm",	"%4g",	0, &reg_proc_E_nm, &reg_format_E_nm),
	REG_DEF(pm.const_R,,,			"Ohm",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_Zp,,,			"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_Ja,,,			"",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_Ja, _kgm2,,		"kgm2",	"%4g",	0, &reg_proc_kgm2, NULL),
	REG_DEF(pm.const_Ja, _kg,,		"kg",	"%4g",	0, &reg_proc_kg, NULL),
	REG_DEF(pm.const_im_L1,,,		"H",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_im_L2,,,		"H",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_im_B,,,		"g",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_im_R,,,		"Ohm",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_ld_S,,,		"mm",	"%3f",	REG_CONFIG, &reg_proc_mm, NULL),

	REG_DEF(pm.watt_wP_maximal,,,		"W",	"%1f",	REG_CONFIG, &reg_proc_watt, NULL),
	REG_DEF(pm.watt_iDC_maximal,,,		"A",	"%3f",	REG_CONFIG, &reg_proc_watt, NULL),
	REG_DEF(pm.watt_wP_reverse,,,		"W",	"%1f",	REG_CONFIG, &reg_proc_watt, NULL),
	REG_DEF(pm.watt_iDC_reverse,,,		"A",	"%3f",	REG_CONFIG, &reg_proc_watt, NULL),
	REG_DEF(pm.watt_dclink_HI,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.watt_dclink_LO,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.watt_lpf_D,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.watt_lpf_Q,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.watt_lpf_wP,,,		"W",	"%1f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.watt_gain_LP_F,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.watt_gain_LP_P,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.i_setpoint_current,,,	"A",	"%3f",	0, NULL, NULL),
	REG_DEF(pm.i_setpoint_current, _pc,,	"%",	"%2f",	0, &reg_proc_current_pc, NULL),
	REG_DEF(pm.i_maximal,,,			"A",	"%3f",	REG_CONFIG, &reg_proc_auto_maximal_current, NULL),
	REG_DEF(pm.i_reverse,,,			"A",	"%3f",	REG_CONFIG, &reg_proc_auto_maximal_current, NULL),
	REG_DEF(pm.i_derated_PCB,,,		"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.i_derated_WEAK,,,		"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.i_track_D,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.i_track_Q,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.i_slew_rate,,,		"A/s",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.i_tol_Z,,,			"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.i_gain_P,,,		"",	"%2e",	REG_CONFIG, &reg_proc_auto_loop_current, NULL),
	REG_DEF(pm.i_gain_I,,,		"",	"%2e",	REG_CONFIG, &reg_proc_auto_loop_current, NULL),

	REG_DEF(pm.weak_maximal,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.weak_D,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.weak_gain_EU,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.v_maximal,,,			"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.v_reverse,,,			"V",	"%3f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.s_setpoint_speed,,,	"rad/s",	"%2f",	0, NULL, NULL),
	REG_DEF(pm.s_setpoint_speed, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.s_setpoint_speed, _mmps,,	"mm/s",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.s_setpoint_speed, _kmh,,	"km/h",	"%1f",	0, &reg_proc_kmh, NULL),
	REG_DEF(pm.s_setpoint_speed, _pc,,	"%",	"%2f",	0, &reg_proc_rpm_pc, NULL),
	REG_DEF(pm.s_maximal,,,		"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_maximal, _rpm,,		"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.s_maximal, _mmps,,		"mm/s",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.s_maximal, _kmh,,		"km/h",	"%1f",	0, &reg_proc_kmh, NULL),
	REG_DEF(pm.s_reverse,,,		"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_reverse, _rpm,,		"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.s_reverse, _mmps,,		"mm/s",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.s_reverse, _kmh,,		"km/h",	"%1f",	0, &reg_proc_kmh, NULL),
	REG_DEF(pm.s_accel,,,		"rad/s2",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_accel, _rpm,,	"rpm/s",	"%1f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.s_accel, _kmh,,	"km/h/s",	"%1f",	0, &reg_proc_kmh, NULL),
	REG_DEF(pm.s_linspan,,,		"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_track,,,		"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_tol_Z,,,		"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_gain_P,,,		"",	"%2e",	REG_CONFIG, &reg_proc_auto_loop_speed, NULL),

	REG_DEF(pm.x_setpoint_location,,,	"rad",	"%3f",	0, NULL, NULL),
	REG_DEF(pm.x_setpoint_location, _g,,	"g",	"%2f",	0, &reg_proc_location_g, NULL),
	REG_DEF(pm.x_setpoint_location, _mm,,	"mm",	"%3f",	0, &reg_proc_location_mm, NULL),
	REG_DEF(pm.x_setpoint_speed,,,	"rad/s",	"%2f",	0, NULL, NULL),
	REG_DEF(pm.x_setpoint_speed, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.x_setpoint_speed, _mmps,,	"mm/s",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_discrepancy,,,		"rad",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.x_discrepancy, _mm,,		"mm",	"%3f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_discrepancy, _ep,,		"ep",	"%1f",	REG_READ_ONLY, &reg_proc_epps, NULL),
	REG_DEF(pm.x_tol_NEAR,,,		"rad",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_tol_NEAR, _mm,,		"mm",	"%3f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_tol_Z,,,			"rad",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_tol_Z, _mm,,		"mm",	"%3f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_tol_Z, _ep,,		"ep",	"%1f",	0, &reg_proc_epps, NULL),
	REG_DEF(pm.x_gain_P,,,			"",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_gain_P, _accel,,	"rad/s2",	"%1f",	0, &reg_proc_gain_accel, NULL),
	REG_DEF(pm.x_gain_P, _accel_mm,,"mm/s2",	"%1f",	0, &reg_proc_gain_accel_mm, NULL),
	REG_DEF(pm.x_gain_Z,,,			"",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.im_distance,,,		"m",	"%1f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.im_distance, _km,,		"km",	"%3f",	REG_READ_ONLY, &reg_proc_km, NULL),
	REG_DEF(pm.im_consumed_Wh,,,		"Wh",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.im_consumed_Ah,,,		"Ah",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.im_reverted_Wh,,,		"Wh",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.im_reverted_Ah,,,		"Ah",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.im_capacity_Ah,,,		"Ah",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.im_fuel_pc,,,		"%",	"%2f",	0, &reg_proc_im_fuel, NULL),

	REG_DEF(tlm.freq_grab_hz,,,		"Hz",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(tlm.freq_live_hz,,,		"Hz",	"%i",	REG_CONFIG, NULL, NULL),

	REG_DEF(tlm.reg_ID, 0, [0],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, 1, [1],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, 2, [2],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, 3, [3],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, 4, [4],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, 5, [5],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, 6, [6],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, 7, [7],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, 8, [8],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, 9, [9],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),

	{ NULL, "", 0, NULL, NULL, NULL }
};

void reg_getval(const reg_t *reg, void *lval)
{
	if (reg->proc != NULL) {

		reg->proc(reg, lval, NULL);
	}
	else {
		*(reg_val_t *) lval = *reg->link;
	}
}

void reg_setval(const reg_t *reg, const void *rval)
{
	if ((reg->mode & REG_READ_ONLY) == 0) {

		if (reg->proc != NULL) {

			reg->proc(reg, NULL, rval);
		}
		else {
			*reg->link = *(reg_val_t *) rval;
		}
	}
}

void reg_format_rval(const reg_t *reg, const void *rval)
{
	reg_val_t		*link = (reg_val_t *) rval;

	if (		   reg->fmt[1] == 'i'
			|| reg->fmt[2] == 'x') {

		printf(reg->fmt, link->i);
	}
	else {
		printf(reg->fmt, &link->f);
	}
}

void reg_format(const reg_t *reg)
{
	reg_val_t		rval;
	const char		*su;
	int			reg_ID;

	if (reg != NULL) {

		reg_ID = (int) (reg - regfile);

		printf("%c%c%c [%i] %s = ",
			(int) (reg->mode & REG_CONFIG) 		? 'C' : ' ',
			(int) (reg->mode & REG_READ_ONLY)	? 'R' : ' ',
			(int) (reg->mode & REG_LINKED)		? 'L' : ' ',
			(int) reg_ID, reg->sym);

		if (reg->format != NULL) {

			reg->format(reg);
		}
		else {
			reg_getval(reg, &rval);

			reg_format_rval(reg, &rval);

			if (reg->mode & REG_LINKED) {

				if (rval.i >= 0 && rval.i < REG_MAX) {

					printf(" (%s)", regfile[rval.i].sym);
				}
			}

			su = reg->sym + strlen(reg->sym) + 1;

			if (*su != 0) {

				printf(" (%s)", su);
			}
		}

		puts(EOL);
	}
}

const reg_t *reg_search(const char *sym)
{
	const reg_t		*reg, *found = NULL;

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (strcmp(reg->sym, sym) == 0) {

			found = reg;
			break;
		}
	}

	return found;
}

const reg_t *reg_search_fuzzy(const char *sym)
{
	const reg_t		*reg, *found = NULL;
	int			n;

	if (stoi(&n, sym) != NULL) {

		if (n >= 0 && n < REG_MAX)
			found = regfile + n;
	}
	else {
		for (reg = regfile; reg->sym != NULL; ++reg) {

			if (strcmp(reg->sym, sym) == 0) {

				found = reg;
				break;
			}
		}

		if (found == NULL && iodef_ECHO != 0) {

			for (reg = regfile; reg->sym != NULL; ++reg) {

				if (strstr(reg->sym, sym) != NULL) {

					if (found == NULL) {

						found = reg;
					}
					else {
						found = NULL;
						break;
					}
				}
			}
		}
	}

	return found;
}

void reg_GET(int reg_ID, void *lval)
{
	if (reg_ID >= 0 && reg_ID < REG_MAX) {

		reg_getval(regfile + reg_ID, lval);
	}
}

void reg_SET(int reg_ID, const void *rval)
{
	if (reg_ID >= 0 && reg_ID < REG_MAX) {

		reg_setval(regfile + reg_ID, rval);
	}
}

int reg_GET_I(int reg_ID)
{
	int		lval;

	reg_GET(reg_ID, &lval);

	return lval;
}

float reg_GET_F(int reg_ID)
{
	float		lval;

	reg_GET(reg_ID, &lval);

	return lval;
}

void reg_SET_I(int reg_ID, int rval)
{
	reg_SET(reg_ID, &rval);
}

void reg_SET_F(int reg_ID, float rval)
{
	reg_SET(reg_ID, &rval);
}

SH_DEF(reg)
{
	reg_val_t		rval;
	const reg_t		*reg, *lreg;

	reg = reg_search_fuzzy(s);

	if (reg != NULL) {

		s = sh_next_arg(s);

		if (		   reg->fmt[1] == 'i'
				|| reg->fmt[2] == 'x') {

			if (reg->mode & REG_LINKED) {

				lreg = reg_search_fuzzy(s);

				if (lreg != NULL) {

					rval.i = (int) (lreg - regfile);
					reg_setval(reg, &rval);
				}
			}
			else if (stoi(&rval.i, s) != NULL) {

				reg_setval(reg, &rval);
			}
			else if (htoi(&rval.i, s) != NULL) {

				reg_setval(reg, &rval);
			}
		}
		else {
			if (stof(&rval.f, s) != NULL) {

				reg_setval(reg, &rval);
			}
		}

		reg_format(reg);
	}
	else {
		for (reg = regfile; reg->sym != NULL; ++reg) {

			if (strstr(reg->sym, s) != NULL) {

				reg_format(reg);
			}
		}
	}
}

SH_DEF(export_reg)
{
	reg_val_t		rval;
	const reg_t		*reg;

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (reg->mode & REG_CONFIG) {

			printf("reg %s ", reg->sym);

			reg_getval(reg, &rval);

			if (reg->mode & REG_LINKED) {

				if (rval.i >= 0 && rval.i < REG_MAX) {

					puts(regfile[rval.i].sym);
				}
			}
			else {
				reg_format_rval(reg, &rval);
			}

			puts(EOL);
		}
	}
}

