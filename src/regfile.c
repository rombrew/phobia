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

#define REG_DEF(l, e, q, u, f, m, p, t)		{ #l #e "\0" u, f, m, \
						(reg_value_t * const) &(l q), \
						(void * const) p, (void * const) t}

#define REG_MAX					(sizeof(regfile) / sizeof(reg_t) - 1U)

static int		null;

static void
reg_proc_PWM(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	int			irq;

	if (lval != NULL) {

		lval->f = reg->link->f;
	}
	else if (rval != NULL) {

		if (pm.lu_MODE == PM_LU_DISABLED) {

			reg->link->f = rval->f;

			irq = hal_lock_irq();

			PWM_configure();

			pm.m_freq = hal.PWM_frequency;
			pm.m_dT = 1.f / pm.m_freq;
			pm.dc_resolution = hal.PWM_resolution;

			hal_unlock_irq(irq);
		}
	}
}

static void
reg_proc_ADC(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	int			irq;

	if (lval != NULL) {

		lval->f = reg->link->f;
	}
	else if (rval != NULL) {

		if (pm.lu_MODE == PM_LU_DISABLED) {

			reg->link->f = rval->f;

			irq = hal_lock_irq();

			ADC_const_build();

			hal_unlock_irq(irq);
		}
	}
}

#ifdef HW_HAVE_NETWORK_EPCAN
static void
reg_proc_net_rate(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = hal.PWM_frequency / (float) reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = (int) (hal.PWM_frequency / rval->f + .5f);
	}
}

static void
reg_proc_net_ID(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->i = reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = rval->i;

		hal_memory_fence();

		EPCAN_filter_ID();
	}
}

static void
reg_proc_net_timeout(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = 1000.f / hal.PWM_frequency * (float) reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = (int) (rval->f * hal.PWM_frequency / 1000.f + .5f);
	}
}
#endif /* HW_HAVE_NETWORK_EPCAN */

#undef APP_DEF
#define APP_DEF(name)		extern void app_ ## name(void *);
#include "app/apdefs.h"

void app_control(const reg_t *reg, void (* pvTask) (void *), const char *pcName);

static void
reg_proc_task(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	int			reg_ID;

	if (lval != NULL) {

		lval->i = reg->link->i;
	}
	else if (rval != NULL) {

		reg_ID = (int) (reg - regfile);

		reg->link->i = rval->i;

		switch (reg_ID) {

#undef APP_DEF
#define APP_DEF(name)		case ID_AP_TASK_ ## name: \
				app_control(reg, app_ ## name, PM_SFI(name)); \
				break;
#include "app/apdefs.h"

			default: break;
		}
	}
}

static void
reg_proc_PPM(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->i = reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = rval->i;

		hal_memory_fence();

		PPM_configure();
	}
}

static void
reg_proc_DPS(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->i = reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = rval->i;

		hal_memory_fence();

		DPS_configure();
	}
}

#ifdef HW_HAVE_DRV_ON_PCB
static void
reg_proc_DRV_part(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->i = reg->link->i;
	}
	else if (rval != NULL) {

		if (pm.lu_MODE == PM_LU_DISABLED) {

			reg->link->i = rval->i;

			hal_memory_fence();

			DRV_halt();
			DRV_startup();
		}
	}
}

static void
reg_proc_DRV_status(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		DRV_status();

		hal_memory_fence();

		lval->i = reg->link->i;
	}
}

static void
reg_proc_DRV_configure(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->i = reg->link->i;
	}
	else if (rval != NULL) {

		if (pm.lu_MODE == PM_LU_DISABLED) {

			reg->link->i = rval->i;

			hal_memory_fence();

			DRV_configure();
		}
	}
}
#endif /* HW_HAVE_DRV_ON_PCB */

static void
reg_proc_CNT_diag(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * hal.PWM_frequency * 100.f;
	}
}

static void
reg_proc_ppm_freq(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = (reg->link->f > M_EPS_F) ? 1.f / reg->link->f : 0.f;
	}
}

static void
reg_proc_rpm(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * (30.f / M_PI_F) / (float) pm.const_Zp;
	}
	else if (rval != NULL) {

		reg->link->f = rval->f * (M_PI_F / 30.f) * (float) pm.const_Zp;
	}
}

static void
reg_proc_mmps(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	reg_value_t		rpm;

	if (lval != NULL) {

		reg_proc_rpm(reg, &rpm, NULL);

		lval->f = rpm.f * pm.const_ld_S * (1000.f / 60.f);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			rpm.f = rval->f / pm.const_ld_S * (60.f / 1000.f);

			reg_proc_rpm(reg, NULL, &rpm);
		}
	}
}

static void
reg_proc_kmh(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	reg_value_t		rpm;

	if (lval != NULL) {

		reg_proc_rpm(reg, &rpm, NULL);

		lval->f = rpm.f * pm.const_ld_S * (3.6f / 60.f);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			rpm.f = rval->f / pm.const_ld_S * (60.f / 3.6f);

			reg_proc_rpm(reg, NULL, &rpm);
		}
	}
}

static void
reg_proc_rpm_pc(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	float			kPC = pm.k_EMAX / 100.f;

	if (lval != NULL) {

		lval->f = reg->link->f * pm.const_lambda / (kPC * pm.const_fb_U);
	}
	else if (rval != NULL) {

		if (pm.const_lambda > M_EPS_F) {

			reg->link->f = rval->f * kPC * pm.const_fb_U / pm.const_lambda;
		}
	}
}

static void
reg_proc_voltage(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * pm.const_lambda;
	}
	else if (rval != NULL) {

		if (pm.const_lambda > M_EPS_F) {

			reg->link->f = rval->f / pm.const_lambda;
		}
	}
}

static void
reg_proc_current_pc(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * 100.f / pm.i_maximal;
	}
	else if (rval != NULL) {

		reg->link->f = rval->f * pm.i_maximal / 100.f;
	}
}

static void
reg_proc_lambda_kv(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
        if (lval != NULL) {

                lval->f = 5.513289f / (reg->link->f * (float) pm.const_Zp);
        }
        else if (rval != NULL) {

                reg->link->f = 5.513289f / (rval->f * (float) pm.const_Zp);
        }
}

static void
reg_proc_lambda_nm(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	float			mQ;

	if (lval != NULL) {

		if (pm.config_RELUCTANCE == PM_ENABLED) {

			/* Unable to calculate torque constant in [Nm/A].
			 * */
			lval->f = 0.f;
		}
		else {
			mQ = pm_torque_equation(&pm, 0.f, 1.f);

			lval->f = mQ * (float) pm.const_Zp;
		}
	}
}

static void
reg_proc_kgm2(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	const float             Zp2 = (float) (pm.const_Zp * pm.const_Zp);

	if (lval != NULL) {

		lval->f = reg->link->f * Zp2;
	}
	else if (rval != NULL) {

		if (pm.const_lambda > M_EPS_F) {

			reg->link->f = rval->f / Zp2;
		}
	}
}

static void
reg_proc_kg(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	const float             Zp2 = (float) (pm.const_Zp * pm.const_Zp);

	const float		lR = pm.const_ld_S / (2.f * M_PI_F);
	const float		lQ = lR * lR;

	if (lval != NULL) {

		if (lQ > M_EPS_F) {

			lval->f = reg->link->f * Zp2 / lQ;
		}
		else {
			lval->f = 0.f;
		}
	}
	else if (rval != NULL) {

		reg->link->f = rval->f * lQ / Zp2;
	}
}

static void
reg_proc_load_nm(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * (float) pm.const_Zp;
	}
	else if (rval != NULL) {

		if (pm.const_lambda > M_EPS_F) {

			reg->link->f = rval->f / (float) pm.const_Zp;
		}
	}
}

static void
reg_proc_auto_maximal_current(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f;
	}
	else if (rval != NULL) {

		if (rval->f < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_MAXIMAL_CURRENT);
		}
		else {
			reg->link->f = rval->f;
		}
	}
}

static void
reg_proc_auto_probe_speed_hold(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f;
	}
	else if (rval != NULL) {

		if (rval->f < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_PROBE_SPEED_HOLD);
		}
		else {
			reg->link->f = rval->f;
		}
	}
}

static void
reg_proc_auto_zone_threshold(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f;
	}
	else if (rval != NULL) {

		if (rval->f < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_ZONE_THRESHOLD);
		}
		else {
			reg->link->f = rval->f;
		}
	}
}

static void
reg_proc_auto_forced_maximal(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f;
	}
	else if (rval != NULL) {

		if (rval->f < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_FORCED_MAXIMAL);
		}
		else {
			reg->link->f = rval->f;
		}
	}
}

static void
reg_proc_auto_forced_accel(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f;
	}
	else if (rval != NULL) {

		if (rval->f < M_EPS_F) {

			pm_auto(&pm, PM_AUTO_FORCED_ACCEL);
		}
		else {
			reg->link->f = rval->f;
		}
	}
}

static void
reg_proc_current_halt(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	float			halt, adjust;

	if (lval != NULL) {

		lval->f = reg->link->f;
	}
	else if (rval != NULL) {

		if (rval->f < M_EPS_F) {

			halt = m_fabsf(hal.const_ADC.GA * ADC_RESOLUTION / 2.f);

			adjust = (pm.scale_iA[1] < pm.scale_iB[1])
				? pm.scale_iA[1] : pm.scale_iB[1];

			halt *= adjust;

			reg->link->f = (float) (int) (halt * .95f);
		}
		else {
			reg->link->f = rval->f;
		}
	}
}

static void
reg_proc_percent(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * 100.f;
	}
	else if (rval != NULL) {

		reg->link->f = rval->f / 100.f;
	}
}

static void
reg_proc_auto_loop_current(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * 100.f;
	}
	else if (rval != NULL) {

		reg->link->f = rval->f / 100.f;

		hal_memory_fence();

		pm_auto(&pm, PM_AUTO_LOOP_CURRENT);
	}
}

static void
reg_proc_auto_loop_speed(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * 100.f;
	}
	else if (rval != NULL) {

		reg->link->f = rval->f / 100.f;

		hal_memory_fence();

		pm_auto(&pm, PM_AUTO_LOOP_SPEED);
	}
}

static void
reg_proc_wattage(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f;
	}
	else if (rval != NULL) {

		reg->link->f = m_fabsf(rval->f);
	}
}

static void
reg_proc_USEABLE(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->i = reg->link->i;
	}
	else if (rval != NULL) {

		if (rval->i != PM_ENABLED) {

			reg->link->i = rval->i;
		}
	}
}

static void
reg_proc_fpos_g(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	float			fcos, fsin, *fpos = (void *) reg->link;
	int			irq;

	if (lval != NULL) {

		irq = hal_lock_irq();

		fcos = fpos[0];
		fsin = fpos[1];

		hal_unlock_irq(irq);

		lval->f = m_atan2f(fsin, fcos) * (180.f / M_PI_F);
	}
}

static void
reg_proc_fpos_nolock_g(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	float			*fpos = (void *) reg->link;

	if (lval != NULL) {

		lval->f = m_atan2f(fpos[1], fpos[0]) * (180.f / M_PI_F);
	}
}

static void
reg_proc_location_g(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * (180.f / M_PI_F) / (float) pm.const_Zp;
	}
	else if (rval != NULL) {

		reg->link->f = rval->f * (M_PI_F / 180.f) * (float) pm.const_Zp;
	}
}

static void
reg_proc_location_mm(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * pm.const_ld_S * 1000.f
			/ (2.f * M_PI_F * (float) pm.const_Zp);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			reg->link->f = rval->f * (2.f * M_PI_F) * (float) pm.const_Zp
				/ (pm.const_ld_S * 1000.f);
		}
	}
}

static void
reg_proc_km(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f / 1000.f;
	}
	else if (rval != NULL) {

		reg->link->f = rval->f * 1000.f;
	}
}

static void
reg_proc_mm(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	if (lval != NULL) {

		lval->f = reg->link->f * 1000.f;
	}
	else if (rval != NULL) {

		reg->link->f = rval->f / 1000.f;
	}
}

static void
reg_proc_x_accel(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
        if (lval != NULL) {

                lval->f = reg->link->f * reg->link->f / 2.f;
        }
        else if (rval != NULL) {

                reg->link->f = m_sqrtf(rval->f * 2.f);
        }
}

static void
reg_proc_x_accel_mm(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	reg_value_t		rad;

	if (lval != NULL) {

		reg_proc_x_accel(reg, &rad, NULL);

		lval->f = rad.f * pm.const_ld_S * 1000.f
			/ (2.f * M_PI_F * (float) pm.const_Zp);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			rad.f = rval->f * (2.f * M_PI_F * (float) pm.const_Zp)
				/ (pm.const_ld_S * 1000.f);

			reg_proc_x_accel(reg, NULL, &rad);
		}
	}
}

static void
reg_proc_tvm_FIR_tau(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	float		tau, *FIR = (void *) reg->link;

	if (lval != NULL) {

		tau = FIR[0] / - FIR[1];
		tau = (tau > M_EPS_F) ? pm.m_dT * 1000000.f / m_logf(tau) : 0.f;

		lval->f = tau;
	}
}

static void
reg_proc_im_fuel(const reg_t *reg, reg_value_t *lval, const reg_value_t *rval)
{
	int			irq;

	if (lval != NULL) {

		lval->f = reg->link->f;
	}
	else if (rval != NULL) {

		if (rval->f < M_EPS_F) {

			irq = hal_lock_irq();

			pm.mi_consumed_Wh = 0.f;
			pm.mi_consumed_Ah = 0.f;
			pm.mi_reverted_Wh = 0.f;
			pm.mi_reverted_Ah = 0.f;

			pm.mi_rem[0] = 0.f;
			pm.mi_rem[1] = 0.f;
			pm.mi_rem[2] = 0.f;
			pm.mi_rem[3] = 0.f;

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

#undef PM_SFI_CASE
#define PM_SFI_CASE(val)	case val: printf("(%s)", PM_SFI(val)); break

static void
reg_format_enum(const reg_t *reg)
{
	int			reg_ID, val;

	reg_ID = (int) (reg - regfile);
	val = reg->link->i;

	printf("%i ", val);

	switch (reg_ID) {

		case ID_HAL_ADC_SAMPLE_TIME:

			switch (val) {

				PM_SFI_CASE(ADC_SMP_3);
				PM_SFI_CASE(ADC_SMP_15);
				PM_SFI_CASE(ADC_SMP_28);
				PM_SFI_CASE(ADC_SMP_56);

				default: break;
			}
			break;

		case ID_HAL_DPS_MODE:

			switch (val) {

				PM_SFI_CASE(DPS_DISABLED);
				PM_SFI_CASE(DPS_DRIVE_HALL);
				PM_SFI_CASE(DPS_DRIVE_EABI);
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
				PM_SFI_CASE(PPM_BACKUP_EABI);

				default: break;
			}
			break;

#ifdef HW_HAVE_DRV_ON_PCB
		case ID_HAL_DRV_PART:

			switch (val) {

				PM_SFI_CASE(DRV_NONE);
				PM_SFI_CASE(DRV_PART_DRV8303);
				PM_SFI_CASE(DRV_PART_DRV8305);

				default: break;
			}
			break;

		case ID_HAL_DRV_AUTO_RESET:

			switch (val) {

				PM_SFI_CASE(PM_DISABLED);
				PM_SFI_CASE(PM_ENABLED);

				default: break;
			}
			break;
#endif /* HW_HAVE_DRV_ON_PCB */

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
				PM_SFI_CASE(EPCAN_PAYLOAD_PACKED);

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
				PM_SFI_CASE(NTC_CMOS);

				default: break;
			}
			break;

#undef APP_DEF
#define APP_DEF(name)		case ID_AP_TASK_ ## name:
#include "app/apdefs.h"

		switch (val) {

			PM_SFI_CASE(PM_DISABLED);
			PM_SFI_CASE(PM_ENABLED);

			default: break;
		}
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
		case ID_PM_CONFIG_VSI_CLAMP:
		case ID_PM_CONFIG_VSI_STRICT:
		case ID_PM_CONFIG_LU_FORCED:
		case ID_PM_CONFIG_HFI_POLARITY:
		case ID_PM_CONFIG_RELUCTANCE:
		case ID_PM_CONFIG_WEAKENING:
		case ID_PM_CONFIG_HOLDING_BRAKE:
		case ID_PM_CONFIG_SPEED_LIMITED:
		case ID_PM_CONFIG_MILEAGE_INFO:

			switch (val) {

				PM_SFI_CASE(PM_DISABLED);
				PM_SFI_CASE(PM_ENABLED);

				default: break;
			}
			break;

		case ID_PM_CONFIG_VSI_ZERO:

			switch (val) {

				PM_SFI_CASE(PM_VSI_GND);
				PM_SFI_CASE(PM_VSI_CENTER);
				PM_SFI_CASE(PM_VSI_EXTREME);

				default: break;
			}
			break;

		case ID_PM_CONFIG_LU_ESTIMATE:

			switch (val) {

				PM_SFI_CASE(PM_FLUX_NONE);
				PM_SFI_CASE(PM_FLUX_ORTEGA);
				PM_SFI_CASE(PM_FLUX_KALMAN);

				default: break;
			}
			break;

		case ID_PM_CONFIG_LU_SENSOR:

			switch (val) {

				PM_SFI_CASE(PM_SENSOR_NONE);
				PM_SFI_CASE(PM_SENSOR_HALL);
				PM_SFI_CASE(PM_SENSOR_EABI);
				PM_SFI_CASE(PM_SENSOR_SINCOS);

				default: break;
			}
			break;

		case ID_PM_CONFIG_LU_LOCATION:

			switch (val) {

				PM_SFI_CASE(PM_LOCATION_INHERITED);
				PM_SFI_CASE(PM_LOCATION_EABI);
				PM_SFI_CASE(PM_LOCATION_SINCOS);

				default: break;
			}
			break;

		case ID_PM_CONFIG_LU_DRIVE:

			switch (val) {

				PM_SFI_CASE(PM_DRIVE_CURRENT);
				PM_SFI_CASE(PM_DRIVE_SPEED);
				PM_SFI_CASE(PM_DRIVE_LOCATION);

				default: break;
			}
			break;

		case ID_PM_CONFIG_HFI_WAVE:

			switch (val) {

				PM_SFI_CASE(PM_HFI_NONE);
				PM_SFI_CASE(PM_HFI_SINE);
				PM_SFI_CASE(PM_HFI_RANDOM);

				default: break;
			}
			break;

		case ID_PM_CONFIG_SALIENCY:

			switch (val) {

				PM_SFI_CASE(PM_SALIENCY_NEGATIVE);
				PM_SFI_CASE(PM_SALIENCY_POSITIVE);

				default: break;
			}
			break;

		case ID_PM_CONFIG_EABI_FRONTEND:

			switch (val) {

				PM_SFI_CASE(PM_EABI_INCREMENTAL);
				PM_SFI_CASE(PM_EABI_ABSOLUTE);

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

		case ID_PM_VSI_MASK_XF:

			switch (val) {

				PM_SFI_CASE(PM_MASK_NONE);
				PM_SFI_CASE(PM_MASK_A);
				PM_SFI_CASE(PM_MASK_B);
				PM_SFI_CASE(PM_MASK_C);

				default: break;
			}
			break;

		case ID_PM_TVM_USEABLE:
		case ID_PM_HALL_USEABLE:
		case ID_PM_EABI_USEABLE:

			switch (val) {

				PM_SFI_CASE(PM_DISABLED);
				PM_SFI_CASE(PM_ENABLED);

				default: break;
			}
			break;

		case ID_PM_LU_MODE:

			switch (val) {

				PM_SFI_CASE(PM_LU_DISABLED);
				PM_SFI_CASE(PM_LU_DETACHED);
				PM_SFI_CASE(PM_LU_FORCED);
				PM_SFI_CASE(PM_LU_ESTIMATE);
				PM_SFI_CASE(PM_LU_ON_HFI);
				PM_SFI_CASE(PM_LU_SENSOR_HALL);
				PM_SFI_CASE(PM_LU_SENSOR_EABI);
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

		case ID_TLM_MODE:

			switch (val) {

				PM_SFI_CASE(TLM_MODE_DISABLED);
				PM_SFI_CASE(TLM_MODE_GRAB);
				PM_SFI_CASE(TLM_MODE_WATCH);
				PM_SFI_CASE(TLM_MODE_LIVE);

				default: break;
			}
			break;

		default: break;
	}
}

const reg_t		regfile[] = {

	REG_DEF(null,,,				"",	"%i",	REG_READ_ONLY, NULL, NULL),

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

	REG_DEF(hal.ADC_sample_time,,,		"",	"%i",	REG_CONFIG, &reg_proc_ADC, &reg_format_enum),
	REG_DEF(hal.ADC_sample_advance,,,	"",	"%i",	REG_CONFIG, &reg_proc_PWM, NULL),

	REG_DEF(hal.DPS_mode,,,		"",	"%i",	REG_CONFIG, &reg_proc_DPS, &reg_format_enum),
	REG_DEF(hal.PPM_mode,,,		"",	"%i",	REG_CONFIG, &reg_proc_PPM, &reg_format_enum),
	REG_DEF(hal.PPM_frequency,,,	"Hz",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(hal.PPM_caught,,,	"",	"%i",	REG_READ_ONLY, NULL, NULL),

#ifdef HW_HAVE_DRV_ON_PCB
	REG_DEF(hal.DRV.part,,,		"",	"%i",	REG_CONFIG, &reg_proc_DRV_part, &reg_format_enum),
	REG_DEF(hal.DRV.auto_RESET,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(hal.DRV.status_raw,,,	"",	"%4x",	REG_READ_ONLY, &reg_proc_DRV_status, NULL),
	REG_DEF(hal.DRV.gate_current,,,	"",	"%i",	REG_CONFIG, &reg_proc_DRV_configure, NULL),
	REG_DEF(hal.DRV.ocp_level,,,	"",	"%i",	REG_CONFIG, &reg_proc_DRV_configure, NULL),
#endif /* HW_HAVE_DRV_ON_PCB */

	REG_DEF(hal.CNT_diag, 0, [0],	"%",	"%1f",	REG_READ_ONLY, &reg_proc_CNT_diag, NULL),
	REG_DEF(hal.CNT_diag, 1, [1],	"%",	"%1f",	REG_READ_ONLY, &reg_proc_CNT_diag, NULL),
	REG_DEF(hal.CNT_diag, 2, [2],	"%",	"%1f",	REG_READ_ONLY, &reg_proc_CNT_diag, NULL),

#ifdef HW_HAVE_NETWORK_EPCAN
	REG_DEF(net.node_ID,,,		"",	"%i",	REG_CONFIG, &reg_proc_net_ID, NULL),
	REG_DEF(net.log_MODE,,,		"",	"%i",	REG_CONFIG, &reg_proc_net_ID, &reg_format_enum),
	REG_DEF(net.timeout_EP,,,	"ms",	"%1f",	REG_CONFIG, &reg_proc_net_timeout, NULL),

	REG_DEF(net.ep, 0_MODE, [0].MODE,"",		"%i",	REG_CONFIG, &reg_proc_net_ID, &reg_format_enum),
	REG_DEF(net.ep, 0_ID, [0].ID,"",		"%i",	REG_CONFIG, &reg_proc_net_ID, NULL),
	REG_DEF(net.ep, 0_clock_ID, [0].clock_ID,"",	"%i",	REG_CONFIG, &reg_proc_net_ID, NULL),
	REG_DEF(net.ep, 0_reg_DATA, [0].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(net.ep, 0_reg_ID, [0].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(net.ep, 0_PAYLOAD, [0].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 0_STARTUP, [0].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 0_rate, [0].rate, "Hz",		"%1f",	REG_CONFIG, &reg_proc_net_rate, NULL),
	REG_DEF(net.ep, 0_range0, [0].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(net.ep, 0_range1, [0].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(net.ep, 1_MODE, [1].MODE,"",		"%i",	REG_CONFIG, &reg_proc_net_ID, &reg_format_enum),
	REG_DEF(net.ep, 1_ID, [1].ID,"",		"%i",	REG_CONFIG, &reg_proc_net_ID, NULL),
	REG_DEF(net.ep, 1_clock_ID, [1].clock_ID,"",	"%i",	REG_CONFIG, &reg_proc_net_ID, NULL),
	REG_DEF(net.ep, 1_reg_DATA, [1].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(net.ep, 1_reg_ID, [1].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(net.ep, 1_PAYLOAD, [1].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 1_STARTUP, [1].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 1_rate, [1].rate, "Hz",		"%1f",	REG_CONFIG, &reg_proc_net_rate, NULL),
	REG_DEF(net.ep, 1_range0, [1].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(net.ep, 1_range1, [1].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(net.ep, 2_MODE, [2].MODE,"",		"%i",	REG_CONFIG, &reg_proc_net_ID, &reg_format_enum),
	REG_DEF(net.ep, 2_ID, [2].ID,"",		"%i",	REG_CONFIG, &reg_proc_net_ID, NULL),
	REG_DEF(net.ep, 2_clock_ID, [2].clock_ID,"",	"%i",	REG_CONFIG, &reg_proc_net_ID, NULL),
	REG_DEF(net.ep, 2_reg_DATA, [2].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(net.ep, 2_reg_ID, [2].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(net.ep, 2_PAYLOAD, [2].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 2_STARTUP, [2].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 2_rate, [2].rate, "Hz",		"%1f",	REG_CONFIG, &reg_proc_net_rate, NULL),
	REG_DEF(net.ep, 2_range0, [2].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(net.ep, 2_range1, [2].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(net.ep, 3_MODE, [3].MODE,"",		"%i",	REG_CONFIG, &reg_proc_net_ID, &reg_format_enum),
	REG_DEF(net.ep, 3_ID, [3].ID,"",		"%i",	REG_CONFIG, &reg_proc_net_ID, NULL),
	REG_DEF(net.ep, 3_clock_ID, [3].clock_ID,"",	"%i",	REG_CONFIG, &reg_proc_net_ID, NULL),
	REG_DEF(net.ep, 3_reg_DATA, [3].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(net.ep, 3_reg_ID, [3].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(net.ep, 3_PAYLOAD, [3].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 3_STARTUP, [3].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(net.ep, 3_rate, [3].rate, "Hz",		"%1f",	REG_CONFIG, &reg_proc_net_rate, NULL),
	REG_DEF(net.ep, 3_range0, [3].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(net.ep, 3_range1, [3].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),
#endif /* HW_HAVE_NETWORK_EPCAN */

	REG_DEF(ap.ppm_in_pulse,,,		"ms",	"%4f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(ap.ppm_in_freq,,,		"Hz",	"%1f",	REG_READ_ONLY, &reg_proc_ppm_freq, NULL),
	REG_DEF(ap.ppm_reg_DATA,,,		"",	"%2f",	0, NULL, &reg_format_referenced_ppm),
	REG_DEF(ap.ppm_reg_ID,,,		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(ap.ppm_STARTUP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.ppm_range_pulse, 0, [0],	"ms",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_range_pulse, 1, [1],	"ms",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_range_pulse, 2, [2],	"ms",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_range_control, 0, [0],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_ppm),
	REG_DEF(ap.ppm_range_control, 1, [1],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_ppm),
	REG_DEF(ap.ppm_range_control, 2, [2],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_ppm),

	REG_DEF(ap.step_reg_DATA,,,		"",	"%2f",	0, NULL, NULL),
	REG_DEF(ap.step_reg_ID,,,		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(ap.step_STARTUP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.step_accu_EP,,,		"",	"%i",	0, NULL, NULL),
	REG_DEF(ap.step_const_ld_EP,,,		"mm",	"%3f",	REG_CONFIG, NULL, NULL),

#ifdef HW_HAVE_ANALOG_KNOB
	REG_DEF(ap.knob_in_ANG,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(ap.knob_in_BRK,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(ap.knob_reg_DATA,,,		"",	"%2f",	0, NULL, &reg_format_referenced_knob),
	REG_DEF(ap.knob_reg_ID,,,		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(ap.knob_ENABLED,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.knob_STARTUP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.knob_range_ANG, 0, [0],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_range_ANG, 1, [1],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_range_ANG, 2, [2],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_range_BRK, 0, [0],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_range_BRK, 1, [1],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_range_LST, 0, [0],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_range_LST, 1, [1],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.knob_control_ANG, 0, [0],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_knob),
	REG_DEF(ap.knob_control_ANG, 1, [1],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_knob),
	REG_DEF(ap.knob_control_ANG, 2, [2],	"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_knob),
	REG_DEF(ap.knob_control_BRK,,,		"",	"%2f",	REG_CONFIG, NULL, &reg_format_referenced_knob),
#endif /* HW_HAVE_ANALOG_KNOB */

	REG_DEF(ap.idle_TIME,,,			"s",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.disarm_TIME,,,		"s",	"%1f",	REG_CONFIG, NULL, NULL),

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
	REG_DEF(ap.temp_MCU,,,			"C",	"%1f",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(ap.tpro_PCB_temp_halt,,,	"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_PCB_temp_derate,,,	"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_PCB_temp_FAN,,,		"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_EXT_temp_derate,,,	"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_derated_PCB,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_derated_EXT,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.tpro_temp_recovery,,,	"C",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.task_PUSHBUTTON,,,		"",	"%i",	REG_CONFIG, &reg_proc_task, &reg_format_enum),
	REG_DEF(ap.task_AS5047,,,		"",	"%i",	REG_CONFIG, &reg_proc_task, &reg_format_enum),
	REG_DEF(ap.task_HX711,,,		"",	"%i",	REG_CONFIG, &reg_proc_task, &reg_format_enum),
	REG_DEF(ap.task_MPU6050,,,		"",	"%i",	REG_CONFIG, &reg_proc_task, &reg_format_enum),

	REG_DEF(ap.adc_load_kg,,,		"kg",	"%4f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(ap.adc_load_scale, 0, [0],	"kg",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.adc_load_scale, 1, [1],	"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.dc_resolution,,,		"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.dc_minimal,,,		"us",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_clearance,,,		"us",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_skip,,,			"us",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_bootstrap,,,		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_clamped,,,		"s",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.self_BST,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_BST),
	REG_DEF(pm.self_BM,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_BM),
	REG_DEF(pm.self_STDi,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_RMSi),
	REG_DEF(pm.self_RMSi,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_RMSi),
	REG_DEF(pm.self_RMSu,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_RMSu),

	REG_DEF(pm.config_NOP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_IFB,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_TVM,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),

	REG_DEF(pm.config_VSI_ZERO,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_VSI_CLAMP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_VSI_STRICT,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_FORCED,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_ESTIMATE,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_SENSOR,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_LOCATION,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_LU_DRIVE,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_HFI_WAVE,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_HFI_POLARITY,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_SALIENCY,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_RELUCTANCE,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_WEAKENING,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_HOLDING_BRAKE,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_SPEED_LIMITED,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_EABI_FRONTEND,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_SINCOS_FRONTEND,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_MILEAGE_INFO,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_BOOST_CHARGE,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),

	REG_DEF(pm.fsm_req,,,			"",	"%i",	0, NULL, NULL),
	REG_DEF(pm.fsm_state,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
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

	REG_DEF(pm.scale_iA, 0, [0],		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_iA, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_iB, 0, [0],		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_iB, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_iC, 0, [0],		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_iC, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_uS, 0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_uS, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_uA, 0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_uA, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_uB, 0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_uB, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_uC, 0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.scale_uC, 1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),

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
	REG_DEF(pm.probe_freq_sine,,,		"Hz",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_speed_hold,,,		"rad/s","%2f",	REG_CONFIG, &reg_proc_auto_probe_speed_hold, NULL),
	REG_DEF(pm.probe_speed_hold, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.probe_speed_detached,,,	"rad/s","%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_speed_detached, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.probe_speed_tol,,,		"rad/s","%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_speed_tol, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.probe_location_tol,,,	"rad",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_location_tol, _mm,,	"mm",	"%3f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.probe_gain_P,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_gain_I,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.auto_loop_current,,,		"%",	"%1f",	REG_CONFIG, &reg_proc_auto_loop_current, NULL),
	REG_DEF(pm.auto_loop_speed,,,		"%",	"%1f",	REG_CONFIG, &reg_proc_auto_loop_speed, NULL),

	REG_DEF(pm.fault_voltage_tol,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.fault_current_tol,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.fault_accuracy_tol,,,	"%",	"%1f",	REG_CONFIG, &reg_proc_percent, NULL),
	REG_DEF(pm.fault_terminal_tol,,,	"V",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.fault_current_halt,,,	"A",	"%3f",	REG_CONFIG, &reg_proc_current_halt, NULL),
	REG_DEF(pm.fault_voltage_halt,,,	"V",	"%3f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.vsi_DC,,,			"",	"%2f",	REG_READ_ONLY, &reg_proc_percent, NULL),
	REG_DEF(pm.vsi_lpf_DC,,,		"",	"%2f",	REG_READ_ONLY, &reg_proc_percent, NULL),
	REG_DEF(pm.vsi_gain_LP,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.vsi_mask_XF,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),

	REG_DEF(pm.vsi_X,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_Y,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_AF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_BF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_CF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_IF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_SF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_UF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_AQ,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_BQ,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_CQ,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(pm.tvm_USEABLE,,,		"",	"%i",	REG_CONFIG, &reg_proc_USEABLE, &reg_format_enum),
	REG_DEF(pm.tvm_clean_zone,,,		"%",	"%1f",	REG_CONFIG, &reg_proc_percent, NULL),
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
	REG_DEF(pm.lu_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_fpos_g, NULL),
	REG_DEF(pm.lu_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.lu_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.lu_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.lu_location,,,		"rad",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_location, _g,,		"g",	"%2f",	REG_READ_ONLY, &reg_proc_location_g, NULL),
	REG_DEF(pm.lu_location, _mm,,		"mm",	"%3f",	REG_READ_ONLY, &reg_proc_location_mm, NULL),
	REG_DEF(pm.lu_total_revol,,,		"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_rate,,,		"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.lu_mq_load,,,		"Nm",	"%3f",	REG_READ_ONLY, &reg_proc_load_nm, NULL),
	REG_DEF(pm.lu_gain_mq_LP,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.forced_hold_D,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_maximal,,,	"rad/s",	"%2f",	REG_CONFIG, &reg_proc_auto_forced_maximal, NULL),
	REG_DEF(pm.forced_maximal, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.forced_reverse,,,	"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_reverse, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.forced_accel,,,	"rad/s2",	"%1f",	REG_CONFIG, &reg_proc_auto_forced_accel, NULL),
	REG_DEF(pm.forced_accel, _rpm,,	"rpm/s",	"%1f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.forced_accel, _mmps,,"mm/s2",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.forced_slew_rate,,,		"A/s",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_maximal_DC,,,		"%",	"%1f",	REG_CONFIG, &reg_proc_percent, NULL),

	REG_DEF(pm.detach_voltage,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.detach_trip_AP,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.detach_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.flux_ZONE,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_enum),
	REG_DEF(pm.flux_lambda,,,		"Wb",	"%4g",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.flux_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_fpos_g, NULL),
	REG_DEF(pm.flux_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.flux_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.flux_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.flux_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.flux_trip_AP,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_IN,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_LO,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_HI,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_IF,,,		"%",	"%1f",	REG_CONFIG, &reg_proc_percent, NULL),

	REG_DEF(pm.kalman_bias_Q,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.kalman_lpf_wS,,,	"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.kalman_gain_Q, 0, [0],	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.kalman_gain_Q, 1, [1],	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.kalman_gain_Q, 2, [2],	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.kalman_gain_Q, 3, [3],	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.kalman_gain_Q, 4, [4],	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.kalman_gain_R,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.zone_speed_noise,,, 	"rad/s", 	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.zone_speed_noise, _v,,	"V",	"%3f",	0, &reg_proc_voltage, NULL),
	REG_DEF(pm.zone_speed_threshold,,, "rad/s",	"%2f",	REG_CONFIG, &reg_proc_auto_zone_threshold, NULL),
	REG_DEF(pm.zone_speed_threshold, _v,, "V",	"%3f",	0, &reg_proc_voltage, NULL),
	REG_DEF(pm.zone_lpf_wS,,,	"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.zone_gain_TH,,,		"%",	"%1f",	REG_CONFIG, &reg_proc_percent, NULL),
	REG_DEF(pm.zone_gain_LP,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.hfi_freq,,,			"Hz",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_sine,,,			"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_pole,,,			"",	"%4e",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hfi_gain_DP,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.hall_ST, 1_X, [1].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 1_Y, [1].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 1, [1],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_fpos_nolock_g, NULL),
	REG_DEF(pm.hall_ST, 2_X, [2].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 2_Y, [2].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 2, [2],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_fpos_nolock_g, NULL),
	REG_DEF(pm.hall_ST, 3_X, [3].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 3_Y, [3].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 3, [3],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_fpos_nolock_g, NULL),
	REG_DEF(pm.hall_ST, 4_X, [4].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 4_Y, [4].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 4, [4],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_fpos_nolock_g, NULL),
	REG_DEF(pm.hall_ST, 5_X, [5].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 5_Y, [5].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 5, [5],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_fpos_nolock_g, NULL),
	REG_DEF(pm.hall_ST, 6_X, [6].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 6_Y, [6].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, 6, [6],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_fpos_nolock_g, NULL),

	REG_DEF(pm.hall_USEABLE,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.hall_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_fpos_g, NULL),
	REG_DEF(pm.hall_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.hall_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.hall_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.hall_trip_AP,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hall_gain_LO,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hall_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hall_gain_IF,,,		"%",	"%1f",	REG_CONFIG, &reg_proc_percent, NULL),

	REG_DEF(pm.eabi_USEABLE,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.eabi_EPPR,,,			"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.eabi_gear_Zs,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.eabi_gear_Zq,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.eabi_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_fpos_g, NULL),
	REG_DEF(pm.eabi_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.eabi_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.eabi_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.eabi_trip_AP,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.eabi_gain_LO,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.eabi_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.eabi_gain_IF,,,		"%",	"%1f",	REG_CONFIG, &reg_proc_percent, NULL),

	REG_DEF(pm.const_fb_U,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.const_lambda,,,		"Wb",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_lambda, _kv,,	"rpm/V",	"%2f",	0, &reg_proc_lambda_kv, NULL),
	REG_DEF(pm.const_lambda, _nm,,		"Nm/A",	"%4g",	0, &reg_proc_lambda_nm, NULL),
	REG_DEF(pm.const_Rs,,,			"Ohm",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_Zp,,,			"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_Ja,,,		"ekgm2",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_Ja, _kgm2,,		"kgm2",	"%4g",	0, &reg_proc_kgm2, NULL),
	REG_DEF(pm.const_Ja, _kg,,		"kg",	"%4g",	0, &reg_proc_kg, NULL),
	REG_DEF(pm.const_im_L1,,,		"H",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_im_L2,,,		"H",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_im_B,,,		"g",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_im_R,,,		"Ohm",	"%4g",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_ld_S,,,		"mm",	"%3f",	REG_CONFIG, &reg_proc_mm, NULL),

	REG_DEF(pm.watt_wP_maximal,,,		"W",	"%1f",	REG_CONFIG, &reg_proc_wattage, NULL),
	REG_DEF(pm.watt_wA_maximal,,,		"A",	"%3f",	REG_CONFIG, &reg_proc_wattage, NULL),
	REG_DEF(pm.watt_wP_reverse,,,		"W",	"%1f",	REG_CONFIG, &reg_proc_wattage, NULL),
	REG_DEF(pm.watt_wA_reverse,,,		"A",	"%3f",	REG_CONFIG, &reg_proc_wattage, NULL),
	REG_DEF(pm.watt_uDC_maximal,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.watt_uDC_minimal,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.watt_lpf_D,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.watt_lpf_Q,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.watt_drain_wP,,,		"W",	"%1f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.watt_drain_wA,,,		"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.watt_gain_LP,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.i_derate_on_HFI,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.i_setpoint_current,,,	"A",	"%3f",	0, NULL, NULL),
	REG_DEF(pm.i_setpoint_current, _pc,,	"%",	"%2f",	0, &reg_proc_current_pc, NULL),
	REG_DEF(pm.i_maximal,,,			"A",	"%3f",	REG_CONFIG, &reg_proc_auto_maximal_current, NULL),
	REG_DEF(pm.i_reverse,,,			"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.i_track_D,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.i_track_Q,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.i_slew_rate,,,		"A/s",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.i_gain_P,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.i_gain_I,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),

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
	REG_DEF(pm.s_track,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.s_accel,,,		"rad/s2",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_accel, _rpm,,	"rpm/s",	"%1f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.s_accel, _kmh,,	"km/h/s",	"%1f",	0, &reg_proc_kmh, NULL),
	REG_DEF(pm.s_gain_P,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_gain_D,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.l_track_tol,,,	"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.l_blend,,,			"",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.l_gain_LP,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.x_location_range, 0, [0],	"rad",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_location_range, 0_g, [0],	"g",	"%2f",	0, &reg_proc_location_g, NULL),
	REG_DEF(pm.x_location_range, 0_mm, [0],	"mm",	"%3f",	0, &reg_proc_location_mm, NULL),
	REG_DEF(pm.x_location_range, 1, [1],	"rad",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_location_range, 1_g, [1],	"g",	"%2f",	0, &reg_proc_location_g, NULL),
	REG_DEF(pm.x_location_range, 1_mm, [1],	"mm",	"%3f",	0, &reg_proc_location_mm, NULL),
	REG_DEF(pm.x_location_home,,,		"rad",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_location_home, _g,,	"g",	"%2f",	0, &reg_proc_location_g, NULL),
	REG_DEF(pm.x_location_home, _mm,,	"mm",	"%3f",	0, &reg_proc_location_mm, NULL),

	REG_DEF(pm.x_setpoint_location,,,	"rad",	"%3f",	0, NULL, NULL),
	REG_DEF(pm.x_setpoint_location, _g,,	"g",	"%2f",	0, &reg_proc_location_g, NULL),
	REG_DEF(pm.x_setpoint_location, _mm,,	"mm",	"%3f",	0, &reg_proc_location_mm, NULL),
	REG_DEF(pm.x_setpoint_speed,,,	"rad/s",	"%2f",	0, NULL, NULL),
	REG_DEF(pm.x_setpoint_speed, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.x_setpoint_speed, _mmps,,	"mm/s",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_weak_zone,,,		"rad",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_weak_zone, _mm,,		"mm",	"%3f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_tolerance,,,		"rad",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_tolerance, _mm,,		"mm",	"%3f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_gain_P,,,			"",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_gain_P, _accel,,	"rad/s2",	"%1f",	0, &reg_proc_x_accel, NULL),
	REG_DEF(pm.x_gain_P, _accel_mm,,"mm/s2",	"%1f",	0, &reg_proc_x_accel_mm, NULL),
	REG_DEF(pm.x_gain_N,,,			"",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.mi_traveled,,,		"m",	"%1f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.mi_traveled, _km,,		"km",	"%3f",	REG_READ_ONLY, &reg_proc_km, NULL),
	REG_DEF(pm.mi_consumed_Wh,,,		"Wh",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.mi_consumed_Ah,,,		"Ah",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.mi_reverted_Wh,,,		"Wh",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.mi_reverted_Ah,,,		"Ah",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.mi_capacity_Ah,,,		"Ah",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.mi_fuel_gauge,,,		"%",	"%2f",	0, &reg_proc_im_fuel, NULL),

	REG_DEF(tlm.grabfreq,,,			"Hz",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(tlm.livefreq,,,			"Hz",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(tlm.mode,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_enum),

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

static void
reg_getval(const reg_t *reg, reg_value_t *lval)
{
	if (reg->proc != NULL) {

		reg->proc(reg, lval, NULL);
	}
	else {
		*(reg_value_t *) lval = *reg->link;
	}
}

static void
reg_setval(const reg_t *reg, const reg_value_t *rval)
{
	if ((reg->mode & REG_READ_ONLY) == 0) {

		if (reg->proc != NULL) {

			reg->proc(reg, NULL, rval);
		}
		else {
			*reg->link = *(reg_value_t *) rval;
		}
	}
}

void reg_format_rval(const reg_t *reg, const reg_value_t *rval)
{
	if (		   reg->fmt[1] == 'i'
			|| reg->fmt[2] == 'x') {

		printf(reg->fmt, rval->i);
	}
	else {
		printf(reg->fmt, &rval->f);
	}
}

void reg_format(const reg_t *reg)
{
	reg_value_t		rval;
	const char		*su;
	int			reg_ID;

	if (reg != NULL) {

		reg_ID = (int) (reg - regfile);

		printf("%i [%i] %s = ", (int) reg->mode, (int) reg_ID, reg->sym);

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

		if (found == NULL) {

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

void reg_GET(int reg_ID, reg_value_t *lval)
{
	if (reg_ID >= 0 && reg_ID < REG_MAX) {

		reg_getval(regfile + reg_ID, lval);
	}
}

void reg_SET(int reg_ID, const reg_value_t *rval)
{
	if (reg_ID >= 0 && reg_ID < REG_MAX) {

		reg_setval(regfile + reg_ID, rval);
	}
}

int reg_GET_I(int reg_ID)
{
	reg_value_t	lval;

	reg_GET(reg_ID, &lval);

	return lval.i;
}

float reg_GET_F(int reg_ID)
{
	reg_value_t	lval;

	reg_GET(reg_ID, &lval);

	return lval.f;
}

void reg_SET_I(int reg_ID, int x)
{
	reg_value_t	rval = {.i = x};

	reg_SET(reg_ID, &rval);
}

void reg_SET_F(int reg_ID, float x)
{
	reg_value_t	rval = {.f = x};

	reg_SET(reg_ID, &rval);
}

void reg_TOUCH_I(int reg_ID)
{
	reg_SET_I(reg_ID, reg_GET_I(reg_ID));
}

SH_DEF(reg)
{
	reg_value_t		rval;
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

SH_DEF(plain_reg)
{
	reg_value_t		rval;
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

