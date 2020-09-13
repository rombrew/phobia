#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "ifcan.h"
#include "regfile.h"
#include "shell.h"
#include "tlm.h"

#define REG_DEF(l, e, i, u, f, m, p, t)	{ #l #e "\0" u, f, m, (void *) &l i, (void *) p, (void *) t}
#define REG_MAX				(sizeof(regfile) / sizeof(reg_t) - 1UL)

static int		null;

static void
reg_proc_pwm(const reg_t *reg, float *lval, const float *rval)
{
	int			irq;

	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		reg->link->f = *rval;

		if (pm.lu_mode == PM_LU_DISABLED) {

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
reg_proc_adc(const reg_t *reg, float *lval, const float *rval)
{
	int			irq;

	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		reg->link->f = *rval;

		if (pm.lu_mode == PM_LU_DISABLED) {

			irq = hal_lock_irq();

			ADC_configure();

			reg_SET_F(ID_PM_FAULT_CURRENT_HALT, 0.f);

			hal_unlock_irq(irq);
		}
	}
}

static void
reg_proc_can_NART(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = *rval;

		CAN_configure();
	}
}

static void
reg_proc_can_TIM(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = (int) (hal.PWM_frequency / (float) reg->link->i + .5f);
	}
	else if (rval != NULL) {

		reg->link->i = (int) (hal.PWM_frequency / (float) *rval + .5f);
	}
}

static void
reg_proc_can_IDs(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = *rval;

		IFCAN_filter_ID();
	}
}

static void
reg_proc_ppm(const reg_t *reg, int *lval, const int *rval)
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
reg_proc_tim(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}
	else if (rval != NULL) {

		reg->link->i = *rval;

		TIM_configure();
	}
}

static void
reg_proc_rpm(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * (60.f / 2.f / M_PI_F) / pm.const_Zp;
	}
	else if (rval != NULL) {

		reg->link->f = (*rval) * (2.f * M_PI_F / 60.f) * pm.const_Zp;
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

		*lval = rpm * pm.abi_PPR * (1.f / 60.f);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			rpm = (*rval) / pm.abi_PPR * (60.f / 1.f);

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
	float			KPC = pm.k_EMAX / 100.f;

	if (lval != NULL) {

		*lval = reg->link->f * pm.const_E / (KPC * pm.const_fb_U);
	}
	else if (rval != NULL) {

		if (pm.const_E > M_EPS_F) {

			reg->link->f = (*rval) * KPC * pm.const_fb_U / pm.const_E;
		}
	}
}

static void
reg_proc_Q_pc(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * 100.f / pm.i_maximal;
	}
	else if (rval != NULL) {

		reg->link->f = (*rval) * pm.i_maximal / 100.f;
	}
}

static void
reg_proc_kv(const reg_t *reg, float *lval, const float *rval)
{
        if (lval != NULL) {

                *lval = 5.513289f / (reg->link->f * pm.const_Zp);
        }
        else if (rval != NULL) {

                reg->link->f = 5.513289f / ((*rval) * pm.const_Zp);
        }
}

static void
reg_proc_kgm2(const reg_t *reg, float *lval, const float *rval)
{
	float			Zp2 = (float) (pm.const_Zp * pm.const_Zp);

	if (lval != NULL) {

		*lval = reg->link->f * 1.5f * Zp2 * pm.const_E;
	}
	else if (rval != NULL) {

		if (pm.const_E > M_EPS_F) {

			reg->link->f = (*rval) / (1.5f * Zp2 * pm.const_E);
		}
	}
}

static void
reg_proc_kg(const reg_t *reg, float *lval, const float *rval)
{
	const float		ld_R = pm.const_ld_S / (2.f * M_PI_F);

	if (lval != NULL) {

		if (ld_R > M_EPS_F) {

			*lval = reg->link->f * 1.5f * pm.const_E / (ld_R * ld_R);
		}
		else {
			*lval = 0.f;
		}
	}
	else if (rval != NULL) {

		if (pm.const_E > M_EPS_F) {

			reg->link->f = (*rval) * ld_R * ld_R / (1.5f * pm.const_E);
		}
	}
}

static void
reg_proc_BEMF(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * pm.lu_MPPE * pm.const_E;
	}
	else if (rval != NULL) {

		if (pm.const_E > M_EPS_F) {

			reg->link->f = (*rval) / (pm.lu_MPPE * pm.const_E);
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

			halt = ADC_RESOLUTION * 95E-2f / 2.f;

			adjust = (pm.ad_IA[1] < pm.ad_IB[1])
				? pm.ad_IA[1] : pm.ad_IB[1];
			adjust = (adjust == 0.f) ? 1.f : adjust;

			halt *= m_fabsf(hal.ADC_const.GA * adjust);

			reg->link->f = (float) (int) (halt);
		}
		else {
			reg->link->f = *rval;
		}
	}
}

static void
reg_proc_maximal_i(const reg_t *reg, float *lval, const float *rval)
{
	float			range, max_1;

	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			range = pm.fault_current_halt * 95E-2f;

			if (pm.const_R > M_EPS_F) {

				max_1 = pm.k_UMAX * pm.const_fb_U / pm.const_R;
				range = (max_1 < range) ? max_1 : range;
			}

			reg->link->f = (float) (int) (range);
		}
		else {
			reg->link->f = *rval;
		}
	}
}

static void
reg_proc_reverse_i(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f;
	}
	else if (rval != NULL) {

		if (*rval < M_EPS_F) {

			reg->link->f = pm.i_maximal;
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
reg_proc_F_g(const reg_t *reg, float *lval, const float *rval)
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
reg_proc_F_nolock_g(const reg_t *reg, float *lval, const float *rval)
{
	float			*F = (void *) reg->link;

	if (lval != NULL) {

		*lval = m_atan2f(F[1], F[0]) * (180.f / M_PI_F);
	}
}

static void
reg_proc_setpoint_F(const reg_t *reg, float *lval, const float *rval)
{
	float			*F = (void *) reg->link;
	float			angle, f_cosine, f_sine;
	int			revol, irq;

        if (lval != NULL) {

		irq = hal_lock_irq();

		f_cosine = F[0];
		f_sine   = F[1];
		revol    = pm.x_setpoint_revol;

		hal_unlock_irq(irq);

		angle = m_atan2f(f_sine, f_cosine);
		*lval = angle + (float) revol * 2.f * M_PI_F;
        }
        else if (rval != NULL) {

		angle = (*rval);
		revol = (int) (angle / (2.f * M_PI_F));
                angle -= (float) (revol * 2.f * M_PI_F);

                if (angle < - M_PI_F) {

                        revol -= 1;
                        angle += 2.f * M_PI_F;
                }

                if (angle > M_PI_F) {

                        revol += 1;
                        angle -= 2.f * M_PI_F;
                }

		f_cosine = m_cosf(angle);
		f_sine   = m_sinf(angle);

		irq = hal_lock_irq();

		F[0] = f_cosine;
		F[1] = f_sine;
		pm.x_setpoint_revol = revol;

		hal_unlock_irq(irq);
        }
}

static void
reg_proc_setpoint_F_g(const reg_t *reg, float *lval, const float *rval)
{
	float			angle;

	if (lval != NULL) {

		reg_proc_setpoint_F(reg, &angle, rval);

		*lval = angle * (180.f / M_PI_F) / pm.const_Zp;
	}
	else if (rval != NULL) {

		angle = (*rval) * (M_PI_F / 180.f) * pm.const_Zp;

		reg_proc_setpoint_F(reg, lval, &angle);
	}
}

static void
reg_proc_setpoint_F_mm(const reg_t *reg, float *lval, const float *rval)
{
	float			angle;

	if (lval != NULL) {

		reg_proc_setpoint_F(reg, &angle, rval);

		*lval = angle * pm.const_ld_S * 1000.f
			/ (2.f * M_PI_F * pm.const_Zp);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			angle = (*rval) * (2.f * M_PI_F * pm.const_Zp)
				/ (pm.const_ld_S * 1000.f);

			reg_proc_setpoint_F(reg, lval, &angle);
		}
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
			/ (2.f * M_PI_F * pm.const_Zp);
	}
	else if (rval != NULL) {

		if (pm.const_ld_S > M_EPS_F) {

			rads = (*rval) * (2.f * M_PI_F * pm.const_Zp)
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

			hal_unlock_irq(irq);

			vTaskDelay((TickType_t) 1);
		}
	}
}

static void
reg_format_self_BST(const reg_t *reg)
{
	int		*BST = (void *) reg->link;

	printf("%4f %4f %4f (ms)", &BST[0], &BST[1], &BST[2]);
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

	printf("%3f %3f (A) %4f (V)", &RMS[0], &RMS[1], &RMS[2]);
}

static void
reg_format_self_RMSu(const reg_t *reg)
{
	float		*RMS = (void *) reg->link;

	printf("%4f %4f %4f (V)", &RMS[0], &RMS[1], &RMS[2]);
}

static void
reg_format_can_TIM_ms(const reg_t *reg)
{
	float			ms;

	ms = 1000.f * pm.dT * (float) reg->link->i;

	printf("%i (%1f ms)", reg->link->i, &ms);
}

#define TEXT_ITEM(t)	case t: printf("(%s)", PM_SFI(t)); break

static void
reg_format_enum(const reg_t *reg)
{
	int			n, val;

	n = (int) (reg - regfile);
	val = reg->link->i;

	printf("%i ", val);

	switch (n) {

		case ID_HAL_TIM_MODE:

			switch (val) {

				TEXT_ITEM(TIM_DISABLED);
				TEXT_ITEM(TIM_DRIVE_HALL);
				TEXT_ITEM(TIM_DRIVE_ABI);

				default: break;
			}
			break;

		case ID_HAL_CAN_MODE_NART:

			switch (val) {

				TEXT_ITEM(CAN_MODE_STANDARD);
				TEXT_ITEM(CAN_MODE_NO_AUTO_RETRANSMIT);

				default: break;
			}
			break;

		case ID_HAL_PPM_MODE:

			switch (val) {

				TEXT_ITEM(PPM_DISABLED);
				TEXT_ITEM(PPM_PULSE_WIDTH);
				TEXT_ITEM(PPM_STEP_DIR);
				TEXT_ITEM(PPM_OUTPULSE);
				TEXT_ITEM(PPM_BACKUP_ABI);

				default: break;
			}
			break;

		case ID_CAN_LOG_MODE:

			switch (val) {

				TEXT_ITEM(IFCAN_LOG_DISABLED);
				TEXT_ITEM(IFCAN_LOG_FILTERED);
				TEXT_ITEM(IFCAN_LOG_PROMISCUOUS);

				default: break;
			}
			break;

		case ID_CAN_PIPE_0_MODE:
		case ID_CAN_PIPE_1_MODE:

			switch (val) {

				TEXT_ITEM(IFCAN_PIPE_DISABLED);
				TEXT_ITEM(IFCAN_PIPE_INCOMING);
				TEXT_ITEM(IFCAN_PIPE_OUTGOING_REGULAR);
				TEXT_ITEM(IFCAN_PIPE_OUTGOING_TRIGGERED);

				default: break;
			}
			break;

		case ID_CAN_PIPE_0_STARTUP:
		case ID_CAN_PIPE_1_STARTUP:

			switch (val) {

				TEXT_ITEM(PM_DISABLED);
				TEXT_ITEM(PM_ENABLED);

				default: break;
			}
			break;

		case ID_CAN_PIPE_0_PAYLOAD:
		case ID_CAN_PIPE_1_PAYLOAD:

			switch (val) {

				TEXT_ITEM(IFCAN_PAYLOAD_FLOAT);
				TEXT_ITEM(IFCAN_PAYLOAD_INT_16);
				TEXT_ITEM(IFCAN_PAYLOAD_INT_32);
				TEXT_ITEM(IFCAN_PAYLOAD_PACKED_INT_16_0);
				TEXT_ITEM(IFCAN_PAYLOAD_PACKED_INT_16_1);
				TEXT_ITEM(IFCAN_PAYLOAD_PACKED_INT_16_2);
				TEXT_ITEM(IFCAN_PAYLOAD_PACKED_INT_16_3);

				default: break;
			}
			break;

		case ID_AP_PPM_STARTUP:
		case ID_AP_STEP_STARTUP:
		case ID_AP_ANALOG_ENABLED:
		case ID_AP_ANALOG_STARTUP:

			switch (val) {

				TEXT_ITEM(PM_DISABLED);
				TEXT_ITEM(PM_ENABLED);

				default: break;
			}
			break;

		case ID_PM_FAIL_REASON:

			printf("(%s)", pm_strerror(pm.fail_reason));
			break;

		case ID_PM_CONFIG_NOP:

			switch (val) {

				TEXT_ITEM(PM_NOP_THREE_PHASE);
				TEXT_ITEM(PM_NOP_TWO_PHASE);

				default: break;
			}
			break;

		case ID_PM_CONFIG_IFB:

			switch (val) {

				TEXT_ITEM(PM_IFB_AB_INLINE);
				TEXT_ITEM(PM_IFB_AB_LOW);

				default: break;
			}
			break;

		case ID_PM_CONFIG_TVM:
		case ID_PM_CONFIG_VSI_SILENT:
		case ID_PM_CONFIG_FORCED:
		case ID_PM_CONFIG_ABI_FORCED_ALIGN:
		case ID_PM_CONFIG_HFI:
		case ID_PM_CONFIG_WEAK:
		case ID_PM_CONFIG_SPEED_FROM_TORQUE:
		case ID_PM_CONFIG_INFO:
		case ID_PM_TVM_ENABLED:
		case ID_PM_HALL_ENABLED:

			switch (val) {

				TEXT_ITEM(PM_DISABLED);
				TEXT_ITEM(PM_ENABLED);

				default: break;
			}
			break;

		case ID_PM_CONFIG_ESTIMATE:

			switch (val) {

				TEXT_ITEM(PM_ESTIMATE_DISABLED);
				TEXT_ITEM(PM_ESTIMATE_FLUX);

				default: break;
			}
			break;

		case ID_PM_CONFIG_SENSOR:

			switch (val) {

				TEXT_ITEM(PM_SENSOR_DISABLED);
				TEXT_ITEM(PM_SENSOR_HALL);
				TEXT_ITEM(PM_SENSOR_ABI);

				default: break;
			}
			break;

		case ID_PM_CONFIG_DRIVE:

			switch (val) {

				TEXT_ITEM(PM_DRIVE_CURRENT);
				TEXT_ITEM(PM_DRIVE_SPEED);
				TEXT_ITEM(PM_DRIVE_COMBINED);
				TEXT_ITEM(PM_DRIVE_SERVO);

				default: break;
			}
			break;

		case ID_PM_FSM_REQ:
		case ID_PM_FSM_STATE:

			switch (val) {

				TEXT_ITEM(PM_STATE_IDLE);
				TEXT_ITEM(PM_STATE_ZERO_DRIFT);
				TEXT_ITEM(PM_STATE_SELF_TEST_BOOTSTRAP);
				TEXT_ITEM(PM_STATE_SELF_TEST_POWER_STAGE);
				TEXT_ITEM(PM_STATE_SELF_TEST_CLEARANCE);
				TEXT_ITEM(PM_STATE_ADJUST_VOLTAGE);
				TEXT_ITEM(PM_STATE_ADJUST_CURRENT);
				TEXT_ITEM(PM_STATE_PROBE_CONST_R);
				TEXT_ITEM(PM_STATE_PROBE_CONST_L);
				TEXT_ITEM(PM_STATE_LU_STARTUP);
				TEXT_ITEM(PM_STATE_LU_SHUTDOWN);
				TEXT_ITEM(PM_STATE_PROBE_CONST_E);
				TEXT_ITEM(PM_STATE_PROBE_CONST_J);
				TEXT_ITEM(PM_STATE_PROBE_LU_MPPE);
				TEXT_ITEM(PM_STATE_ADJUST_HALL);
				TEXT_ITEM(PM_STATE_HALT);

				default: break;
			}
			break;

		case ID_PM_LU_MODE:

			switch (val) {

				TEXT_ITEM(PM_LU_DISABLED);
				TEXT_ITEM(PM_LU_DETACHED);
				TEXT_ITEM(PM_LU_FORCED);
				TEXT_ITEM(PM_LU_ESTIMATE_FLUX);
				TEXT_ITEM(PM_LU_ESTIMATE_HFI);
				TEXT_ITEM(PM_LU_SENSOR_HALL);
				TEXT_ITEM(PM_LU_SENSOR_ABI);

				default: break;
			}
			break;

		case ID_PM_LU_FLUX_ZONE:

			switch (val) {

				TEXT_ITEM(PM_FLUX_UNCERTAIN);
				TEXT_ITEM(PM_FLUX_HIGH);
				TEXT_ITEM(PM_FLUX_DETACHED);

				default: break;
			}
			break;

		default: break;
	}
}

const reg_t		regfile[] = {

	REG_DEF(null,,,				"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(iodef_ECHO,,,			"",	"%i",	0, 0, NULL),

	REG_DEF(hal.USART_baud_rate,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(hal.PWM_frequency,,,		"Hz",	"%1f",	REG_CONFIG, &reg_proc_pwm, NULL),
	REG_DEF(hal.PWM_deadtime,,,		"ns",	"%1f",	REG_CONFIG, &reg_proc_pwm, NULL),
	REG_DEF(hal.ADC_reference_voltage,,,	"V",	"%3f",	REG_CONFIG, &reg_proc_adc, NULL),
	REG_DEF(hal.ADC_shunt_resistance,,,	"Ohm",	"%4e",	REG_CONFIG, &reg_proc_adc, NULL),
	REG_DEF(hal.ADC_amplifier_gain,,,	"",	"%1f",	REG_CONFIG, &reg_proc_adc, NULL),
	REG_DEF(hal.ADC_voltage_ratio,,,	"",	"%4e",	REG_CONFIG, &reg_proc_adc, NULL),
	REG_DEF(hal.ADC_terminal_ratio,,,	"",	"%4e",	REG_CONFIG, &reg_proc_adc, NULL),
	REG_DEF(hal.ADC_terminal_bias,,,	"",	"%4e",	REG_CONFIG, &reg_proc_adc, NULL),
	REG_DEF(hal.TIM_mode,,,		"",	"%i", REG_CONFIG, &reg_proc_tim, &reg_format_enum),
	REG_DEF(hal.CAN_mode_NART,,,	"",	"%i", REG_CONFIG, &reg_proc_can_NART, &reg_format_enum),
	REG_DEF(hal.PPM_mode,,,		"",	"%i", REG_CONFIG, &reg_proc_ppm, &reg_format_enum),
	REG_DEF(hal.PPM_timebase,,,		"Hz",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(hal.PPM_signal_caught,,,	"",	"%i",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(can.node_ID,,,		"",	"%i",	REG_CONFIG, &reg_proc_can_IDs, NULL),
	REG_DEF(can.log_MODE,,,		"",	"%i",	REG_CONFIG, &reg_proc_can_IDs, &reg_format_enum),
	REG_DEF(can.startup_LOST, _ms,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_can_TIM_ms),

	REG_DEF(can.pipe, _0_ID, [0].ID,"",		"%i",	REG_CONFIG, &reg_proc_can_IDs, NULL),
	REG_DEF(can.pipe, _0_reg_DATA, [0].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(can.pipe, _0_reg_ID, [0].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(can.pipe, _0_MODE, [0].MODE,"", "%i",	REG_CONFIG, &reg_proc_can_IDs, &reg_format_enum),
	REG_DEF(can.pipe, _0_STARTUP, [0].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(can.pipe, _0_tim_hz, [0].tim,	"Hz",	"%i",	REG_CONFIG, &reg_proc_can_TIM, NULL),
	REG_DEF(can.pipe, _0_trigger_ID, [0].trigger_ID,"","%i",REG_CONFIG, &reg_proc_can_IDs, NULL),
	REG_DEF(can.pipe, _0_PAYLOAD, [0].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(can.pipe, _0_range_0, [0].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(can.pipe, _0_range_1, [0].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(can.pipe, _1_ID, [1].ID,"",		"%i",	REG_CONFIG, &reg_proc_can_IDs, NULL),
	REG_DEF(can.pipe, _1_reg_DATA, [1].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(can.pipe, _1_reg_ID, [1].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(can.pipe, _1_MODE, [1].MODE,"", "%i",	REG_CONFIG, &reg_proc_can_IDs, &reg_format_enum),
	REG_DEF(can.pipe, _1_STARTUP, [1].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(can.pipe, _1_tim_hz, [1].tim,	"Hz",	"%i",	REG_CONFIG, &reg_proc_can_TIM, NULL),
	REG_DEF(can.pipe, _1_trigger_ID, [1].trigger_ID,"","%i",REG_CONFIG, &reg_proc_can_IDs, NULL),
	REG_DEF(can.pipe, _1_PAYLOAD, [1].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(can.pipe, _1_range_0, [1].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(can.pipe, _1_range_1, [1].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(can.pipe, _2_ID, [2].ID,"",		"%i",	REG_CONFIG, &reg_proc_can_IDs, NULL),
	REG_DEF(can.pipe, _2_reg_DATA, [2].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(can.pipe, _2_reg_ID, [2].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(can.pipe, _2_MODE, [2].MODE,"", "%i",	REG_CONFIG, &reg_proc_can_IDs, &reg_format_enum),
	REG_DEF(can.pipe, _2_STARTUP, [2].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(can.pipe, _2_tim_hz, [2].tim,	"Hz",	"%i",	REG_CONFIG, &reg_proc_can_TIM, NULL),
	REG_DEF(can.pipe, _2_trigger_ID, [2].trigger_ID,"","%i",REG_CONFIG, &reg_proc_can_IDs, NULL),
	REG_DEF(can.pipe, _2_PAYLOAD, [2].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(can.pipe, _2_range_0, [2].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(can.pipe, _2_range_1, [2].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(can.pipe, _3_ID, [3].ID,"",		"%i",	REG_CONFIG, &reg_proc_can_IDs, NULL),
	REG_DEF(can.pipe, _3_reg_DATA, [3].reg_DATA,"",	"%4e",	0, NULL, NULL),
	REG_DEF(can.pipe, _3_reg_ID, [3].reg_ID,"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(can.pipe, _3_MODE, [3].MODE,"", "%i",	REG_CONFIG, &reg_proc_can_IDs, &reg_format_enum),
	REG_DEF(can.pipe, _3_STARTUP, [3].STARTUP,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(can.pipe, _3_tim_hz, [3].tim,	"Hz",	"%i",	REG_CONFIG, &reg_proc_can_TIM, NULL),
	REG_DEF(can.pipe, _3_trigger_ID, [3].trigger_ID,"","%i",REG_CONFIG, &reg_proc_can_IDs, NULL),
	REG_DEF(can.pipe, _3_PAYLOAD, [3].PAYLOAD,"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(can.pipe, _3_range_0, [3].range[0],"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(can.pipe, _3_range_1, [3].range[1],"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.ppm_reg_ID,,,		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(ap.ppm_STARTUP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.ppm_in_range, _0, [0],	"us",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_in_range, _1, [1],	"us",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_in_range, _2, [2],	"us",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_control_range, _0, [0],	"",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_control_range, _1, [1],	"",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ppm_control_range, _2, [2],	"",	"%2f",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.step_reg_ID,,,		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(ap.step_STARTUP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.step_accuEP,,,		"",	"%i",	0, NULL, NULL),
	REG_DEF(ap.step_const_ld_EP,,,		"mm",	"%3f",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.analog_ENABLED,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.analog_reg_ID,,,		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(ap.analog_STARTUP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(ap.analog_in_ANG, _0, [0],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.analog_in_ANG, _1, [1],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.analog_in_ANG, _2, [2],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.analog_in_BRK, _0, [0],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.analog_in_BRK, _1, [1],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.analog_in_lost, _0, [0],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.analog_in_lost, _1, [1],	"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.analog_control_ANG, _0, [0],	"",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.analog_control_ANG, _1, [1],	"",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.analog_control_ANG, _2, [2],	"",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.analog_control_BRK,,,	"",	"%2f",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.timeout_current_tol,,,	"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.timeout_IDLE_s,,,		"s",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.ntc_PCB.r_balance,,,		"Ohm",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_PCB.r_ntc_0,,,		"Ohm",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_PCB.ta_0,,,		"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_PCB.betta,,,		"",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_EXT.r_balance,,,		"Ohm",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_EXT.r_ntc_0,,,		"Ohm",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_EXT.ta_0,,,		"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.ntc_EXT.betta,,,		"",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.temp_PCB,,,			"C",	"%1f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(ap.temp_EXT,,,			"C",	"%1f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(ap.temp_INT,,,			"C",	"%1f",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(ap.heat_PCB,,,			"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.heat_PCB_derated_1,,,	"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.heat_EXT,,,			"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.heat_EXT_derated_1,,,	"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.heat_PCB_FAN,,,		"C",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.heat_recovery_gap,,,		"C",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.servo_SPAN_mm, _0, [0],	"mm",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.servo_SPAN_mm, _1, [1],	"mm",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.servo_UNIFORM_mmps,,,	"mm/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.servo_mice_role,,,		"",	"%i",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.FT_grab_hz,,,		"Hz",	"%i",	REG_CONFIG, NULL, NULL),

	REG_DEF(ap.hx711_kg,,,			"kg",	"%4f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(ap.hx711_scale, _0, [0],	"kg",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(ap.hx711_scale, _1, [1],	"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.dc_resolution,,,		"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.dc_minimal,,,		"us",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_clearance,,,		"us",	"%4f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_bootstrap,,,		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.dc_clamped,,,		"s",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.fail_reason,,,	"",	"%i",	REG_READ_ONLY, NULL, &reg_format_enum),
	REG_DEF(pm.self_BST,,,		"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_BST),
	REG_DEF(pm.self_BM,,,		"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_BM),
	REG_DEF(pm.self_RMSi,,,		"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_RMSi),
	REG_DEF(pm.self_RMSu,,,		"",	"%i",	REG_READ_ONLY, NULL, &reg_format_self_RMSu),

	REG_DEF(pm.config_NOP,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_TVM,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_IFB,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_VSI_SILENT,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_FORCED,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_ABI_FORCED_ALIGN,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_ESTIMATE,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_HFI,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_SENSOR,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_WEAK,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_DRIVE,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_SPEED_FROM_TORQUE,,,	"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_INFO,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),
	REG_DEF(pm.config_BOOST,,,		"",	"%i",	REG_CONFIG, NULL, &reg_format_enum),

	REG_DEF(pm.fsm_req,,,			"",	"%i",	0, NULL, &reg_format_enum),
	REG_DEF(pm.fsm_state,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_enum),
	REG_DEF(pm.fsm_phase,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(pm.tm_transient_slow,,,		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_transient_fast,,,		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_voltage_hold,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_current_hold,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_instant_probe,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_average_probe,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_average_drift,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_average_inertia,,, 	"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tm_startup,,,		"ms",	"%1f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.ad_IA, _0, [0],		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_IA, _1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_IB, _0, [0],		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_IB, _1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_US, _0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_US, _1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UA, _0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UA, _1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UB, _0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UB, _1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UC, _0, [0],		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.ad_UC, _1, [1],		"",	"%4e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.fb_iA,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_iB,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_uA,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_uB,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_uC,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_HS,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.fb_EP,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(pm.probe_current_hold,,,	"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_hold_angle,,,		"g",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_current_weak,,,	"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_current_sine,,,	"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_freq_sine_hz,,,	"Hz",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_speed_hold_pc,,,	"%",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_speed_spinup_pc,,,	"%",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_speed_detached,,,	"rad/s","%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_speed_detached, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.probe_speed_detached, _mmps,,"mm/s",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.probe_gain_P,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.probe_gain_I,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.fault_voltage_tol,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.fault_current_tol,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.fault_accuracy_tol,,,	"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.fault_current_halt,,,	"A",	"%3f",	REG_CONFIG, &reg_proc_halt, NULL),
	REG_DEF(pm.fault_voltage_halt,,,	"V",	"%3f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.vsi_DC,,,			"",	"%4f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_X,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_Y,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_DX,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_DY,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_AF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_BF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.vsi_UF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(pm.tvm_ENABLED,,,"",	"%i",	REG_CONFIG | REG_READ_ONLY, NULL, &reg_format_enum),
	REG_DEF(pm.tvm_range_DC,,,		"",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.tvm_A,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_B,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_C,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_A, _0, [0],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_A, _1, [1],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_A, _2, [2],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_A, _tau,,	"us",	"%3f",	REG_READ_ONLY, &reg_proc_tvm_FIR_tau, NULL),
	REG_DEF(pm.tvm_FIR_B, _0, [0],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_B, _1, [1],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_B, _2, [2],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_B, _tau,,	"us",	"%3f",	REG_READ_ONLY, &reg_proc_tvm_FIR_tau, NULL),
	REG_DEF(pm.tvm_FIR_C, _0, [0],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_C, _1, [1],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_C, _2, [2],	"",	"%4e",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_FIR_C, _tau,,	"us",	"%3f",	REG_READ_ONLY, &reg_proc_tvm_FIR_tau, NULL),
	REG_DEF(pm.tvm_DX,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.tvm_DY,,,			"V",	"%3f",	REG_READ_ONLY, NULL, NULL),

	REG_DEF(pm.lu_iX,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_iY,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_iD,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_iQ,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_g, NULL),
	REG_DEF(pm.lu_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.lu_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.lu_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.lu_mode,,,			"",	"%i",	REG_READ_ONLY, NULL, &reg_format_enum),

	REG_DEF(pm.lu_flux_lpf_wS,,,	"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.lu_flux_lpf_wS, _rpm,,	"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.lu_flux_lpf_wS, _mmps,,	"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.lu_flux_lpf_wS, _kmh,,	"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.lu_MPPE,,,		"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.lu_MPPE, _rpm,,		"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.lu_flux_zone,,,		"",	"%i",	REG_READ_ONLY, NULL, &reg_format_enum),
	REG_DEF(pm.lu_gain_TAKE,,,		"",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.lu_gain_GIVE,,,		"",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.lu_gain_TAKE, _BEMF,,	"V",	"%3f",	0, &reg_proc_BEMF, NULL),
	REG_DEF(pm.lu_gain_GIVE, _BEMF,,	"V",	"%3f",	0, &reg_proc_BEMF, NULL),
	REG_DEF(pm.lu_gain_LEVE,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.lu_gain_LP,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.forced_hold_D,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_maximal,,,	"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_maximal, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.forced_reverse,,,	"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_reverse, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.forced_accel,,,	"rad/s2",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.forced_accel, _rpm,,	"rpm/s",	"%1f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.forced_accel, _mmps,,"mm/s2",	"%2f",	0, &reg_proc_mmps, NULL),

	REG_DEF(pm.detach_take_U,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.detach_gain_AD,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.detach_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.flux_X,,,			"Wb",	"%4e",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.flux_Y,,,			"Wb",	"%4e",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.flux_E,,,			"Wb",	"%4e",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.flux_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_g, NULL),
	REG_DEF(pm.flux_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.flux_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.flux_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.flux_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.flux_gain_IN,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_LO,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_HI,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_AD,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.flux_gain_IF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.hfi_tm_DIV,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_tm_SKIP,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_tm_SUM,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_tm_POLAR,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_inject_sine,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_maximal,,,	"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_maximal, _rpm,,		"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.hfi_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_g, NULL),
	REG_DEF(pm.hfi_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hfi_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.hfi_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.hfi_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.hfi_const_L1,,,		"H",	"%4e",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hfi_const_L2,,,		"H",	"%4e",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hfi_const_R,,,		"Ohm",	"%4e",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hfi_const_POLAR,,,		"",	"%2e",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hfi_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hfi_gain_IF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.hall_ST, _1_X, [1].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _1_Y, [1].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _1_g, [1],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_nolock_g, NULL),
	REG_DEF(pm.hall_ST, _2_X, [2].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _2_Y, [2].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _2_g, [2],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_nolock_g, NULL),
	REG_DEF(pm.hall_ST, _3_X, [3].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _3_Y, [3].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _3_g, [3],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_nolock_g, NULL),
	REG_DEF(pm.hall_ST, _4_X, [4].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _4_Y, [4].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _4_g, [4],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_nolock_g, NULL),
	REG_DEF(pm.hall_ST, _5_X, [5].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _5_Y, [5].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _5_g, [5],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_nolock_g, NULL),
	REG_DEF(pm.hall_ST, _6_X, [6].X,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _6_Y, [6].Y,"",	"%3f",	REG_CONFIG | REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_ST, _6_g, [6],	"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_nolock_g, NULL),

	REG_DEF(pm.hall_ENABLED,,,"",	"%i",	REG_CONFIG | REG_READ_ONLY, NULL, &reg_format_enum),
	REG_DEF(pm.hall_DIRF,,,			"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_prolS,,,		"rad",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_prolTIM,,,		"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_g, NULL),
	REG_DEF(pm.hall_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.hall_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.hall_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.hall_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.hall_prol_T,,, 		"ms",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hall_gain_PF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hall_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.hall_gain_IF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.abi_baseEP,,,		"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.abi_lastEP,,,		"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.abi_rotEP,,,		"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.abi_prolTIM,,,		"",	"%i",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.abi_prolS,,,		"rad",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.abi_PPR,,,			"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.abi_FILTER,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.abi_Zq,,,			"",	"%5f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.abi_F, _g,,			"g",	"%2f",	REG_READ_ONLY, &reg_proc_F_g, NULL),
	REG_DEF(pm.abi_wS,,,		"rad/s",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.abi_wS, _rpm,,		"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm, NULL),
	REG_DEF(pm.abi_wS, _mmps,,		"mm/s",	"%2f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.abi_wS, _kmh,,		"km/h",	"%1f",	REG_READ_ONLY, &reg_proc_kmh, NULL),
	REG_DEF(pm.abi_gain_PF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.abi_gain_SF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.abi_gain_IF,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.const_fb_U,,,		"V",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.const_E,,,			"Wb",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_E, _kv,,	"rpm/v",	"%2f",	0, &reg_proc_kv, NULL),
	REG_DEF(pm.const_R,,,			"Ohm",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_L,,,			"H",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_Zp,,,			"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_Ja,,,		"A/rad/s2",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_Ja, _kgm2,,		"kgm2",	"%4e",	0, &reg_proc_kgm2, NULL),
	REG_DEF(pm.const_Ja, _kg,,		"kg",	"%4e",	0, &reg_proc_kg, NULL),
	REG_DEF(pm.const_im_L1,,,		"H",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_im_L2,,,		"H",	"%4e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_im_B,,,		"g",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.const_im_R,,,		"Ohm",	"%4e",	REG_CONFIG, NULL, NULL),
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

	REG_DEF(pm.i_maximal,,,			"A",	"%3f",	REG_CONFIG, &reg_proc_maximal_i, NULL),
	REG_DEF(pm.i_reverse,,,			"A",	"%3f",	REG_CONFIG, &reg_proc_reverse_i, NULL),
	REG_DEF(pm.i_derated_1,,,		"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.i_derated_HFI,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.i_setpoint_D,,,		"A",	"%3f",	0, NULL, NULL),
	REG_DEF(pm.i_setpoint_Q,,,		"A",	"%3f",	0, NULL, NULL),
	REG_DEF(pm.i_setpoint_torque,,,		"A",	"%3f",	0, NULL, NULL),
	REG_DEF(pm.i_setpoint_torque, _pc,,	"%",	"%2f",	0, &reg_proc_Q_pc, NULL),
	REG_DEF(pm.i_tol_Z,,,			"A",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.i_gain_P,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.i_gain_I,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.inject_ratio_D,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.inject_gain_AD,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.weak_maximal,,,		"A",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.weak_bias_U,,,		"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.weak_D,,,			"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.weak_gain_EU,,,		"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.v_maximal,,,			"V",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.v_reverse,,,			"V",	"%3f",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.s_maximal,,,		"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_maximal, _rpm,,		"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.s_maximal, _mmps,,		"mm/s",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.s_maximal, _kmh,,		"km/h",	"%1f",	0, &reg_proc_kmh, NULL),
	REG_DEF(pm.s_reverse,,,		"rad/s",	"%2f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_reverse, _rpm,,		"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.s_reverse, _mmps,,		"mm/s",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.s_reverse, _kmh,,		"km/h",	"%1f",	0, &reg_proc_kmh, NULL),
	REG_DEF(pm.s_setpoint_speed,,,	"rad/s",	"%2f",	0, NULL, NULL),
	REG_DEF(pm.s_setpoint_speed, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.s_setpoint_speed, _mmps,,	"mm/s",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.s_setpoint_speed, _kmh,,	"km/h",	"%1f",	0, &reg_proc_kmh, NULL),
	REG_DEF(pm.s_setpoint_speed, _pc,,	"%",	"%2f",	0, &reg_proc_rpm_pc, NULL),
	REG_DEF(pm.s_accel,,,		"rad/s2",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_accel, _rpm,,	"rpm/s",	"%1f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.s_accel, _kmh,,	"km/h/s",	"%1f",	0, &reg_proc_kmh, NULL),
	REG_DEF(pm.s_integral,,,		"A",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.s_tol_Z,,,		"rad/s",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_gain_P,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_gain_I,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_gain_S,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.s_gain_D,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.x_setpoint_F,,,		"rad",	"%2f",	0, &reg_proc_setpoint_F, NULL),
	REG_DEF(pm.x_setpoint_F, _g,,		"g",	"%2f",	0, &reg_proc_setpoint_F_g, NULL),
	REG_DEF(pm.x_setpoint_F, _mm,,		"mm",	"%3f",	0, &reg_proc_setpoint_F_mm, NULL),
	REG_DEF(pm.x_setpoint_speed,,,	"rad/s",	"%2f",	0, NULL, NULL),
	REG_DEF(pm.x_setpoint_speed, _rpm,,	"rpm",	"%2f",	0, &reg_proc_rpm, NULL),
	REG_DEF(pm.x_setpoint_speed, _mmps,,	"mm/s",	"%2f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_residual,,,		"rad",	"%2f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.x_residual, _mm,,		"mm",	"%3f",	REG_READ_ONLY, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_residual, _ep,,		"ep",	"%1f",	REG_READ_ONLY, &reg_proc_epps, NULL),
	REG_DEF(pm.x_tol_N,,,			"rad",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_tol_N, _mm,,		"mm",	"%3f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_tol_Z,,,			"rad",	"%2e",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_tol_Z, _mm,,		"mm",	"%3f",	0, &reg_proc_mmps, NULL),
	REG_DEF(pm.x_tol_Z, _ep,,		"ep",	"%1f",	0, &reg_proc_epps, NULL),
	REG_DEF(pm.x_gain_P,,,			"",	"%1f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.x_gain_P, _accel,,	"rad/s2",	"%1f",	0, &reg_proc_gain_accel, NULL),
	REG_DEF(pm.x_gain_P, _accel_mm,,"mm/s2",	"%1f",	0, &reg_proc_gain_accel_mm, NULL),
	REG_DEF(pm.x_gain_N,,,			"",	"%2e",	REG_CONFIG, NULL, NULL),

	REG_DEF(pm.im_revol_total,,,		"",	"%i",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.im_distance,,,		"m",	"%1f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.im_distance, _km,,		"km",	"%3f",	REG_READ_ONLY, &reg_proc_km, NULL),
	REG_DEF(pm.im_consumed_Wh,,,		"Wh",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.im_consumed_Ah,,,		"Ah",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.im_reverted_Wh,,,		"Wh",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.im_reverted_Ah,,,		"Ah",	"%3f",	REG_READ_ONLY, NULL, NULL),
	REG_DEF(pm.im_capacity_Ah,,,		"Ah",	"%3f",	REG_CONFIG, NULL, NULL),
	REG_DEF(pm.im_fuel_pc,,,		"%",	"%2f",	0, &reg_proc_im_fuel, NULL),

	REG_DEF(tlm.reg_ID, _0, [0],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, _1, [1],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, _2, [2],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, _3, [3],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, _4, [4],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, _5, [5],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, _6, [6],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, _7, [7],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, _8, [8],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),
	REG_DEF(tlm.reg_ID, _9, [9],		"",	"%i",	REG_CONFIG | REG_LINKED, NULL, NULL),

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

	if (reg->fmt[1] == 'i') {

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

	if (reg != NULL) {

		printf("%c%c%c [%i] %s = ",
			(int) (reg->mode & REG_CONFIG) 		? 'C' : ' ',
			(int) (reg->mode & REG_READ_ONLY)	? 'R' : ' ',
			(int) (reg->mode & REG_LINKED)		? 'L' : ' ',
			(int) (reg - regfile), reg->sym);

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

void reg_GET(int n, void *lval)
{
	if (n >= 0 && n < REG_MAX) {

		reg_getval(regfile + n, lval);
	}
}

void reg_SET(int n, const void *rval)
{
	if (n >= 0 && n < REG_MAX) {

		reg_setval(regfile + n, rval);
	}
}

int reg_GET_I(int n)
{
	int		lval;

	reg_GET(n, &lval);

	return lval;
}

float reg_GET_F(int n)
{
	float		lval;

	reg_GET(n, &lval);

	return lval;
}

void reg_SET_I(int n, int rval)
{
	reg_SET(n, &rval);
}

void reg_SET_F(int n, float rval)
{
	reg_SET(n, &rval);
}

SH_DEF(reg)
{
	reg_val_t		rval;
	const reg_t		*reg, *lreg;

	reg = reg_search_fuzzy(s);

	if (reg != NULL) {

		s = sh_next_arg(s);

		if (reg->fmt[1] == 'i') {

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
		}
		else {
			if (stof(&rval.f, s) != NULL) {

				reg_setval(reg, &rval);
			}
		}

		if (iodef_ECHO != 0) {

			reg_format(reg);
		}
	}
	else if (iodef_ECHO != 0) {

		for (reg = regfile; reg->sym != NULL; ++reg) {

			if (strstr(reg->sym, s) != NULL) {

				reg_format(reg);
			}
		}
	}
}

SH_DEF(config_export)
{
	reg_val_t		rval;
	const reg_t		*reg;

	printf("reg %s 0" EOL, regfile[ID_IODEF_ECHO].sym);

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

	printf("reg %s 1" EOL, regfile[ID_IODEF_ECHO].sym);
}

