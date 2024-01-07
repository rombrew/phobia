#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "tlm.h"
#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

static uint16_t
tlm_fp_half(float x)
{
	union {
		float		f;
		uint32_t	l;
	}
	u = { x };

	return    ((u.l >> 16) & 0xC000U)
		| ((u.l >> 14) & 0x3FFFU);
}

static float
tlm_fp_float(uint16_t x)
{
	union {
		uint32_t	l;
		float		f;
	}
	u = { 		  ((x & 0xC000U) << 16)
			| ((x & 0x3FFFU) << 14) };

	if (		   (x & 0x4000U) == 0
			&& (x & 0x3E00U) != 0) {

		u.l |= 0x3000U << 16;
	}
	else if ((x & 0x7E00U) == 0x7E00U) {

		u.l |= 0x3000U << 16;
	}

	return u.f;
}

void tlm_reg_default(tlm_t *tlm)
{
	tlm->grabfreq = 0.f;
	tlm->livefreq = 20.f;

	tlm->reg_ID[0] = ID_PM_LU_IX;
	tlm->reg_ID[1] = ID_PM_LU_IY;
	tlm->reg_ID[2] = ID_PM_LU_ID;
	tlm->reg_ID[3] = ID_PM_LU_IQ;
	tlm->reg_ID[4] = ID_PM_LU_WS_RPM;
	tlm->reg_ID[5] = ID_PM_VSI_DC;
	tlm->reg_ID[6] = ID_PM_WATT_DRAIN_WA;
	tlm->reg_ID[7] = ID_PM_CONST_FB_U;
	tlm->reg_ID[8] = ID_AP_TEMP_PCB;
	tlm->reg_ID[9] = ID_HAL_CNT_DIAG2_PC;
}

void tlm_reg_grab(tlm_t *tlm)
{
	int			N;

	if (tlm->mode == TLM_MODE_DISABLED)
		return ;

	if (tlm->count == 0) {

		uint16_t	*vm = tlm->vm[tlm->line];

		for (N = 0; N < TLM_INPUT_MAX; ++N) {

			rval_t	*link = tlm->layout[N].reg->link;

			if (tlm->layout[N].type == TLM_TYPE_FLOAT) {

				vm[N] = tlm_fp_half(link->f);
			}
			else if (tlm->layout[N].type == TLM_TYPE_INT) {

				vm[N] = (uint16_t) link->i;
			}
		}
	}

	tlm->count += 1;

	if (tlm->count >= tlm->span) {

		tlm->clock += 1;
		tlm->count = 0;

		tlm->line = (tlm->line < (TLM_DATA_MAX - 1)) ? tlm->line + 1 : 0;

		if (tlm->mode == TLM_MODE_GRAB) {

			if (tlm->clock >= TLM_DATA_MAX) {

				tlm->mode = TLM_MODE_DISABLED;
			}
		}
		else if (tlm->mode == TLM_MODE_WATCH) {

			if (pm.fsm_errno != PM_OK) {

				tlm->mode = TLM_MODE_DISABLED;
			}
		}
	}
}

void tlm_startup(tlm_t *tlm, float freq, int mode)
{
	int			N;

	tlm->mode = TLM_MODE_DISABLED;

	hal_memory_fence();

	for (N = 0; N < TLM_INPUT_MAX; ++N) {

		if (tlm->reg_ID[N] != ID_NULL) {

			const reg_t	*reg = &regfile[tlm->reg_ID[N]];

			if (		   reg->fmt[2] == 'i'
					|| reg->fmt[2] == 'x') {

				tlm->layout[N].type = TLM_TYPE_INT;
			}
			else {
				tlm->layout[N].type = TLM_TYPE_FLOAT;
			}

			tlm->layout[N].reg = reg;
		}
		else {
			tlm->layout[N].type = TLM_TYPE_NONE;
		}
	}

	tlm->clock = 0;
	tlm->count = 0;

	tlm->span = (freq >= 0.1f && freq <= hal.PWM_frequency)
		? (int) (hal.PWM_frequency / freq + .5f) : 1;

	hal_memory_fence();

	tlm->mode = mode;
}

void tlm_halt(tlm_t *tlm)
{
	tlm->mode = TLM_MODE_DISABLED;

	hal_memory_fence();
}

SH_DEF(tlm_default)
{
	tlm_reg_default(&tlm);
}

SH_DEF(tlm_grab)
{
	float		freq = tlm.grabfreq;

	stof(&freq, s);

	tlm_startup(&tlm, freq, TLM_MODE_GRAB);
}

SH_DEF(tlm_watch)
{
	float		freq = tlm.grabfreq;

	stof(&freq, s);

	tlm_startup(&tlm, freq, TLM_MODE_WATCH);
}

SH_DEF(tlm_stop)
{
	tlm_halt(&tlm);
}

static void
tlm_reg_label(tlm_t *tlm)
{
	const char		*su;
	int			N;

	printf("time@s;");

	for (N = 0; N < TLM_INPUT_MAX; ++N) {

		if (tlm->layout[N].type != TLM_TYPE_NONE) {

			const reg_t	*reg = tlm->layout[N].reg;

			puts(reg->sym);

			su = reg->sym + strlen(reg->sym) + 1;

			if (*su != 0) {

				printf("@%s", su);
			}

			puts(";");
		}
	}

	puts(EOL);
}

static void
tlm_reg_flush_line(tlm_t *tlm, int line)
{
	const uint16_t		*vm = tlm->vm[line];
	int			N;

	for (N = 0; N < TLM_INPUT_MAX; ++N) {

		if (tlm->layout[N].type != TLM_TYPE_NONE) {

			const reg_t	*reg = tlm->layout[N].reg;
			rval_t		rval;

			if (tlm->layout[N].type == TLM_TYPE_INT) {

				rval.i = (int) vm[N];
			}
			else {
				rval.f = tlm_fp_float(vm[N]);
			}

			if (reg->proc != NULL) {

				reg_t		lreg = { .link = &rval };

				reg->proc(&lreg, &rval, NULL);
			}

			reg_format_rval(reg, &rval);

			puts(";");
		}
	}

	puts(EOL);
}

SH_DEF(tlm_flush_sync)
{
	float			time, dT;
	int			line, clock, precision;

	if (tlm.mode != TLM_MODE_DISABLED)
		return ;

	line = tlm.line;
	clock = 0;

	dT = (float) tlm.span / hal.PWM_frequency;

	precision = (int) (2.9f - m_log10f(dT));
	precision = (precision < 2) ? 2
		  : (precision > 7) ? 7 : precision;

	tlm_reg_label(&tlm);

	do {
		time = (float) clock * dT;

		printf("%*f;", precision, &time);

		tlm_reg_flush_line(&tlm, line);

		line = (line < (TLM_DATA_MAX - 1)) ? line + 1 : 0;

		clock += 1;

		if (		   poll() != 0
				&& getc() != K_LF)
			break;
	}
	while (line != tlm.line);
}

SH_DEF(tlm_live_sync)
{
	float			time, freq, dT;
	int			line, clock, precision;

	if (tlm.mode != TLM_MODE_DISABLED)
		return ;

	freq = tlm.livefreq;

	if (stof(&freq, s) != NULL) {

		freq =    (freq < 1.f) ? 1.f
			: (freq > tlm.livefreq) ? tlm.livefreq : freq;
	}

	tlm_startup(&tlm, freq, TLM_MODE_LIVE);

	line = tlm.line;
	clock = 0;

	dT = (float) tlm.span / hal.PWM_frequency;

	precision = (int) (2.9f - m_log10f(dT));
	precision = (precision < 2) ? 2
		  : (precision > 7) ? 7 : precision;

	tlm_reg_label(&tlm);

	do {
		vTaskDelay((TickType_t) 1);

		while (tlm.line != line) {

			time = (float) clock * dT;

			printf("%*f;", precision, &time);

			tlm_reg_flush_line(&tlm, line);

			line = (line < (TLM_DATA_MAX - 1)) ? line + 1 : 0;

			clock += 1;

			hal_memory_fence();
		}

		if (		   poll() != 0
				&& getc() != K_LF)
			break;
	}
	while (1);

	tlm_halt(&tlm);
}

