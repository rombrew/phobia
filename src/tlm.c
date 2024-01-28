#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "tlm.h"
#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

void tlm_reg_default(tlm_t *tlm)
{
	tlm->rate_grab = 5;
	tlm->rate_live = (int) (hal.PWM_frequency / 10.f + 0.5f);

	tlm->reg_ID[0] = ID_PM_FB_IA;
	tlm->reg_ID[1] = ID_PM_FB_IB;
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

	if (unlikely(tlm->mode == TLM_MODE_DISABLED))
		return ;

	if (tlm->skip == 0) {

		rval_t		*rdata = tlm->rdata + tlm->layout_N * tlm->line;

		for (N = 0; N < tlm->layout_N; ++N) {

			rdata[N] = *(tlm->layout_reg[N]->link);
		}
	}

	tlm->skip += 1;

	if (tlm->skip >= tlm->rate) {

		tlm->clock += 1;
		tlm->skip = 0;

		tlm->line = (tlm->line < (tlm->length_MAX - 1)) ? tlm->line + 1 : 0;

		if (tlm->mode == TLM_MODE_GRAB) {

			if (tlm->clock >= tlm->length_MAX) {

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

void tlm_startup(tlm_t *tlm, int rate, int mode)
{
	int			N, layout_N = 0;

	tlm->mode = TLM_MODE_DISABLED;

	hal_memory_fence();

	for (N = 0; N < TLM_INPUT_MAX; ++N) {

		if (tlm->reg_ID[N] != ID_NULL) {

			const reg_t	*reg = &regfile[tlm->reg_ID[N]];

			tlm->layout_reg[layout_N++] = reg;
		}
	}

	tlm->layout_N = layout_N;
	tlm->length_MAX = TLM_DATA_MAX / layout_N;

	tlm->clock = 0;
	tlm->skip = 0;

	tlm->rate = rate;

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
	int		rate = tlm.rate_grab;

	stoi(&rate, s);

	tlm_startup(&tlm, rate, TLM_MODE_GRAB);
}

SH_DEF(tlm_watch)
{
	int		rate = tlm.rate_grab;

	stoi(&rate, s);

	tlm_startup(&tlm, rate, TLM_MODE_WATCH);
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

	for (N = 0; N < tlm->layout_N; ++N) {

		const reg_t	*reg = tlm->layout_reg[N];

		puts(reg->sym);

		su = reg->sym + strlen(reg->sym) + 1;

		if (*su != 0) {

			printf("@%s", su);
		}

		puts(";");
	}

	puts(EOL);
}

static void
tlm_reg_flush_line(tlm_t *tlm, int line)
{
	const rval_t		*rdata = tlm->rdata + tlm->layout_N * line;
	int			N;

	for (N = 0; N < tlm->layout_N; ++N) {

		const reg_t	*reg = tlm->layout_reg[N];
		rval_t		rval = rdata[N];

		if (reg->proc != NULL) {

			reg_t		lreg = { .link = &rval };

			reg->proc(&lreg, &rval, NULL);
		}

		reg_format_rval(reg, &rval);

		puts(";");
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

	dT = (float) tlm.rate / hal.PWM_frequency;

	precision = (int) (2.9f - m_log10f(dT));
	precision = (precision < 2) ? 2
		  : (precision > 7) ? 7 : precision;

	tlm_reg_label(&tlm);

	do {
		time = (float) clock * dT;

		printf("%*f;", precision, &time);

		tlm_reg_flush_line(&tlm, line);

		line = (line < (tlm.length_MAX - 1)) ? line + 1 : 0;

		clock += 1;

		if (		   poll() != 0
				&& getc() != K_LF)
			break;
	}
	while (line != tlm.line);
}

SH_DEF(tlm_live_sync)
{
	float			time, dT;
	int			line, clock, rate, precision;

	if (tlm.mode != TLM_MODE_DISABLED)
		return ;

	rate = tlm.rate_live;

	if (stoi(&rate, s) != NULL) {

		int		rate_minimal = (int) (hal.PWM_frequency / 100.f + 0.5f);

		rate =    (rate < rate_minimal) ? rate_minimal
			: (rate > tlm.rate_live) ? tlm.rate_live : rate;
	}

	tlm_startup(&tlm, rate, TLM_MODE_LIVE);

	line = tlm.line;
	clock = 0;

	dT = (float) tlm.rate / hal.PWM_frequency;

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

			line = (line < (tlm.length_MAX - 1)) ? line + 1 : 0;

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

