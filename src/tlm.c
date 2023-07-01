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
	tlm->grabfreq = 0;
	tlm->livefreq = 20;

	tlm->reg_ID[0] = ID_PM_LU_IX;
	tlm->reg_ID[1] = ID_PM_LU_IY;
	tlm->reg_ID[2] = ID_PM_LU_ID;
	tlm->reg_ID[3] = ID_PM_LU_IQ;
	tlm->reg_ID[4] = ID_PM_LU_WS_RPM;
	tlm->reg_ID[5] = ID_PM_VSI_DC;
	tlm->reg_ID[6] = ID_PM_WATT_DRAIN_WA;
	tlm->reg_ID[7] = ID_PM_CONST_FB_U;
	tlm->reg_ID[8] = ID_AP_TEMP_PCB;
	tlm->reg_ID[9] = ID_NULL;
}

void tlm_reg_grab(tlm_t *tlm)
{
	if (tlm->mode == TLM_MODE_DISABLED)
		return ;

	if (tlm->skip == 0) {

		int		N;

		for (N = 0; N < TLM_INPUT_MAX; ++N) {

			if (tlm->reg_ID[N] != ID_NULL) {

				const reg_t	*reg = &regfile[tlm->reg_ID[N]];
				reg_value_t	lval;

				reg_getval(reg, &lval);

				if (		   reg->fmt[1] == 'i'
						|| reg->fmt[2] == 'x') {

					tlm->vm[tlm->line][N] = (uint16_t) lval.i;
				}
				else {
					tlm->vm[tlm->line][N] = tlm_fp_half(lval.f);
				}
			}
		}
	}

	tlm->skip += 1;

	if (tlm->skip >= tlm->span) {

		tlm->clock += 1;
		tlm->skip = 0;

		if (tlm->mode == TLM_MODE_GRAB) {

			tlm->line++;

			if (tlm->line >= TLM_DATA_MAX) {

				tlm->mode = TLM_MODE_DISABLED;
				tlm->line = 0;
			}
		}
		else if (tlm->mode == TLM_MODE_WATCH) {

			tlm->line = (tlm->line < (TLM_DATA_MAX - 1)) ? tlm->line + 1 : 0;

			if (pm.fsm_errno != PM_OK) {

				tlm->mode = TLM_MODE_DISABLED;
			}
		}
		else if (tlm->mode == TLM_MODE_LIVE) {

			tlm->line = (tlm->line < (TLM_DATA_MAX - 1)) ? tlm->line + 1 : 0;
		}
	}
}

void tlm_startup(tlm_t *tlm, int freq, int mode)
{
	tlm->mode = TLM_MODE_DISABLED;

	hal_memory_fence();

	if (freq > 0 && freq < hal.PWM_frequency) {

		tlm->span = (int) (hal.PWM_frequency / (float) freq + .5f);
	}
	else {
		tlm->span = 1;
	}

	tlm->clock = 0;
	tlm->skip = 0;
	tlm->line = 0;

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
	int		freq = tlm.grabfreq;

	stoi(&freq, s);

	tlm_startup(&tlm, freq, TLM_MODE_GRAB);
}

SH_DEF(tlm_watch)
{
	int		freq = tlm.grabfreq;

	stoi(&freq, s);

	tlm_startup(&tlm, freq, TLM_MODE_WATCH);
}

SH_DEF(tlm_stop)
{
	tlm_halt(&tlm);
}

static void
tlm_reg_label(tlm_t *tlm)
{
	const reg_t		*reg;
	const char		*su;
	int			N;

	printf("time@s;");

	for (N = 0; N < TLM_INPUT_MAX; ++N) {

		if (tlm->reg_ID[N] != ID_NULL) {

			reg = &regfile[tlm->reg_ID[N]];

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
	int			N;

	for (N = 0; N < TLM_INPUT_MAX; ++N) {

		if (tlm->reg_ID[N] != ID_NULL) {

			const reg_t	*reg = &regfile[tlm->reg_ID[N]];
			reg_value_t	rval;

			if (		   reg->fmt[1] == 'i'
					|| reg->fmt[2] == 'x') {

				rval.i = (int) tlm->vm[line][N];
			}
			else {
				rval.f = tlm_fp_float(tlm->vm[line][N]);
			}

			reg_format_rval(reg, &rval);

			puts(";");
		}
	}

	puts(EOL);
}

static void
tlm_reg_flush(tlm_t *tlm)
{
	float			time;
	int			line, clock;

	line = tlm->line;
	clock = 0;

	do {
		time = (float) (clock * tlm->span) / hal.PWM_frequency;

		printf("%6f;", &time);

		tlm_reg_flush_line(tlm, line);

		line = (line < (TLM_DATA_MAX - 1)) ? line + 1 : 0;

		clock += 1;
	}
	while (line != tlm->line);
}

SH_DEF(tlm_flush_sync)
{
	if (tlm.mode == TLM_MODE_DISABLED) {

		tlm_reg_label(&tlm);
		tlm_reg_flush(&tlm);
	}
}

void task_tlm_FLUSH(void *pData)
{
	tlm_t		*tlm = (tlm_t *) pData;
	float		time;
	int		line = tlm->line;

	tlm_reg_label(tlm);

	do {
		vTaskDelay((TickType_t) 1);

		while (tlm->line != line) {

			time = (float) (tlm->clock * tlm->span) / hal.PWM_frequency;

			printf("%4f;", &time);

			tlm_reg_flush_line(tlm, line);

			line = (line < (TLM_DATA_MAX - 1)) ? line + 1 : 0;

			hal_memory_fence();
		}
	}
	while (1);
}

SH_DEF(tlm_live_sync)
{
	TaskHandle_t		xHandle;
	int			xC, freq = tlm.livefreq;

	xTaskCreate(task_tlm_FLUSH, "FLUSH", configMINIMAL_STACK_SIZE,
			(void *) &tlm, 1, &xHandle);

	if (stoi(&freq, s) != NULL) {

		freq = (freq < 1) ? 1 : (freq > tlm.livefreq)
			? tlm.livefreq : freq;
	}

	tlm_startup(&tlm, freq, TLM_MODE_LIVE);

	do {
		xC = getc();

		if (xC != K_LF)
			break;
	}
	while (1);

	tlm_halt(&tlm);

	vTaskDelete(xHandle);
}

