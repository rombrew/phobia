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
	tlm->grabfreq = 0;
	tlm->livefreq = 10;

	tlm->reg_ID[0] = ID_PM_LU_ID;
	tlm->reg_ID[1] = ID_PM_LU_IQ;
	tlm->reg_ID[2] = ID_PM_LU_WS_RPM;
	tlm->reg_ID[3] = ID_PM_LU_MQ_LOAD;
	tlm->reg_ID[4] = ID_PM_WATT_DRAIN_WA;
	tlm->reg_ID[5] = ID_PM_CONST_FB_U;
	tlm->reg_ID[6] = ID_AP_TEMP_PCB;
	tlm->reg_ID[7] = ID_NULL;
	tlm->reg_ID[8] = ID_NULL;
	tlm->reg_ID[9] = ID_NULL;
}

void tlm_reg_grab(tlm_t *tlm)
{
	int			N;

	if (tlm->mode != TLM_MODE_DISABLED) {

		if (tlm->skip == 0) {

			for (N = 0; N < TLM_INPUT_MAX; ++N) {

				if (tlm->reg_ID[N] != ID_NULL) {

					/* Grab the register VALUE.
					 * */
					reg_GET(tlm->reg_ID[N], &tlm->data[tlm->N][N]);
				}
			}
		}

		tlm->skip += 1;

		if (tlm->skip >= tlm->span) {

			tlm->clock += 1;
			tlm->skip = 0;

			if (tlm->mode == TLM_MODE_GRAB) {

				tlm->N++;

				if (tlm->N >= TLM_DATA_MAX) {

					tlm->mode = TLM_MODE_DISABLED;
					tlm->N = 0;
				}
			}
			else if (tlm->mode == TLM_MODE_WATCH) {

				tlm->N = (tlm->N < (TLM_DATA_MAX - 1)) ? tlm->N + 1 : 0;

				if (pm.lu_MODE == PM_DISABLED) {

					tlm->mode = TLM_MODE_DISABLED;
				}
			}
			else if (tlm->mode == TLM_MODE_LIVE) {

				tlm->N = (tlm->N < (TLM_DATA_MAX - 1)) ? tlm->N + 1 : 0;
			}
		}
	}
}

void tlm_startup(tlm_t *tlm, int freq, int mode)
{
	if (tlm->mode != TLM_MODE_DISABLED) {

		tlm->mode = TLM_MODE_DISABLED;

		hal_memory_fence();
	}

	if (freq > 0 && freq < hal.PWM_frequency) {

		tlm->span = (int) (hal.PWM_frequency / (float) freq + .5f);
	}
	else {
		tlm->span = 1;
	}

	tlm->clock = 0;
	tlm->skip = 0;

	tlm->N = 0;

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
tlm_reg_flush_line(tlm_t *tlm, int tlm_N)
{
	const reg_t		*reg;
	int			N;

	for (N = 0; N < TLM_INPUT_MAX; ++N) {

		if (tlm->reg_ID[N] != ID_NULL) {

			reg = &regfile[tlm->reg_ID[N]];
			reg_format_rval(reg, &tlm->data[tlm_N][N]);

			puts(";");
		}
	}

	puts(EOL);
}

static void
tlm_reg_flush(tlm_t *tlm)
{
	float			time;
	int			N, clock;

	N = tlm->N;
	clock = 0;

	do {
		time = (float) (clock * tlm->span) / hal.PWM_frequency;

		printf("%6f;", &time);

		tlm_reg_flush_line(tlm, N);

		N = (N < (TLM_DATA_MAX - 1)) ? N + 1 : 0;

		clock += 1;
	}
	while (N != tlm->N);
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
	int		N = tlm->N;

	tlm_reg_label(tlm);

	do {
		vTaskDelay((TickType_t) 5);

		while (tlm->N != N) {

			time = (float) (tlm->clock * tlm->span) / hal.PWM_frequency;

			printf("%6f;", &time);

			tlm_reg_flush_line(tlm, N);

			N = (N < (TLM_DATA_MAX - 1)) ? N + 1 : 0;

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

