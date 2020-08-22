#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "tlm.h"
#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

void TLM_reg_default(TLM_t *tlm)
{
	tlm->reg_ID[0] = ID_PM_FB_IA;
	tlm->reg_ID[1] = ID_PM_FB_IB;
	tlm->reg_ID[2] = ID_PM_LU_ID;
	tlm->reg_ID[3] = ID_PM_LU_IQ;
	tlm->reg_ID[4] = ID_PM_LU_F_G;
	tlm->reg_ID[5] = ID_PM_LU_WS_RPM;
	tlm->reg_ID[6] = ID_PM_S_INTEGRAL;
	tlm->reg_ID[7] = ID_PM_WATT_LPF_WP;
	tlm->reg_ID[8] = ID_PM_CONST_FB_U;
	tlm->reg_ID[9] = ID_AP_TEMP_PCB;
}

void TLM_reg_grab(TLM_t *tlm)
{
	const reg_t		*reg;
	int			N;

	if (tlm->mode != TLM_MODE_DISABLED) {

		tlm->i++;

		if (tlm->i == 1) {

			for (N = 0; N < TLM_INPUT_MAX; ++N) {

				if (tlm->reg_ID[N] != ID_NULL) {

					/* Grab the value if register is not null.
					 * */
					reg = &regfile[tlm->reg_ID[N]];
					reg_getval(reg, &tlm->data[tlm->n][N]);
				}
			}
		}

		if (tlm->i >= tlm->tim) {

			tlm->i = 0;

			if (tlm->mode == TLM_MODE_SINGLE_GRAB) {

				tlm->n++;

				if (tlm->n >= TLM_DATA_MAX) {

					tlm->mode = TLM_MODE_DISABLED;
				}
			}
			else if (tlm->mode == TLM_MODE_LIVE) {

				tlm->n = (tlm->n < (TLM_DATA_MAX - 1)) ? tlm->n + 1 : 0;
			}
		}
	}
}

void TLM_startup(TLM_t *tlm, int freq, int mode)
{
	if (freq > 0 && freq < hal.PWM_frequency) {

		tlm->tim = (int) (hal.PWM_frequency / (float) freq + .5f);
	}
	else {
		tlm->tim = 1;
	}

	tlm->i = 0;
	tlm->n = 0;

	hal_fence();
	tlm->mode = mode;
}

void TLM_halt(TLM_t *tlm)
{
	tlm->mode = TLM_MODE_DISABLED;
	hal_fence();
}

SH_DEF(tlm_grab)
{
	int		freq = 0;

	stoi(&freq, s);
	TLM_startup(&tlm, freq, TLM_MODE_SINGLE_GRAB);
}

SH_DEF(tlm_stop)
{
	TLM_halt(&tlm);
}

void TLM_reg_labels(TLM_t *tlm)
{
	const reg_t		*reg;
	const char		*su;
	int			N;

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

void TLM_reg_flush(TLM_t *tlm)
{
	const reg_t		*reg;
	int			K, N;

	for (K = 0; K < TLM_DATA_MAX; ++K) {

		for (N = 0; N < TLM_INPUT_MAX; ++N) {

			if (tlm->reg_ID[N] != ID_NULL) {

				reg = &regfile[tlm->reg_ID[N]];
				reg_format_rval(reg, &tlm->data[K][N]);

				puts(";");
			}
		}

		puts(EOL);
	}
}

void TLM_reg_flush_1(TLM_t *tlm, int nR)
{
	const reg_t		*reg;
	int			N;

	for (N = 0; N < TLM_INPUT_MAX; ++N) {

		if (tlm->reg_ID[N] != ID_NULL) {

			reg = &regfile[tlm->reg_ID[N]];
			reg_format_rval(reg, &tlm->data[nR][N]);

			puts(";");
		}
	}

	puts(EOL);
}

SH_DEF(tlm_flush_sync)
{
	if (tlm.mode == TLM_MODE_DISABLED) {

		TLM_reg_labels(&tlm);
		TLM_reg_flush(&tlm);
	}
}

void task_LIVE(void *pData)
{
	TLM_t		*tlm = (TLM_t *) pData;
	int		nR = tlm->n;

	TLM_reg_labels(tlm);

	do {
		vTaskDelay((TickType_t) 5);

		while (tlm->n != nR) {

			TLM_reg_flush_1(tlm, nR);
			nR = (nR < (TLM_DATA_MAX - 1)) ? nR + 1 : 0;

			hal_fence();
		}
	}
	while (1);
}

SH_DEF(tlm_live_sync)
{
	TaskHandle_t		xHandle;
	int			freq = 20;

	xTaskCreate(task_LIVE, "LIVE", configMINIMAL_STACK_SIZE, (void *) &tlm, 1, &xHandle);

	if (stoi(&freq, s) != NULL) {

		freq = (freq < 1) ? 1 : (freq > 100) ? 100 : freq;
	}

	TLM_startup(&tlm, freq, TLM_MODE_LIVE);

	getc();

	TLM_halt(&tlm);
	vTaskDelete(xHandle);
}

