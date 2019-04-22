#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "tel.h"
#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

void tel_reg_default(tel_t *ti)
{
	ti->reg_ID[0] = ID_PM_FB_CURRENT_A;
	ti->reg_ID[1] = ID_PM_FB_CURRENT_B;
	ti->reg_ID[2] = ID_PM_LU_X_0;
	ti->reg_ID[3] = ID_PM_LU_X_1;
	ti->reg_ID[4] = ID_PM_LU_X_4_RPM;
	ti->reg_ID[5] = ID_PM_FLUX_DRIFT_Q;
	ti->reg_ID[6] = ID_PM_FLUX_RESIDUE_LPF;
	ti->reg_ID[7] = ID_PM_VSI_LPF_WATT;
	ti->reg_ID[8] = ID_PM_CONST_LPF_U;
	ti->reg_ID[9] = ID_AP_TEMP_PCB;
}

void tel_reg_grab(tel_t *ti)
{
	const reg_t		*reg;
	int			N;

	if (ti->mode != 0) {

		ti->i++;

		if (ti->i == 1) {

			for (N = 0; N < TEL_INPUT_MAX; ++N) {

				if (ti->reg_ID[N] != ID_NULL) {

					/* Grab the value is register is not null.
					 * */
					reg = &regfile[ti->reg_ID[N]];
					reg_getval(reg, &ti->data[ti->n][N]);
				}
			}
		}

		if (ti->i >= ti->d) {

			ti->i = 0;

			if (ti->mode == TEL_MODE_SINGLE_GRAB) {

				ti->n++;

				if (ti->n >= TEL_DATA_MAX) {

					ti->mode = 0;
				}
			}
			else if (ti->mode == TEL_MODE_LIVE) {

				ti->n = (ti->n < (TEL_DATA_MAX - 1)) ? ti->n + 1 : 0;
			}
		}
	}
}

void tel_startup(tel_t *ti, int freq, int mode)
{
	if (freq > 0 && freq <= hal.PWM_frequency) {

		ti->d = (hal.PWM_frequency + freq / 2) / freq;
	}
	else {
		ti->d = 1;
	}

	ti->i = 0;
	ti->n = 0;

	hal_fence();
	ti->mode = mode;
}

void tel_halt(tel_t *ti)
{
	ti->mode = 0;
	hal_fence();
}

SH_DEF(tel_grab)
{
	int		freq = 0;

	stoi(&freq, s);
	tel_startup(&ti, freq, TEL_MODE_SINGLE_GRAB);
}

SH_DEF(tel_stop)
{
	tel_halt(&ti);
}

void tel_reg_labels(tel_t *ti)
{
	const reg_t		*reg;
	const char		*su;
	int			N;

	for (N = 0; N < TEL_INPUT_MAX; ++N) {

		if (ti->reg_ID[N] != ID_NULL) {

			reg = &regfile[ti->reg_ID[N]];

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

void tel_reg_flush(tel_t *ti)
{
	const reg_t		*reg;
	int			K, N;

	for (K = 0; K < TEL_DATA_MAX; ++K) {

		for (N = 0; N < TEL_INPUT_MAX; ++N) {

			if (ti->reg_ID[N] != ID_NULL) {

				reg = &regfile[ti->reg_ID[N]];
				reg_format_rval(reg, &ti->data[K][N]);

				puts(";");
			}
		}

		puts(EOL);
	}
}

void tel_reg_flush_1(tel_t *ti, int nR)
{
	const reg_t		*reg;
	int			N;

	for (N = 0; N < TEL_INPUT_MAX; ++N) {

		if (ti->reg_ID[N] != ID_NULL) {

			reg = &regfile[ti->reg_ID[N]];
			reg_format_rval(reg, &ti->data[nR][N]);

			puts(";");
		}
	}

	puts(EOL);
}

SH_DEF(tel_flush_sync)
{
	if (ti.mode == TEL_MODE_DISABLED) {

		tel_reg_labels(&ti);
		tel_reg_flush(&ti);
	}
}

void task_LIVE(void *pData)
{
	tel_t		*ti = (tel_t *) pData;
	int		nR = ti->n;

	tel_reg_labels(ti);

	do {
		vTaskDelay((TickType_t) 5);

		while (ti->n != nR) {

			tel_reg_flush_1(ti, nR);
			nR = (nR < (TEL_DATA_MAX - 1)) ? nR + 1 : 0;

			hal_fence();
		}
	}
	while (1);
}

SH_DEF(tel_live_sync)
{
	TaskHandle_t		xHandle;
	int			freq = 20;

	xTaskCreate(task_LIVE, "LIVE", configMINIMAL_STACK_SIZE, (void *) &ti, 1, &xHandle);

	if (stoi(&freq, s) != NULL) {

		freq = (freq < 1) ? 1 : (freq > 50) ? 50 : freq;
	}

	tel_startup(&ti, freq, TEL_MODE_LIVE);

	getc();

	tel_halt(&ti);
	vTaskDelete(xHandle);
}

