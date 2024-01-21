/*
   Graph Plotter is a tool to analyse numerical data.
   Copyright (C) 2023 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "plot.h"
#include "read.h"
#include "draw.h"
#include "lse.h"
#include "lz4.h"
#include "scheme.h"

extern SDL_RWops *TTF_RW_roboto_mono_normal();
extern SDL_RWops *TTF_RW_roboto_mono_thin();

double fp_nan()
{
	union {
		unsigned long long	l;
		double			f;
	}
	u = { 0xFFF8000000000000ULL };

	return u.f;
}

int fp_isfinite(double x)
{
	union {
		double			f;
		unsigned long long	l;
	}
	u = { x };

	return ((0x7FFUL & (unsigned long) (u.l >> 52)) != 0x7FFUL) ? 1 : 0;
}

plot_t *plotAlloc(draw_t *dw, scheme_t *sch)
{
	plot_t		*pl;
	int		N;

	pl = calloc(1, sizeof(plot_t));

	pl->dw = dw;
	pl->sch = sch;

	for (N = 0; N < PLOT_SKETCH_MAX - 1; ++N)
		pl->sketch[N].linked = N + 1;

	pl->sketch[PLOT_SKETCH_MAX - 1].linked = -1;

	pl->sketch_list_garbage = 0;
	pl->sketch_list_todraw = -1;
	pl->sketch_list_current = -1;
	pl->sketch_list_current_end = -1;

	for (N = 0; N < PLOT_FIGURE_MAX; ++N)
		pl->draw[N].list_self = -1;

	pl->layout_font_long = 11;
	pl->layout_font_space = 16;
	pl->layout_border = 5;
	pl->layout_tick_tooth = 5;
	pl->layout_grid_dash = 2;
	pl->layout_grid_space = 8;
	pl->layout_drawing_dash = 8;
	pl->layout_drawing_space = 12;
	pl->layout_fence_dash = 10;
	pl->layout_fence_space = 10;
	pl->layout_fence_point = 10;

	pl->interpolation = 1;
	pl->defungap = 10;

	pl->mark_size = 40;
	pl->mark_density = 40;

	pl->default_drawing = FIGURE_DRAWING_LINE;
	pl->default_width = 2;

	pl->transparency = 1;
	pl->fprecision = 9;
	pl->lz4_compress = 1;

	return pl;
}

static void
plotSketchFree(plot_t *pl)
{
	int		N;

	plotSketchClean(pl);

	for (N = 0; N < PLOT_SKETCH_MAX; ++N) {

		if (pl->sketch[N].chunk != NULL) {

			free(pl->sketch[N].chunk);

			pl->sketch[N].chunk = NULL;
		}
	}
}

void plotClean(plot_t *pl)
{
	int		dN;

	drawPixmapClean(pl->dw);
	plotSketchFree(pl);

	for (dN = 0; dN < PLOT_DATASET_MAX; ++dN) {

		if (pl->data[dN].column_N != 0)
			plotDataClean(pl, dN);
	}

	free(pl);
}

static void
plotFontLayout(plot_t *pl)
{
	TTF_SizeUTF8(pl->font, "M", &pl->layout_font_long, &pl->layout_font_height);

	pl->layout_font_height = TTF_FontHeight(pl->font);

	pl->layout_font_space = pl->layout_font_long * 14 / 10;
	pl->layout_axis_box = pl->layout_tick_tooth + pl->layout_font_height;
	pl->layout_label_box = pl->layout_font_height;

	pl->layout_mark_size = pl->layout_font_height * pl->mark_size / 200;
	pl->layout_mark_size = (pl->layout_mark_size < 1) ? 1 : pl->layout_mark_size;
}

void plotFontDefault(plot_t *pl, int ttfnum, int ptsize, int style)
{
	if (pl->font != NULL) {

		TTF_CloseFont(pl->font);

		pl->font = NULL;
	}

	switch (ttfnum) {

		default:
			ttfnum = TTF_ID_ROBOTO_MONO_NORMAL;

		case TTF_ID_ROBOTO_MONO_NORMAL:
			pl->font = TTF_OpenFontRW(TTF_RW_roboto_mono_normal(), 1, ptsize);
			break;

		case TTF_ID_ROBOTO_MONO_THIN:
			pl->font = TTF_OpenFontRW(TTF_RW_roboto_mono_thin(), 1, ptsize);
			break;
	}

	TTF_SetFontStyle(pl->font, style);

	pl->layout_font_ttf = ttfnum;
	pl->layout_font_pt = ptsize;

	plotFontLayout(pl);
}

int plotFontOpen(plot_t *pl, const char *ttf, int ptsize, int style)
{
	if (pl->font != NULL) {

		TTF_CloseFont(pl->font);

		pl->font = NULL;
	}

	pl->font = TTF_OpenFont(ttf, ptsize);

	if (pl->font == NULL) {

		ERROR("TTF_OpenFont: \"%s\"\n", TTF_GetError());
		return -1;
	}

	TTF_SetFontStyle(pl->font, style);

	pl->layout_font_ttf = 0;
	pl->layout_font_pt = ptsize;

	plotFontLayout(pl);

	return 0;
}

static void
plotDataChunkAlloc(plot_t *pl, int dN, int lN)
{
	int		N, kN, lSHIFT;

	lSHIFT = pl->data[dN].chunk_SHIFT;

	kN = (lN & pl->data[dN].chunk_MASK) ? 1 : 0;
	kN += lN >> lSHIFT;

	if (kN > PLOT_CHUNK_MAX) {

		kN = PLOT_CHUNK_MAX;
		lN = kN * (1UL << lSHIFT);
	}

	if (pl->data[dN].lz4_compress != 0) {

		for (N = kN; N < PLOT_CHUNK_MAX; ++N) {

			if (pl->data[dN].compress[N].raw != NULL) {

				free(pl->data[dN].compress[N].raw);

				pl->data[dN].compress[N].raw = NULL;
			}
		}
	}
	else {
		for (N = 0; N < kN; ++N) {

			if (pl->data[dN].raw[N] == NULL) {

				pl->data[dN].raw[N] = (fval_t *) malloc(pl->data[dN].chunk_bSIZE);

				if (pl->data[dN].raw[N] == NULL) {

					lN = N * (1UL << lSHIFT);

					ERROR("Unable to allocate memory of %i dataset\n", dN);
					break;
				}
			}
		}

		for (N = kN; N < PLOT_CHUNK_MAX; ++N) {

			if (pl->data[dN].raw[N] != NULL) {

				free(pl->data[dN].raw[N]);

				pl->data[dN].raw[N] = NULL;
			}
		}
	}

	pl->data[dN].length_N = lN;
}

unsigned long long plotDataMemoryUsage(plot_t *pl, int dN)
{
	int			N;
	unsigned long long	bUSAGE;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return 0;
	}

	bUSAGE = 0;

	for (N = 0; N < PLOT_CHUNK_MAX; ++N) {

		if (pl->data[dN].raw[N] != NULL) {

			bUSAGE += pl->data[dN].chunk_bSIZE;
		}

		if (pl->data[dN].compress[N].raw != NULL) {

			bUSAGE += pl->data[dN].compress[N].length;
		}
	}

	return bUSAGE;
}

unsigned long long plotDataMemoryUncompressed(plot_t *pl, int dN)
{
	int			N;
	unsigned long long	bUSAGE;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return 0;
	}

	bUSAGE = 0;

	for (N = 0; N < PLOT_CHUNK_MAX; ++N) {

		if (		pl->data[dN].raw[N] != NULL
				|| pl->data[dN].compress[N].raw != NULL) {

			bUSAGE += pl->data[dN].chunk_bSIZE;
		}
	}

	return bUSAGE;
}

unsigned long long plotDataMemoryCached(plot_t *pl, int dN)
{
	int			N;
	unsigned long long	bUSAGE;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return 0;
	}

	bUSAGE = 0;

	for (N = 0; N < PLOT_CHUNK_CACHE; ++N) {

		if (pl->data[dN].cache[N].raw != NULL) {

			bUSAGE += pl->data[dN].chunk_bSIZE;
		}
	}

	return bUSAGE;
}

static int
plotDataCacheGetNode(plot_t *pl, int dN, int kN)
{
	int		N, kNOT, xN = -1;

	for (N = 0; N < PLOT_CHUNK_CACHE; ++N) {

		if (pl->data[dN].cache[N].raw == NULL) {

			xN = N;
			break;
		}
	}

	if (xN < 0) {

		kNOT = pl->data[dN].tail_N >> pl->data[dN].chunk_SHIFT;

		N = (pl->data[dN].cache_ID < PLOT_CHUNK_CACHE - 1)
			? pl->data[dN].cache_ID + 1 : 0;

		if (pl->data[dN].cache[N].chunk_N == kNOT) {

			N = (N < PLOT_CHUNK_CACHE - 1) ? N + 1 : 0;
		}

		xN = N;

		pl->data[dN].cache_ID = N;
	}

	return xN;
}

static void
plotDataCacheFetch(plot_t *pl, int dN, int kN)
{
	int		xN, kNZ, lzLEN;

	xN = plotDataCacheGetNode(pl, dN, kN);

	if (pl->data[dN].cache[xN].raw != NULL) {

		kNZ = pl->data[dN].cache[xN].chunk_N;

		if (pl->data[dN].cache[xN].dirty != 0) {

			lzLEN = LZ4_compressBound(pl->data[dN].chunk_bSIZE);

			if (pl->data[dN].compress[kNZ].raw != NULL) {

				free(pl->data[dN].compress[kNZ].raw);
			}

			pl->data[dN].compress[kNZ].raw = (void *) malloc(lzLEN);

			if (pl->data[dN].compress[kNZ].raw == NULL) {

				ERROR("Unable to allocate LZ4 memory of %i dataset\n", dN);
			}

			lzLEN = LZ4_compress_default(
					(const char *) pl->data[dN].cache[xN].raw,
					(char *) pl->data[dN].compress[kNZ].raw,
					pl->data[dN].chunk_bSIZE, lzLEN);

			if (lzLEN > 0) {

				pl->data[dN].compress[kNZ].raw =
					realloc(pl->data[dN].compress[kNZ].raw, lzLEN);
				pl->data[dN].compress[kNZ].length = lzLEN;
			}
			else {
				ERROR("Unable to compress the chunk of %i dataset\n", dN);

				free(pl->data[dN].compress[kNZ].raw);

				pl->data[dN].compress[kNZ].raw = NULL;
				pl->data[dN].compress[kNZ].length = 0;
			}
		}

		pl->data[dN].raw[kNZ] = NULL;
	}
	else {
		pl->data[dN].cache[xN].raw = (fval_t *) malloc(pl->data[dN].chunk_bSIZE);

		if (pl->data[dN].cache[xN].raw == NULL) {

			ERROR("Unable to allocate cache of %i dataset\n", dN);
		}
	}

	pl->data[dN].cache[xN].chunk_N = kN;
	pl->data[dN].cache[xN].dirty = 0;

	pl->data[dN].raw[kN] = pl->data[dN].cache[xN].raw;

	if (pl->data[dN].compress[kN].raw != NULL) {

		lzLEN = LZ4_decompress_safe(
				(const char *) pl->data[dN].compress[kN].raw,
				(char *) pl->data[dN].raw[kN],
				pl->data[dN].compress[kN].length,
				pl->data[dN].chunk_bSIZE);

		if (lzLEN != pl->data[dN].chunk_bSIZE) {

			ERROR("Unable to decompress the chunk of %i dataset\n", dN);
		}
	}
}

static void
plotDataChunkFetch(plot_t *pl, int dN, int kN)
{
	if (		   pl->data[dN].raw[kN] == NULL
			&& pl->data[dN].length_N != 0) {

		plotDataCacheFetch(pl, dN, kN);
	}
}

static void
plotDataChunkWrite(plot_t *pl, int dN, int kN)
{
	int		N;

	if (		   pl->data[dN].raw[kN] == NULL
			&& pl->data[dN].length_N != 0) {

		plotDataCacheFetch(pl, dN, kN);
	}

	if (pl->data[dN].raw[kN] != NULL) {

		for (N = 0; N < PLOT_CHUNK_CACHE; ++N) {

			if (pl->data[dN].cache[N].chunk_N == kN) {

				pl->data[dN].cache[N].dirty = 1;
				break;
			}
		}
	}
}

void plotDataAlloc(plot_t *pl, int dN, int cN, int lN)
{
	int		*map;
	int		N, bSIZE;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return ;
	}

	if (cN < 1) {

		ERROR("Number of columns is too few\n");
		return ;
	}

	if (lN < 1) {

		ERROR("Length of dataset is too short\n");
		return ;
	}

	if (pl->data[dN].column_N != 0) {

		if (pl->data[dN].column_N != cN) {

			ERROR("Number of columns cannot be changed\n");
			return ;
		}

		plotSketchClean(pl);

		plotDataRangeCacheClean(pl, dN);
		plotDataChunkAlloc(pl, dN, lN);

		pl->data[dN].head_N = 0;
		pl->data[dN].tail_N = 0;
		pl->data[dN].id_N = 0;
		pl->data[dN].sub_N = 0;
	}
	else {
		pl->data[dN].column_N = cN;

		for (N = 0; N < 30; ++N) {

			bSIZE = sizeof(fval_t) * (cN + PLOT_SUBTRACT) * (1UL << N);

			if (bSIZE >= PLOT_CHUNK_SIZE) {

				pl->data[dN].chunk_SHIFT = N;
				pl->data[dN].chunk_MASK = (1UL << N) - 1UL;
				pl->data[dN].chunk_bSIZE = bSIZE;
				break;
			}
		}

		pl->data[dN].lz4_compress = pl->lz4_compress;

		plotDataChunkAlloc(pl, dN, lN);

		pl->data[dN].cache_ID = 0;

		pl->data[dN].head_N = 0;
		pl->data[dN].tail_N = 0;
		pl->data[dN].id_N = 0;
		pl->data[dN].sub_N = 0;

		for (N = 0; N < PLOT_SUBTRACT; ++N) {

			pl->data[dN].sub[N].busy = SUBTRACT_FREE;
		}

		map = (int *) malloc(sizeof(int) * (cN + PLOT_SUBTRACT + 1));

		if (map == NULL) {

			ERROR("No memory allocated for %i map\n", dN);
			return ;
		}

		pl->data[dN].map = (int *) map + 1;

		for (N = -1; N < (cN + PLOT_SUBTRACT); ++N) {

			pl->data[dN].map[N] = -1;
		}
	}
}

void plotDataResize(plot_t *pl, int dN, int lN)
{
	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return ;
	}

	if (lN < 1) {

		ERROR("Length of dataset is too short\n");
		return ;
	}

	if (pl->data[dN].column_N != 0) {

		if (lN < pl->data[dN].length_N) {

			pl->data[dN].head_N = 0;
			pl->data[dN].tail_N = 0;
			pl->data[dN].id_N = 0;
			pl->data[dN].sub_N = 0;
		}

		plotDataChunkAlloc(pl, dN, lN);
	}
}

int plotDataSpaceLeft(plot_t *pl, int dN)
{
	int		N;

	N = pl->data[dN].tail_N - pl->data[dN].head_N;
	N += (N < 0) ? pl->data[dN].length_N : 0;

	return pl->data[dN].length_N - N;
}

void plotDataGrowUp(plot_t *pl, int dN)
{
	int			lSHIFT, lN;

	lSHIFT = pl->data[dN].chunk_SHIFT;

	lN = pl->data[dN].length_N;
	lN = ((lN >> lSHIFT) + 1) << lSHIFT;

	plotDataResize(pl, dN, lN);
}

static const fval_t *
plotDataGet(plot_t *pl, int dN, int *rN)
{
	const fval_t	*row = NULL;
	int		lN, kN, jN;

	if (*rN != pl->data[dN].tail_N) {

		kN = *rN >> pl->data[dN].chunk_SHIFT;
		jN = *rN & pl->data[dN].chunk_MASK;

		if (pl->data[dN].lz4_compress != 0) {

			plotDataChunkFetch(pl, dN, kN);
		}

		row = pl->data[dN].raw[kN];

		if (row != NULL) {

			row += (pl->data[dN].column_N + PLOT_SUBTRACT) * jN;

			lN = pl->data[dN].length_N;
			*rN = (*rN < lN - 1) ? *rN + 1 : 0;
		}
	}

	return row;
}

static void
plotDataRangeCacheWipe(plot_t *pl, int dN, int kN)
{
	int		N;

	for (N = 0; N < PLOT_RCACHE_SIZE; ++N) {

		if (		pl->rcache[N].busy != 0
				&& pl->rcache[N].data_N == dN) {

			pl->rcache[N].chunk[kN].computed = 0;
			pl->rcache[N].cached = 0;
		}
	}
}

static fval_t *
plotDataWrite(plot_t *pl, int dN, int *rN)
{
	fval_t		*row = NULL;
	int		lN, kN, jN;

	if (*rN != pl->data[dN].tail_N) {

		kN = *rN >> pl->data[dN].chunk_SHIFT;
		jN = *rN & pl->data[dN].chunk_MASK;

		if (pl->data[dN].lz4_compress != 0) {

			plotDataChunkWrite(pl, dN, kN);
		}

		if (		   pl->rcache_wipe_data_N != dN
				|| pl->rcache_wipe_chunk_N != kN) {

			plotDataRangeCacheWipe(pl, dN, kN);

			pl->rcache_wipe_data_N = dN;
			pl->rcache_wipe_chunk_N = kN;
		}

		row = pl->data[dN].raw[kN];

		if (row != NULL) {

			row += (pl->data[dN].column_N + PLOT_SUBTRACT) * jN;

			lN = pl->data[dN].length_N;
			*rN = (*rN < lN - 1) ? *rN + 1 : 0;
		}
	}

	return row;
}

static void
plotDataSkip(plot_t *pl, int dN, int *rN, int *id_N, int iN)
{
	int		N, lN, tN;

	lN = pl->data[dN].length_N;

	N = *rN - pl->data[dN].head_N;
	N = (N < 0) ? N + lN : N;

	tN = pl->data[dN].tail_N - pl->data[dN].head_N;
	tN = (tN < 0) ? tN + lN : tN;

	iN = (N + iN < 0) ? - N : iN;
	iN = (N + iN > tN) ? tN - N : iN;

	N += iN;

	N = pl->data[dN].head_N + N;
	N = (N > lN - 1) ? N - lN : N;

	if (rN != NULL) {

		*rN = N;
	}

	if (id_N != NULL) {

		*id_N += iN;
	}
}

static int
plotDataChunkN(plot_t *pl, int dN, int rN)
{
	int		kN;

	kN = rN >> pl->data[dN].chunk_SHIFT;

	return kN;
}

static void
plotDataChunkSkip(plot_t *pl, int dN, int *rN, int *id_N)
{
	int		skip_N, wrap_N;

	skip_N = (1UL << pl->data[dN].chunk_SHIFT)
		- (*rN & pl->data[dN].chunk_MASK);

	wrap_N = pl->data[dN].length_N - *rN;
	skip_N = (wrap_N < skip_N) ? wrap_N : skip_N;

	plotDataSkip(pl, dN, rN, id_N, skip_N);
}

static tuple_t
plotDataMedianAdd(plot_t *pl, int dN, int sN, double fval, double fpay)
{
	int		index[PLOT_MEDIAN_MAX];
	int		N, Nq, N0, N1, length, keep, tail, N_len;

	tuple_t		mN = { -1, -1 };

	length = pl->data[dN].sub[sN].op.median.length;
	keep = pl->data[dN].sub[sN].op.median.keep;
	tail = pl->data[dN].sub[sN].op.median.tail;

	pl->data[dN].sub[sN].op.median.window[tail].fval = fval;
	pl->data[dN].sub[sN].op.median.window[tail].fpay = fpay;

	keep = (keep < length - 1) ? keep + 1 : length;
	tail = (tail < length - 1) ? tail + 1 : 0;

	pl->data[dN].sub[sN].op.median.keep = keep;
	pl->data[dN].sub[sN].op.median.tail = tail;

	for (N = 0, N_len = 0; N < keep; ++N) {

		fval = pl->data[dN].sub[sN].op.median.window[N].fval;

		if (fp_isfinite(fval)) {

			index[N_len++] = N;

			for (Nq = N_len - 1; Nq > 0; --Nq) {

				N0 = index[Nq - 1];
				N1 = index[Nq];

				fval = pl->data[dN].sub[sN].op.median.window[N0].fval;
				fpay = pl->data[dN].sub[sN].op.median.window[N1].fval;

				if (fval < fpay) {

					index[Nq - 1] = N1;
					index[Nq] = N0;
				}
			}
		}
	}

	if (N_len > 0) {

		mN.X = index[N_len / 2];
		mN.Y = mN.X;
	}

	if (pl->data[dN].sub[sN].op.median.opdata != 0) {

		for (N = 0, N_len = 0; N < keep; ++N) {

			fval = pl->data[dN].sub[sN].op.median.window[N].fval;
			fpay = pl->data[dN].sub[sN].op.median.window[N].fpay;

			if (fp_isfinite(fval) && fp_isfinite(fpay)) {

				index[N_len++] = N;

				for (Nq = N_len - 1; Nq > 0; --Nq) {

					N0 = index[Nq - 1];
					N1 = index[Nq];

					fval = pl->data[dN].sub[sN].op.median.window[N0].fpay;
					fpay = pl->data[dN].sub[sN].op.median.window[N1].fpay;

					if (fval < fpay) {

						index[Nq - 1] = N1;
						index[Nq] = N0;
					}
				}
			}
		}

		if (N_len > 0) {

			mN.Y = index[N_len / 2];
		}
	}

	return mN;
}

static void
plotDataResample(plot_t *pl, int dN, int cNX, int cNY, int in_dN, int in_cNX, int in_cNY)
{
	fval_t		*row, X, Y, X2, Y2, prev_X2, prev_Y2, Qf;
	const fval_t	*prey;

	int		rN, id_N, rN2, id_N2;

	rN = pl->data[dN].head_N;
	id_N = pl->data[dN].id_N;

	rN2 = pl->data[in_dN].head_N;
	id_N2 = pl->data[in_dN].id_N;

	do {
		prey = plotDataGet(pl, in_dN, &rN2);

		if (prey == NULL)
			break;

		X2 = (in_cNX < 0) ? id_N2 : prey[in_cNX];
		Y2 = (in_cNY < 0) ? id_N2 : prey[in_cNY];

		id_N2++;

		if (fp_isfinite(X2))
			break;
	}
	while (1);

	if (id_N2 != pl->data[in_dN].id_N) {

		prev_X2 = X2;
		prev_Y2 = Y2;
	}
	else {
		ERROR("No data to resample in dataset %i column %i\n", in_dN, in_cNX);
		return ;
	}

	do {
		row = plotDataWrite(pl, dN, &rN);

		if (row == NULL)
			break;

		X = (cNX < 0) ? id_N : row[cNX];

		if (fp_isfinite(X)) {

			do {
				if (X2 >= X)
					break;

				prey = plotDataGet(pl, in_dN, &rN2);

				if (prey == NULL)
					break;

				if (fp_isfinite(X2)) {

					prev_X2 = X2;
					prev_Y2 = Y2;
				}

				X2 = (in_cNX < 0) ? id_N2 : prey[in_cNX];
				Y2 = (in_cNY < 0) ? id_N2 : prey[in_cNY];

				id_N2++;
			}
			while (1);

			if (		pl->interpolation != 0
					&& X2 >= X) {

				if (		prev_X2 <= X
						&& prev_X2 < X2) {

					Qf = (X - prev_X2) / (X2 - prev_X2);
					Y = prev_Y2 + (Y2 - prev_Y2) * Qf;
				}
				else {
					Y = prev_Y2;
				}
			}
			else {
				Y = Y2;
			}
		}
		else {
			Y = FP_NAN;
		}

		row[cNY] = Y;

		id_N++;
	}
	while (1);
}

static void
plotDataPolyfit(plot_t *pl, int dN, int cNX, int cNY,
		double scale_X, double offset_X,
		double scale_Y, double offset_Y, int N0, int N1)
{
	const fval_t	*row;
	double		fval_X, fval_Y, fvec[LSE_FULL_MAX];
	int		N, xN, yN, kN, rN, id_N, job;

	lse_construct(&pl->lsq, LSE_CASCADE_MAX, N1 - N0 + 1, 1);

	xN = plotDataRangeCacheFetch(pl, dN, cNX);
	yN = plotDataRangeCacheFetch(pl, dN, cNY);

	rN = pl->data[dN].head_N;
	id_N = pl->data[dN].id_N;

	do {
		kN = plotDataChunkN(pl, dN, rN);
		job = 1;

		if (xN >= 0 && pl->rcache[xN].chunk[kN].computed != 0) {

			if (pl->rcache[xN].chunk[kN].finite != 0) {

				fvec[0] = pl->rcache[xN].chunk[kN].fmin * scale_X + offset_X;
				fvec[1] = pl->rcache[xN].chunk[kN].fmax * scale_X + offset_X;

				if (fvec[0] > 1. || fvec[1] < 0.) {

					job = 0;
				}
			}
			else {
				job = 0;
			}
		}

		if (yN >= 0 && pl->rcache[yN].chunk[kN].computed != 0) {

			if (pl->rcache[yN].chunk[kN].finite != 0) {

				fvec[0] = pl->rcache[yN].chunk[kN].fmin * scale_Y + offset_Y;
				fvec[1] = pl->rcache[yN].chunk[kN].fmax * scale_Y + offset_Y;

				if (fvec[0] > 1. || fvec[1] < 0.) {

					job = 0;
				}
			}
			else {
				job = 0;
			}
		}

		if (job != 0) {

			do {
				if (kN != plotDataChunkN(pl, dN, rN))
					break;

				row = plotDataGet(pl, dN, &rN);

				if (row == NULL)
					break;

				fval_X = (cNX < 0) ? id_N : row[cNX];
				fval_Y = (cNY < 0) ? id_N : row[cNY];

				if (fp_isfinite(fval_X) && fp_isfinite(fval_Y)) {

					fvec[0] = fval_X * scale_X + offset_X;
					fvec[1] = fval_Y * scale_Y + offset_Y;

					if (		   fvec[0] >= 0. && fvec[0] <= 1.
							&& fvec[1] >= 0. && fvec[1] <= 1.) {

						fvec[0] = 1.;

						for (N = 0; N < N1; ++N)
							fvec[N + 1] = fvec[N] * fval_X;

						for (N = 0; N < N1 - N0 + 1; ++N)
							fvec[N] = fvec[N + N0];

						fvec[N1 - N0 + 1] = fval_Y;

						lse_insert(&pl->lsq, fvec);
					}
				}

				id_N++;
			}
			while (1);
		}
		else {
			plotDataChunkSkip(pl, dN, &rN, &id_N);
		}

		if (rN == pl->data[dN].tail_N)
			break;
	}
	while (1);

	lse_solve(&pl->lsq);
	lse_std(&pl->lsq);
}

static void
plotDataFileCSV(plot_t *pl, int *list_dN, int *list_cN, int len_N, FILE *fd_csv)
{
	char		numfmt[PLOT_STRING_MAX];

	fval_t		fval;
	int		N, dN, job;

	struct {

		const fval_t	*row;

		int		rN;
		int		id_N;
	}
	local[PLOT_DATASET_MAX];

	for (dN = 0; dN < PLOT_DATASET_MAX; ++dN) {

		job = 0;

		if (pl->data[dN].column_N != 0) {

			for (N = 0; N < len_N; ++N) {

				if (list_dN[N] == dN) {

					job = 1;
					break;
				}
			}
		}

		if (job != 0) {

			local[dN].row = (const fval_t *) 1;
			local[dN].rN = pl->data[dN].head_N;
			local[dN].id_N = pl->data[dN].id_N;
		}
		else {
			local[dN].row = NULL;
		}
	}

	do {
		job = 0;

		for (dN = 0; dN < PLOT_DATASET_MAX; ++dN) {

			if (local[dN].row != NULL) {

				local[dN].row = plotDataGet(pl, dN, &local[dN].rN);
			}

			if (local[dN].row != NULL)
				job = 1;
		}

		if (job == 0)
			break;

		for (N = 0; N < len_N; ++N) {

			dN = list_dN[N];

			if (local[dN].row != NULL) {

				fval = (list_cN[N] < 0) ? local[dN].id_N
					: local[dN].row[list_cN[N]];

				sprintf(numfmt, "%%.%iE;", pl->fprecision - 1);
				fprintf(fd_csv, numfmt, fval);
			}
			else {
				fprintf(fd_csv, "NAN;");
			}
		}

		fprintf(fd_csv, "\n");

		for (dN = 0; dN < PLOT_DATASET_MAX; ++dN) {

			if (local[dN].row != NULL)
				local[dN].id_N++;
		}
	}
	while (1);
}

static void
plotDataSubtractWrite(plot_t *pl, int dN, int sN, int rN_beg, int id_N_beg, int rN_end)
{
	fval_t		*row, X1, X2, X3, X4;
	double		scale, offset, gain;
	int		cN, rN, id_N, cN1, cN2, cN3, mode;

	mode = pl->data[dN].sub[sN].busy;

	if (mode != SUBTRACT_FREE) {

		cN = sN + pl->data[dN].column_N;

		rN = rN_beg;
		id_N = id_N_beg;
	}

	if (mode == SUBTRACT_TIME_MEDIAN) {

		/* Do not to calculate HERE */
	}
	else if (mode == SUBTRACT_DATA_MEDIAN) {

		tuple_t		mN;

		if (rN_beg == pl->data[dN].head_N) {

			pl->data[dN].sub[sN].op.median.keep = 0;
			pl->data[dN].sub[sN].op.median.tail = 0;

			pl->data[dN].sub[sN].op.median.offset = (double) 0.;
			pl->data[dN].sub[sN].op.median.prev[0] = FP_NAN;
			pl->data[dN].sub[sN].op.median.prev[1] = FP_NAN;
		}

		cN1 = pl->data[dN].sub[sN].op.median.column_1;
		cN2 = pl->data[dN].sub[sN].op.median.column_2;
		cN3 = pl->data[dN].sub[sN].op.median.column_3;

		offset = pl->data[dN].sub[sN].op.median.offset;

		if (pl->data[dN].sub[sN].op.median.unwrap != 0) {

			X3 = (fval_t) pl->data[dN].sub[sN].op.median.prev[0];
			X4 = (fval_t) pl->data[dN].sub[sN].op.median.prev[1];
		}

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];
			X2 = (cN2 < 0) ? id_N : row[cN2];

			mN = plotDataMedianAdd(pl, dN, sN, X1, X2);

			if (mN.X < 0) {

				X1 = FP_NAN;
				X2 = FP_NAN;
			}
			else {
				X1 = pl->data[dN].sub[sN].op.median.window[mN.X].fval;
				X2 = pl->data[dN].sub[sN].op.median.window[mN.Y].fpay;
			}

			if (pl->data[dN].sub[sN].op.median.unwrap != 0) {

				if (X1 + (fval_t) pl->defungap < X3) {

					offset += X3 - X1;

					if (X4 < X3) {

						offset += X3 - X4;
					}
				}

				if (fp_isfinite(X1)) {

					X4 = X3;
					X3 = X1;
				}
			}

			row[cN3] = X1 + offset;
			row[cN] = X2;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);

		pl->data[dN].sub[sN].op.median.offset = offset;

		if (pl->data[dN].sub[sN].op.median.unwrap != 0) {

			pl->data[dN].sub[sN].op.median.prev[0] = (double) X3;
			pl->data[dN].sub[sN].op.median.prev[1] = (double) X4;
		}
	}
	else if (mode == SUBTRACT_SCALE) {

		cN1 = pl->data[dN].sub[sN].op.scale.column_1;
		scale = pl->data[dN].sub[sN].op.scale.scale;
		offset = pl->data[dN].sub[sN].op.scale.offset;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];
			X1 = X1 * scale + offset;

			row[cN] = X1;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);
	}
	else if (mode == SUBTRACT_RESAMPLE) {

		/* Do not to calculate HERE */
	}
	else if (mode == SUBTRACT_POLYFIT) {

		const double	*coefs;
		int		N, N0, N1;

		cN1 = pl->data[dN].sub[sN].op.polyfit.column_X;
		N0 = pl->data[dN].sub[sN].op.polyfit.poly_N0;
		N1 = pl->data[dN].sub[sN].op.polyfit.poly_N1;
		coefs = pl->data[dN].sub[sN].op.polyfit.coefs;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];
			X2 = coefs[N1 - N0];

			for (N = N1 - N0 - 1; N >= 0; --N)
				X2 = X2 * X1 + coefs[N];

			for (N = N0 - 1; N >= 0; --N)
				X2 = X2 * X1;

			row[cN] = X2;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);
	}
	else if (mode == SUBTRACT_BINARY_SUBTRACTION) {

		cN1 = pl->data[dN].sub[sN].op.binary.column_1;
		cN2 = pl->data[dN].sub[sN].op.binary.column_2;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];
			X2 = (cN2 < 0) ? id_N : row[cN2];

			row[cN] = X1 - X2;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);
	}
	else if (mode == SUBTRACT_BINARY_ADDITION) {

		cN1 = pl->data[dN].sub[sN].op.binary.column_1;
		cN2 = pl->data[dN].sub[sN].op.binary.column_2;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];
			X2 = (cN2 < 0) ? id_N : row[cN2];

			row[cN] = X1 + X2;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);
	}
	else if (mode == SUBTRACT_BINARY_MULTIPLICATION) {

		cN1 = pl->data[dN].sub[sN].op.binary.column_1;
		cN2 = pl->data[dN].sub[sN].op.binary.column_2;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];
			X2 = (cN2 < 0) ? id_N : row[cN2];

			row[cN] = X1 * X2;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);
	}
	else if (mode == SUBTRACT_BINARY_HYPOTENUSE) {

		cN1 = pl->data[dN].sub[sN].op.binary.column_1;
		cN2 = pl->data[dN].sub[sN].op.binary.column_2;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];
			X2 = (cN2 < 0) ? id_N : row[cN2];

			row[cN] = sqrt(X1 * X1 + X2 * X2);

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);
	}
	else if (mode == SUBTRACT_FILTER_DIFFERENCE) {

		if (rN_beg == pl->data[dN].head_N) {

			pl->data[dN].sub[sN].op.filter.state = FP_NAN;
		}

		cN1 = pl->data[dN].sub[sN].op.filter.column_1;
		X2 = (fval_t) pl->data[dN].sub[sN].op.filter.state;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];

			row[cN] = X1 - X2;

			X2 = X1;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);

		pl->data[dN].sub[sN].op.filter.state = (double) X2;
	}
	else if (mode == SUBTRACT_FILTER_CUMULATIVE) {

		if (rN_beg == pl->data[dN].head_N) {

			pl->data[dN].sub[sN].op.filter.state = 0.;
		}

		cN1 = pl->data[dN].sub[sN].op.filter.column_1;
		X2 = (fval_t) pl->data[dN].sub[sN].op.filter.state;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];

			if (fp_isfinite(X1)) {

				X2 += X1;
			}

			row[cN] = X2;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);

		pl->data[dN].sub[sN].op.filter.state = (double) X2;
	}
	else if (mode == SUBTRACT_FILTER_BITMASK) {

		unsigned long	shift, mask, ulval;

		cN1 = pl->data[dN].sub[sN].op.filter.column_1;
		ulval = (unsigned long) pl->data[dN].sub[sN].op.filter.gain;

		shift = ulval & 0xFFU;
		ulval = ulval >> 8;

		mask = ((1U << (ulval - shift + 1U)) - 1U) << shift;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];

			ulval = ((unsigned long) X1 & mask) >> shift;
			row[cN] = (fval_t) ulval;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);
	}
	else if (mode == SUBTRACT_FILTER_LOW_PASS) {

		if (rN_beg == pl->data[dN].head_N) {

			pl->data[dN].sub[sN].op.filter.state = FP_NAN;
		}

		cN1 = pl->data[dN].sub[sN].op.filter.column_1;
		gain = pl->data[dN].sub[sN].op.filter.gain;
		X2 = (fval_t) pl->data[dN].sub[sN].op.filter.state;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];

			if (fp_isfinite(X1)) {

				if (fp_isfinite(X2)) {

					X2 += (X1 - X2) * gain;
				}
				else {
					X2 = X1;
				}
			}

			row[cN] = X2;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);

		pl->data[dN].sub[sN].op.filter.state = (double) X2;
	}
	else if (mode == SUBTRACT_FILTER_MEDIAN) {

		tuple_t		mN;

		if (rN_beg == pl->data[dN].head_N) {

			pl->data[dN].sub[sN].op.median.keep = 0;
			pl->data[dN].sub[sN].op.median.tail = 0;
		}

		cN1 = pl->data[dN].sub[sN].op.median.column_1;

		do {
			row = plotDataWrite(pl, dN, &rN);

			if (row == NULL)
				break;

			X1 = (cN1 < 0) ? id_N : row[cN1];

			mN = plotDataMedianAdd(pl, dN, sN, X1, X1);

			if (mN.X < 0) {

				X2 = FP_NAN;
			}
			else {
				X2 = pl->data[dN].sub[sN].op.median.window[mN.X].fval;
			}

			row[cN] = X2;

			id_N++;

			if (rN == rN_end)
				break;
		}
		while (1);
	}
}

static void
plotDataSubtractResample(plot_t *pl, int dN, int sN)
{
	int		cNX, cNY, in_dN, in_cNX, in_cNY;

	if (pl->data[dN].sub[sN].busy == SUBTRACT_RESAMPLE) {

		cNX = pl->data[dN].sub[sN].op.resample.column_X;
		cNY = sN + pl->data[dN].column_N;

		in_dN = pl->data[dN].sub[sN].op.resample.in_data_N;
		in_cNX = pl->data[dN].sub[sN].op.resample.in_column_X;
		in_cNY = pl->data[dN].sub[sN].op.resample.in_column_Y;

		plotDataResample(pl, dN, cNX, cNY, in_dN, in_cNX, in_cNY);
	}
}

static void
plotDataSubtractWriteSeq(plot_t *pl, int dN, int rN_beg, int id_N_beg, int rN_end)
{
	int		N;

	for (N = 0; N < PLOT_SUBTRACT; ++N) {

		plotDataSubtractWrite(pl, dN, N, rN_beg, id_N_beg, rN_end);
	}
}

static void
plotDataSubtractResampleSeq(plot_t *pl, int dN)
{
	int		N;

	for (N = 0; N < PLOT_SUBTRACT; ++N) {

		plotDataSubtractResample(pl, dN, N);
	}
}

void plotDataSubtractCompute(plot_t *pl, int dN, int sN)
{
	int		rN, id_N, rN_end;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return ;
	}

	if (sN < 0 || sN >= PLOT_SUBTRACT) {

		ERROR("Subtract number %i is out of range\n", sN);
		return ;
	}

	if (pl->data[dN].sub_paused != 0)
		return ;

	rN = pl->data[dN].head_N;
	id_N = pl->data[dN].id_N;

	rN_end = pl->data[dN].tail_N;

	if (rN == rN_end)
		return ;

	plotDataSubtractWrite(pl, dN, sN, rN, id_N, rN_end);
	plotDataSubtractResample(pl, dN, sN);
}

void plotDataSubtractResidual(plot_t *pl, int dN)
{
	int		rN, id_N, rN_end;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return ;
	}

	rN = pl->data[dN].sub_N;
	id_N = pl->data[dN].id_N;

	rN_end = pl->data[dN].tail_N;

	if (rN == rN_end)
		return ;

	plotDataSubtractWriteSeq(pl, dN, rN, id_N, rN_end);

	pl->data[dN].sub_N = rN_end;
}

void plotDataSubtractClean(plot_t *pl)
{
	int		dN, N;

	for (dN = 0; dN < PLOT_DATASET_MAX; ++dN) {

		if (pl->data[dN].column_N != 0) {

			for (N = 0; N < PLOT_SUBTRACT; ++N) {

				pl->data[dN].sub[N].busy = SUBTRACT_FREE;
			}
		}
	}
}

void plotDataSubtractPaused(plot_t *pl)
{
	int		dN;

	for (dN = 0; dN < PLOT_DATASET_MAX; ++dN) {

		if (pl->data[dN].column_N != 0) {

			pl->data[dN].sub_paused = 1;
		}
	}
}

void plotDataSubtractAlternate(plot_t *pl)
{
	int		dN, rN, id_N, rN_end, id_N_end, lCHUNK;

	for (dN = 0; dN < PLOT_DATASET_MAX; ++dN) {

		if (pl->data[dN].column_N != 0) {

			rN = pl->data[dN].head_N;
			id_N = pl->data[dN].id_N;

			if (rN != pl->data[dN].tail_N) {

				lCHUNK = (1UL << pl->data[dN].chunk_SHIFT);

				rN_end = rN;
				id_N_end = id_N;

				do {
					plotDataSkip(pl, dN, &rN_end, &id_N_end, lCHUNK);
					plotDataSubtractWriteSeq(pl, dN, rN, id_N, rN_end);

					rN = rN_end;
					id_N = id_N_end;
				}
				while (rN != pl->data[dN].tail_N);

				pl->data[dN].sub_N = rN_end;

				plotDataSubtractResampleSeq(pl, dN);
			}

			pl->data[dN].sub_paused = 0;
		}
	}
}

void plotDataInsert(plot_t *pl, int dN, const fval_t *row)
{
	fval_t		*place;
	int		cN, lN, hN, tN, kN, jN, sN;

	cN = pl->data[dN].column_N;
	lN = pl->data[dN].length_N;
	hN = pl->data[dN].head_N;
	tN = pl->data[dN].tail_N;

	kN = tN >> pl->data[dN].chunk_SHIFT;
	jN = tN & pl->data[dN].chunk_MASK;

	if (pl->data[dN].lz4_compress != 0) {

		plotDataChunkWrite(pl, dN, kN);
	}

	if (		   pl->rcache_wipe_data_N != dN
			|| pl->rcache_wipe_chunk_N != kN) {

		plotDataRangeCacheWipe(pl, dN, kN);

		pl->rcache_wipe_data_N = dN;
		pl->rcache_wipe_chunk_N = kN;
	}

	place = pl->data[dN].raw[kN];

	if (place != NULL) {

		place += (cN + PLOT_SUBTRACT) * jN;

		memcpy(place, row, cN * sizeof(fval_t));
		memset(place + cN, 0, PLOT_SUBTRACT * sizeof(fval_t));

		tN = (tN < lN - 1) ? tN + 1 : 0;

		if (hN == tN) {

			pl->data[dN].id_N++;

			hN = (hN < lN - 1) ? hN + 1 : 0;
			pl->data[dN].head_N = hN;

			sN = pl->data[dN].sub_N;
			pl->data[dN].sub_N = (sN == tN) ? hN : sN;
		}

		pl->data[dN].tail_N = tN;
	}
}

void plotDataClean(plot_t *pl, int dN)
{
	int		N;

	if (pl->data[dN].column_N != 0) {

		pl->data[dN].column_N = 0;
		pl->data[dN].length_N = 0;

		if (pl->data[dN].lz4_compress != 0) {

			for (N = 0; N < PLOT_CHUNK_CACHE; ++N) {

				if (pl->data[dN].cache[N].raw) {

					free(pl->data[dN].cache[N].raw);

					pl->data[dN].cache[N].raw = NULL;
				}
			}

			for (N = 0; N < PLOT_CHUNK_MAX; ++N) {

				pl->data[dN].raw[N] = NULL;

				if (pl->data[dN].compress[N].raw != NULL) {

					free(pl->data[dN].compress[N].raw);

					pl->data[dN].compress[N].raw = NULL;
				}
			}
		}
		else {
			for (N = 0; N < PLOT_CHUNK_MAX; ++N) {

				if (pl->data[dN].raw[N] != NULL) {

					free(pl->data[dN].raw[N]);

					pl->data[dN].raw[N] = NULL;
				}
			}
		}

		free(pl->data[dN].map - 1);

		pl->data[dN].map = NULL;
	}
}

static int
plotDataRangeCacheGetNode(plot_t *pl, int dN, int cN)
{
	int		N, xN = -1;

	for (N = 0; N < PLOT_RCACHE_SIZE; ++N) {

		if (		pl->rcache[N].busy != 0
				&& pl->rcache[N].data_N == dN
				&& pl->rcache[N].column_N == cN) {

			xN = N;
			break;
		}
	}

	return xN;
}

void plotDataRangeCacheClean(plot_t *pl, int dN)
{
	int		N;

	for (N = 0; N < PLOT_RCACHE_SIZE; ++N) {

		if (pl->rcache[N].data_N == dN)
			pl->rcache[N].busy = 0;
	}
}

void plotDataRangeCacheSubtractClean(plot_t *pl)
{
	int		N, dN;

	for (N = 0; N < PLOT_RCACHE_SIZE; ++N) {

		if (pl->rcache[N].busy != 0) {

			dN = pl->rcache[N].data_N;

			if (		dN >= 0 && dN < PLOT_DATASET_MAX
					&& pl->data[dN].column_N != 0) {

				if (pl->rcache[N].column_N >= pl->data[dN].column_N)
					pl->rcache[N].busy = 0;
			}
		}
	}
}

int plotDataRangeCacheFetch(plot_t *pl, int dN, int cN)
{
	const fval_t	*row;
	fval_t		fval, fmin, fmax, ymin, ymax;
	int		N, xN, rN, id_N, kN;
	int		job, finite, started;

	xN = plotDataRangeCacheGetNode(pl, dN, cN);

	if (xN >= 0) {

		if (pl->rcache[xN].cached != 0)
			return xN;
	}
	else {
		xN = pl->rcache_ID;

		pl->rcache_ID = (pl->rcache_ID < PLOT_RCACHE_SIZE - 1)
			? pl->rcache_ID + 1 : 0;

		for (N = 0; N < PLOT_CHUNK_MAX; ++N) {

			pl->rcache[xN].chunk[N].computed = 0;
		}
	}

	rN = pl->data[dN].head_N;
	id_N = pl->data[dN].id_N;

	fmin = (fval_t) 0.;
	fmax = (fval_t) 0.;

	started = 0;

	do {
		kN = plotDataChunkN(pl, dN, rN);

		if (pl->rcache[xN].chunk[kN].computed != 0) {

			if (kN == plotDataChunkN(pl, dN, pl->data[dN].tail_N)) {

				job = 1;

				finite = pl->rcache[xN].chunk[kN].finite;
				ymin = pl->rcache[xN].chunk[kN].fmin;
				ymax = pl->rcache[xN].chunk[kN].fmax;
			}
			else {
				job = 0;
			}
		}
		else {
			finite = 0;
			job = 1;
		}

		if (job != 0) {

			do {
				if (kN != plotDataChunkN(pl, dN, rN))
					break;

				row = plotDataGet(pl, dN, &rN);

				if (row == NULL)
					break;

				fval = (cN < 0) ? id_N : row[cN];

				if (fp_isfinite(fval)) {

					if (finite != 0) {

						ymin = (fval < ymin) ? fval : ymin;
						ymax = (fval > ymax) ? fval : ymax;
					}
					else {
						finite = 1;

						ymin = fval;
						ymax = fval;
					}
				}

				id_N++;
			}
			while (1);

			pl->rcache[xN].chunk[kN].computed = 1;
			pl->rcache[xN].chunk[kN].finite = finite;

			if (finite != 0) {

				pl->rcache[xN].chunk[kN].fmin = ymin;
				pl->rcache[xN].chunk[kN].fmax = ymax;
			}
		}
		else {
			plotDataChunkSkip(pl, dN, &rN, &id_N);
		}

		if (pl->rcache[xN].chunk[kN].finite != 0) {

			if (started != 0) {

				fmin = (pl->rcache[xN].chunk[kN].fmin < fmin)
					? pl->rcache[xN].chunk[kN].fmin : fmin;

				fmax = (pl->rcache[xN].chunk[kN].fmax > fmax)
					? pl->rcache[xN].chunk[kN].fmax : fmax;
			}
			else {
				started = 1;

				fmin = pl->rcache[xN].chunk[kN].fmin;
				fmax = pl->rcache[xN].chunk[kN].fmax;
			}
		}

		if (rN == pl->data[dN].tail_N)
			break;
	}
	while (1);

	pl->rcache[xN].busy = 1;
	pl->rcache[xN].data_N = dN;
	pl->rcache[xN].column_N = cN;
	pl->rcache[xN].cached = 1;
	pl->rcache[xN].fmin = fmin;
	pl->rcache[xN].fmax = fmax;

	pl->rcache_wipe_data_N = -1;
	pl->rcache_wipe_chunk_N = -1;

	return xN;
}

static void
plotDataRangeGet(plot_t *pl, int dN, int cN, double *pmin, double *pmax)
{
	int		xN;

	xN = plotDataRangeCacheFetch(pl, dN, cN);

	*pmin = (double) pl->rcache[xN].fmin;
	*pmax = (double) pl->rcache[xN].fmax;
}

static void
plotDataRangeCond(plot_t *pl, int dN, int cN, int cN_cond, int *pflag,
		double scale, double offset, double *pmin, double *pmax)
{
	const fval_t	*row;
	double		fval, fmin, fmax, fcond, vmin, vmax;
	int		xN, yN, kN, rN, id_N, job, started;

	started = *pflag;
	fmin = *pmin;
	fmax = *pmax;

	xN = plotDataRangeCacheFetch(pl, dN, cN_cond);
	yN = plotDataRangeCacheFetch(pl, dN, cN);

	if (xN >= 0 && yN >= 0) {

		vmin = pl->rcache[xN].fmin * scale + offset;
		vmax = pl->rcache[xN].fmax * scale + offset;

		if (		   vmin >= 0. && vmin <= 1.
				&& vmax >= 0. && vmax <= 1.) {

			if (started != 0) {

				fmin = (pl->rcache[yN].fmin < fmin)
					? pl->rcache[yN].fmin : fmin;

				fmax = (pl->rcache[yN].fmax > fmax)
					? pl->rcache[yN].fmax : fmax;
			}
			else {
				started = 1;

				fmin = pl->rcache[yN].fmin;
				fmax = pl->rcache[yN].fmax;
			}

			*pflag = started;
			*pmin = fmin;
			*pmax = fmax;

			return ;
		}
	}

	rN = pl->data[dN].head_N;
	id_N = pl->data[dN].id_N;

	do {
		kN = plotDataChunkN(pl, dN, rN);
		job = 1;

		if (xN >= 0 && pl->rcache[xN].chunk[kN].computed != 0) {

			if (pl->rcache[xN].chunk[kN].finite != 0) {

				vmin = pl->rcache[xN].chunk[kN].fmin * scale + offset;
				vmax = pl->rcache[xN].chunk[kN].fmax * scale + offset;

				if (yN >= 0	&& pl->rcache[yN].chunk[kN].computed != 0
						&& vmin >= 0. && vmin <= 1.
						&& vmax >= 0. && vmax <= 1.) {

					job = 0;

					if (pl->rcache[yN].chunk[kN].finite != 0) {

						if (started != 0) {

							fmin = (pl->rcache[yN].chunk[kN].fmin < fmin)
								? pl->rcache[yN].chunk[kN].fmin : fmin;

							fmax = (pl->rcache[yN].chunk[kN].fmax > fmax)
								? pl->rcache[yN].chunk[kN].fmax : fmax;
						}
						else {
							started = 1;

							fmin = pl->rcache[yN].chunk[kN].fmin;
							fmax = pl->rcache[yN].chunk[kN].fmax;
						}
					}
				}
				else if (vmin > 1. || vmax < 0.) {

					job = 0;
				}
			}
			else {
				job = 0;
			}
		}

		if (job != 0) {

			do {
				if (kN != plotDataChunkN(pl, dN, rN))
					break;

				row = plotDataGet(pl, dN, &rN);

				if (row == NULL)
					break;

				fval = (cN < 0) ? id_N : row[cN];
				fcond = (cN_cond < 0) ? id_N : row[cN_cond];

				fcond = fcond * scale + offset;

				if (fcond >= 0. && fcond <= 1.) {

					if (fp_isfinite(fval)) {

						if (started != 0) {

							fmin = (fval < fmin) ? fval : fmin;
							fmax = (fval > fmax) ? fval : fmax;
						}
						else {
							started = 1;

							fmin = fval;
							fmax = fval;
						}
					}
				}

				id_N++;
			}
			while (1);
		}
		else {
			plotDataChunkSkip(pl, dN, &rN, &id_N);
		}

		if (rN == pl->data[dN].tail_N)
			break;
	}
	while (1);

	*pflag = started;
	*pmin = fmin;
	*pmax = fmax;
}

static int
plotDataRangeAxis(plot_t *pl, int dN, int cN, int aN, double *pmin, double *pmax)
{
	double		scale, offset, fmin, fmax;
	int		xN, yN, fN, cN_cond, job, started;

	started = 0;

	fmin = 0.;
	fmax = 0.;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		job = 0;

		if (		pl->figure[fN].busy != 0
				&& pl->figure[fN].hidden == 0
				&& pl->figure[fN].data_N == dN) {

			if (		pl->figure[fN].axis_X == aN
					&& pl->figure[fN].column_Y == cN) {

				scale = 1.;
				offset = 0.;

				cN_cond = pl->figure[fN].column_X;
				job = 1;
			}
			else if (	pl->figure[fN].axis_Y == aN
					&& pl->figure[fN].column_X == cN) {

				scale = 1.;
				offset = 0.;

				cN_cond = pl->figure[fN].column_Y;
				job = 1;
			}

			xN = pl->figure[fN].axis_X;
			yN = pl->figure[fN].axis_Y;

			if (		pl->axis[xN].slave != 0
					&& pl->axis[xN].slave_N == aN
					&& pl->figure[fN].column_Y == cN) {

				scale = pl->axis[xN].scale;
				offset = pl->axis[xN].offset;

				cN_cond = pl->figure[fN].column_X;
				job = 1;
			}
			else if (	pl->axis[yN].slave != 0
					&& pl->axis[yN].slave_N == aN
					&& pl->figure[fN].column_X == cN) {

				scale = pl->axis[yN].scale;
				offset = pl->axis[yN].offset;

				cN_cond = pl->figure[fN].column_Y;
				job = 1;
			}
		}

		if (job != 0) {

			scale *= pl->axis[aN].scale;
			offset = offset * pl->axis[aN].scale + pl->axis[aN].offset;

			plotDataRangeCond(pl, dN, cN, cN_cond, &started,
					scale, offset, &fmin, &fmax);
		}
	}

	*pmin = fmin;
	*pmax = fmax;

	return started;
}

static const fval_t *
plotDataSliceGet(plot_t *pl, int dN, int cN, double fsamp, int *m_id_N)
{
	const fval_t	*row;
	double		fval, fbest, fmin, fmax, fneard;
	int		xN, lN, rN, id_N, kN, kN_rep, best_N;
	int		job, started, span;

	xN = plotDataRangeCacheFetch(pl, dN, cN);

	rN = pl->data[dN].head_N;
	id_N = pl->data[dN].id_N;

	kN_rep = -1;

	started = 0;
	span = 0;

	do {
		kN = plotDataChunkN(pl, dN, rN);
		job = 1;

		if (xN >= 0 && pl->rcache[xN].chunk[kN].computed != 0) {

			if (pl->rcache[xN].chunk[kN].finite != 0) {

				fmin = pl->rcache[xN].chunk[kN].fmin;
				fmax = pl->rcache[xN].chunk[kN].fmax;

				if (fsamp < fmin || fsamp > fmax) {

					job = 0;

					fmin = fabs(fmin - fsamp);
					fmax = fabs(fmax - fsamp);

					if (kN_rep >= 0) {

						if (fmin < fneard) {

							fneard = fmin;
							kN_rep = kN;
						}

						if (fmax < fneard) {

							fneard = fmax;
							kN_rep = kN;
						}
					}
					else {
						fneard = (fmin < fmax)
							? fmin : fmax;

						kN_rep = kN;
					}
				}
			}
			else {
				job = 0;
			}
		}

		if (job != 0) {

			span++;

			do {
				if (kN != plotDataChunkN(pl, dN, rN))
					break;

				row = plotDataGet(pl, dN, &rN);

				if (row == NULL)
					break;

				fval = (cN < 0) ? id_N : row[cN];

				if (fp_isfinite(fval)) {

					if (started != 0) {

						fval = fabs(fsamp - fval);

						if (fval < fbest) {

							fbest = fval;
							best_N = id_N;
						}
					}
					else {
						started = 1;

						fbest = fabs(fsamp - fval);
						best_N = id_N;
					}
				}

				id_N++;
			}
			while (1);

			if (span >= PLOT_SLICE_SPAN)
				break;
		}
		else {
			plotDataChunkSkip(pl, dN, &rN, &id_N);
		}

		if (rN == pl->data[dN].tail_N)
			break;
	}
	while (1);

	if (		started == 0
			&& kN_rep >= 0) {

		rN = pl->data[dN].head_N;
		id_N = pl->data[dN].id_N;

		do {
			kN = plotDataChunkN(pl, dN, rN);
			job = 1;

			if (kN == kN_rep) {

				do {
					if (kN != plotDataChunkN(pl, dN, rN))
						break;

					row = plotDataGet(pl, dN, &rN);

					if (row == NULL)
						break;

					fval = (cN < 0) ? id_N : row[cN];

					if (fp_isfinite(fval)) {

						if (started != 0) {

							fval = fabs(fsamp - fval);

							if (fval < fbest) {

								fbest = fval;
								best_N = id_N;
							}
						}
						else {
							started = 1;

							fbest = fabs(fsamp - fval);
							best_N = id_N;
						}
					}

					id_N++;
				}
				while (1);
			}
			else {
				plotDataChunkSkip(pl, dN, &rN, &id_N);
			}

			if (rN == pl->data[dN].tail_N)
				break;
		}
		while (1);
	}

	if (started != 0) {

		*m_id_N = best_N;

		lN = pl->data[dN].length_N;

		rN = pl->data[dN].head_N + (best_N - pl->data[dN].id_N);
		rN = (rN > lN - 1) ? rN - lN : rN;

		row = plotDataGet(pl, dN, &rN);
	}
	else {
		row = NULL;
	}

	return row;
}

void plotAxisLabel(plot_t *pl, int aN, const char *label)
{
	if (aN < 0 || aN >= PLOT_AXES_MAX) {

		ERROR("Axis number is out of range\n");
		return ;
	}

	if (label[0] != 0) {

		strcpy(pl->axis[aN].label, label);

		pl->axis[aN].compact = (strlen(pl->axis[aN].label) >= 3) ? 0 : 1;
	}
}

static int
plotAxisRangeGet(plot_t *pl, int aN, double *pmin, double *pmax)
{
	double		min, max, fmin, fmax, scale, offset;
	int		fN, dN, cN, xN, yN, started = 0;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].busy != 0 && pl->figure[fN].hidden == 0) {

			dN = pl->figure[fN].data_N;

			do {
				if (pl->figure[fN].axis_X == aN) {

					cN = pl->figure[fN].column_X;
				}
				else if (pl->figure[fN].axis_Y == aN) {

					cN = pl->figure[fN].column_Y;
				}
				else
					break;

				plotDataRangeGet(pl, dN, cN, &min, &max);

				if (started != 0) {

					fmin = (min < fmin) ? min : fmin;
					fmax = (max > fmax) ? max : fmax;
				}
				else {
					started = 1;

					fmin = min;
					fmax = max;
				}
			}
			while (0);

			do {
				xN = pl->figure[fN].axis_X;
				yN = pl->figure[fN].axis_Y;

				if (		pl->axis[xN].slave != 0
						&& pl->axis[xN].slave_N == aN) {

					cN = pl->figure[fN].column_X;

					scale = pl->axis[xN].scale;
					offset = pl->axis[xN].offset;
				}
				else if (	pl->axis[yN].slave != 0
						&& pl->axis[yN].slave_N == aN) {

					cN = pl->figure[fN].column_Y;

					scale = pl->axis[yN].scale;
					offset = pl->axis[yN].offset;
				}
				else
					break;

				plotDataRangeGet(pl, dN, cN, &min, &max);

				min = min * scale + offset;
				max = max * scale + offset;

				if (started != 0) {

					fmin = (min < fmin) ? min : fmin;
					fmax = (max > fmax) ? max : fmax;
				}
				else {
					started = 1;

					fmin = min;
					fmax = max;
				}
			}
			while (0);
		}
	}

	*pmin = fmin;
	*pmax = fmax;

	return started;
}

static int
plotAxisRangeCond(plot_t *pl, int aN, int bN, double *pmin, double *pmax)
{
	double		min, max, fmin, fmax, scale, offset;
	int		fN, dN, cN, xN, yN, nN, started, cond;

	started = 0;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].busy != 0 && pl->figure[fN].hidden == 0) {

			dN = pl->figure[fN].data_N;

			do {
				if (pl->figure[fN].axis_X == aN) {

					cN = pl->figure[fN].column_X;
					nN = pl->figure[fN].axis_Y;
				}
				else if (pl->figure[fN].axis_Y == aN) {

					cN = pl->figure[fN].column_Y;
					nN = pl->figure[fN].axis_X;
				}
				else
					break;

				if (bN >= 0) {

					cond = plotDataRangeAxis(pl, dN, cN, bN, &min, &max);
				}
				else {
					cond = plotDataRangeAxis(pl, dN, cN, nN, &min, &max);
				}

				if (cond != 0) {

					if (started != 0) {

						fmin = (min < fmin) ? min : fmin;
						fmax = (max > fmax) ? max : fmax;
					}
					else {
						started = 1;

						fmin = min;
						fmax = max;
					}
				}
			}
			while (0);

			do {
				xN = pl->figure[fN].axis_X;
				yN = pl->figure[fN].axis_Y;

				if (		pl->axis[xN].slave != 0
						&& pl->axis[xN].slave_N == aN) {

					cN = pl->figure[fN].column_X;
					nN = pl->figure[fN].axis_Y;

					scale = pl->axis[xN].scale;
					offset = pl->axis[xN].offset;
				}
				else if (	pl->axis[yN].slave != 0
						&& pl->axis[yN].slave_N == aN) {

					cN = pl->figure[fN].column_Y;
					nN = pl->figure[fN].axis_X;

					scale = pl->axis[yN].scale;
					offset = pl->axis[yN].offset;
				}
				else
					break;


				if (bN >= 0) {

					cond = plotDataRangeAxis(pl, dN, cN, bN, &min, &max);
				}
				else {
					cond = plotDataRangeAxis(pl, dN, cN, nN, &min, &max);
				}

				if (cond != 0) {

					min = min * scale + offset;
					max = max * scale + offset;

					if (started != 0) {

						fmin = (min < fmin) ? min : fmin;
						fmax = (max > fmax) ? max : fmax;
					}
					else {
						started = 1;

						fmin = min;
						fmax = max;
					}
				}
			}
			while (0);
		}
	}

	*pmin = fmin;
	*pmax = fmax;

	return started;
}

void plotAxisScaleManual(plot_t *pl, int aN, double min, double max)
{
	if (aN < 0 || aN >= PLOT_AXES_MAX) {

		ERROR("Axis number is out of range\n");
		return ;
	}

	if (pl->axis[aN].busy == AXIS_FREE)
		return ;

	if (pl->axis[aN].slave != 0)
		return ;

	pl->axis[aN].scale = 1. / (max - min);
	pl->axis[aN].offset = - min / (max - min);
}

void plotAxisScaleAuto(plot_t *pl, int aN)
{
	double		fmin, fmax;

	if (aN < 0 || aN >= PLOT_AXES_MAX) {

		ERROR("Axis number is out of range\n");
		return ;
	}

	if (pl->axis[aN].busy == AXIS_FREE)
		return ;

	if (pl->axis[aN].slave != 0)
		return ;

	if (plotAxisRangeGet(pl, aN, &fmin, &fmax) != 0) {

		if (fmin == fmax) {

			fmin += (double) - 1.;
			fmax += (double) + 1.;
		}

		plotAxisScaleManual(pl, aN, fmin, fmax);

		if (pl->axis[aN].busy == AXIS_BUSY_X) {

			fmin = plotAxisConvBackward(pl, aN, pl->viewport.min_x - pl->layout_mark_size);
			fmax = plotAxisConvBackward(pl, aN, pl->viewport.max_x + pl->layout_mark_size);

			plotAxisScaleManual(pl, aN, fmin, fmax);
		}
		else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

			fmin = plotAxisConvBackward(pl, aN, pl->viewport.max_y + pl->layout_mark_size);
			fmax = plotAxisConvBackward(pl, aN, pl->viewport.min_y - pl->layout_mark_size);

			plotAxisScaleManual(pl, aN, fmin, fmax);
		}

		pl->axis[aN].lock_scale = LOCK_AUTO;
		pl->axis[aN].lock_tick = 0;
	}
}

void plotAxisScaleAutoCond(plot_t *pl, int aN, int bN)
{
	double		fmin, fmax;

	if (aN < 0 || aN >= PLOT_AXES_MAX) {

		ERROR("Axis number is out of range\n");
		return ;
	}

	if (bN < -1 || bN >= PLOT_AXES_MAX) {

		ERROR("Conditional axis number is out of range\n");
		return ;
	}

	if (pl->axis[aN].busy == AXIS_FREE)
		return ;

	if (pl->axis[aN].slave != 0)
		return ;

	if (plotAxisRangeCond(pl, aN, bN, &fmin, &fmax) != 0) {

		if (fmin == fmax) {

			fmin += (double) - 1.;
			fmax += (double) + 1.;
		}

		plotAxisScaleManual(pl, aN, fmin, fmax);

		if (pl->axis[aN].busy == AXIS_BUSY_X) {

			fmin = plotAxisConvBackward(pl, aN, pl->viewport.min_x - pl->layout_mark_size);
			fmax = plotAxisConvBackward(pl, aN, pl->viewport.max_x + pl->layout_mark_size);

			plotAxisScaleManual(pl, aN, fmin, fmax);
		}
		else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

			fmin = plotAxisConvBackward(pl, aN, pl->viewport.max_y + pl->layout_mark_size);
			fmax = plotAxisConvBackward(pl, aN, pl->viewport.min_y - pl->layout_mark_size);

			plotAxisScaleManual(pl, aN, fmin, fmax);
		}

		pl->axis[aN].lock_tick = 0;
	}
}

void plotAxisScaleLock(plot_t *pl, int knob)
{
	int		aN;

	for (aN = 0; aN < PLOT_AXES_MAX; ++aN)
		pl->axis[aN].lock_scale = knob;
}

void plotAxisScaleDefault(plot_t *pl)
{
	int		aN;

	for (aN = 0; aN < PLOT_AXES_MAX; ++aN) {

		if (		pl->axis[aN].busy != AXIS_FREE
				&& pl->axis[aN].lock_scale == LOCK_AUTO) {

			plotAxisScaleAuto(pl, aN);
		}
	}

	for (aN = 0; aN < PLOT_AXES_MAX; ++aN) {

		if (		pl->axis[aN].busy == AXIS_BUSY_Y
				&& pl->axis[aN].lock_scale == LOCK_STACKED) {

			plotAxisScaleStacked(pl, -1);
			break;
		}
	}
}

void plotAxisScaleZoom(plot_t *pl, int aN, int origin, double zoom)
{
	if (aN < 0 || aN >= PLOT_AXES_MAX) {

		ERROR("Axis number is out of range\n");
		return ;
	}

	if (pl->axis[aN].slave != 0)
		return ;

	if (pl->axis[aN].busy == AXIS_BUSY_X) {

		pl->axis[aN].scale *= zoom;
		pl->axis[aN].offset = pl->axis[aN].offset * zoom
			+ (double) (pl->viewport.min_x - origin) /
			(double) (pl->viewport.max_x - pl->viewport.min_x) * (zoom - 1.);
	}
	else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

		pl->axis[aN].scale *= zoom;
		pl->axis[aN].offset = pl->axis[aN].offset * zoom
			+ (double) (pl->viewport.max_y - origin) /
			(double) (pl->viewport.min_y - pl->viewport.max_y) * (zoom - 1.);
	}

	pl->axis[aN].lock_scale = LOCK_FREE;
}

void plotAxisScaleMove(plot_t *pl, int aN, double move)
{
	double		span;

	if (aN < 0 || aN >= PLOT_AXES_MAX) {

		ERROR("Axis number is out of range\n");
		return ;
	}

	if (pl->axis[aN].slave != 0)
		return ;

	if (pl->axis[aN].busy == AXIS_BUSY_X) {

		span = (double) (pl->viewport.max_x - pl->viewport.min_x);
		pl->axis[aN].offset += move / span;
	}
	else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

		span = (double) (pl->viewport.min_y - pl->viewport.max_y);
		pl->axis[aN].offset += move / span;
	}

	pl->axis[aN].lock_scale = LOCK_FREE;
}

void plotAxisScaleEqual(plot_t *pl)
{
	double		zoom, aspect_x, aspect_y;

	if (pl->on_X < 0 || pl->on_X >= PLOT_AXES_MAX)
		return ;

	if (pl->on_Y < 0 || pl->on_Y >= PLOT_AXES_MAX)
		return ;

	aspect_x = (double) (pl->viewport.max_x - pl->viewport.min_x);
	aspect_y = (double) (pl->viewport.max_y - pl->viewport.min_y);

	if (pl->axis[pl->on_Y].scale < pl->axis[pl->on_X].scale) {

		zoom = pl->axis[pl->on_Y].scale / pl->axis[pl->on_X].scale;
		zoom *= aspect_y / aspect_x;

		pl->axis[pl->on_X].offset *= zoom;
		pl->axis[pl->on_X].offset += (1. - zoom) / 2.;
		pl->axis[pl->on_X].scale *= zoom;
	}
	else {
		zoom = pl->axis[pl->on_X].scale / pl->axis[pl->on_Y].scale;
		zoom *= aspect_x / aspect_y;

		pl->axis[pl->on_Y].offset *= zoom;
		pl->axis[pl->on_Y].offset += (1. - zoom) / 2.;
		pl->axis[pl->on_Y].scale *= zoom;
	}

	pl->axis[pl->on_X].lock_scale = LOCK_FREE;
	pl->axis[pl->on_Y].lock_scale = LOCK_FREE;

	pl->axis[pl->on_X].lock_tick = 0;
	pl->axis[pl->on_Y].lock_tick = 0;
}

static void
plotAxisGridPairwiseAlign(plot_t *pl, int aN, int bN)
{
	double		scale, offset;

	if (pl->axis[aN].slave != 0)
		return ;

	if (aN != bN) {

		scale = pl->axis[bN].ruler_tih / pl->axis[aN].ruler_tih;
		offset = pl->axis[bN].ruler_tis - pl->axis[aN].ruler_tis;

		pl->axis[aN].scale *= scale;
		pl->axis[aN].offset = pl->axis[aN].offset * scale
			+ offset - pl->axis[aN].ruler_tis * (scale - 1.);

		pl->axis[aN].lock_scale = LOCK_FREE;
		pl->axis[aN].lock_tick = 0;
	}
}

void plotAxisScaleGridAlign(plot_t *pl)
{
	int		aN;

	if (pl->on_X < 0 || pl->on_Y < 0)
		return ;

	for (aN = 0; aN < PLOT_AXES_MAX; ++aN) {

		if (pl->axis[aN].busy == AXIS_BUSY_X) {

			plotAxisGridPairwiseAlign(pl, aN, pl->on_X);
		}
		else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

			plotAxisGridPairwiseAlign(pl, aN, pl->on_Y);
		}
	}

	pl->axis[pl->on_X].lock_scale = LOCK_FREE;
	pl->axis[pl->on_Y].lock_scale = LOCK_FREE;

	pl->axis[pl->on_X].lock_tick = 0;
	pl->axis[pl->on_Y].lock_tick = 0;
}

void plotAxisScaleGridLock(plot_t *pl, int aN)
{
	double		scale, offset, ymin, ymax, fmin, fmax;
	int		bN;

	if (aN < 0 || aN >= PLOT_AXES_MAX) {

		ERROR("Axis number is out of range\n");
		return ;
	}

	scale = pl->axis[aN].scale;
	offset = pl->axis[aN].offset;

	if (pl->axis[aN].slave != 0) {

		bN = pl->axis[aN].slave_N;
		scale *= pl->axis[bN].scale;
		offset = offset * pl->axis[bN].scale + pl->axis[bN].offset;
	}

	ymin = - offset / scale;
	ymax = 1. / scale + ymin;

	if (plotAxisRangeCond(pl, aN, -1, &fmin, &fmax) != 0) {

		if (fmin == fmax) {

			fmin += (double) - 1.;
			fmax += (double) + 1.;
		}

		if (pl->axis[aN].busy == AXIS_BUSY_X) {

			fmin = plotAxisConvBackward(pl, aN, plotAxisConvForward(pl, aN, fmin) - pl->layout_mark_size);
			fmax = plotAxisConvBackward(pl, aN, plotAxisConvForward(pl, aN, fmax) + pl->layout_mark_size);
		}
		else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

			fmin = plotAxisConvBackward(pl, aN, plotAxisConvForward(pl, aN, fmin) + pl->layout_mark_size);
			fmax = plotAxisConvBackward(pl, aN, plotAxisConvForward(pl, aN, fmax) - pl->layout_mark_size);
		}

		ymin = (ymin < fmin) ? (fmin < ymax) ? fmin : ymax : ymin;
		ymax = (ymax > fmax) ? (fmax > ymin) ? fmax : ymin : ymax;
	}

	pl->axis[aN].lock_tick = 1;

	pl->axis[aN].ruler_min = ymin;
	pl->axis[aN].ruler_max = ymax;
}

typedef struct {

	int		aN;
	double		yval;

	double		fmin;
	double		fmax;
}
yaxis_t;

static int
plotAxisGetSorted(plot_t *pl, int bN, yaxis_t *map)
{
	int		aN, N, job, cond, yN = 0;

	for (aN = 0; aN < PLOT_AXES_MAX; ++aN) {

		if (		pl->axis[aN].busy == AXIS_BUSY_Y
				&& pl->axis[aN].slave == 0) {

			job = 0;

			if (bN >= 0) {

				job = 1;
			}
			else if (pl->axis[aN].lock_scale == LOCK_STACKED) {

				job = 1;
			}

			if (job != 0) {

				cond = plotAxisRangeCond(pl, aN, -1, &map[yN].fmin,
						&map[yN].fmax);

				if (cond != 0) {

					map[yN].aN = aN;
					map[yN].yval = plotAxisConvForward(pl, aN,
							(map[yN].fmin + map[yN].fmax) / 2.);

					yN++;
				}
			}
		}
	}

	do {
		yaxis_t		ybackup;

		job = 0;

		for (N = 1; N < yN; ++N) {

			if (map[N - 1].yval < map[N].yval) {

				memcpy(&ybackup, &map[N - 1], sizeof(yaxis_t));
				memcpy(&map[N - 1], &map[N], sizeof(yaxis_t));
				memcpy(&map[N], &ybackup, sizeof(yaxis_t));

				job = 1;
			}
		}
	}
	while (job != 0);

	return yN;
}

void plotAxisScaleStacked(plot_t *pl, int bN)
{
	double		scale, offset, ypad, yself;
	int		aN, yN, N;

	yaxis_t		ymap[PLOT_AXES_MAX];

	yN = plotAxisGetSorted(pl, bN, ymap);

	if (yN >= 2) {

		scale = 1. / (double) yN;
		offset = 0.;

		for (N = 0; N < yN; ++N) {

			aN = ymap[N].aN;

			if (ymap[N].fmin == ymap[N].fmax) {

				ymap[N].fmin += (double) - 1.;
				ymap[N].fmax += (double) + 1.;
			}

			ypad = (double) pl->layout_mark_size / (double)
				(pl->viewport.max_y - pl->viewport.min_y);

			ypad *= (ymap[N].fmax - ymap[N].fmin) / (scale - ypad * 2.);

			ymap[N].fmin += (double) - ypad;
			ymap[N].fmax += (double) + ypad;

			yself = ymap[N].fmin / (ymap[N].fmax - ymap[N].fmin);

			pl->axis[aN].scale = scale / (ymap[N].fmax - ymap[N].fmin);
			pl->axis[aN].offset = offset - yself * scale;

			pl->axis[aN].lock_scale = LOCK_STACKED;
			pl->axis[aN].lock_tick = 1;

			pl->axis[aN].ruler_min = ymap[N].fmin;
			pl->axis[aN].ruler_max = ymap[N].fmax;

			offset += scale;
		}
	}
	else {
		for (N = 0; N < PLOT_AXES_MAX; ++N) {

			if (pl->axis[N].lock_scale == LOCK_STACKED)
				pl->axis[N].lock_scale = LOCK_FREE;
		}
	}
}

int plotAxisGetByClick(plot_t *pl, int cur_X, int cur_Y)
{
	int		aN, len, rN = -1;

	cur_X = pl->viewport.min_x - pl->layout_border - cur_X;
	cur_Y = cur_Y - pl->viewport.max_y - pl->layout_border;

	for (aN = 0; aN < PLOT_AXES_MAX; ++aN) {

		if (pl->axis[aN].busy == AXIS_BUSY_X) {

			len = pl->layout_axis_box;
			len += (pl->axis[aN].compact == 0) ? pl->layout_label_box : 0;

			if (		cur_Y < pl->axis[aN].layout_pos + len
					&& cur_Y > pl->axis[aN].layout_pos) {

				rN = aN;
				break;
			}
		}

		if (pl->axis[aN].busy == AXIS_BUSY_Y) {

			len = pl->layout_axis_box;
			len += (pl->axis[aN].compact == 0) ? pl->layout_label_box : 0;

			if (		cur_X < pl->axis[aN].layout_pos + len
					&& cur_X > pl->axis[aN].layout_pos) {

				rN = aN;
				break;
			}
		}
	}

	pl->hover_axis = rN;

	return rN;
}

double plotAxisConvForward(plot_t *pl, int aN, double fval)
{
	double		scale, offset, temp;
	int		bN;

	scale = pl->axis[aN].scale;
	offset = pl->axis[aN].offset;

	if (pl->axis[aN].slave != 0) {

		bN = pl->axis[aN].slave_N;
		scale *= pl->axis[bN].scale;
		offset = offset * pl->axis[bN].scale + pl->axis[bN].offset;
	}

	if (pl->axis[aN].busy == AXIS_BUSY_X) {

		temp = (double) (pl->viewport.max_x - pl->viewport.min_x);
		scale *= temp;
		offset = offset * temp + pl->viewport.min_x;
	}
	else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

		temp = (double) (pl->viewport.min_y - pl->viewport.max_y);
		scale *= temp;
		offset = offset * temp + pl->viewport.max_y;
	}

	return fval * scale + offset;
}

double plotAxisConvBackward(plot_t *pl, int aN, double xval)
{
	double		scale, offset, temp;
	int		bN;

	scale = pl->axis[aN].scale;
	offset = pl->axis[aN].offset;

	if (pl->axis[aN].slave != 0) {

		bN = pl->axis[aN].slave_N;
		scale *= pl->axis[bN].scale;
		offset = offset * pl->axis[bN].scale + pl->axis[bN].offset;
	}

	if (pl->axis[aN].busy == AXIS_BUSY_X) {

		temp = (double) (pl->viewport.max_x - pl->viewport.min_x);
		scale *= temp;
		offset = offset * temp + pl->viewport.min_x;
	}
	else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

		temp = (double) (pl->viewport.min_y - pl->viewport.max_y);
		scale *= temp;
		offset = offset * temp + pl->viewport.max_y;
	}

	return (xval - offset) / scale;
}

void plotAxisSlave(plot_t *pl, int aN, int bN, double scale, double offset, int action)
{
	int		N, base = 0;

	if (aN < 0 || aN >= PLOT_AXES_MAX) {

		ERROR("Slave axis number is out of range\n");
		return ;
	}

	if (action == AXIS_SLAVE_DISABLE) {

		bN = pl->axis[aN].slave_N;
	}

	if (bN < 0 || bN >= PLOT_AXES_MAX) {

		ERROR("Base axis number is out of range\n");
		return ;
	}

	if (bN == aN) {

		ERROR("Axes must not be the same\n");
		return ;
	}

	if (pl->axis[bN].slave != 0) {

		ERROR("Base axis must not be slave\n");
		return ;
	}

	for (N = 0; N < PLOT_AXES_MAX; ++N) {

		if (pl->axis[N].busy != AXIS_FREE
				&& pl->axis[N].slave != 0) {

			if (pl->axis[N].slave_N == aN) {

				base = 1;
				break;
			}
		}
	}

	if (base) {

		ERROR("The axis is base for another slave\n");
		return ;
	}

	if (action == AXIS_SLAVE_ENABLE) {

		if (pl->axis[aN].slave == 0) {

			pl->axis[aN].slave = 1;
			pl->axis[aN].slave_N = bN;
			pl->axis[aN].scale = scale;
			pl->axis[aN].offset = offset;

			pl->on_X = (aN == pl->on_X) ? bN : pl->on_X;
			pl->on_Y = (aN == pl->on_Y) ? bN : pl->on_Y;
		}
	}
	else if (action == AXIS_SLAVE_HOLD_AS_IS) {

		if (bN < 0 || bN >= PLOT_AXES_MAX) {

			ERROR("Base axis number is out of range\n");
			return ;
		}

		if (pl->axis[aN].slave == 0) {

			pl->axis[aN].slave = 1;
			pl->axis[aN].slave_N = bN;

			pl->axis[aN].scale = pl->axis[aN].scale / pl->axis[bN].scale;
			pl->axis[aN].offset = (pl->axis[aN].offset - pl->axis[bN].offset)
				/ pl->axis[bN].scale;

			pl->on_X = (aN == pl->on_X) ? bN : pl->on_X;
			pl->on_Y = (aN == pl->on_Y) ? bN : pl->on_Y;
		}
	}
	else {
		if (pl->axis[aN].slave != 0) {

			pl->axis[aN].slave = 0;

			pl->axis[aN].scale = pl->axis[aN].scale * pl->axis[bN].scale;
			pl->axis[aN].offset = pl->axis[aN].offset * pl->axis[bN].scale
				+ pl->axis[bN].offset;
		}
	}
}

void plotAxisRemove(plot_t *pl, int aN)
{
	int		N, cN;

	if (aN < 0 || aN >= PLOT_AXES_MAX) {

		ERROR("Axis number is out of range\n");
		return ;
	}

	if (aN == pl->on_X || aN == pl->on_Y) {

		ERROR("Unable to remove active axis\n");
		return ;
	}

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		if (pl->figure[N].busy != 0) {

			if (pl->figure[N].axis_X == aN) {

				if (pl->axis[aN].slave != 0) {

					cN = plotGetSubtractScale(pl, pl->figure[N].data_N,
							pl->figure[N].column_X,
							pl->axis[aN].scale,
							pl->axis[aN].offset);

					if (cN != -1) {

						pl->figure[N].column_X = cN;
					}

					pl->figure[N].axis_X = pl->axis[aN].slave_N;
				}
				else {
					pl->figure[N].axis_X = pl->on_X;
				}
			}

			if (pl->figure[N].axis_Y == aN) {

				if (pl->axis[aN].slave != 0) {

					cN = plotGetSubtractScale(pl, pl->figure[N].data_N,
							pl->figure[N].column_Y,
							pl->axis[aN].scale,
							pl->axis[aN].offset);

					if (cN != -1) {

						pl->figure[N].column_Y = cN;
					}

					pl->figure[N].axis_Y = pl->axis[aN].slave_N;
				}
				else {
					pl->figure[N].axis_Y = pl->on_Y;
				}
			}
		}
	}

	for (N = 0; N < PLOT_AXES_MAX; ++N) {

		if (pl->axis[N].busy != AXIS_FREE
				&& pl->axis[N].slave != 0) {

			if (pl->axis[N].slave_N == aN) {

				plotAxisSlave(pl, N, -1, 0., 0., AXIS_SLAVE_DISABLE);
			}
		}
	}

	pl->axis[aN].busy = AXIS_FREE;
	pl->axis[aN].slave = 0;
	pl->axis[aN].label[0] = 0;
	pl->axis[aN].compact = 1;
	pl->axis[aN].exponential = 0;
}

void plotFigureAdd(plot_t *pl, int fN, int dN, int nX, int nY, int aX, int aY, const char *label)
{
	int		gN;

	if (fN < 0 || fN >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return ;
	}

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return ;
	}

	if (pl->data[dN].column_N < 1) {

		ERROR("Dataset %i has no DATA\n", dN);
		return ;
	}

	if (nX < -1 || nX >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("X column number %i is out of range\n", nX);
		return ;
	}

	if (nY < -1 || nY >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Y column number %i is out of range\n", nY);
		return ;
	}

	if (aX < 0 || aX >= PLOT_AXES_MAX) {

		ERROR("X axis number %i is out of range\n", aX);
		return ;
	}

	if (aY < 0 || aY >= PLOT_AXES_MAX) {

		ERROR("Y axis number %i is out of range\n", aY);
		return ;
	}

	if (aX == aY || pl->axis[aX].busy == AXIS_BUSY_Y
		|| pl->axis[aY].busy == AXIS_BUSY_X) {

		ERROR("Invalid axes mapping %i %i\n", aX, aY);
		return ;
	}

	pl->draw[fN].sketch = SKETCH_FINISHED;

	pl->figure[fN].busy = 1;
	pl->figure[fN].hidden = 0;
	pl->figure[fN].drawing = pl->default_drawing;
	pl->figure[fN].width = pl->default_width;
	pl->figure[fN].data_N = dN;
	pl->figure[fN].column_X = nX;
	pl->figure[fN].column_Y = nY;
	pl->figure[fN].axis_X = aX;
	pl->figure[fN].axis_Y = aY;

	if (pl->axis[aX].busy == AXIS_FREE) {

		pl->axis[aX].busy = AXIS_BUSY_X;
		pl->axis[aX].lock_scale = LOCK_AUTO;
		pl->axis[aX].compact = 1;
	}

	if (pl->axis[aY].busy == AXIS_FREE) {

		pl->axis[aY].busy = AXIS_BUSY_Y;
		pl->axis[aY].lock_scale = LOCK_AUTO;
		pl->axis[aY].compact = 1;
	}

	gN = pl->data[dN].map[nX];

	if (gN != -1) {

		plotAxisLabel(pl, aX, pl->group[gN].label);
	}

	gN = pl->data[dN].map[nY];

	if (gN != -1) {

		plotAxisLabel(pl, aY, pl->group[gN].label);
	}

	strcpy(pl->figure[fN].label, label);

	pl->on_X = (pl->on_X < 0) ? aX : pl->on_X;
	pl->on_Y = (pl->on_Y < 0) ? aY : pl->on_Y;
}

static void
plotDataBoxTextFmt(plot_t *pl, int fN, double val)
{
	char		tfmt[PLOT_STRING_MAX];
	char		tbuf[PLOT_STRING_MAX];

	int		fexp = 1;

	if (val != 0.) {

		fexp += (int) floor(log10(fabs(val)));
	}

	if (fexp >= -2 && fexp < pl->fprecision) {

		fexp = (fexp < 1) ? 1 : fexp;

		sprintf(tfmt, "%% .%df ", pl->fprecision - fexp);
	}
	else {
		sprintf(tfmt, "%% .%dE ", pl->fprecision - 1);
	}

	sprintf(tbuf, tfmt, val);
	strcat(pl->data_box_text[fN], tbuf);
}

static int
plotCheckColumnLinked(plot_t *pl, int dN, int cN)
{
	int		sN, fN, dNf, linked = 0;

	for (sN = 0; sN < PLOT_SUBTRACT; ++sN) {

		if (pl->data[dN].sub[sN].busy == SUBTRACT_TIME_MEDIAN) {

			if (cN == pl->data[dN].sub[sN].op.median.column_1) {

				linked = 1;
				break;
			}
		}
		else if (pl->data[dN].sub[sN].busy == SUBTRACT_DATA_MEDIAN) {

			if (cN == pl->data[dN].sub[sN].op.median.column_2) {

				linked = 1;
				break;
			}
		}
		else if (pl->data[dN].sub[sN].busy == SUBTRACT_SCALE) {

			if (cN == pl->data[dN].sub[sN].op.scale.column_1) {

				linked = 1;
				break;
			}
		}
		else if (pl->data[dN].sub[sN].busy == SUBTRACT_RESAMPLE) {

			if (cN == pl->data[dN].sub[sN].op.resample.column_X) {

				linked = 1;
				break;
			}
		}
		else if (pl->data[dN].sub[sN].busy == SUBTRACT_POLYFIT) {

			if (		cN == pl->data[dN].sub[sN].op.polyfit.column_X
					|| cN == pl->data[dN].sub[sN].op.polyfit.column_Y) {

				linked = 1;
				break;
			}
		}
		else if (	pl->data[dN].sub[sN].busy == SUBTRACT_BINARY_SUBTRACTION
				|| pl->data[dN].sub[sN].busy == SUBTRACT_BINARY_ADDITION
				|| pl->data[dN].sub[sN].busy == SUBTRACT_BINARY_MULTIPLICATION
				|| pl->data[dN].sub[sN].busy == SUBTRACT_BINARY_HYPOTENUSE) {

			if (		cN == pl->data[dN].sub[sN].op.binary.column_1
					|| cN == pl->data[dN].sub[sN].op.binary.column_2) {

				linked = 1;
				break;
			}
		}
		else if (	pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_DIFFERENCE
				|| pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_CUMULATIVE
				|| pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_BITMASK
				|| pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_LOW_PASS) {

			if (cN == pl->data[dN].sub[sN].op.filter.column_1) {

				linked = 1;
				break;
			}
		}
		else if (pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_MEDIAN) {

			if (cN == pl->data[dN].sub[sN].op.median.column_1) {

				linked = 1;
				break;
			}
		}
	}

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].busy != 0) {

			if (		cN == pl->figure[fN].column_X
					|| cN == pl->figure[fN].column_Y) {

				linked = 1;
				break;
			}

			dNf = pl->figure[fN].data_N;

			for (sN = 0; sN < PLOT_SUBTRACT; ++sN) {

				if (pl->data[dNf].sub[sN].busy == SUBTRACT_RESAMPLE) {

					if (		dN == pl->data[dNf].sub[sN].op.resample.in_data_N
							&& cN == pl->data[dNf].sub[sN].op.resample.in_column_X) {

						linked = 1;
						break;
					}
					else if (	dN == pl->data[dNf].sub[sN].op.resample.in_data_N
							&& cN == pl->data[dNf].sub[sN].op.resample.in_column_Y) {

						linked = 1;
						break;
					}
				}
			}

			if (linked != 0)
				break;
		}
	}

	return linked;
}

static void
plotSubtractGarbage(plot_t *pl, int dN)
{
	int		sN, cN, N;

	do {
		N = 0;

		for (sN = 0; sN < PLOT_SUBTRACT; ++sN) {

			if (pl->data[dN].sub[sN].busy != SUBTRACT_FREE) {

				cN = sN + pl->data[dN].column_N;

				if (plotCheckColumnLinked(pl, dN, cN) == 0) {

					pl->data[dN].sub[sN].busy = SUBTRACT_FREE;

					N++;
				}
			}
		}
	}
	while (N != 0);
}

void plotFigureRemove(plot_t *pl, int fN)
{
	int		N, aN, rX = 1, rY = 1;

	if (fN < 0 || fN >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return ;
	}

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		if (pl->figure[N].busy != 0 && N != fN) {

			if (pl->figure[N].axis_X == pl->figure[fN].axis_X)
				rX = 0;

			if (pl->figure[N].axis_Y == pl->figure[fN].axis_Y)
				rY = 0;
		}
	}

	pl->figure[fN].busy = 0;

	if (rX != 0) {

		aN = pl->figure[fN].axis_X;

		if (pl->on_X == aN) {

			for (N = 0; N < PLOT_AXES_MAX; ++N) {

				if (N != aN && pl->axis[N].busy == AXIS_BUSY_X
						&& pl->axis[N].slave == 0) {

					pl->on_X = N;
					break;
				}
			}
		}

		if (pl->on_X != aN) {

			plotAxisRemove(pl, aN);
		}
	}

	if (rY != 0) {

		aN = pl->figure[fN].axis_Y;

		if (pl->on_Y == aN) {

			for (N = 0; N < PLOT_AXES_MAX; ++N) {

				if (N != aN && pl->axis[N].busy == AXIS_BUSY_Y
						&& pl->axis[N].slave == 0) {

					pl->on_Y = N;
					break;
				}
			}
		}

		if (pl->on_Y != aN) {

			plotAxisRemove(pl, aN);
		}
	}

	plotSubtractGarbage(pl, pl->figure[fN].data_N);
}

void plotFigureGarbage(plot_t *pl, int dN)
{
	int		fN;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (		pl->figure[fN].busy != 0
				&& pl->figure[fN].data_N == dN) {

			plotFigureRemove(pl, fN);
		}
	}
}

void plotFigureMoveAxes(plot_t *pl, int fN)
{
	int		N, aN, rX = 1, rY = 1;

	if (fN < 0 || fN >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return ;
	}

	if (pl->on_X < 0 || pl->on_X >= PLOT_AXES_MAX)
		return ;

	if (pl->on_Y < 0 || pl->on_Y >= PLOT_AXES_MAX)
		return ;

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		if (pl->figure[N].busy != 0 && N != fN) {

			if (pl->figure[N].axis_X == pl->figure[fN].axis_X)
				rX = 0;

			if (pl->figure[N].axis_Y == pl->figure[fN].axis_Y)
				rY = 0;
		}
	}

	if (pl->figure[fN].axis_X != pl->on_X) {

		aN = pl->figure[fN].axis_X;
		pl->figure[fN].axis_X = pl->on_X;

		if (rX != 0) {

			plotAxisRemove(pl, aN);
		}
	}

	if (pl->figure[fN].axis_Y != pl->on_Y) {

		aN = pl->figure[fN].axis_Y;
		pl->figure[fN].axis_Y = pl->on_Y;

		if (rY != 0) {

			plotAxisRemove(pl, aN);
		}
	}
}

static int
plotGetFreeAxis(plot_t *pl)
{
	int		N, aN = -1;

	for (N = 0; N < PLOT_AXES_MAX; ++N) {

		if (pl->axis[N].busy == AXIS_FREE) {

			aN = N;
			break;
		}
	}

	return aN;
}

void plotFigureMakeIndividualAxes(plot_t *pl, int fN)
{
	int		N, aN, rX = 1, rY = 1;

	if (fN < 0 || fN >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return ;
	}

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		if (pl->figure[N].busy != 0 && N != fN) {

			if (pl->figure[N].axis_X == pl->figure[fN].axis_X)
				rX = 0;

			if (pl->figure[N].axis_Y == pl->figure[fN].axis_Y)
				rY = 0;
		}
	}

	if (rX == 0) {

		aN = plotGetFreeAxis(pl);

		if (aN != -1) {

			N = pl->figure[fN].axis_X;

			pl->axis[aN].busy = AXIS_BUSY_X;
			pl->figure[fN].axis_X = aN;

			plotAxisScaleAuto(pl, aN);
			plotAxisLabel(pl, aN, pl->axis[N].label);
		}
		else {
			ERROR("Unable to get free axis on X\n");
			return ;
		}
	}

	if (rY == 0) {

		aN = plotGetFreeAxis(pl);

		if (aN != -1) {

			N = pl->figure[fN].axis_Y;

			pl->axis[aN].busy = AXIS_BUSY_Y;
			pl->figure[fN].axis_Y = aN;

			plotAxisScaleAuto(pl, aN);
			plotAxisLabel(pl, aN, pl->axis[N].label);
		}
		else {
			ERROR("Unable to get free axis on Y\n");
			return ;
		}
	}
}

void plotFigureExchange(plot_t *pl, int fN, int fN_1)
{
	char		backup[sizeof(pl->figure[0])];

	if (fN < 0 || fN >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return ;
	}

	if (fN_1 < 0 || fN_1 >= PLOT_FIGURE_MAX) {

		ERROR("Figure number (exchange) is out of range\n");
		return ;
	}

	memcpy(backup, &pl->figure[fN_1], sizeof(pl->figure[0]));
	memcpy(&pl->figure[fN_1], &pl->figure[fN], sizeof(pl->figure[0]));
	memcpy(&pl->figure[fN], backup, sizeof(pl->figure[0]));
}

int plotFigureSelected(plot_t *pl)
{
	int		fN, N = 0;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (		pl->figure[fN].busy != 0
				&& pl->figure[fN].hidden == 0) {

			N++;
		}
	}

	return N;
}

static int
plotGetSubtractTimeMedianByMatch(plot_t *pl, int dN, int cNX, int length, int unwrap)
{
	int		sN, rN = -1;

	for (sN = 0; sN < PLOT_SUBTRACT; ++sN) {

		if (		pl->data[dN].sub[sN].busy == SUBTRACT_TIME_MEDIAN
				&& pl->data[dN].sub[sN].op.median.column_1 == cNX
				&& pl->data[dN].sub[sN].op.median.length == length
				&& pl->data[dN].sub[sN].op.median.unwrap == unwrap) {

			rN = sN;
			break;
		}
	}

	return rN;
}

static int
plotGetSubtractScaleByMatch(plot_t *pl, int dN, int cN, double scale, double offset)
{
	int		sN, rN = -1;

	for (sN = 0; sN < PLOT_SUBTRACT; ++sN) {

		if (pl->data[dN].sub[sN].busy == SUBTRACT_SCALE
				&& pl->data[dN].sub[sN].op.scale.column_1 == cN
				&& pl->data[dN].sub[sN].op.scale.scale == scale
				&& pl->data[dN].sub[sN].op.scale.offset == offset) {

			rN = sN;
			break;
		}
	}

	return rN;
}

static int
plotGetFreeSubtract(plot_t *pl, int dN)
{
	int		sN, rN = -1;

	for (sN = 0; sN < PLOT_SUBTRACT; ++sN) {

		if (pl->data[dN].sub[sN].busy == 0) {

			rN = sN;
			break;
		}
	}

	return rN;
}

tuple_t plotGetSubtractTimeMedian(plot_t *pl, int dN, int cNX, int cNY,
		int length, int unwrap, int opdata)
{
	tuple_t		uN = { -1, -1 };
	int		sNX, sNY;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return uN;
	}

	if (length < 1 || length > PLOT_MEDIAN_MAX) {

		ERROR("Median length %i is out of range\n", length);
		return uN;
	}

	sNX = plotGetSubtractTimeMedianByMatch(pl, dN, cNX, length, unwrap);

	if (sNX < 0) {

		sNX = plotGetFreeSubtract(pl, dN);

		if (sNX < 0) {

			ERROR("Unable to get free subtract\n");
			return uN;
		}

		pl->data[dN].sub[sNX].busy = SUBTRACT_TIME_MEDIAN;
		pl->data[dN].sub[sNX].op.median.column_1 = cNX;
		pl->data[dN].sub[sNX].op.median.length = length;
		pl->data[dN].sub[sNX].op.median.unwrap = unwrap;
		pl->data[dN].sub[sNX].op.median.opdata = 0;
	}

	sNY = plotGetFreeSubtract(pl, dN);

	if (sNY < 0) {

		ERROR("Unable to get free subtract\n");
		return uN;
	}

	pl->data[dN].sub[sNY].busy = SUBTRACT_DATA_MEDIAN;
	pl->data[dN].sub[sNY].op.median.column_1 = cNX;
	pl->data[dN].sub[sNY].op.median.column_2 = cNY;
	pl->data[dN].sub[sNY].op.median.column_3 = sNX + pl->data[dN].column_N;
	pl->data[dN].sub[sNY].op.median.length = length;
	pl->data[dN].sub[sNY].op.median.unwrap = unwrap;
	pl->data[dN].sub[sNY].op.median.opdata = opdata;

	plotDataSubtractCompute(pl, dN, sNY);

	uN.X = sNX + pl->data[dN].column_N;
	uN.Y = sNY + pl->data[dN].column_N;

	return uN;
}

int plotGetSubtractScale(plot_t *pl, int dN, int cN, double scale, double offset)
{
	int		sN;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return -1;
	}

	sN = plotGetSubtractScaleByMatch(pl, dN, cN, scale, offset);

	if (sN < 0) {

		sN = plotGetFreeSubtract(pl, dN);

		if (sN < 0) {

			ERROR("Unable to get free subtract\n");
			return -1;
		}

		pl->data[dN].sub[sN].busy = SUBTRACT_SCALE;
		pl->data[dN].sub[sN].op.scale.column_1 = cN;
		pl->data[dN].sub[sN].op.scale.scale = scale;
		pl->data[dN].sub[sN].op.scale.offset = offset;

		plotDataSubtractCompute(pl, dN, sN);
	}

	cN = sN + pl->data[dN].column_N;

	return cN;
}

int plotGetSubtractResample(plot_t *pl, int dN, int cNX, int in_dN, int in_cNX, int in_cNY)
{
	int		sN, cN;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return -1;
	}

	sN = plotGetFreeSubtract(pl, dN);

	if (sN < 0) {

		ERROR("Unable to get free subtract\n");
		return -1;
	}

	pl->data[dN].sub[sN].busy = SUBTRACT_RESAMPLE;
	pl->data[dN].sub[sN].op.resample.column_X = cNX;
	pl->data[dN].sub[sN].op.resample.in_data_N = in_dN;
	pl->data[dN].sub[sN].op.resample.in_column_X = in_cNX;
	pl->data[dN].sub[sN].op.resample.in_column_Y = in_cNY;

	plotDataSubtractCompute(pl, dN, sN);

	cN = sN + pl->data[dN].column_N;

	return cN;
}

int plotGetSubtractBinary(plot_t *pl, int dN, int opSUB, int cN1, int cN2)
{
	int		sN, cN;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return -1;
	}

	if (cN1 < -1 || cN1 >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Column number %i is out of range\n", cN1);
		return -1;
	}

	if (cN2 < -1 || cN2 >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Column number %i is out of range\n", cN2);
		return -1;
	}

	sN = plotGetFreeSubtract(pl, dN);

	if (sN < 0) {

		ERROR("Unable to get free subtract\n");
		return -1;
	}

	pl->data[dN].sub[sN].busy = opSUB;
	pl->data[dN].sub[sN].op.binary.column_1 = cN1;
	pl->data[dN].sub[sN].op.binary.column_2 = cN2;

	plotDataSubtractCompute(pl, dN, sN);

	cN = sN + pl->data[dN].column_N;

	return cN;
}

int plotGetSubtractFilter(plot_t *pl, int dN, int cN, int opSUB, double gain)
{
	int		sN;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return -1;
	}

	if (cN < -1 || cN >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Column number %i is out of range\n", cN);
		return -1;
	}

	sN = plotGetFreeSubtract(pl, dN);

	if (sN < 0) {

		ERROR("Unable to get free subtract\n");
		return -1;
	}

	pl->data[dN].sub[sN].busy = opSUB;
	pl->data[dN].sub[sN].op.filter.column_1 = cN;
	pl->data[dN].sub[sN].op.filter.gain = gain;

	plotDataSubtractCompute(pl, dN, sN);

	cN = sN + pl->data[dN].column_N;

	return cN;
}

int plotGetSubtractMedian(plot_t *pl, int dN, int cN, int opSUB, int length)
{
	int		sN;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return -1;
	}

	if (cN < -1 || cN >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Column number %i is out of range\n", cN);
		return -1;
	}

	if (length < 1 || length > PLOT_MEDIAN_MAX) {

		ERROR("Median length %i is out of range\n", length);
		return -1;
	}

	sN = plotGetFreeSubtract(pl, dN);

	if (sN < 0) {

		ERROR("Unable to get free subtract\n");
		return -1;
	}

	pl->data[dN].sub[sN].busy = opSUB;
	pl->data[dN].sub[sN].op.median.column_1 = cN;
	pl->data[dN].sub[sN].op.median.length = length;
	pl->data[dN].sub[sN].op.median.unwrap = 0;
	pl->data[dN].sub[sN].op.median.opdata = 0;

	plotDataSubtractCompute(pl, dN, sN);

	cN = sN + pl->data[dN].column_N;

	return cN;
}

int plotGetFreeFigure(plot_t *pl)
{
	int		N, fN = -1;

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		if (pl->figure[N].busy == 0) {

			fN = N;
			break;
		}
	}

	return fN;
}

int plotFigureSubtractGetMedianConfig(plot_t *pl, int fN, int config[3])
{
	int		dN, cN, sN;

	if (fN < 0 || fN >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return -1;
	}

	dN = pl->figure[fN].data_N;
	cN = pl->figure[fN].column_Y;

	sN = cN - pl->data[dN].column_N;

	if (		sN >= 0 && sN < PLOT_SUBTRACT
			&& pl->data[dN].sub[sN].busy == SUBTRACT_DATA_MEDIAN) {

		config[0] = pl->data[dN].sub[sN].op.median.length;
		config[1] = pl->data[dN].sub[sN].op.median.unwrap;
		config[2] = pl->data[dN].sub[sN].op.median.opdata;

		return sN;
	}

	return -1;
}

void plotFigureSubtractTimeMedian(plot_t *pl, int fN, int length, int unwrap, int opdata)
{
	tuple_t		uN;
	int		dN, cNX, cNY, sN;

	if (fN < 0 || fN >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return ;
	}

	dN = pl->figure[fN].data_N;
	cNX = pl->figure[fN].column_X;
	cNY = pl->figure[fN].column_Y;

	if (length < 1) {

		sN = cNY - pl->data[dN].column_N;

		if (		sN >= 0 && sN < PLOT_SUBTRACT
				&& pl->data[dN].sub[sN].busy == SUBTRACT_DATA_MEDIAN) {

			cNX = pl->data[dN].sub[sN].op.median.column_1;
			cNY = pl->data[dN].sub[sN].op.median.column_2;

			pl->figure[fN].column_X = cNX;
			pl->figure[fN].column_Y = cNY;

			plotSubtractGarbage(pl, dN);
		}
	}
	else {
		sN = cNY - pl->data[dN].column_N;

		if (		sN >= 0 && sN < PLOT_SUBTRACT
				&& pl->data[dN].sub[sN].busy == SUBTRACT_DATA_MEDIAN) {

			if (		pl->data[dN].sub[sN].op.median.length != length
					|| pl->data[dN].sub[sN].op.median.unwrap != unwrap
					|| pl->data[dN].sub[sN].op.median.opdata != opdata) {

				cNX = pl->data[dN].sub[sN].op.median.column_1;
				cNY = pl->data[dN].sub[sN].op.median.column_2;

				pl->figure[fN].column_X = cNX;
				pl->figure[fN].column_Y = cNY;

				plotSubtractGarbage(pl, dN);

				uN = plotGetSubtractTimeMedian(pl, dN, cNX, cNY, length, unwrap, opdata);

				if (uN.X != -1) {

					pl->figure[fN].column_X = uN.X;
					pl->figure[fN].column_Y = uN.Y;
				}
			}
		}
		else {
			uN = plotGetSubtractTimeMedian(pl, dN, cNX, cNY, length, unwrap, opdata);

			if (uN.X != -1) {

				pl->figure[fN].column_X = uN.X;
				pl->figure[fN].column_Y = uN.Y;
			}
		}
	}
}

void plotFigureSubtractScale(plot_t *pl, int fN, int aBUSY, double scale, double offset)
{
	int		dN, cN;

	if (fN < 0 || fN >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return ;
	}

	dN = pl->figure[fN].data_N;

	if (aBUSY == AXIS_BUSY_X) {

		cN = plotGetSubtractScale(pl, dN, pl->figure[fN].column_X, scale, offset);

		if (cN != -1) {

			pl->figure[fN].column_X = cN;
		}
	}
	else if (aBUSY == AXIS_BUSY_Y) {

		cN = plotGetSubtractScale(pl, dN, pl->figure[fN].column_Y, scale, offset);

		if (cN != -1) {

			pl->figure[fN].column_Y = cN;
		}
	}
}

static int
plotFigureSubtractAdd(plot_t *pl, int fN, int fN_1, int fN_2, int opSUB)
{
	const char	*label_1, *label_2, *delim;
	int		dN, aNX, aNY, cNX, cNY;

	dN = pl->figure[fN_1].data_N;

	cNX = pl->figure[fN_1].column_X;
	aNX = pl->figure[fN_1].axis_X;

	if (aNX != pl->figure[fN_2].axis_X) {

		ERROR("Both figures must be on the same axis on X\n");
		return 0;
	}

	if (		dN != pl->figure[fN_2].data_N
			|| cNX != pl->figure[fN_2].column_X) {

		cNY = plotGetSubtractResample(pl, dN, cNX,
				pl->figure[fN_2].data_N,
				pl->figure[fN_2].column_X,
				pl->figure[fN_2].column_Y);

		if (cNY < 0) {

			ERROR("Unable to get resample subtract\n");
			return 0;
		}
	}
	else {
		cNY = pl->figure[fN_2].column_Y;
	}

	cNY = plotGetSubtractBinary(pl, dN, opSUB, pl->figure[fN_1].column_Y, cNY);

	if (cNY < 0) {

		return 0;
	}

	aNY = plotGetFreeAxis(pl);

	if (aNY != -1) {

		pl->axis[aNY].busy = AXIS_BUSY_Y;
		plotAxisLabel(pl, aNY, pl->axis[pl->figure[fN_1].axis_Y].label);
	}
	else {
		aNY = pl->figure[fN_1].axis_Y;
	}

	plotFigureAdd(pl, fN, dN, cNX, cNY, aNX, aNY, "");

	label_1 = pl->figure[fN_1].label;
	label_2 = pl->figure[fN_2].label;

	delim = strrchr(label_1, ' ');
	label_1 = (delim != NULL) ? delim + 1 : label_1;

	delim = strrchr(label_2, ' ');
	label_2 = (delim != NULL) ? delim + 1 : label_2;

	if (opSUB == SUBTRACT_BINARY_SUBTRACTION) {

		sprintf(pl->figure[fN].label, "R: %.35s - %.35s", label_1, label_2);
	}
	else if (opSUB == SUBTRACT_BINARY_ADDITION) {

		sprintf(pl->figure[fN].label, "A: %.35s + %.35s", label_1, label_2);
	}
	else if (opSUB == SUBTRACT_BINARY_MULTIPLICATION) {

		sprintf(pl->figure[fN].label, "X: %.35s * %.35s", label_1, label_2);
	}
	else if (opSUB == SUBTRACT_BINARY_HYPOTENUSE) {

		sprintf(pl->figure[fN].label, "H: %.35s ~ %.35s", label_1, label_2);
	}

	pl->figure[fN].drawing = pl->figure[fN_1].drawing;
	pl->figure[fN].width = pl->figure[fN_1].width;

	return AXIS_BUSY_Y;
}

void plotFigureSubtractFilter(plot_t *pl, int fN_1, int opSUB, double gain)
{
	int		fN, dN, cN, aN;

	if (fN_1 < 0 || fN_1 >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return ;
	}

	dN = pl->figure[fN_1].data_N;
	cN = pl->figure[fN_1].column_Y;

	fN = plotGetFreeFigure(pl);

	if (fN < 0) {

		ERROR("Unable to get free figure to subtract\n");
		return ;
	}

	if (opSUB == SUBTRACT_FILTER_MEDIAN) {

		cN = plotGetSubtractMedian(pl, dN, cN, opSUB, (int) gain);
	}
	else {
		cN = plotGetSubtractFilter(pl, dN, cN, opSUB, gain);
	}

	if (cN < 0) {

		return ;
	}

	if (		opSUB == SUBTRACT_FILTER_LOW_PASS
			|| opSUB == SUBTRACT_FILTER_MEDIAN) {

		aN = pl->figure[fN_1].axis_Y;
	}
	else {
		aN = plotGetFreeAxis(pl);

		if (aN != -1) {

			pl->axis[aN].busy = AXIS_BUSY_Y;
			plotAxisLabel(pl, aN, pl->axis[pl->figure[fN_1].axis_Y].label);
		}
		else {
			aN = pl->figure[fN_1].axis_Y;
		}
	}

	plotFigureAdd(pl, fN, dN, pl->figure[fN_1].column_X, cN,
			pl->figure[fN_1].axis_X, aN, "");

	if (opSUB == SUBTRACT_FILTER_DIFFERENCE) {

		sprintf(pl->figure[fN].label, "D: %.75s", pl->figure[fN_1].label);
	}
	else if (opSUB == SUBTRACT_FILTER_CUMULATIVE) {

		sprintf(pl->figure[fN].label, "C: %.75s", pl->figure[fN_1].label);
	}
	else if (opSUB == SUBTRACT_FILTER_BITMASK) {

		int	bf[2] = { 0, (int) gain };

		bf[0] = bf[1] & 0xFFU;
		bf[1] = bf[1] >> 8;

		if (bf[0] == bf[1]) {

			sprintf(pl->figure[fN].label, "B(%d): %.75s",
					(int) bf[0], pl->figure[fN_1].label);
		}
		else {
			sprintf(pl->figure[fN].label, "B(%d-%d): %.75s",
					(int) bf[0], (int) bf[1], pl->figure[fN_1].label);
		}
	}
	else if (opSUB == SUBTRACT_FILTER_LOW_PASS) {

		sprintf(pl->figure[fN].label, "L(%.2E): %.75s",
				gain, pl->figure[fN_1].label);
	}
	else if (opSUB == SUBTRACT_FILTER_MEDIAN) {

		sprintf(pl->figure[fN].label, "M(%d): %.75s",
				(int) gain, pl->figure[fN_1].label);
	}

	pl->figure[fN].drawing = pl->figure[fN_1].drawing;
	pl->figure[fN].width = pl->figure[fN_1].width;

	if (opSUB == SUBTRACT_FILTER_LOW_PASS) {

		/* Do nothing */
	}
	else {
		plotAxisScaleAutoCond(pl, pl->figure[fN].axis_Y, pl->figure[fN].axis_X);

		pl->on_X = pl->figure[fN].axis_X;
		pl->on_Y = pl->figure[fN].axis_Y;

		if (pl->axis[pl->on_X].slave != 0) {

			pl->on_X = pl->axis[pl->on_X].slave_N;
		}

		if (pl->axis[pl->on_Y].slave != 0) {

			pl->on_Y = pl->axis[pl->on_Y].slave_N;
		}
	}
}

static void
plotFigureSubtractBinaryLinked(plot_t *pl, int fN, int opSUB, int fNP[2])
{
	int		dN, dN1, sN, sE, cN, cN1, fN_1, fN_2;

	dN = pl->figure[fN].data_N;
	sN = pl->figure[fN].column_Y - pl->data[dN].column_N;

	fN_1 = -1;
	fN_2 = -1;

	if (		sN >= 0 && sN < PLOT_SUBTRACT
			&& pl->data[dN].sub[sN].busy == opSUB) {

		cN = pl->data[dN].sub[sN].op.binary.column_1;
		sE = cN - pl->data[dN].column_N;

		dN1 = dN;
		cN1 = cN;

		if (		sE >= 0 && sE < PLOT_SUBTRACT
				&& pl->data[dN].sub[sE].busy == SUBTRACT_RESAMPLE) {

			dN1 = pl->data[dN].sub[sE].op.resample.in_data_N;
			cN1 = pl->data[dN].sub[sE].op.resample.in_column_Y;
		}

		for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

			if (pl->figure[fN].busy != 0) {

				if (		dN == pl->figure[fN].data_N
						&& cN == pl->figure[fN].column_Y) {

					fN_1 = fN;
					break;
				}

				if (		dN1 == pl->figure[fN].data_N
						&& cN1 == pl->figure[fN].column_Y) {

					fN_1 = fN;
					break;
				}
			}
		}

		cN = pl->data[dN].sub[sN].op.binary.column_2;
		sE = cN - pl->data[dN].column_N;

		dN1 = dN;
		cN1 = cN;

		if (		sE >= 0 && sE < PLOT_SUBTRACT
				&& pl->data[dN].sub[sE].busy == SUBTRACT_RESAMPLE) {

			dN1 = pl->data[dN].sub[sE].op.resample.in_data_N;
			cN1 = pl->data[dN].sub[sE].op.resample.in_column_Y;
		}

		for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

			if (pl->figure[fN].busy != 0) {

				if (		dN == pl->figure[fN].data_N
						&& cN == pl->figure[fN].column_Y) {

					fN_2 = fN;
					break;
				}

				if (		dN1 == pl->figure[fN].data_N
						&& cN1 == pl->figure[fN].column_Y) {

					fN_2 = fN;
					break;
				}
			}
		}
	}

	fNP[0] = fN_1;
	fNP[1] = fN_2;
}

void plotFigureSubtractSwitch(plot_t *pl, int opSUB)
{
	int		fN, fN_1, fN_2, fNQ[2], rBUSY, N = 0;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].busy != 0 && pl->figure[fN].hidden == 0) {

			if (N < 2) {

				fNQ[N] = fN;
			}

			N++;
		}
	}

	if (N == 1) {

		fN = fNQ[0];

		plotFigureSubtractBinaryLinked(pl, fN, opSUB, fNQ);

		fN_1 = fNQ[0];
		fN_2 = fNQ[1];

		if (fN_1 != -1 && fN_2 != -1) {

			pl->figure[fN].hidden = 1;

			pl->figure[fN_1].hidden = 0;
			pl->figure[fN_2].hidden = 0;

			pl->on_X = pl->figure[fN_1].axis_X;
			pl->on_Y = pl->figure[fN_1].axis_Y;
		}
	}
	else if (N == 2) {

		fN_1 = fNQ[0];
		fN_2 = fNQ[1];

		fN = -1;

		for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

			if (pl->figure[N].busy != 0) {

				plotFigureSubtractBinaryLinked(pl, N, opSUB, fNQ);

				if (fNQ[0] == fN_1 && fNQ[1] == fN_2) {

					fN = N;
					break;
				}

				if (fNQ[0] == fN_2 && fNQ[1] == fN_1) {

					fN = N;
					break;
				}
			}
		}

		if (fN != -1) {

			pl->figure[fN].hidden = 0;

			pl->figure[fN_1].hidden = 1;
			pl->figure[fN_2].hidden = 1;

			if (		pl->figure[fN].axis_X == pl->figure[fN_1].axis_X
					&& pl->figure[fN].axis_X == pl->figure[fN_2].axis_X) {

				plotAxisScaleAutoCond(pl, pl->figure[fN].axis_Y,
						pl->figure[fN].axis_X);
			}
			else if (	pl->figure[fN].axis_Y == pl->figure[fN_1].axis_Y
					&& pl->figure[fN].axis_Y == pl->figure[fN_2].axis_Y) {

				plotAxisScaleAutoCond(pl, pl->figure[fN].axis_X,
						pl->figure[fN].axis_Y);
			}

			pl->on_X = pl->figure[fN].axis_X;
			pl->on_Y = pl->figure[fN].axis_Y;
		}
		else {
			fN = plotGetFreeFigure(pl);

			if (fN < 0) {

				ERROR("Unable to get free figure to subtract\n");
				return ;
			}

			rBUSY = plotFigureSubtractAdd(pl, fN, fN_1, fN_2, opSUB);

			if (rBUSY != 0) {

				pl->figure[fN_1].hidden = 1;
				pl->figure[fN_2].hidden = 1;

				if (rBUSY == AXIS_BUSY_X) {

					plotAxisScaleAutoCond(pl, pl->figure[fN].axis_X,
							pl->figure[fN].axis_Y);
				}
				else if (rBUSY == AXIS_BUSY_Y) {

					plotAxisScaleAutoCond(pl, pl->figure[fN].axis_Y,
							pl->figure[fN].axis_X);
				}
				else {
					plotAxisScaleAuto(pl, pl->figure[fN].axis_X);
					plotAxisScaleAuto(pl, pl->figure[fN].axis_Y);
				}

				pl->on_X = pl->figure[fN].axis_X;
				pl->on_Y = pl->figure[fN].axis_Y;
			}
		}
	}

	if (pl->axis[pl->on_X].slave != 0) {

		pl->on_X = pl->axis[pl->on_X].slave_N;
	}

	if (pl->axis[pl->on_Y].slave != 0) {

		pl->on_Y = pl->axis[pl->on_Y].slave_N;
	}
}

void plotFigureSubtractResample(plot_t *pl, int fN)
{
	int		N, dN, aN, cNX, cNY;

	dN = pl->figure[fN].data_N;
	aN = pl->figure[fN].axis_X;
	cNX = pl->figure[fN].column_X;

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		if (		pl->figure[N].busy != 0
				&& pl->figure[N].hidden == 0
				&& N != fN) {

			if (aN != pl->figure[N].axis_X) {

				ERROR("All figures must be on the same axis on X\n");
				return ;
			}
		}
	}

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		if (		pl->figure[N].busy != 0
				&& pl->figure[N].hidden == 0
				&& N != fN) {

			if (		dN != pl->figure[N].data_N
					|| cNX != pl->figure[N].column_X) {

				cNY = plotGetSubtractResample(pl, dN, cNX,
						pl->figure[N].data_N,
						pl->figure[N].column_X,
						pl->figure[N].column_Y);

				if (cNY < 0) {

					ERROR("Unable to get resample subtract\n");
					return ;
				}

				pl->figure[N].data_N = dN;
				pl->figure[N].column_X = cNX;
				pl->figure[N].column_Y = cNY;
			}
		}
	}
}

int plotDataBoxPolyfit(plot_t *pl, int fN)
{
	int		N, dN, sN, N0, N1;
	double		*coefs, std;

	if (fN < 0 || fN >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return 0;
	}

	dN = pl->figure[fN].data_N;
	sN = pl->figure[fN].column_Y - pl->data[dN].column_N;

	if (sN < 0 || sN >= PLOT_SUBTRACT) {

		return 0;
	}
	else if (pl->data[dN].sub[sN].busy != SUBTRACT_POLYFIT) {

		return 0;
	}

	N0 = pl->data[dN].sub[sN].op.polyfit.poly_N0;
	N1 = pl->data[dN].sub[sN].op.polyfit.poly_N1;

	coefs = pl->data[dN].sub[sN].op.polyfit.coefs;
	std = pl->data[dN].sub[sN].op.polyfit.std;

	for (N = 0; N < PLOT_DATA_BOX_MAX; ++N) {

		pl->data_box_text[N][0] = 0;

		if (N == 0 && N1 == 0) {

			sprintf(pl->data_box_text[N], " [%i] = ", N);
			plotDataBoxTextFmt(pl, N, coefs[N]);
		}
		else if (N < N1 - N0 + 1) {

			char		sfmt[PLOT_STRING_MAX];

			sprintf(sfmt, " [%%i] = %% .%iE ", pl->fprecision - 1);
			sprintf(pl->data_box_text[N], sfmt, N + N0, coefs[N]);
		}
		else if (N == N1 - N0 + 1) {

			sprintf(pl->data_box_text[N], " STD = ");
			plotDataBoxTextFmt(pl, N, std);
		}
	}

	if (pl->data_box_on != DATA_BOX_POLYFIT) {

		pl->data_box_on = DATA_BOX_POLYFIT;
		pl->data_box_X = pl->viewport.max_x;
		pl->data_box_Y = 0;
	}

	return 1;
}

void plotFigureSubtractPolyfit(plot_t *pl, int fN_1, int N0, int N1)
{
	int		N, fN, dN, sN, cN, aN, bN;
	double		scale_X, offset_X, scale_Y, offset_Y;

	if (fN_1 < 0 || fN_1 >= PLOT_FIGURE_MAX) {

		ERROR("Figure number is out of range\n");
		return ;
	}

	if (N0 < 0 || N0 > N1) {

		ERROR("Polynomial base %i is out of range\n", N0);
		return ;
	}

	if (N1 < 0 || N1 > PLOT_POLYFIT_MAX) {

		ERROR("Polynomial degree %i is out of range\n", N1);
		return ;
	}

	fN = plotGetFreeFigure(pl);

	if (fN < 0) {

		ERROR("Unable to get free figure to subtract\n");
		return ;
	}

	dN = pl->figure[fN_1].data_N;
	sN = plotGetFreeSubtract(pl, dN);

	if (sN < 0) {

		ERROR("Unable to get free subtract\n");
		return ;
	}

	aN = pl->figure[fN_1].axis_X;

	scale_X = pl->axis[aN].scale;
	offset_X = pl->axis[aN].offset;

	if (pl->axis[aN].slave != 0) {

		bN = pl->axis[aN].slave_N;
		scale_X *= pl->axis[bN].scale;
		offset_X = offset_X * pl->axis[bN].scale + pl->axis[bN].offset;
	}

	aN = pl->figure[fN_1].axis_Y;

	scale_Y = pl->axis[aN].scale;
	offset_Y = pl->axis[aN].offset;

	if (pl->axis[aN].slave != 0) {

		bN = pl->axis[aN].slave_N;
		scale_Y *= pl->axis[bN].scale;
		offset_Y = offset_X * pl->axis[bN].scale + pl->axis[bN].offset;
	}

	plotDataPolyfit(pl, dN, pl->figure[fN_1].column_X, pl->figure[fN_1].column_Y,
			scale_X, offset_X, scale_Y, offset_Y, N0, N1);

	pl->data[dN].sub[sN].busy = SUBTRACT_POLYFIT;
	pl->data[dN].sub[sN].op.polyfit.column_X = pl->figure[fN_1].column_X;
	pl->data[dN].sub[sN].op.polyfit.column_Y = pl->figure[fN_1].column_Y;
	pl->data[dN].sub[sN].op.polyfit.poly_N0 = N0;
	pl->data[dN].sub[sN].op.polyfit.poly_N1 = N1;

	for (N = 0; N < N1 - N0 + 1; ++N) {

		pl->data[dN].sub[sN].op.polyfit.coefs[N] = pl->lsq.sol.m[N];
	}

	pl->data[dN].sub[sN].op.polyfit.std = pl->lsq.std.m[0];

	plotDataSubtractCompute(pl, dN, sN);

	cN = sN + pl->data[dN].column_N;
	aN = pl->figure[fN_1].axis_Y;

	plotFigureAdd(pl, fN, dN, pl->figure[fN_1].column_X, cN,
			pl->figure[fN_1].axis_X, aN, "");

	sprintf(pl->figure[fN].label, "P: %.75s", pl->figure[fN_1].label);

	pl->figure[fN].drawing = pl->figure[fN_1].drawing;
	pl->figure[fN].width = pl->figure[fN_1].width;

	plotDataBoxPolyfit(pl, fN);
}

static void
plotLabelFusedCSV(plot_t *pl, char *label, const char *name, const char *unit)
{
	read_t		*rd = (read_t *) pl->ld;

	const char	*s;
	char		*l = label;
	int		n;

	s = strrchr(name, ' ');
	s = (s != NULL) ? s + 1 : name;
	n = 0;

	while (*s != 0) {

		if (*s == '@')
			break;

		*l++ = (strchr(rd->mk_text.space, *s) == NULL) ? *s : '_';

		++s;
		++n;

		if (n >= 50)
			break;
	}

	s = unit;
	n = 0;

	if (*s != 0) {

		*l++ = '@';
	}

	while (*s != 0) {

		if (*s == '@')
			break;

		*l++ = (strchr(rd->mk_text.space, *s) == NULL) ? *s : '_';

		++s;
		++n;

		if (n >= 20)
			break;
	}

	*l = 0;
}

void plotFigureExportCSV(plot_t *pl, const char *file)
{
	int		list_dN[16], list_cN[16], list_fN[16];
	int		N, fN, aN, job, len_N = 0;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (		pl->figure[fN].busy != 0
				&& pl->figure[fN].hidden == 0) {

			job = 1;

			for (N = 0; N < len_N; ++N) {

				if (		list_dN[N] == pl->figure[fN].data_N
						&& list_cN[N] == pl->figure[fN].column_X) {

					job = 0;
					break;
				}
			}

			if (job != 0) {

				list_dN[len_N] = pl->figure[fN].data_N;
				list_cN[len_N] = pl->figure[fN].column_X;
				list_fN[len_N] = fN;

				len_N++;
			}

			job = 1;

			for (N = 0; N < len_N; ++N) {

				if (		list_dN[N] == pl->figure[fN].data_N
						&& list_cN[N] == pl->figure[fN].column_Y) {

					job = 0;
					break;
				}
			}

			if (job != 0) {

				list_dN[len_N] = pl->figure[fN].data_N;
				list_cN[len_N] = pl->figure[fN].column_Y;
				list_fN[len_N] = fN;

				len_N++;
			}
		}
	}

	if (len_N >= 2) {

		FILE		*fd_csv;
#ifdef _WINDOWS
		read_t		*rd = (read_t *) pl->ld;
#endif /* _WINDOWS */
		char		labelbuf[PLOT_STRING_MAX];

		fd_csv = unified_fopen(file, "w");

		if (fd_csv == NULL) {

			ERROR("fopen(\"%s\"): %s\n", file, strerror(errno));
			return ;
		}

		for (N = 0; N < len_N; ++N) {

			fN = list_fN[N];

			if (list_cN[N] == pl->figure[fN].column_X) {

				aN = pl->figure[fN].axis_X;

				plotLabelFusedCSV(pl, labelbuf, "time",
						pl->axis[aN].label);
			}
			else {
				aN = pl->figure[fN].axis_Y;

				plotLabelFusedCSV(pl, labelbuf,
						pl->figure[fN].label,
						pl->axis[aN].label);
			}

#ifdef _WINDOWS
			if (rd->legacy_label == 1) {

				legacy_UTF8_to_ACP(labelbuf, labelbuf, sizeof(labelbuf));
			}
			else if (rd->legacy_label == 2) {

				legacy_UTF8_to_OEM(labelbuf, labelbuf, sizeof(labelbuf));
			}
#endif /* _WINDOWS */

			fprintf(fd_csv, "%s;", labelbuf);
		}

		fprintf(fd_csv, "\n");

		plotDataFileCSV(pl, list_dN, list_cN, len_N, fd_csv);

		fclose(fd_csv);
	}
}

void plotFigureClean(plot_t *pl)
{
	int		N;

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		pl->figure[N].busy = 0;
		pl->figure[N].hidden = 0;
		pl->figure[N].label[0] = 0;
	}

	for (N = 0; N < PLOT_AXES_MAX; ++N) {

		pl->axis[N].busy = AXIS_FREE;
		pl->axis[N].slave = 0;
		pl->axis[N].label[0] = 0;
		pl->axis[N].compact = 1;
		pl->axis[N].exponential = 0;
	}

	pl->legend_X = 0;
	pl->legend_Y = 0;

	pl->data_box_on = DATA_BOX_FREE;
	pl->data_box_X = pl->viewport.max_x;
	pl->data_box_Y = 0;

	pl->slice_on = 0;
	pl->slice_mode_N = 0;

	pl->on_X = -1;
	pl->on_Y = -1;

	pl->hover_figure = -1;
	pl->hover_legend = -1;
	pl->hover_data_box = -1;
	pl->hover_axis = -1;

	pl->mark_on = 0;

	plotSketchClean(pl);
}

static void
plotMarkLayout(plot_t *pl)
{
	const fval_t	*row;
	double		shuffle, total, scale, offset, fval_X, fval_Y;
	int		fN, vN, aN, bN, cX, cY, cZ, N, id_N, fMAX = 0;

	const int	ltdense[PLOT_FIGURE_MAX] = {

		250, 353, 433, 500, 559, 612, 661, 707		/* 250 * sqrt(N) */
	};

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (		pl->figure[fN].busy != 0
				&& pl->figure[fN].hidden == 0) {

			fMAX++;
		}
	}

	if (fMAX == 0)
		return ;

	pl->layout_mark_size = pl->layout_font_height * pl->mark_size / 200;
	pl->layout_mark_size = (pl->layout_mark_size < 1) ? 1 : pl->layout_mark_size;

	pl->mark_count = (pl->viewport.max_x - pl->viewport.min_x)
		* pl->mark_density / (pl->layout_mark_size * ltdense[fMAX - 1]);

	pl->mark_count = (pl->mark_count > PLOT_MARK_MAX) ? PLOT_MARK_MAX
		: (pl->mark_count < 4) ? 4 : pl->mark_count;

	shuffle = (double) (SDL_GetTicks() % 100) / (double) (pl->mark_count * 100);
	total = (double) (pl->mark_count * fMAX);

	for (fN = 0, vN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (		pl->figure[fN].busy != 0
				&& pl->figure[fN].hidden == 0) {

			aN = pl->figure[fN].axis_X;
			cZ = pl->figure[fN].column_X;

			scale = pl->axis[aN].scale;
			offset = pl->axis[aN].offset;

			if (pl->axis[aN].slave != 0) {

				bN = pl->axis[aN].slave_N;

				scale *= pl->axis[bN].scale;
				offset = offset * pl->axis[bN].scale
					+ pl->axis[bN].offset;
			}

			for (N = 0; N < pl->mark_count; ++N) {

				fval_X = shuffle + (double) (N * fMAX + vN) / total;
				fval_X = (fval_X > 1.) ? fval_X - 1. : fval_X;

				fval_X = (fval_X - offset) / scale;

				row = plotDataSliceGet(pl, pl->figure[fN].data_N,
							cZ, fval_X, &id_N);

				if (row != NULL) {

					cX = pl->figure[fN].column_X;
					cY = pl->figure[fN].column_Y;

					fval_X = (cX < 0) ? id_N : row[cX];
					fval_Y = (cY < 0) ? id_N : row[cY];

					pl->figure[fN].mark_X[N] = fval_X;
					pl->figure[fN].mark_Y[N] = fval_Y;
				}
				else {
					pl->figure[fN].mark_X[N] = FP_NAN;
					pl->figure[fN].mark_Y[N] = FP_NAN;
				}
			}

			vN++;
		}
	}
}

static void
plotMarkDraw(plot_t *pl, SDL_Surface *surface)
{
	double		X, Y, scale_X, scale_Y, offset_X, offset_Y;
	int		N, fN, aN, bN, fwidth;

	int		ncolor;

	SDL_LockSurface(surface);

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (		pl->figure[fN].busy != 0
				&& pl->figure[fN].hidden == 0) {

			ncolor = (pl->figure[fN].hidden != 0) ? 9 : fN + 1;

			fwidth = pl->figure[fN].width;
			fwidth = (fwidth < 1) ? 1 : fwidth;

			aN = pl->figure[fN].axis_X;
			scale_X = pl->axis[aN].scale;
			offset_X = pl->axis[aN].offset;

			if (pl->axis[aN].slave != 0) {

				bN = pl->axis[aN].slave_N;

				scale_X *= pl->axis[bN].scale;
				offset_X = offset_X * pl->axis[bN].scale
					+ pl->axis[bN].offset;
			}

			aN = pl->figure[fN].axis_Y;
			scale_Y = pl->axis[aN].scale;
			offset_Y = pl->axis[aN].offset;

			if (pl->axis[aN].slave != 0) {

				bN = pl->axis[aN].slave_N;

				scale_Y *= pl->axis[bN].scale;
				offset_Y = offset_Y * pl->axis[bN].scale
					+ pl->axis[bN].offset;
			}

			X = (double) (pl->viewport.max_x - pl->viewport.min_x);
			Y = (double) (pl->viewport.min_y - pl->viewport.max_y);

			scale_X *= X;
			offset_X = offset_X * X + pl->viewport.min_x;
			scale_Y *= Y;
			offset_Y = offset_Y * Y + pl->viewport.max_y;

			for (N = 0; N < pl->mark_count; ++N) {

				X = pl->figure[fN].mark_X[N] * scale_X + offset_X;
				Y = pl->figure[fN].mark_Y[N] * scale_Y + offset_Y;

				if (fp_isfinite(X) && fp_isfinite(Y)) {

					drawMarkCanvas(pl->dw, surface, &pl->viewport, X, Y,
							pl->layout_mark_size, fN, ncolor, fwidth);
				}
			}
		}
	}

	SDL_UnlockSurface(surface);
}

void plotGroupAdd(plot_t *pl, int dN, int gN, int cN)
{
	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return ;
	}

	if (gN < 0 || gN >= PLOT_GROUP_MAX) {

		ERROR("Group number is out of range\n");
		return ;
	}

	if (cN < -1 || cN >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Column number %i is out of range\n", cN);
		return ;
	}

	pl->data[dN].map[cN] = gN;
}

void plotGroupLabel(plot_t *pl, int gN, const char *label)
{
	if (gN < 0 || gN >= PLOT_GROUP_MAX) {

		ERROR("Group number is out of range\n");
		return ;
	}

	if (label[0] != 0) {

		strcpy(pl->group[gN].label, label);
	}
}

void plotGroupMedian(plot_t *pl, int gN, int length, int unwrap, int opdata)
{
	if (gN < 0 || gN >= PLOT_GROUP_MAX) {

		ERROR("Group number is out of range\n");
		return ;
	}

	pl->group[gN].op_time_median = (length >= 1) ? 1 : 0;
	pl->group[gN].op_time_unwrap = (unwrap != 0) ? 1 : 0;
	pl->group[gN].op_time_opdata = (opdata != 0) ? 1 : 0;

	pl->group[gN].length = length;
}

void plotGroupScale(plot_t *pl, int gN, int knob, double scale, double offset)
{
	if (gN < 0 || gN >= PLOT_GROUP_MAX) {

		ERROR("Group number is out of range\n");
		return ;
	}

	pl->group[gN].op_scale = (knob != 0) ? 1 : 0;

	pl->group[gN].scale = scale;
	pl->group[gN].offset = offset;
}

void plotSliceSwitch(plot_t *pl)
{
	int		fN;

	if (pl->slice_mode_N == 0) {

		pl->slice_mode_N = 1;

		for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

			if (pl->figure[fN].slice_busy != 0) {

				pl->figure[fN].slice_base_X = pl->figure[fN].slice_X;
				pl->figure[fN].slice_base_Y = pl->figure[fN].slice_Y;
			}
		}
	}
	else if (pl->slice_mode_N == 1) {

		pl->slice_mode_N = 2;
	}
	else if (pl->slice_mode_N == 2) {

		pl->slice_mode_N = 0;
	}
}

void plotSliceTrack(plot_t *pl, int cur_X, int cur_Y)
{
	const fval_t	*row = NULL;
	double		fval_X, fval_Y;
	int		fN, aN, bN, dN, cX, cY, id_N;
	int		dN_s, aN_s, cX_s, job;

	if (pl->slice_mode_N == 2)
		return ;

	if (pl->slice_axis_N < 0) {

		pl->slice_axis_N = pl->on_X;
	}

	if (pl->slice_axis_N < 0) {

		ERROR("No valid axis number to slice\n");
		return ;
	}

	dN_s = -1;
	aN_s = -1;
	cX_s = -1;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		pl->figure[fN].slice_busy = 0;

		job = 0;

		if (pl->figure[fN].busy != 0 && pl->figure[fN].hidden == 0) {

			aN = pl->slice_axis_N;

			if (pl->axis[aN].busy == AXIS_BUSY_X) {

				if (pl->figure[fN].axis_X == aN) {

					job = 1;
				}
				else {
					bN = pl->figure[fN].axis_X;

					if (pl->axis[bN].slave != 0) {

						if (pl->axis[bN].slave_N == aN)
							job = 1;
					}
					else if (pl->axis[aN].slave != 0) {

						if (pl->axis[aN].slave_N == bN)
							job = 1;
					}
				}

				aN = pl->figure[fN].axis_X;
				cX = pl->figure[fN].column_X;

				fval_X = plotAxisConvBackward(pl, aN, cur_X);
			}
			else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

				if (pl->figure[fN].axis_Y == aN) {

					job = 1;
				}
				else {
					bN = pl->figure[fN].axis_Y;

					if (pl->axis[bN].slave != 0) {

						if (pl->axis[bN].slave_N == aN)
							job = 1;
					}
					else if (pl->axis[aN].slave != 0) {

						if (pl->axis[aN].slave_N == bN)
							job = 1;
					}
				}

				aN = pl->figure[fN].axis_Y;
				cX = pl->figure[fN].column_Y;

				fval_X = plotAxisConvBackward(pl, aN, cur_Y);
			}
		}

		if (job) {

			dN = pl->figure[fN].data_N;

			if (dN_s != dN || aN_s != aN || cX_s != cX) {

				row = plotDataSliceGet(pl, dN, cX,
						fval_X, &id_N);

				dN_s = dN;
				aN_s = aN;
				cX_s = cX;
			}

			if (row != NULL) {

				cX = pl->figure[fN].column_X;
				cY = pl->figure[fN].column_Y;

				fval_X = (cX < 0) ? id_N : row[cX];
				fval_Y = (cY < 0) ? id_N : row[cY];

				pl->figure[fN].slice_busy = 1;
				pl->figure[fN].slice_X = fval_X;
				pl->figure[fN].slice_Y = fval_Y;
			}
		}
	}

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		pl->data_box_text[fN][0] = 0;

		if (pl->figure[fN].slice_busy != 0) {

			if (pl->slice_mode_N != 0) {

				fval_X = pl->figure[fN].slice_base_X;
				fval_Y = pl->figure[fN].slice_base_Y;

				strcat(pl->data_box_text[fN], " \xCE\x94");
				plotDataBoxTextFmt(pl, fN, pl->figure[fN].slice_X - fval_X);

				strcat(pl->data_box_text[fN], "\xCE\x94");
				plotDataBoxTextFmt(pl, fN, pl->figure[fN].slice_Y - fval_Y);
			}
			else {
				plotDataBoxTextFmt(pl, fN, pl->figure[fN].slice_X);
				plotDataBoxTextFmt(pl, fN, pl->figure[fN].slice_Y);
			}
		}
	}

	if (pl->data_box_on != DATA_BOX_SLICE) {

		pl->data_box_on = DATA_BOX_SLICE;
		pl->data_box_X = pl->viewport.max_x;
		pl->data_box_Y = 0;
	}
}

static void
plotSliceLightDraw(plot_t *pl, SDL_Surface *surface)
{
	double		base_X, base_Y, data_X, data_Y, temp;
	int		fN, aN, bN;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].slice_busy != 0) {

			aN = pl->figure[fN].axis_X;
			bN = pl->figure[fN].axis_Y;

			base_X = plotAxisConvForward(pl, aN, pl->figure[fN].slice_base_X);
			base_Y = plotAxisConvForward(pl, bN, pl->figure[fN].slice_base_Y);

			data_X = plotAxisConvForward(pl, aN, pl->figure[fN].slice_X);
			data_Y = plotAxisConvForward(pl, bN, pl->figure[fN].slice_Y);

			if (data_X < base_X) {

				temp = base_X;
				base_X = data_X;
				data_X = temp;
			}

			if (data_Y < base_Y) {

				temp = base_Y;
				base_Y = data_Y;
				data_Y = temp;
			}

			SDL_LockSurface(surface);

			if (pl->axis[pl->slice_axis_N].busy == AXIS_BUSY_X) {

				if (fp_isfinite(base_X) && fp_isfinite(data_X)) {

					drawClipRect(surface, &pl->viewport,
							base_X, pl->viewport.min_y,
							data_X, pl->viewport.max_y,
							pl->sch->plot_hidden);
				}
			}
			else if (pl->axis[pl->slice_axis_N].busy == AXIS_BUSY_Y) {

				if (fp_isfinite(base_Y) && fp_isfinite(data_Y)) {

					drawClipRect(surface, &pl->viewport,
							pl->viewport.min_x, base_Y,
							pl->viewport.max_x, data_Y,
							pl->sch->plot_hidden);
				}
			}

			SDL_UnlockSurface(surface);
		}
	}
}

static void
plotSliceDraw(plot_t *pl, SDL_Surface *surface)
{
	double		base_X, base_Y, data_X, data_Y;
	int		fN, aN, bN;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].slice_busy != 0) {

			aN = pl->figure[fN].axis_X;
			bN = pl->figure[fN].axis_Y;

			if (pl->slice_mode_N != 0) {

				base_X = plotAxisConvForward(pl, aN, pl->figure[fN].slice_base_X);
				base_Y = plotAxisConvForward(pl, bN, pl->figure[fN].slice_base_Y);
			}

			data_X = plotAxisConvForward(pl, aN, pl->figure[fN].slice_X);
			data_Y = plotAxisConvForward(pl, bN, pl->figure[fN].slice_Y);

			SDL_LockSurface(surface);

			if (pl->axis[pl->slice_axis_N].busy == AXIS_BUSY_X) {

				if (pl->slice_mode_N != 0) {

					if (fp_isfinite(base_X)) {

						drawDashReset(pl->dw);
						drawDash(pl->dw, surface, &pl->viewport,
								base_X, pl->viewport.min_y,
								base_X, pl->viewport.max_y,
								pl->sch->plot_text,
								pl->layout_fence_dash,
								pl->layout_fence_space);
					}
				}

				if (fp_isfinite(data_X)) {

					drawDashReset(pl->dw);
					drawDash(pl->dw, surface, &pl->viewport,
							data_X, pl->viewport.min_y,
							data_X, pl->viewport.max_y,
							pl->sch->plot_text,
							pl->layout_fence_dash,
							pl->layout_fence_space);
				}
			}
			else if (pl->axis[pl->slice_axis_N].busy == AXIS_BUSY_Y) {

				if (pl->slice_mode_N != 0) {

					if (fp_isfinite(base_Y)) {

						drawDashReset(pl->dw);
						drawDash(pl->dw, surface, &pl->viewport,
								pl->viewport.min_x, base_Y,
								pl->viewport.max_x, base_Y,
								pl->sch->plot_text,
								pl->layout_fence_dash,
								pl->layout_fence_space);
					}
				}

				if (fp_isfinite(data_Y)) {

					drawDashReset(pl->dw);
					drawDash(pl->dw, surface, &pl->viewport,
							pl->viewport.min_x, data_Y,
							pl->viewport.max_x, data_Y,
							pl->sch->plot_text,
							pl->layout_fence_dash,
							pl->layout_fence_space);
				}
			}

			if (pl->slice_mode_N != 0) {

				if (fp_isfinite(base_X) && fp_isfinite(base_Y)) {

					drawDotCanvas(pl->dw, surface, &pl->viewport,
							base_X, base_Y,
							pl->layout_fence_point, 10, 0);
				}
			}

			if (fp_isfinite(data_X) && fp_isfinite(data_Y)) {

				drawDotCanvas(pl->dw, surface, &pl->viewport,
						data_X, data_Y,
						pl->layout_fence_point, 10, 0);
			}

			SDL_UnlockSurface(surface);
		}
	}
}

static void
plotSketchDataChunkSetUp(plot_t *pl, int fN)
{
	int		hN;

	hN = pl->draw[fN].list_self;

	if (hN >= 0	&& pl->sketch[hN].figure_N == fN
			&& pl->sketch[hN].drawing == pl->figure[fN].drawing
			&& pl->sketch[hN].width == pl->figure[fN].width
			&& pl->sketch[hN].length < PLOT_SKETCH_CHUNK_SIZE) {

		/* Keep using this chunk */
	}
	else if (pl->sketch_list_garbage >= 0) {

		hN = pl->sketch_list_garbage;
		pl->sketch_list_garbage = pl->sketch[hN].linked;

		pl->sketch[hN].figure_N = fN;
		pl->sketch[hN].drawing = pl->figure[fN].drawing;
		pl->sketch[hN].width = pl->figure[fN].width;

		if (pl->sketch[hN].chunk == NULL) {

			pl->sketch[hN].chunk = (double *) malloc(sizeof(double) * PLOT_SKETCH_CHUNK_SIZE);

			if (pl->sketch[hN].chunk == NULL) {

				ERROR("Unable to allocate memory of %i sketch chunk\n", hN);
			}
		}

		pl->sketch[hN].length = 0;

		if (pl->draw[fN].list_self >= 0) {

			pl->sketch[hN].linked = pl->sketch[pl->draw[fN].list_self].linked;
			pl->sketch[pl->draw[fN].list_self].linked = hN;

			if (pl->draw[fN].list_self == pl->sketch_list_current_end)
				pl->sketch_list_current_end = hN;
		}
		else {
			pl->sketch[hN].linked = -1;

			if (pl->sketch_list_current >= 0) {

				pl->sketch[pl->sketch_list_current_end].linked = hN;
				pl->sketch_list_current_end = hN;
			}
			else {
				pl->sketch_list_current = hN;
				pl->sketch_list_current_end = hN;
			}
		}

		pl->draw[fN].list_self = hN;
	}
	else {
		ERROR("Unable to get free sketch chunk\n");

		pl->draw[fN].list_self = -1;
	}
}

static void
plotSketchDataAdd(plot_t *pl, int fN, double X, double Y)
{
	int		hN, length;

	hN = pl->draw[fN].list_self;

	if (hN >= 0) {

		length = pl->sketch[hN].length;

		pl->sketch[hN].chunk[length++] = X;
		pl->sketch[hN].chunk[length++] = Y;

		pl->sketch[hN].length = length;

		if (length >= PLOT_SKETCH_CHUNK_SIZE) {

			plotSketchDataChunkSetUp(pl, fN);
		}
	}
}

static void
plotSketchGarbage(plot_t *pl)
{
	int		N, hN, linked;

	hN = pl->sketch_list_todraw;

	while (hN >= 0) {

		linked = pl->sketch[hN].linked;

		pl->sketch[hN].linked = pl->sketch_list_garbage;
		pl->sketch_list_garbage = hN;

		hN = linked;
	}

	pl->sketch_list_todraw = pl->sketch_list_current;
	pl->sketch_list_current = -1;
	pl->sketch_list_current_end = -1;

	for (N = 0; N < PLOT_FIGURE_MAX; ++N)
		pl->draw[N].list_self = -1;
}

void plotSketchClean(plot_t *pl)
{
	int		N, hN, linked;

	hN = pl->sketch_list_todraw;

	while (hN >= 0) {

		linked = pl->sketch[hN].linked;

		pl->sketch[hN].linked = pl->sketch_list_garbage;
		pl->sketch_list_garbage = hN;

		hN = linked;
	}

	hN = pl->sketch_list_current;

	while (hN >= 0) {

		linked = pl->sketch[hN].linked;

		pl->sketch[hN].linked = pl->sketch_list_garbage;
		pl->sketch_list_garbage = hN;

		hN = linked;
	}

	pl->sketch_list_todraw = -1;
	pl->sketch_list_current = -1;
	pl->sketch_list_current_end = -1;

	for (N = 0; N < PLOT_FIGURE_MAX; ++N)
		pl->draw[N].list_self = -1;

	pl->draw_in_progress = 0;
}

static void
plotDrawPalette(plot_t *pl)
{
	draw_t			*dw = pl->dw;
	scheme_t		*sch = pl->sch;
	Uint32			*palette;

	palette = dw->palette;

	palette[0] = drawRGBMap(dw, sch->plot_background);
	palette[1] = drawRGBMap(dw, sch->plot_figure[0]);
	palette[2] = drawRGBMap(dw, sch->plot_figure[1]);
	palette[3] = drawRGBMap(dw, sch->plot_figure[2]);
	palette[4] = drawRGBMap(dw, sch->plot_figure[3]);
	palette[5] = drawRGBMap(dw, sch->plot_figure[4]);
	palette[6] = drawRGBMap(dw, sch->plot_figure[5]);
	palette[7] = drawRGBMap(dw, sch->plot_figure[6]);
	palette[8] = drawRGBMap(dw, sch->plot_figure[7]);
	palette[9] = drawRGBMap(dw, sch->plot_hidden);
	palette[10] = drawRGBMap(dw, sch->plot_text);
}

static int
plotGetTickCached(plot_t *pl)
{
	if (pl->tick_skip > 0) {

		pl->tick_skip--;
	}
	else {
		pl->tick_cached = SDL_GetTicks();
		pl->tick_skip = 63;
	}

	return pl->tick_cached;
}

static void
plotDrawFigureTrial(plot_t *pl, int fN, int tTOP)
{
	const fval_t	*row;
	double		scale_X, scale_Y, offset_X, offset_Y, im_MIN, im_MAX;
	double		X, Y, last_X, last_Y, im_X, im_Y, last_im_X, last_im_Y;
	int		dN, rN, xN, yN, xNR, yNR, aN, bN, id_N, id_N_top, kN, kN_cached;
	int		job, skipped, line, rc, ncolor, fdrawing, fwidth;

	ncolor = (pl->figure[fN].hidden != 0) ? 9 : fN + 1;

	fdrawing = pl->figure[fN].drawing;
	fwidth = pl->figure[fN].width;

	dN = pl->figure[fN].data_N;
	xN = pl->figure[fN].column_X;
	yN = pl->figure[fN].column_Y;

	xNR = plotDataRangeCacheFetch(pl, dN, xN);
	yNR = plotDataRangeCacheFetch(pl, dN, yN);

	aN = pl->figure[fN].axis_X;
	scale_X = pl->axis[aN].scale;
	offset_X = pl->axis[aN].offset;

	if (pl->axis[aN].slave != 0) {

		bN = pl->axis[aN].slave_N;
		scale_X *= pl->axis[bN].scale;
		offset_X = offset_X * pl->axis[bN].scale + pl->axis[bN].offset;
	}

	aN = pl->figure[fN].axis_Y;
	scale_Y = pl->axis[aN].scale;
	offset_Y = pl->axis[aN].offset;

	if (pl->axis[aN].slave != 0) {

		bN = pl->axis[aN].slave_N;
		scale_Y *= pl->axis[bN].scale;
		offset_Y = offset_Y * pl->axis[bN].scale + pl->axis[bN].offset;
	}

	X = (double) (pl->viewport.max_x - pl->viewport.min_x);
	Y = (double) (pl->viewport.min_y - pl->viewport.max_y);

	scale_X *= X;
	offset_X = offset_X * X + pl->viewport.min_x;
	scale_Y *= Y;
	offset_Y = offset_Y * Y + pl->viewport.max_y;

	rN = pl->draw[fN].rN;
	id_N = pl->draw[fN].id_N;

	id_N_top = id_N + (1UL << pl->data[dN].chunk_SHIFT);
	kN_cached = -1;

	plotSketchDataChunkSetUp(pl, fN);

	if (		fdrawing == FIGURE_DRAWING_LINE
			|| fdrawing == FIGURE_DRAWING_DASH) {

		skipped = pl->draw[fN].skipped;
		line = pl->draw[fN].line;

		last_X = pl->draw[fN].last_X;
		last_Y = pl->draw[fN].last_Y;

		last_im_X = last_X * scale_X + offset_X;
		last_im_Y = last_Y * scale_Y + offset_Y;

		do {
			kN = plotDataChunkN(pl, dN, rN);
			job = 1;

			if (kN != kN_cached) {

				if (xNR >= 0 && pl->rcache[xNR].chunk[kN].computed != 0) {

					if (pl->rcache[xNR].chunk[kN].finite != 0) {

						im_MIN = pl->rcache[xNR].chunk[kN].fmin * scale_X + offset_X;
						im_MAX = pl->rcache[xNR].chunk[kN].fmax * scale_X + offset_X;

						job = (	   im_MAX < pl->viewport.min_x - 16
							|| im_MIN > pl->viewport.max_x + 16) ? 0 : job;
					}
					else {
						job = 0;
					}
				}

				if (yNR >= 0 && pl->rcache[yNR].chunk[kN].computed != 0) {

					if (pl->rcache[yNR].chunk[kN].finite != 0) {

						im_MIN = pl->rcache[yNR].chunk[kN].fmin * scale_Y + offset_Y;
						im_MAX = pl->rcache[yNR].chunk[kN].fmax * scale_Y + offset_Y;

						job = (	   im_MIN < pl->viewport.min_y - 16
							|| im_MAX > pl->viewport.max_y + 16) ? 0 : job;
					}
					else {
						job = 0;
					}
				}

				kN_cached = kN;
			}

			if (job != 0 || line != 0) {

				if (skipped != 0) {

					plotDataSkip(pl, dN, &rN, &id_N, -1);

					skipped = 0;
				}

				row = plotDataGet(pl, dN, &rN);

				if (row == NULL) {

					pl->draw[fN].sketch = SKETCH_FINISHED;
					break;
				}

				X = (xN < 0) ? id_N : row[xN];
				Y = (yN < 0) ? id_N : row[yN];

				im_X = X * scale_X + offset_X;
				im_Y = Y * scale_Y + offset_Y;

				if (fp_isfinite(im_X) && fp_isfinite(im_Y)) {

					if (line != 0) {

						rc = drawLineTrial(pl->dw, &pl->viewport,
								last_im_X, last_im_Y, im_X, im_Y,
								ncolor, fwidth);

						if (rc != 0) {

							plotSketchDataAdd(pl, fN, last_X, last_Y);
							plotSketchDataAdd(pl, fN, X, Y);
						}
					}
					else {
						line = 1;
					}

					last_X = X;
					last_Y = Y;

					last_im_X = im_X;
					last_im_Y = im_Y;
				}
				else {
					line = 0;
				}

				id_N++;
			}

			if (job == 0) {

				plotDataChunkSkip(pl, dN, &rN, &id_N);

				skipped = 1;
				line = 0;
			}

			if (id_N > id_N_top || plotGetTickCached(pl) > tTOP) {

				pl->draw[fN].sketch = SKETCH_INTERRUPTED;
				pl->draw[fN].rN = rN;
				pl->draw[fN].id_N = id_N;
				pl->draw[fN].skipped = skipped;
				pl->draw[fN].line = line;
				pl->draw[fN].last_X = last_X;
				pl->draw[fN].last_Y = last_Y;
				break;
			}
		}
		while (1);
	}
	else if (fdrawing == FIGURE_DRAWING_DOT) {

		do {
			kN = plotDataChunkN(pl, dN, rN);
			job = 1;

			if (kN != kN_cached) {

				if (xNR >= 0 && pl->rcache[xNR].chunk[kN].computed != 0) {

					if (pl->rcache[xNR].chunk[kN].finite != 0) {

						im_MIN = pl->rcache[xNR].chunk[kN].fmin * scale_X + offset_X;
						im_MAX = pl->rcache[xNR].chunk[kN].fmax * scale_X + offset_X;

						job = (	   im_MAX < pl->viewport.min_x - 16
							|| im_MIN > pl->viewport.max_x + 16) ? 0 : job;
					}
					else {
						job = 0;
					}
				}

				if (yNR >= 0 && pl->rcache[yNR].chunk[kN].computed != 0) {

					if (pl->rcache[yNR].chunk[kN].finite != 0) {

						im_MIN = pl->rcache[yNR].chunk[kN].fmin * scale_Y + offset_Y;
						im_MAX = pl->rcache[yNR].chunk[kN].fmax * scale_Y + offset_Y;

						job = (	   im_MIN < pl->viewport.min_y - 16
							|| im_MAX > pl->viewport.max_y + 16) ? 0 : job;
					}
					else {
						job = 0;
					}
				}

				kN_cached = kN;
			}

			if (job != 0) {

				row = plotDataGet(pl, dN, &rN);

				if (row == NULL) {

					pl->draw[fN].sketch = SKETCH_FINISHED;
					break;
				}

				X = (xN < 0) ? id_N : row[xN];
				Y = (yN < 0) ? id_N : row[yN];

				im_X = X * scale_X + offset_X;
				im_Y = Y * scale_Y + offset_Y;

				if (fp_isfinite(im_X) && fp_isfinite(im_Y)) {

					rc = drawDotTrial(pl->dw, &pl->viewport,
							im_X, im_Y, fwidth,
							ncolor, 1);

					if (rc != 0) {

						plotSketchDataAdd(pl, fN, X, Y);
					}
				}

				id_N++;
			}

			if (job == 0) {

				plotDataChunkSkip(pl, dN, &rN, &id_N);
			}

			if (id_N > id_N_top || plotGetTickCached(pl) > tTOP) {

				pl->draw[fN].sketch = SKETCH_INTERRUPTED;
				pl->draw[fN].rN = rN;
				pl->draw[fN].id_N = id_N;
				break;
			}
		}
		while (1);
	}
}

int plotGetSketchLength(plot_t *pl)
{
	int		hN, length = 0;

	hN = pl->sketch_list_todraw;

	while (hN >= 0) {

		length += pl->sketch[hN].length;
		hN = pl->sketch[hN].linked;
	}

	return length;
}

static void
plotDrawSketch(plot_t *pl, SDL_Surface *surface)
{
	double		scale_X, offset_X, scale_Y, offset_Y;
	double		X, Y, last_X, last_Y, *chunk, *lend;
	int		hN, fN, aN, bN;

	int		fdrawing, fwidth, ncolor;

	hN = pl->sketch_list_todraw;

	drawDashReset(pl->dw);

	SDL_LockSurface(surface);

	while (hN >= 0) {

		fN = pl->sketch[hN].figure_N;

		ncolor = (pl->figure[fN].hidden != 0) ? 9 : fN + 1;

		fdrawing = pl->sketch[hN].drawing;
		fwidth = pl->sketch[hN].width;

		aN = pl->figure[fN].axis_X;
		scale_X = pl->axis[aN].scale;
		offset_X = pl->axis[aN].offset;

		if (pl->axis[aN].slave != 0) {

			bN = pl->axis[aN].slave_N;
			scale_X *= pl->axis[bN].scale;
			offset_X = offset_X * pl->axis[bN].scale + pl->axis[bN].offset;
		}

		aN = pl->figure[fN].axis_Y;
		scale_Y = pl->axis[aN].scale;
		offset_Y = pl->axis[aN].offset;

		if (pl->axis[aN].slave != 0) {

			bN = pl->axis[aN].slave_N;
			scale_Y *= pl->axis[bN].scale;
			offset_Y = offset_Y * pl->axis[bN].scale + pl->axis[bN].offset;
		}

		X = (double) (pl->viewport.max_x - pl->viewport.min_x);
		Y = (double) (pl->viewport.min_y - pl->viewport.max_y);

		scale_X *= X;
		offset_X = offset_X * X + pl->viewport.min_x;
		scale_Y *= Y;
		offset_Y = offset_Y * Y + pl->viewport.max_y;

		chunk = pl->sketch[hN].chunk;
		lend = chunk + pl->sketch[hN].length;

		if (fdrawing == FIGURE_DRAWING_LINE) {

			while (chunk < lend) {

				X = *chunk++;
				Y = *chunk++;

				last_X = X * scale_X + offset_X;
				last_Y = Y * scale_Y + offset_Y;

				X = *chunk++;
				Y = *chunk++;

				X = X * scale_X + offset_X;
				Y = Y * scale_Y + offset_Y;

				drawLineCanvas(pl->dw, surface, &pl->viewport,
						last_X, last_Y, X, Y,
						ncolor, fwidth);
			}
		}
		else if (fdrawing == FIGURE_DRAWING_DASH) {

			while (chunk < lend) {

				X = *chunk++;
				Y = *chunk++;

				last_X = X * scale_X + offset_X;
				last_Y = Y * scale_Y + offset_Y;

				X = *chunk++;
				Y = *chunk++;

				X = X * scale_X + offset_X;
				Y = Y * scale_Y + offset_Y;

				drawDashCanvas(pl->dw, surface, &pl->viewport,
						last_X, last_Y, X, Y,
						ncolor, fwidth, pl->layout_drawing_dash,
						pl->layout_drawing_space);
			}
		}
		else if (fdrawing == FIGURE_DRAWING_DOT) {

			while (chunk < lend) {

				X = *chunk++;
				Y = *chunk++;

				X = X * scale_X + offset_X;
				Y = Y * scale_Y + offset_Y;

				drawDotCanvas(pl->dw, surface, &pl->viewport,
						X, Y, fwidth,
						ncolor, 1);
			}
		}

		hN = pl->sketch[hN].linked;
	}

	SDL_UnlockSurface(surface);
}

static void
plotDrawAxis(plot_t *pl, SDL_Surface *surface, int aN)
{
	char		numfmt[PLOT_STRING_MAX];
	char		numbuf[PLOT_STRING_MAX];

	double		scale, offset, fmin, fmax, fpow, tih, tis, tik, la;
	int		fN, bN, texp, lpos, tpos, tdec, hovered;

	Uint32		axCol = pl->sch->plot_hidden;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].busy != 0 && pl->figure[fN].hidden == 0) {

			if (	pl->figure[fN].axis_X == aN
				|| pl->figure[fN].axis_Y == aN) {

				if (axCol != pl->sch->plot_hidden) {

					axCol = pl->sch->plot_text;
				}
				else {
					axCol = pl->sch->plot_figure[fN];
				}
			}
		}
	}

	scale = pl->axis[aN].scale;
	offset = pl->axis[aN].offset;

	if (pl->axis[aN].slave != 0) {

		bN = pl->axis[aN].slave_N;
		scale *= pl->axis[bN].scale;
		offset = offset * pl->axis[bN].scale + pl->axis[bN].offset;
	}

	fmin = - offset / scale;
	fmax = 1. / scale + fmin;

	if (pl->axis[aN].lock_tick != 0) {

		if (		   pl->axis[aN].ruler_min > fmin
				|| pl->axis[aN].ruler_max < fmax) {

			fmin = pl->axis[aN].ruler_min;
			fmax = pl->axis[aN].ruler_max;
		}
	}

	if (pl->axis[aN].busy == AXIS_BUSY_X) {

		la = (double) (pl->viewport.max_x - pl->viewport.min_x);
	}
	else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

		la = (double) (pl->viewport.max_y - pl->viewport.min_y);
	}

	if (fmin < fmax) {

		texp = (int) ceil(log10((fmax - fmin) / 10.));
		tih = pow(10., (double) texp);

		if ((fmax - fmin) / tih < 2.) {

			tdec = (int) (tih * scale * la / 5.);

			if (tdec > pl->layout_font_height) {

				tih /= 5.;
				texp--;
			}
		}

		if ((fmax - fmin) / tih < 4.) {

			tdec = (int) (tih * scale * la / 2.);

			if (tdec > pl->layout_font_height) {

				tih /= 2.;
				texp--;
			}
		}

		tdec = (int) (tih * scale * la);

		if (tdec < pl->layout_font_height) {

			tih *= 2.;
		}

		tis = floor(fmin / tih) * tih;
		tis += (tis < fmin) ? tih : 0.;
		tih = (tis + tih == tis) ? fmax - tis : tih;
	}
	else {
		texp = 0;
		tis = fmax;
		tih = fmax;
	}

	pl->axis[aN].ruler_tih = tih * scale;
	pl->axis[aN].ruler_tis = tis * scale + offset;

	fpow = 1.;

	if (pl->axis[aN].busy == AXIS_BUSY_X) {

		la = (double) (pl->viewport.max_x - pl->viewport.min_x);
		scale *= la;
		offset = offset * la + pl->viewport.min_x;
	}
	else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

		la = (double) (pl->viewport.min_y - pl->viewport.max_y);
		scale *= la;
		offset = offset * la + pl->viewport.max_y;
	}

	SDL_LockSurface(surface);

	if (pl->axis[aN].busy == AXIS_BUSY_X) {

		lpos = pl->viewport.max_y + pl->layout_border + pl->axis[aN].layout_pos;
		hovered = (pl->hover_axis == aN) ? 1 : 0;

		if (pl->hover_figure != -1 && pl->shift_on != 0) {

			fN = pl->hover_figure;

			hovered = (pl->figure[fN].axis_X == aN) ? 1 : hovered;
			hovered = (pl->figure[fN].axis_Y == aN) ? 1 : hovered;
		}

		if (hovered != 0) {

			tdec = pl->layout_axis_box;
			tdec += (pl->axis[aN].compact == 0) ? pl->layout_label_box : 0;

			drawFillRect(surface, pl->viewport.min_x, lpos, pl->viewport.max_x,
					lpos + tdec, pl->sch->plot_hovered);
		}

		drawLine(pl->dw, surface, &pl->screen, pl->viewport.min_x, lpos,
				pl->viewport.max_x, lpos, pl->sch->plot_axis);

		for (tik = tis; tik < fmax; tik += tih) {

			tpos = (int) (tik * scale + offset);

			if (tpos < pl->viewport.min_x || tpos > pl->viewport.max_x)
				continue ;

			drawLine(pl->dw, surface, &pl->screen, tpos, lpos, tpos,
					lpos + pl->layout_tick_tooth, pl->sch->plot_axis);

			if (		pl->axis[aN].lock_tick != 0
					|| pl->on_X == aN) {

				drawDashReset(pl->dw);
				drawDash(pl->dw, surface, &pl->screen, tpos,
						pl->viewport.min_y, tpos,
						pl->viewport.max_y, pl->sch->plot_axis,
						pl->layout_grid_dash, pl->layout_grid_space);
			}
		}

		if (pl->on_X == aN) {

			drawLine(pl->dw, surface, &pl->screen, pl->viewport.min_x,
					lpos + 1, pl->viewport.max_x, lpos + 1,
					pl->sch->plot_axis);
		}

		if (pl->axis[aN].slave != 0) {

			drawLine(pl->dw, surface, &pl->screen, pl->viewport.min_x,
					lpos + pl->layout_tick_tooth, pl->viewport.max_x,
					lpos + pl->layout_tick_tooth, pl->sch->plot_axis);
		}
	}
	else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

		lpos = pl->viewport.min_x - pl->layout_border - pl->axis[aN].layout_pos;
		hovered = (pl->hover_axis == aN) ? 1 : 0;

		if (pl->hover_figure != -1 && pl->shift_on != 0) {

			fN = pl->hover_figure;

			hovered = (pl->figure[fN].axis_X == aN) ? 1 : hovered;
			hovered = (pl->figure[fN].axis_Y == aN) ? 1 : hovered;
		}

		if (hovered != 0) {

			tdec = pl->layout_axis_box;
			tdec += (pl->axis[aN].compact == 0) ? pl->layout_label_box : 0;

			drawFillRect(surface, lpos - tdec, pl->viewport.min_y, lpos,
					pl->viewport.max_y, pl->sch->plot_hovered);
		}

		drawLine(pl->dw, surface, &pl->screen, lpos, pl->viewport.min_y,
				lpos, pl->viewport.max_y, pl->sch->plot_axis);

		for (tik = tis; tik < fmax; tik += tih) {

			tpos = (int) (tik * scale + offset);

			if (tpos < pl->viewport.min_y || tpos > pl->viewport.max_y)
				continue ;

			drawLine(pl->dw, surface, &pl->screen, lpos, tpos,
					lpos - pl->layout_tick_tooth, tpos,
					pl->sch->plot_axis);

			if (		pl->axis[aN].lock_tick != 0
					|| pl->on_Y == aN) {

				drawDashReset(pl->dw);
				drawDash(pl->dw, surface, &pl->screen,
						pl->viewport.min_x, tpos,
						pl->viewport.max_x, tpos,
						pl->sch->plot_axis, pl->layout_grid_dash,
						pl->layout_grid_space);
			}
		}

		if (pl->on_Y == aN) {

			drawLine(pl->dw, surface, &pl->screen, lpos - 1,
					pl->viewport.min_y, lpos - 1,
					pl->viewport.max_y, pl->sch->plot_axis);
		}

		if (pl->axis[aN].slave != 0) {

			drawLine(pl->dw, surface, &pl->screen, lpos - pl->layout_tick_tooth,
					pl->viewport.min_y, lpos - pl->layout_tick_tooth,
					pl->viewport.max_y, pl->sch->plot_axis);
		}
	}

	SDL_UnlockSurface(surface);

	if (pl->axis[aN].busy == AXIS_BUSY_X) {

		int		tmove, taway, tleft, tright, txlen;

		lpos = pl->viewport.max_y + pl->layout_border + pl->axis[aN].layout_pos;
		tmove = pl->screen.min_x;
		taway = pl->viewport.max_x;

		if (		pl->axis[aN].exponential != 0
				|| abs(texp) > 16) {

			tdec = 3 * (- texp / 3);

			if (tdec != 0) {

				texp += tdec;
				fpow *= pow(10., (double) tdec);

				sprintf(numbuf, "E%+i", - tdec);

				tpos = (pl->axis[aN].compact == 0) ?
					lpos + pl->layout_axis_box :
					lpos + pl->layout_tick_tooth;

				tpos += pl->layout_font_height / 2;

				TTF_SizeUTF8(pl->font, numbuf, &txlen, &tdec);

				drawText(pl->dw, surface, pl->font, taway - txlen, tpos,
						numbuf, TEXT_CENTERED_ON_Y, axCol);

				if (pl->axis[aN].compact != 0)
					taway += - (txlen + pl->layout_font_space);

				if (pl->axis[aN].exponential == 0)
					pl->axis[aN].exponential = 1;
			}
		}

		if (		pl->axis[aN].label[0] != 0
				&& pl->axis[aN].compact != 0) {

			TTF_SizeUTF8(pl->font, pl->axis[aN].label, &txlen, &tdec);

			taway += - (txlen + pl->layout_font_space);
		}

		if (abs(texp) < 170) {

			sprintf(numfmt, "%%.%df", (texp < 0) ? - texp : 0);
		}
		else {
			numfmt[0] = 0;
		}

		for (tik = tis; tik < fmax; tik += tih) {

			tpos = (int) (tik * scale + offset);

			if (tpos < pl->viewport.min_x || tpos > pl->viewport.max_x)
				continue ;

			sprintf(numbuf, numfmt, tik * fpow);

			TTF_SizeUTF8(pl->font, numbuf, &txlen, &tdec);

			tleft = tpos - txlen / 2 - pl->layout_font_long;
			tright = tpos + (txlen - txlen / 2);

			if (tmove < tleft && tright < taway) {

				drawText(pl->dw, surface, pl->font, tpos,
						lpos + pl->layout_tick_tooth
						+ pl->layout_font_height / 2, numbuf,
						TEXT_CENTERED, axCol);

				tmove = tright;
			}
		}

		if (pl->axis[aN].compact != 0) {

			tpos = taway + pl->layout_font_height / 2;
			lpos = lpos + pl->layout_tick_tooth + pl->layout_font_height / 2;

			if (pl->axis[aN].label[0] != 0) {

				drawText(pl->dw, surface, pl->font, tpos, lpos,
						pl->axis[aN].label,
						TEXT_CENTERED_ON_Y, axCol);
			}
		}
		else {
			tpos = (pl->viewport.min_x + pl->viewport.max_x) / 2;
			lpos = lpos + pl->layout_axis_box + pl->layout_font_height / 2;

			if (pl->axis[aN].label[0] != 0) {

				drawText(pl->dw, surface, pl->font, tpos, lpos,
						pl->axis[aN].label,
						TEXT_CENTERED, axCol);
			}
		}
	}
	else if (pl->axis[aN].busy == AXIS_BUSY_Y) {

		int		tmove, taway, tawayb, tleft, tright, txlen;

		lpos = pl->viewport.min_x - pl->layout_border - pl->axis[aN].layout_pos;
		tmove = pl->screen.max_y;
		taway = pl->viewport.min_y;

		if (		pl->axis[aN].exponential != 0
				|| abs(texp) > 16) {

			tdec = 3 * (- texp / 3);

			if (tdec != 0) {

				texp += tdec;
				fpow *= pow(10., (double) tdec);

				sprintf(numbuf, "E%+i", - tdec);

				tpos = (pl->axis[aN].compact == 0) ?
					lpos - pl->layout_axis_box :
					lpos - pl->layout_tick_tooth;

				tpos -= pl->layout_font_height / 2;

				TTF_SizeUTF8(pl->font, numbuf, &txlen, &tdec);

				drawText(pl->dw, surface, pl->font, tpos, taway, numbuf,
						TEXT_CENTERED_ON_X | TEXT_VERTICAL, axCol);

				if (pl->axis[aN].compact != 0)
					taway += txlen + pl->layout_font_space;

				if (pl->axis[aN].exponential == 0)
					pl->axis[aN].exponential = 1;
			}
		}

		tawayb = taway;

		if (		pl->axis[aN].label[0] != 0
				&& pl->axis[aN].compact != 0) {

			TTF_SizeUTF8(pl->font, pl->axis[aN].label, &txlen, &tdec);

			taway += txlen + pl->layout_font_space;
		}

		if (abs(texp) < 170) {

			sprintf(numfmt, "%%.%df", (texp < 0) ? - texp : 0);
		}
		else {
			numfmt[0] = 0;
		}

		for (tik = tis; tik < fmax; tik += tih) {

			tpos = (int) (tik * scale + offset);

			if (tpos < pl->viewport.min_y || tpos > pl->viewport.max_y)
				continue ;

			sprintf(numbuf, numfmt, tik * fpow);

			TTF_SizeUTF8(pl->font, numbuf, &txlen, &tdec);

			tleft = tpos + txlen / 2 + pl->layout_font_long;
			tright = tpos - (txlen - txlen / 2);

			if (tmove > tleft && tright > taway) {

				drawText(pl->dw, surface, pl->font,
						lpos - pl->layout_tick_tooth
						- pl->layout_font_height / 2, tpos, numbuf,
						TEXT_CENTERED | TEXT_VERTICAL, axCol);

				tmove = tright;
			}
		}

		if (pl->axis[aN].compact != 0) {

			lpos = lpos - pl->layout_tick_tooth - pl->layout_font_height / 2;
			tpos = tawayb;

			if (pl->axis[aN].label[0] != 0) {

				drawText(pl->dw, surface, pl->font, lpos, tpos,
						pl->axis[aN].label,
						TEXT_CENTERED_ON_X | TEXT_VERTICAL, axCol);
			}
		}
		else {
			lpos = lpos - pl->layout_axis_box - pl->layout_font_height / 2;
			tpos = (pl->viewport.min_y + pl->viewport.max_y) / 2;

			if (pl->axis[aN].label[0] != 0) {

				drawText(pl->dw, surface, pl->font, lpos, tpos,
						pl->axis[aN].label,
						TEXT_CENTERED | TEXT_VERTICAL, axCol);
			}
		}
	}
}

static void
plotLegendLayout(plot_t *pl)
{
	int		fN, size_X, size_Y;
	int		size_N = 0, size_MAX = 0;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].busy != 0) {

			TTF_SizeUTF8(pl->font, pl->figure[fN].label, &size_X, &size_Y);
			size_MAX = (size_MAX < size_X) ? size_X : size_MAX;

			size_N++;
		}
	}

	pl->legend_size_X = size_MAX + pl->layout_font_long * 2;
	pl->legend_N = size_N;

	if (pl->legend_X > pl->viewport.max_x - (size_MAX + pl->layout_font_height * 3))
		pl->legend_X = pl->viewport.max_x - (size_MAX + pl->layout_font_height * 3);

	if (pl->legend_Y > pl->viewport.max_y - pl->layout_font_height * (size_N + 1))
		pl->legend_Y = pl->viewport.max_y - pl->layout_font_height * (size_N + 1);

	if (pl->legend_X < pl->viewport.min_x + pl->layout_font_height)
		pl->legend_X = pl->viewport.min_x + pl->layout_font_height;

	if (pl->legend_Y < pl->viewport.min_y + pl->layout_font_height)
		pl->legend_Y = pl->viewport.min_y + pl->layout_font_height;
}

static void
plotLegendDraw(plot_t *pl, SDL_Surface *surface)
{
	int		boxX, boxY, size_X, size_Y;
	int		fN, legX, legY, ncolor, fwidth, hovered;

	legX = pl->legend_X;
	legY = pl->legend_Y;

	size_X = pl->layout_font_height * 2 + pl->legend_size_X;
	size_Y = pl->layout_font_height * pl->legend_N;

	SDL_LockSurface(surface);

	if (pl->hover_legend != -1) {

		drawFillRect(surface, legX, legY, legX + size_X,
				legY + size_Y, pl->sch->plot_hovered);
	}
	else if (	pl->transparency == 0
			&& pl->legend_hidden == 0) {

		drawFillRect(surface, legX, legY, legX + size_X,
				legY + size_Y, pl->sch->plot_background);
	}

	SDL_UnlockSurface(surface);

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].busy != 0) {

			SDL_LockSurface(surface);

			ncolor = (pl->figure[fN].hidden != 0) ? 9 : fN + 1;
			hovered = (pl->hover_figure == fN) ? 1 : 0;

			if (pl->shift_on != 0) {

				hovered = (pl->figure[fN].axis_X == pl->hover_axis) ? 1 : hovered;
				hovered = (pl->figure[fN].axis_Y == pl->hover_axis) ? 1 : hovered;
			}

			if (hovered != 0) {

				boxX = legX + pl->layout_font_height * 2;
				size_X = pl->legend_size_X;
				size_Y = pl->layout_font_height;

				drawFillRect(surface, boxX, legY, boxX + size_X,
						legY + size_Y, pl->sch->plot_hovered);
			}

			if (pl->legend_hidden == 0) {

				double		padY;

				fwidth = pl->figure[fN].width;
				boxY = legY + pl->layout_font_height / 2;

				padY = (fwidth < 1 || (fwidth % 2) != 0) ? 0.5 : 0.;

				if (pl->figure[fN].drawing == FIGURE_DRAWING_LINE) {

					boxX = legX + pl->layout_font_height / 2;

					drawLineCanvas(pl->dw, surface, &pl->viewport, boxX,
							boxY + padY, boxX + pl->layout_font_height,
							boxY + padY, ncolor, fwidth);
				}
				else if (pl->figure[fN].drawing == FIGURE_DRAWING_DASH) {

					boxX = legX + pl->layout_font_height / 2;

					drawDashReset(pl->dw);

					drawDashCanvas(pl->dw, surface, &pl->viewport, boxX,
							boxY + padY, boxX + pl->layout_font_height,
							boxY + padY, ncolor, fwidth,
							pl->layout_drawing_dash, pl->layout_drawing_space);
				}
				else if (pl->figure[fN].drawing == FIGURE_DRAWING_DOT) {

					boxX = legX + pl->layout_font_height;

					drawDotCanvas(pl->dw, surface, &pl->viewport,
							boxX + padY, boxY + padY,
							(fwidth > 4) ? fwidth : 4, ncolor, 1);
				}

				if (pl->mark_on != 0) {

					boxX = legX + pl->layout_font_height;

					drawMarkCanvas(pl->dw, surface, &pl->viewport, boxX,
							boxY + padY, pl->layout_mark_size,
							fN, ncolor, (fwidth < 1) ? 1 : fwidth);
				}
			}

			SDL_UnlockSurface(surface);

			if (pl->legend_hidden == 0) {

				drawText(pl->dw, surface, pl->font, legX + pl->layout_font_height * 2
						+ pl->layout_font_long, boxY, pl->figure[fN].label,
						TEXT_CENTERED_ON_Y, (pl->figure[fN].hidden != 0)
						? pl->sch->plot_hidden : pl->sch->plot_text);
			}

			legY += pl->layout_font_height;
		}
	}
}

int plotLegendGetByClick(plot_t *pl, int cur_X, int cur_Y)
{
	int		legX, legY, relX, relY;
	int		fN, rN = -1;

	legX = pl->legend_X;
	legY = pl->legend_Y;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].busy != 0) {

			relX = cur_X - (legX + pl->layout_font_height * 2);
			relY = cur_Y - legY;

			if (		relX > 0 && relX < pl->legend_size_X
					&& relY > 0 && relY < pl->layout_font_height) {

				rN = fN;
				break;
			}

			legY += pl->layout_font_height;
		}
	}

	pl->hover_figure = rN;

	return rN;
}

int plotLegendBoxGetByClick(plot_t *pl, int cur_X, int cur_Y)
{
	int		relX, relY, lenY, rN = -1;

	relX = cur_X - pl->legend_X;
	relY = cur_Y - pl->legend_Y;

	lenY = pl->layout_font_height * pl->legend_N;

	if (		relX > 0 && relX < pl->layout_font_height * 2
			&& relY > 0 && relY < lenY) {

		rN = 0;
	}

	pl->hover_legend = rN;

	return rN;
}

static void
plotDataBoxLayout(plot_t *pl)
{
	int		N, size_X, size_Y;
	int		size_N = 0, size_MAX = 0;

	if (pl->data_box_on == DATA_BOX_SLICE) {

		for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

			if (pl->figure[N].busy != 0) {

				TTF_SizeUTF8(pl->font, pl->data_box_text[N], &size_X, &size_Y);

				size_MAX = (size_MAX < size_X) ? size_X : size_MAX;
				size_N++;
			}
		}
	}
	else if (pl->data_box_on == DATA_BOX_POLYFIT) {

		for (N = 0; N < PLOT_DATA_BOX_MAX; ++N) {

			if (pl->data_box_text[N][0] != 0) {

				TTF_SizeUTF8(pl->font, pl->data_box_text[N], &size_X, &size_Y);

				size_MAX = (size_MAX < size_X) ? size_X : size_MAX;
				size_N++;
			}
		}
	}

	pl->data_box_size_X = size_MAX;
	pl->data_box_N = size_N;

	if (pl->data_box_X > pl->viewport.max_x - (size_MAX + pl->layout_font_height))
		pl->data_box_X = pl->viewport.max_x - (size_MAX + pl->layout_font_height);

	if (pl->data_box_Y > pl->viewport.max_y - pl->layout_font_height * (size_N + 1))
		pl->data_box_Y = pl->viewport.max_y - pl->layout_font_height * (size_N + 1);

	if (pl->data_box_X < pl->viewport.min_x + pl->layout_font_height)
		pl->data_box_X = pl->viewport.min_x + pl->layout_font_height;

	if (pl->data_box_Y < pl->viewport.min_y + pl->layout_font_height)
		pl->data_box_Y = pl->viewport.min_y + pl->layout_font_height;
}

static void
plotDataBoxDraw(plot_t *pl, SDL_Surface *surface)
{
	int		boxY, size_X, size_Y;
	int		N, legX, legY;

	legX = pl->data_box_X;
	legY = pl->data_box_Y;
	size_X = pl->data_box_size_X;
	size_Y = pl->layout_font_height * pl->data_box_N;

	SDL_LockSurface(surface);

	if (pl->hover_data_box != -1) {

		drawFillRect(surface, legX, legY, legX + size_X,
				legY + size_Y, pl->sch->plot_hovered);
	}
	else if (pl->transparency == 0) {

		drawFillRect(surface, legX, legY, legX + size_X,
				legY + size_Y, pl->sch->plot_background);
	}

	SDL_UnlockSurface(surface);

	if (pl->data_box_on == DATA_BOX_SLICE) {

		for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

			if (pl->figure[N].busy != 0) {

				if (pl->data_box_text[N][0] != 0) {

					boxY = legY + pl->layout_font_height / 2;

					drawText(pl->dw, surface, pl->font, legX, boxY,
							pl->data_box_text[N], TEXT_CENTERED_ON_Y,
							pl->sch->plot_figure[N]);
				}

				legY += pl->layout_font_height;
			}
		}
	}
	else if (pl->data_box_on == DATA_BOX_POLYFIT) {

		for (N = 0; N < PLOT_DATA_BOX_MAX; ++N) {

			if (pl->data_box_text[N][0] != 0) {

				boxY = legY + pl->layout_font_height / 2;

				drawText(pl->dw, surface, pl->font, legX, boxY,
						pl->data_box_text[N], TEXT_CENTERED_ON_Y,
						pl->sch->plot_text);

				legY += pl->layout_font_height;
			}
		}
	}
}

int plotDataBoxGetByClick(plot_t *pl, int cur_X, int cur_Y)
{
	int		relX, relY, lenY, rN = -1;

	if (pl->data_box_on != DATA_BOX_FREE) {

		relX = cur_X - pl->data_box_X;
		relY = cur_Y - pl->data_box_Y;

		lenY = pl->layout_font_height * pl->data_box_N;

		if (		relX > 0 && relX < pl->data_box_size_X
				&& relY > 0 && relY < lenY) {

			rN = 0;
		}
	}

	pl->hover_data_box = rN;

	return rN;
}

void plotDataBoxCopyClipboard(plot_t *pl)
{
	int		N;

	if (pl->data_box_on != DATA_BOX_FREE) {

		pl->data_box_clipboard[0] = 0;

		for (N = 0; N < PLOT_DATA_BOX_MAX; ++N) {

			if (pl->data_box_text[N][0] != 0) {

				strcat(pl->data_box_clipboard, pl->data_box_text[N]);
				strcat(pl->data_box_clipboard, "\r\n");
			}
		}

		if (pl->data_box_clipboard[0] != 0) {

			SDL_SetClipboardText(pl->data_box_clipboard);
		}
	}
}

void plotLayout(plot_t *pl)
{
	int		aN, posX, posY;

	posX = 0;
	posY = 0;

	for (aN = 0; aN < PLOT_AXES_MAX; ++aN) {

		if (pl->axis[aN].busy == AXIS_BUSY_X) {

			if (pl->axis[aN].label[0] == 0)
				pl->axis[aN].compact = 1;

			pl->axis[aN].layout_pos = posX;

			posX += pl->layout_axis_box;
			posX += (pl->axis[aN].compact == 0) ? pl->layout_label_box : 0;
		}

		if (pl->axis[aN].busy == AXIS_BUSY_Y) {

			if (pl->axis[aN].label[0] == 0)
				pl->axis[aN].compact = 1;

			pl->axis[aN].layout_pos = posY;

			posY += pl->layout_axis_box;
			posY += (pl->axis[aN].compact == 0) ? pl->layout_label_box : 0;
		}
	}

	pl->viewport.min_x = pl->screen.min_x + posY + pl->layout_border;
	pl->viewport.max_x = pl->screen.max_x - pl->layout_border;
	pl->viewport.min_y = pl->screen.min_y + pl->layout_border;
	pl->viewport.max_y = pl->screen.max_y - posX - pl->layout_border;

	plotLegendLayout(pl);

	if (pl->data_box_on != DATA_BOX_FREE) {

		plotDataBoxLayout(pl);
	}

	if (pl->mark_on != 0) {

		if (pl->mark_count == 0) {

			plotMarkLayout(pl);
		}
	}
	else {
		pl->mark_count = 0;
	}
}

static void
plotDrawFigureTrialAll(plot_t *pl)
{
	int		FIGS[PLOT_FIGURE_MAX];
	int		N, fN, fQ, lN, dN, tTOP;

	lN = 0;

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].busy != 0 && pl->figure[fN].hidden != 0)
			FIGS[lN++] = fN;
	}

	for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

		if (pl->figure[fN].busy != 0 && pl->figure[fN].hidden == 0)
			FIGS[lN++] = fN;
	}

	if (pl->draw_in_progress == 0) {

		for (N = 0; N < lN; ++N) {

			fN = FIGS[N];
			dN = pl->figure[fN].data_N;

			pl->draw[fN].sketch = SKETCH_STARTED;
			pl->draw[fN].rN = pl->data[dN].head_N;
			pl->draw[fN].id_N = pl->data[dN].id_N;

			pl->draw[fN].skipped = 0;
			pl->draw[fN].line = 0;
		}

		pl->draw_in_progress = 1;
	}

	if (pl->draw_in_progress != 0) {

		tTOP = SDL_GetTicks() + 20;

		drawClearTrial(pl->dw);

		do {
			fN = -1;

			for (N = 0; N < lN; ++N) {

				fQ = FIGS[N];

				if (pl->draw[fQ].sketch != SKETCH_FINISHED) {

					if (fN < 0) {

						fN = fQ;
					}
					else if (pl->draw[fQ].id_N < pl->draw[fN].id_N) {

						fN = fQ;
					}
				}
			}

			if (fN >= 0) {

				if (SDL_GetTicks() > tTOP)
					break;

				plotDrawFigureTrial(pl, fN, tTOP);
			}
			else {
				plotSketchGarbage(pl);

				pl->draw_in_progress = 0;
				break;
			}
		}
		while (1);
	}
}

static void
plotDrawAxisAll(plot_t *pl, SDL_Surface *surface)
{
	int		aN;

	for (aN = 0; aN < PLOT_AXES_MAX; ++aN) {

		if (pl->axis[aN].busy != AXIS_FREE) {

			plotDrawAxis(pl, surface, aN);
		}
	}
}

void plotDraw(plot_t *pl, SDL_Surface *surface)
{
	if (pl->slice_mode_N != 0) {

		plotSliceLightDraw(pl, surface);
	}

	drawPixmapAlloc(pl->dw, surface);

	plotDrawPalette(pl);
	plotDrawFigureTrialAll(pl);

	drawClearCanvas(pl->dw);

	plotDrawSketch(pl, surface);

	if (pl->mark_on != 0) {

		plotMarkDraw(pl, surface);
	}

	SDL_LockSurface(surface);

	drawFlushCanvas(pl->dw, surface, &pl->viewport);

	SDL_UnlockSurface(surface);

	drawClearCanvas(pl->dw);

	plotDrawAxisAll(pl, surface);

	if (pl->slice_on != 0) {

		plotSliceDraw(pl, surface);
	}

	plotLegendDraw(pl, surface);

	SDL_LockSurface(surface);

	drawFlushCanvas(pl->dw, surface, &pl->viewport);

	SDL_UnlockSurface(surface);

	if (pl->data_box_on != DATA_BOX_FREE) {

		plotDataBoxDraw(pl, surface);
	}
}

