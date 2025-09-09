/*
   Graph Plotter is a tool to analyse numerical data.
   Copyright (C) 2025 Roman Belov <romblv@gmail.com>

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

#ifndef _H_PLOT_
#define _H_PLOT_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "draw.h"
#include "lse.h"
#include "scheme.h"

#ifdef ERROR
#undef ERROR
#endif

#ifdef FP_NAN
#undef FP_NAN
#endif

#define ERROR(fmt, ...)		fprintf(stderr, "%s:%i: " fmt, __FILE__, __LINE__, ## __VA_ARGS__)
#define FP_NAN			fp_nan()

#define PLOT_DATASET_MAX			10
#define PLOT_CHUNK_SIZE				16777216
#define PLOT_CHUNK_MAX				2000
#define PLOT_CHUNK_CACHE			4
#define PLOT_RCACHE_SIZE			32
#define PLOT_SLICE_SPAN				4
#define PLOT_AXES_MAX				10
#define PLOT_FIGURE_MAX 			10
#define PLOT_DATA_BOX_MAX			10
#define PLOT_MEDIAN_MAX 			57
#define PLOT_POLYFIT_MAX			7
#define PLOT_SUBTRACT				20
#define PLOT_GROUP_MAX				40
#define PLOT_MARK_MAX				80
#define PLOT_SKETCH_CHUNK_SIZE			32768
#define PLOT_SKETCH_MAX				800
#define PLOT_STRING_MAX				200
#define PLOT_RUNTIME_MAX			20

enum {
	TTF_ID_NONE			= 0,
	TTF_ID_ROBOTO_MONO_NORMAL,
	TTF_ID_ROBOTO_MONO_THIN
};

enum {
	AXIS_FREE			= 0,
	AXIS_BUSY_X,
	AXIS_BUSY_Y
};

enum {
	AXIS_SLAVE_DISABLE		= 0,
	AXIS_SLAVE_ENABLE,
	AXIS_SLAVE_HOLD_AS_IS
};

enum {
	LOCK_FREE			= 0,
	LOCK_AUTO,
	LOCK_CONDITION,
	LOCK_STACKED
};

enum {
	FIGURE_DRAWING_LINE		= 0,
	FIGURE_DRAWING_DASH,
	FIGURE_DRAWING_DOT
};

enum {
	SUBTRACT_FREE			= 0,
	SUBTRACT_TIME_MEDIAN,
	SUBTRACT_DATA_MEDIAN,
	SUBTRACT_SCALE,
	SUBTRACT_RESAMPLE,
	SUBTRACT_POLYFIT,
	SUBTRACT_BINARY_SUBTRACTION,
	SUBTRACT_BINARY_ADDITION,
	SUBTRACT_BINARY_MULTIPLICATION,
	SUBTRACT_BINARY_HYPOTENUSE,
	SUBTRACT_FILTER_DIFFERENCE,
	SUBTRACT_FILTER_CUMULATIVE,
	SUBTRACT_FILTER_BITFIELD,
	SUBTRACT_FILTER_LOW_PASS,
	SUBTRACT_FILTER_MEDIAN,
	SUBTRACT_FILTER_DEMULTIPLEX
};

enum {
	UNWRAP_NONE			= 0,
	UNWRAP_OVERFLOW,
	UNWRAP_BURST
};

enum {
	SKETCH_STARTED			= 0,
	SKETCH_INTERRUPTED,
	SKETCH_FINISHED
};

enum {
	DATA_BOX_FREE			= 0,
	DATA_BOX_SLICE,
	DATA_BOX_PICK,
	DATA_BOX_POLYFIT
};

typedef double			fval_t;

typedef struct {

	int		X;
	int		Y;
}
tuple_t;

typedef struct {

	draw_t			*dw;
	scheme_t		*sch;

	void			*ld;

	struct {

		int		column_N;
		int		length_N;

		int		chunk_SHIFT;
		int		chunk_MASK;
		int		chunk_bSIZE;

		int		lz4_compress;
		void		*lz4_reserved;

		struct {

			fval_t		*raw;

			int		chunk_N;
			int		dirty;
		}
		cache[PLOT_CHUNK_CACHE];

		int		cache_ID;

		struct {

			void		*raw;
			int		length;
		}
		compress[PLOT_CHUNK_MAX];

		fval_t		*raw[PLOT_CHUNK_MAX];
		int		*map;

		int		head_N;
		int		tail_N;
		int		id_N;

		struct {

			int	busy;

			union {

				struct {

					int	column_X;
					int	column_Y;

					int	column_T;

					int	length;
					int	unwrap;
					int	opdata;

					struct {

						double	fval;
						double	fpay;
					}
					window[PLOT_MEDIAN_MAX];

					int	keep;
					int	tail;

					double	prev[2];
					double	offset;
				}
				median;

				struct {

					int	column_X;
					int	modified;

					double	scale;
					double	offset;
				}
				scale;

				struct {

					int	column_X;

					int	in_data_N;
					int	in_column_X;
					int	in_column_Y;
				}
				resample;

				struct {

					int	column_X;
					int	column_Y;

					int	poly_N0;
					int	poly_N1;

					double	coefs[PLOT_POLYFIT_MAX + 1];
					double	std;
				}
				polyfit;

				struct {

					int	column_X;
					int	column_Y;
				}
				binary;

				struct {

					int	column_X;
					int	column_Y;

					double	value;
					double	state[2];
				}
				filter;
			}
			op;
		}
		sub[PLOT_SUBTRACT];

		int		sub_N;
		int		sub_paused;
	}
	data[PLOT_DATASET_MAX];

	struct {

		int		busy;

		int		data_N;
		int		column_N;

		struct {

			int		computed;
			int		finite;

			fval_t		fmin;
			fval_t		fmax;
		}
		chunk[PLOT_CHUNK_MAX];

		int		cached;

		fval_t		fmin;
		fval_t		fmax;
	}
	rcache[PLOT_RCACHE_SIZE];

	struct {

		int		busy;

		int		lock_scale;
		int		lock_tick;

		int		slave;
		int		slave_N;

		double		scale;
		double		offset;

		char		label[PLOT_STRING_MAX];

		int		compact;
		int		exponential;

		int		layout_pos;

		double		ruler_tih;
		double		ruler_tis;
		double		ruler_min;
		double		ruler_max;
	}
	axis[PLOT_AXES_MAX];

	struct {

		int		busy;
		int		hidden;

		int		drawing;
		int		width;

		int		data_N;
		int		column_X;
		int		column_Y;
		int		axis_X;
		int		axis_Y;

		double		mark_X[PLOT_MARK_MAX];
		double		mark_Y[PLOT_MARK_MAX];

		int		slice_busy;
		const fval_t	*slice_row;
		int		slice_id_N;
		double		slice_X;
		double		slice_Y;

		int		slice_base_catch;
		double		slice_base_X;
		double		slice_base_Y;

		int		brush_N;

		char		label[PLOT_STRING_MAX];
	}
	figure[PLOT_FIGURE_MAX];

	struct {

		int		op_time_median;
		int		op_time_unwrap;
		int		op_time_opdata;
		int		op_scale;

		int		length;
		double		ungap;

		double		scale;
		double		offset;

		char		label[PLOT_STRING_MAX];
	}
	group[PLOT_GROUP_MAX];

	clipBox_t		viewport;
	clipBox_t		screen;

	TTF_Font		*font;

	lse_t			lsq;

	int			rcache_ID;
	int			rcache_wipe_data_N;
	int			rcache_wipe_chunk_N;

	int			legend_hidden;
	int			legend_X;
	int			legend_Y;
	int			legend_size_X;
	int			legend_N;

	int			data_box_on;
	int			data_box_X;
	int			data_box_Y;
	int			data_box_size_X;
	int			data_box_N;
	char			data_box_text[PLOT_DATA_BOX_MAX][PLOT_STRING_MAX];
	char			data_box_clipboard[PLOT_DATA_BOX_MAX * PLOT_STRING_MAX];

	int			slice_on;
	int			slice_mode_N;
	int			slice_axis_N;

	int			pick_on;

	int			mark_on;
	int			mark_length;
	int			mark_size;
	int			mark_density;

	int			brush_on;
	int			brush_box_X;
	int			brush_box_Y;
	int			brush_cur_X;
	int			brush_cur_Y;

	struct {

		int		sketch;

		int		rN;
		int		id_N;

		int		skipped;
		int		line;

		double		last_X;
		double		last_Y;

		int		list_self;
	}
	draw[PLOT_FIGURE_MAX];

	int			draw_in_progress;

	Uint32			tick_cached;
	int			tick_skip;

	struct {

		int		figure_N;

		int		drawing;
		int		width;

		double		*chunk;
		int		length;

		int		linked;
	}
	sketch[PLOT_SKETCH_MAX];

	int			sketch_list_garbage;
	int			sketch_list_todraw;
	int			sketch_list_current;
	int			sketch_list_current_end;

	int			layout_font_ttf;
	int			layout_font_pt;
	int			layout_font_height;
	int			layout_font_long;
	int			layout_font_space;
	int			layout_border;
	int			layout_ruler_box;
	int			layout_label_box;
	int			layout_tick_tooth;
	int			layout_grid_dash;
	int			layout_grid_space;
	int			layout_drawing_dash;
	int			layout_drawing_space;
	int			layout_mark_size;
	int			layout_fence_dash;
	int			layout_fence_space;
	int			layout_fence_point;

	int			on_X;
	int			on_Y;

	int			hover_figure;
	int			hover_legend;
	int			hover_data_box;
	int			hover_axis;

	int			interpolation;
	double			defungap;

	int			default_drawing;
	int			default_width;

	int			transparency;
	int			fprecision;
	int			fhexadecimal;
	int			lz4_compress;

	int			shift_on;
}
plot_t;

double fp_nan();
int fp_isfinite(double x);

plot_t *plotAlloc(draw_t *dw, scheme_t *sch);
void plotClean(plot_t *pl);

void plotFontDefault(plot_t *pl, int ttfnum, int ptsize, int style);
int plotFontOpen(plot_t *pl, const char *ttf, int ptsize, int style);

unsigned long long plotDataMemoryUsage(plot_t *pl, int dN);
unsigned long long plotDataMemoryUncompressed(plot_t *pl, int dN);
unsigned long long plotDataMemoryCached(plot_t *pl, int dN);

void plotDataAlloc(plot_t *pl, int dN, int cN, int lN);
void plotDataResize(plot_t *pl, int dN, int lN);
int plotDataLength(plot_t *pl, int dN);
int plotDataSpaceLeft(plot_t *pl, int dN);
void plotDataGrowUp(plot_t *pl, int dN);
void plotDataSubtractCompute(plot_t *pl, int dN, int sN);
void plotDataSubtractResidual(plot_t *pl, int dN);
void plotDataSubtractClean(plot_t *pl);
void plotDataSubtractPaused(plot_t *pl);
void plotDataSubtractAlternate(plot_t *pl);
void plotDataInsert(plot_t *pl, int dN, const fval_t *row);
void plotDataClean(plot_t *pl, int dN);

void plotDataRangeCacheClean(plot_t *pl, int dN);
void plotDataRangeCacheSubtractClean(plot_t *pl);
int plotDataRangeCacheFetch(plot_t *pl, int dN, int cN);

void plotAxisLabel(plot_t *pl, int aN, const char *label);
int plotAxisRangeGet(plot_t *pl, int aN, double *pmin, double *pmax);
void plotAxisScaleManual(plot_t *pl, int aN, double min, double max);
void plotAxisScaleAuto(plot_t *pl, int aN);
void plotAxisScaleAutoCond(plot_t *pl, int aN, int bN);
void plotAxisScaleLock(plot_t *pl, int knob);
void plotAxisScaleDefault(plot_t *pl);
void plotAxisScaleZoom(plot_t *pl, int aN, int origin, double zoom);
void plotAxisScaleMove(plot_t *pl, int aN, double move);
void plotAxisScaleEqual(plot_t *pl);
void plotAxisScaleGridAlign(plot_t *pl);
void plotAxisScaleGridLock(plot_t *pl, int aN);
void plotAxisScaleStacked(plot_t *pl, int bN);

int plotAxisGetByClick(plot_t *pl, int cur_X, int cur_Y);
double plotAxisConvForward(plot_t *pl, int aN, double fval);
double plotAxisConvBackward(plot_t *pl, int aN, double xval);
void plotAxisSlave(plot_t *pl, int aN, int bN, double scale, double offset, int action);
void plotAxisRemove(plot_t *pl, int aN);

void plotFigureAdd(plot_t *pl, int fN, int dN, int nX, int nY, int aX, int aY, const char *label);
void plotFigureSubtractGarbage(plot_t *pl);
void plotFigureScaleMerge(plot_t *pl);
void plotFigureRemove(plot_t *pl, int fN);
void plotFigureGarbage(plot_t *pl, int dN);
void plotFigureMoveAxes(plot_t *pl, int fN);
void plotFigureMakeIndividualAxes(plot_t *pl, int fN);
void plotFigureExchange(plot_t *pl, int fN_1, int fN_2);

int plotFigureSelected(plot_t *pl);
int plotFigureAnother(plot_t *pl, int fN);
int plotFigureAnyData(plot_t *pl);
int plotFigureHaveData(plot_t *pl, int dN);

tuple_t plotGetSubtractTimeMedian(plot_t *pl, int dN, int cNX, int cNY, int length, int unwrap, int opdata);
int plotGetSubtractScale(plot_t *pl, int dN, int cN, double scale, double offset);
int plotGetSubtractResample(plot_t *pl, int dN, int cN_X, int in_dN, int in_cN_X, int in_cN_Y);
int plotGetSubtractBinary(plot_t *pl, int dN, int opSUB, int cN_1, int cN_2);
int plotGetSubtractFilter(plot_t *pl, int dN, int cNX, int cNY, int opSUB, double value);
int plotGetSubtractMedian(plot_t *pl, int dN, int cN, int opSUB, int length);
int plotGetFreeFigure(plot_t *pl);

int plotFigureSubtractGetMedianConfig(plot_t *pl, int fN, int *length, int *unwrap, int *opdata);
void plotFigureSubtractTimeMedian(plot_t *pl, int fN, int length, int unwrap, int opdata);
void plotFigureSubtractScale(plot_t *pl, int fN, int aBUSY, double scale, double offset);
void plotFigureSubtractFilter(plot_t *pl, int fN, int opSUB, double value);
void plotFigureSubtractDemux(plot_t *pl, int fN, int opSUB, int N);
void plotFigureSubtractClean(plot_t *pl, int fN, int opSUB);
void plotFigureSubtractSwitch(plot_t *pl, int opSUB);
void plotTotalSubtractResample(plot_t *pl, int dN, double tmin, double tmax);

int plotDataBoxPolyfit(plot_t *pl, int fN);
void plotFigureSubtractPolyfit(plot_t *pl, int fN, int N0, int N1);
int plotFigureExportCSV(plot_t *pl, const char *file);
void plotFigureClean(plot_t *pl);
void plotSketchClean(plot_t *pl);
int plotGetSketchLength(plot_t *pl);

void plotGroupAdd(plot_t *pl, int dN, int gN, int cN);
void plotGroupLabel(plot_t *pl, int gN, const char *label);
void plotGroupMedian(plot_t *pl, int gN, int length, int unwrap, int opdata);
void plotGroupScale(plot_t *pl, int gN, int knob, double scale, double offset);

void plotSliceSwitch(plot_t *pl);
void plotSliceTrack(plot_t *pl, int cur_X, int cur_Y);
void plotPickTrack(plot_t *pl, int cur_X, int cur_Y);
void plotBrushErase(plot_t *pl);

int plotLegendGetByClick(plot_t *pl, int cur_X, int cur_Y);
int plotLegendBoxGetByClick(plot_t *pl, int cur_X, int cur_Y);
int plotDataBoxGetByClick(plot_t *pl, int cur_X, int cur_Y);
void plotDataBoxCopyClipboard(plot_t *pl);

void plotLayout(plot_t *pl);
void plotDraw(plot_t *pl, SDL_Surface *surface);

#endif /* _H_PLOT_ */

