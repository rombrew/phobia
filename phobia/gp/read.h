/*
   Graph Plotter for numerical data analysis.
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

#ifndef _H_READ_
#define _H_READ_

#include <stdlib.h>
#include <stdio.h>

#include <SDL2/SDL.h>

#include "async.h"
#include "draw.h"
#include "plot.h"

#define READ_PAGE_MAX		1000
#define READ_COLUMN_MAX		2000
#define READ_TOKEN_MAX		80
#define READ_FILE_PATH_MAX	800
#define READ_TEXT_HEADER_MAX	9

#define GP_MIN_SIZE_X		640
#define GP_MIN_SIZE_Y		480

#define GP_CONFIG_VERSION	5

enum {
	FORMAT_NONE			= 0,
	FORMAT_PLAIN_TEXT,
	FORMAT_BINARY_FLOAT,
	FORMAT_BINARY_DOUBLE,

#ifdef _WINDOWS
	FORMAT_BINARY_LEGACY_V1,
	FORMAT_BINARY_LEGACY_V2,
#endif /* _WINDOWS */
};

enum {
	DATA_HINT_NONE			= 0,
	DATA_HINT_FLOAT,
	DATA_HINT_HEX,
	DATA_HINT_OCT
};

typedef unsigned long long	ulen_t;

typedef struct {

	char		delim;
	char		space[READ_TOKEN_MAX];
	char		lend[READ_TOKEN_MAX];
}
markup_t;

typedef struct {

	int		busy;

	int		column_2;
	double		scale;
	double		offset;
}
fig_ops_t;

typedef struct {

	int		busy;
	char		title[PLOT_STRING_MAX];

	struct {

		int		busy;

		int		drawing;
		int		width;

		int		dN;
		int		cX;
		int		cY;
		int		aX;
		int		aY;

		fig_ops_t	ops[2];

		char		label[PLOT_STRING_MAX];
	}
	fig[PLOT_FIGURE_MAX];

	struct {

		int		slave;
		int		slave_N;
		double		scale;
		double		offset;

		char		label[PLOT_STRING_MAX];
	}
	ax[PLOT_AXES_MAX];
}
page_t;

typedef struct {

	char		file[READ_FILE_PATH_MAX];
	const char	*path;

	FILE		*fd;
	const char	*in;

	char		tbuf[READ_FILE_PATH_MAX];
	int		unchar;
	int		line_N;
	int		newline;
	int		fromUI;

	int		dmap[PLOT_DATASET_MAX];
}
parse_t;

typedef struct {

	draw_t		*dw;
	plot_t		*pl;

	char		screenpath[READ_FILE_PATH_MAX];

	int		config_version;
	int		window_size_x;
	int		window_size_y;
	int		language;
	int		colorscheme;
	int		timecol;
	int		shortfilename;

	markup_t	mk_config;
	markup_t	mk_text;

#ifdef _WINDOWS
	int		legacy_label_enc;
#endif /* _WINDOWS */

	int		preload;
	int		chunk;
	int		timeout;
	int		length_N;

	struct {

		int		format;
		int		column_N;
		int		length_N;

		char		file[READ_FILE_PATH_MAX];
		int		line_N;

		FILE		*fd;
		async_FILE	*afd;

		char		buf[READ_TOKEN_MAX * READ_COLUMN_MAX];
		fval_t		row[READ_COLUMN_MAX];

		char		label[READ_COLUMN_MAX][READ_TOKEN_MAX];

		int		hint[READ_COLUMN_MAX];
	}
	data[PLOT_DATASET_MAX];

	page_t		page[READ_PAGE_MAX];

	int		files_N;

	int		bind_N;
	int		page_N;
	int		figure_N;
}
read_t;

read_t *readAlloc(draw_t *dw, plot_t *pl);
void readClean(read_t *rd);
void readOpenUnified(read_t *rd, int dN, int cN, int lN, const char *file, int fmt);
void readToggleHint(read_t *rd, int dN, int cN);
int readUpdate(read_t *rd);

#ifdef _WINDOWS
void legacy_ACP_to_UTF8(char *ustr, const char *text, int n);
void legacy_OEM_to_UTF8(char *ustr, const char *text, int n);
void legacy_readConfigGRM(read_t *rd, const char *path, const char *confile, const char *file, int fromUI);
#endif /* _WINDOWS */

FILE *unified_fopen(const char *file, const char *mode);

void readConfigIN(read_t *rd, const char *config, int fromUI);
void readConfigGP(read_t *rd, const char *file, int fromUI);
void readConfigVerify(read_t *rd);

void readMakePages(read_t *rd, int dN, int cX, int fromUI);
void readDatasetClean(read_t *rd, int dN);
int readGetTimeColumn(read_t *rd, int dN);
void readSetTimeColumn(read_t *rd, int dN, int cX);

void readSelectPage(read_t *rd, int pN);
void readCombinePage(read_t *rd, int pN, int remap);
void readDataReload(read_t *rd);

#endif /* _H_READ_ */

