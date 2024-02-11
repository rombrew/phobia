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
#include <errno.h>
#include <math.h>
#include <string.h>
#include <locale.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>

#ifdef _WINDOWS
#include <windows.h>
#endif /* _WINDOWS */

#include "gp.h"
#include "dirent.h"
#include "draw.h"
#include "edit.h"
#include "lang.h"
#include "menu.h"
#include "plot.h"
#include "read.h"
#include "scheme.h"
#include "svg.h"

#undef main

#define GP_FILE_DIR_MAX			4000

enum {
	GP_TAKE_NONE		= 0,
	GP_TAKE_PNG,
	GP_TAKE_SVG,
	GP_TAKE_CSV
};

enum {
	GP_COMBINE_NONE		= 0,
	GP_COMBINE_AXES_REMAP,
	GP_COMBINE_NO_REMAP
};

struct gp_struct {

	scheme_t	*sch;
	lang_t		*la;

	draw_t		*dw;
	plot_t		*pl;
	read_t		*rd;
	menu_t		*mu;
	edit_t		*ed;

	SDL_Window	*window;
	SDL_Surface	*fb;
	SDL_Surface	*surface;

	int		window_ID;

	char		sbuf[4][READ_FILE_PATH_MAX];

	char		rcfile[READ_FILE_PATH_MAX];
	char		tempfile[READ_FILE_PATH_MAX];

	int		done;
	int		stat;

	int		active;
	int		unfinished;
	int		drawn;

	int		clock;
	int		idled;
	int		updated;
	int		level;

	int		ctrl_on;
	int		shift_on;

	int		i_show_fps;
	int		i_frames;
	int		i_clocked;
	int		i_FPS;

	int		fullscreen;
	int		hinting;

	int		cur_X;
	int		cur_Y;
	int		box_X;
	int		box_Y;
	int		ax_N;
	int		fig_N;
	int		data_N;
	int		grp_N;
	int		line_N;

	int		screen_take;
	int		screen_yank;
	int		legend_drag;
	int		data_box_drag;
	int		combine_on;
	int		hover_box;

	int		layout_page_box;
	int		layout_page_title_offset;
	int		layout_menu_page_margin;
	int		layout_menu_dir_margin;
	int		layout_menu_dataset_margin;
	int		layout_menu_dataset_minimal;

	struct dirent_stat	sb;

	char		cwd[READ_FILE_PATH_MAX];
	int		cwd_ok;

	char		d_names[GP_FILE_DIR_MAX][PLOT_STRING_MAX];
	char		la_menu[GP_FILE_DIR_MAX * PLOT_STRING_MAX + 1];
};

enum {
	GP_IDLE			= 0,
	GP_MOVING,
	GP_RANGE_SELECT,
	GP_BOX_SELECT,
	GP_MENU,
	GP_EDIT,
};

#ifndef _EMBED_GP
static void
gpMakeHello(gp_t *gp)
{
	const fval_t omul[] = {

		21.180, 25.120, 20.298, 42.448, 19.203, 72.868, 18.649,
		106.511, 19.393, 133.510, 22.521, 159.491, 26.570, 178.031,
		30.850, 190.678, 34.669, 198.980, 41.013, 186.502, 45.839,
		176.055, 49.991, 165.249, 54.311, 151.696, 65.768, 151.514,
		77.225, 151.332, 88.682, 151.150, 100.140, 150.968, 101.891,
		159.968, 104.920, 169.501, 110.001, 181.771, 117.909, 198.980,
		125.652, 182.521, 130.473, 168.671, 133.481, 156.518, 135.785,
		145.149, 137.191, 112.465, 136.247, 72.753, 134.526, 39.231,
		133.602, 25.120, 105.497, 25.120, 77.391, 25.120, 49.286,
		25.120, 21.180, 25.120, FP_NAN, FP_NAN, 32.487, 120.416, 41.887,
		125.997, 50.408, 127.691, 58.540, 125.820, 66.771, 120.710,
		FP_NAN, FP_NAN, 90.832, 120.397, 100.879, 126.139, 109.671,
		128.016, 117.918, 126.115, 126.329, 120.519, FP_NAN, FP_NAN,
		51.817, 91.523, 66.133, 83.048, 79.760, 81.249, 92.873, 84.769,
		105.648, 92.251, FP_NAN, FP_NAN, 36.994, 25.848, 43.501, 32.639,
		50.371, 35.609, 58.851, 33.517, 70.189, 25.120, FP_NAN, FP_NAN,
		85.189, 25.120, 92.850, 33.737, 101.987, 36.883, 111.918,
		34.147, 121.963, 25.120, FP_NAN, FP_NAN, 134.434, 40.169,
		135.268, 39.659, 139.077, 39.450, 147.824, 41.521, 163.470,
		47.850, 175.241, 59.465, 176.895, 75.080, 177.196, 93.201,
		184.908, 112.335, 189.761, 118.670, 193.306, 123.087, 195.480,
		125.673, 196.219, 126.515, 196.803, 130.617, 194.349, 133.181,
		190.339, 133.762, 186.259, 131.917, 182.574, 127.680, 177.766,
		121.809, 172.436, 114.071, 167.183, 104.232, 164.575, 91.517,
		165.070, 78.686, 164.466, 67.214, 158.560, 58.576, 151.353,
		55.039, 143.859, 52.910, 137.973, 51.867, 135.590, 51.590,
		FP_NAN, FP_NAN, -55.581, 25.377
	};

	const int lN = sizeof(omul) / sizeof(omul[0]) / 2;

	int		N, dN = 0, pN = 1;

	plotDataAlloc(gp->pl, dN, 2, lN + 1);

	for (N = 0; N < lN; ++N) {

		plotDataInsert(gp->pl, dN, omul + gp->pl->data[dN].column_N * N);
	}

	gp->rd->data[dN].format = FORMAT_PLAIN_TEXT;
	gp->rd->data[dN].column_N = 2;
	gp->rd->data[dN].file[0] = 0;
	gp->rd->files_N = 1;
	gp->rd->bind_N = dN;

	gp->rd->page_N = pN;
	gp->rd->figure_N = -1;

	gp->rd->page[pN].busy = 1;

	strcpy(gp->rd->page[pN].title, "Hello, I am Omul");
	strcpy(gp->rd->page[pN].fig[0].label, "Omul");

	gp->rd->page[pN].fig[0].busy = 1;
	gp->rd->page[pN].fig[0].drawing = FIGURE_DRAWING_LINE;
	gp->rd->page[pN].fig[0].width = 6;
	gp->rd->page[pN].fig[0].dN = dN;
	gp->rd->page[pN].fig[0].cX = 0;
	gp->rd->page[pN].fig[0].cY = 1;
	gp->rd->page[pN].fig[0].aX = 0;
	gp->rd->page[pN].fig[0].aY = 1;
}
#endif /* _EMBED_GP */

static void
gpFileGetPath(gp_t *gp)
{
	char		*home;

#ifdef _WINDOWS
	home = getenv("APPDATA");

	if (home != NULL) {

		legacy_ACP_to_UTF8(gp->rcfile, home, READ_FILE_PATH_MAX);
		strcat(gp->rcfile, "/_gprc");
	}
#else /* _WINDOWS */

	home = getenv("HOME");

	if (home != NULL) {

		strcpy(gp->rcfile, home);
		strcat(gp->rcfile, "/.gprc");
	}
#endif
}

static void
gpFileGetLocal(gp_t *gp)
{
#ifdef _WINDOWS
	strcpy(gp->rcfile, "_gprc");
#else /* _WINDOWS */
	strcpy(gp->rcfile, ".gprc");
#endif
}

static void
gpDefaultFile(gp_t *gp)
{
	FILE		*fd;

	fd = unified_fopen(gp->rcfile, "w");

	if (fd == NULL) {

		ERROR("fopen(\"%s\"): %s\n", gp->rcfile, strerror(errno));
	}
	else {
		fprintf(fd,	"gpconfig %i\n", GP_CONFIG_VERSION);

		fprintf(fd,	"font 24 \"normal\"\n"
				"preload 8388608\n"
				"chunk 4096\n"
				"timeout 5000\n"
				"windowsize 1200 900\n"
				"language 0\n"
				"colorscheme 0\n"
				"antialiasing 1\n"
				"blendfont 1\n"
				"thickness 1\n"
				"gamma 50\n"
				"drawing line 2\n"
				"marker 40\n"
				"density 40\n"
				"transparency 1\n"
				"precision 9\n"
				"timecol -1\n"
				"shortfilename 1\n"
				"fastdraw 200\n"
				"interpolation 1\n"
				"defungap 10\n"
				"lz4_compress 1\n");

#ifdef _WINDOWS
		fprintf(fd,	"legacy_label 1\n");
#endif /* _WINDOWS */

		fclose(fd);
	}
}

static void
gpWriteFile(gp_t *gp)
{
	draw_t		*dw = gp->dw;
	plot_t		*pl = gp->pl;
	read_t		*rd = gp->rd;

	const char	*ttfname, *drawing;
	FILE		*fd;

	fd = unified_fopen(gp->rcfile, "w");

	if (fd == NULL) {

		ERROR("fopen(\"%s\"): %s\n", gp->rcfile, strerror(errno));
	}
	else {
		fprintf(fd, "gpconfig %i\n", GP_CONFIG_VERSION);

		ttfname = (pl->layout_font_ttf == TTF_ID_ROBOTO_MONO_NORMAL) ? "normal"
			: (pl->layout_font_ttf == TTF_ID_ROBOTO_MONO_THIN) ? "thin" : rd->ttfname;

		fprintf(fd, "font %i \"%s\"\n", pl->layout_font_pt, ttfname);
		fprintf(fd, "preload %i\n", rd->preload);
		fprintf(fd, "chunk %i\n", rd->chunk);
		fprintf(fd, "timeout %i\n", rd->timeout);

		SDL_GetWindowSize(gp->window, &rd->window_size_x, &rd->window_size_y);

		fprintf(fd, "windowsize %i %i\n", rd->window_size_x, rd->window_size_y);
		fprintf(fd, "language %i\n", rd->language);
		fprintf(fd, "colorscheme %i\n", rd->colorscheme);
		fprintf(fd, "antialiasing %i\n", dw->antialiasing);
		fprintf(fd, "blendfont %i\n", dw->blendfont);
		fprintf(fd, "thickness %i\n", dw->thickness);
		fprintf(fd, "gamma %i\n", dw->gamma);

		drawing = (pl->default_drawing == FIGURE_DRAWING_LINE) ? "line"
			: (pl->default_drawing == FIGURE_DRAWING_DASH) ? "dash"
			: (pl->default_drawing == FIGURE_DRAWING_DOT) ? "dot" : "unknown";

		fprintf(fd, "drawing %s %i\n", drawing, pl->default_width);
		fprintf(fd, "marker %i\n", pl->mark_size);
		fprintf(fd, "density %i\n", pl->mark_density);
		fprintf(fd, "transparency %i\n", pl->transparency);
		fprintf(fd, "precision %i\n", pl->fprecision);
		fprintf(fd, "timecol %i\n", rd->timecol);
		fprintf(fd, "shortfilename %i\n", rd->shortfilename);
		fprintf(fd, "fastdraw %i\n", rd->fastdraw);
		fprintf(fd, "interpolation %i\n", pl->interpolation);
		fprintf(fd, "defungap %i\n", pl->defungap);
		fprintf(fd, "lz4_compress %i\n", pl->lz4_compress);

#ifdef _WINDOWS
		fprintf(fd, "legacy_label %i\n", rd->legacy_label);
#endif /* _WINDOWS */

		fclose(fd);
	}
}

static int
gpFileExist(const char *file)
{
	unsigned long long	nsize;

	return (file_stat(file, &nsize) == ENT_OK) ? 1 : 0;
}

static int
gpScreenLength(plot_t *pl)
{
	return pl->screen.max_x / pl->layout_font_long;
}

static void
gpTextLeftCrop(plot_t *pl, char *sbuf, const char *text, int margin)
{
	int		length, allowed;

	allowed = gpScreenLength(pl) - margin;
	length = utf8_length(text);

	if (length > (allowed - 1)) {

		text = utf8_skip(text, length - (allowed - 1));

		strcpy(sbuf, "~");
		strcat(sbuf, text);
	}
	else {
		strcpy(sbuf, text);
	}
}

static void
gpTextSepFill(char *sbuf, int len)
{
	memset(sbuf, '-', len);
	sbuf[len] = 0;
}

static void
gpTextFloat(plot_t *pl, char *sbuf, double val)
{
	char		sfmt[PLOT_STRING_MAX];
	int		fexp = 1;

	if (val != 0.) {

		fexp += (int) floor(log10(fabs(val)));
	}

	if (fexp >= -2 && fexp < pl->fprecision) {

		fexp = (fexp < 1) ? 1 : fexp;

		sprintf(sfmt, "%%.%df", pl->fprecision - fexp);
	}
	else {
		sprintf(sfmt, "%%.%dE", pl->fprecision - 1);
	}

	sprintf(sbuf, sfmt, val);
}

static void
gpScreenLayout(gp_t *gp)
{
	plot_t		*pl = gp->pl;
	menu_t		*mu = gp->mu;
	edit_t		*ed = gp->ed;

	mu->screen.min_x = 0;
	mu->screen.max_x = gp->surface->w - 1;
	mu->screen.min_y = 0;
	mu->screen.max_y = gp->surface->h - 1;

	ed->screen.min_x = 0;
	ed->screen.max_x = gp->surface->w - 1;
	ed->screen.min_y = 0;
	ed->screen.max_y = gp->surface->h - 1;

	pl->screen.min_x = 0;
	pl->screen.max_x = gp->surface->w - 1;
	pl->screen.min_y = gp->layout_page_box;
	pl->screen.max_y = gp->surface->h - 1;

	menuLayout(mu);
	editLayout(ed);
}

static void
gpFontLayout(gp_t *gp)
{
	plot_t		*pl = gp->pl;
	menu_t		*mu = gp->mu;
	edit_t		*ed = gp->ed;

	mu->font = pl->font;
	mu->layout_height = pl->layout_font_height;

	ed->font = pl->font;
	ed->layout_height = pl->layout_font_height;
	ed->layout_long = pl->layout_font_long;

	gp->layout_page_box = pl->layout_font_height;
	gp->layout_page_title_offset = - pl->layout_font_height / 2;
	gp->layout_menu_page_margin = 8;
	gp->layout_menu_dir_margin = 36;
	gp->layout_menu_dataset_margin = 16;
	gp->layout_menu_dataset_minimal = 42;
}

static void
gpFontHinting(gp_t *gp)
{
	plot_t		*pl = gp->pl;

	if (gp->hinting == 0) {

		TTF_SetFontHinting(pl->font, TTF_HINTING_NONE);
	}
	else if (gp->hinting == 1) {

		TTF_SetFontHinting(pl->font, TTF_HINTING_LIGHT);
	}
	else if (gp->hinting == 2) {

		TTF_SetFontHinting(pl->font, TTF_HINTING_NORMAL);
	}
}

static void
gpFontToggle(gp_t *gp, int toggle, int font_pt)
{
	plot_t		*pl = gp->pl;
	int		style;

	if (toggle != 0) {

		pl->layout_font_ttf = (pl->layout_font_ttf != TTF_ID_ROBOTO_MONO_NORMAL)
			? TTF_ID_ROBOTO_MONO_NORMAL : TTF_ID_ROBOTO_MONO_THIN;
	}

	font_pt = (font_pt == 0) ? pl->layout_font_pt
		: (font_pt < 8) ? 8 : (font_pt > 92) ? 92 : font_pt;

	style = TTF_STYLE_NORMAL;

	style |= (gp->ctrl_on == 1) ? TTF_STYLE_BOLD : 0;
	style |= (gp->shift_on == 1) ? TTF_STYLE_ITALIC : 0;

	plotFontDefault(pl, pl->layout_font_ttf, font_pt, style);

	gpFontHinting(gp);

	gpFontLayout(gp);
	gpScreenLayout(gp);
}

static void
gpMakePageMenu(gp_t *gp)
{
	read_t		*rd = gp->rd;
	char		*la = gp->la_menu;
	int		pN = 0;

	do {
		if (rd->page[pN].busy != 0) {

			gpTextLeftCrop(gp->pl, gp->sbuf[1], rd->page[pN].title,
					gp->layout_menu_page_margin);

			sprintf(gp->sbuf[0], "%3d %s", pN, gp->sbuf[1]);

			strcpy(la, gp->sbuf[0]);
			la += strlen(la) + 1;
		}

		pN += 1;

		if (pN >= READ_PAGE_MAX)
			break;
	}
	while (1);

	*la = 0;
}

static int
gpFileIsGP(gp_t *gp, const char *file)
{
	int		rc = 0;

	if (strlen(file) > 3) {

		file += strlen(file) - 3;

		rc = (strcmp(file, ".gp") == 0 || strcmp(file, ".GP") == 0) ? 1 : 0;
	}

	return rc;
}

#ifdef _LEGACY
static int
legacy_FileIsBAT(gp_t *gp, const char *file)
{
	int		rc = 0;

	if (strlen(file) > 4) {

		file += strlen(file) - 4;

		rc = (strcmp(file, ".bat") == 0 || strcmp(file, ".BAT") == 0) ? 1 : 0;
	}

	return rc;
}

static char *
legacy_DirName(char *path)
{
	char		*eol;

	eol = path + strlen(path) - 1;

	do {
		if (*eol == '/' || *eol == '\\')
			break;

		if (path == eol)
			break;

		eol--;
	}
	while (1);

	if (eol != path) {

		*eol = 0;
	}
	else {
		path = NULL;
	}

	return path;
}

static void
legacy_FileOpenBAT(gp_t *gp, const char *file, int fromUI)
{
	FILE		*fd;
	char		*s, *path, *argv[2];
	int		line_N = 0;

	fd = unified_fopen(file, "r");

	if (fd == NULL) {

		ERROR("fopen(\"%s\"): %s\n", file, strerror(errno));
	}
	else {
		do {
			if ((s = fgets(gp->sbuf[0], sizeof(gp->sbuf[0]), fd)) == NULL)
				break;

			s = strtok(s, " \t");

			if (s != NULL && (strcmp(s, "grm") == 0 || strcmp(s, "GRM") == 0)) {

				if ((argv[0] = strtok(NULL, " \t")) != NULL) {

					if ((argv[1] = strtok(NULL, " \t\r\n")) != NULL) {

						strcpy(gp->sbuf[1], file);
						path = legacy_DirName(gp->sbuf[1]);

						legacy_ConfigGRM(gp->rd, path, argv[0], argv[1], fromUI);
						break;
					}
				}
			}

			line_N++;

			if (line_N >= 10)
				break;
		}
		while (1);

		fclose(fd);
	}
}
#endif /* _LEGACY */

static void
gpUnifiedFileOpen(gp_t *gp, const char *file, int fromUI)
{
	read_t		*rd = gp->rd;

	if (gpFileIsGP(gp, file) != 0) {

		readConfigGP(rd, file, fromUI);
	}

#ifdef _LEGACY
	else if (legacy_FileIsBAT(gp, file) != 0) {

		legacy_FileOpenBAT(gp, file, fromUI);
	}
#endif /* _LEGACY */

	else {
		sprintf(gp->sbuf[0],	"load 0 0 text \"%s\"\n"
					"mkpages -2\n", file);

		readConfigIN(rd, gp->sbuf[0], fromUI);
	}
}

static int
gpDirWalk(gp_t *gp, int dir_N, int revert);

static void
gpMakeDirMenu(gp_t *gp)
{
	char			sfmt[PLOT_STRING_MAX];
	char			*la = gp->la_menu;
	int			len, pad, N, kmg;

	len = gpScreenLength(gp->pl) - gp->layout_menu_dir_margin;

	if (dirent_open(&gp->sb, gp->cwd) == ENT_OK) {

		gp->cwd_ok = 1;
	}
	else {
		gpDirWalk(gp, 2, 1);

		gp->cwd_ok = (dirent_open(&gp->sb, gp->cwd) == ENT_OK) ? 1 : 0;
	}

	gpTextLeftCrop(gp->pl, gp->sbuf[1], gp->cwd, gp->layout_menu_dir_margin);

	sprintf(gp->sbuf[0], "[%s]", gp->sbuf[1]);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	gpTextSepFill(gp->sbuf[0], len + 26);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	strcpy(la, "/..");
	la += strlen(la) + 1;

	strcpy(gp->d_names[0], "/..");
	N = 1;

	if (gp->cwd_ok != 0) {

		while (dirent_read(&gp->sb) == ENT_OK) {

			if (gp->sb.ntype == ENT_TYPE_DIRECTORY) {

				if (		strcmp(gp->sb.name, ".") == 0
						|| strcmp(gp->sb.name, "..") == 0) {

					continue;
				}

				if (N >= GP_FILE_DIR_MAX)
					break;

				sprintf(gp->sbuf[0], "/%s", gp->sb.name);

				if (strlen(gp->sbuf[0]) < PLOT_STRING_MAX) {

					strcpy(gp->d_names[N], gp->sbuf[0]);

					gpTextLeftCrop(gp->pl, gp->sbuf[1], gp->sb.name,
							gp->layout_menu_dir_margin);

					pad = len - utf8_length(gp->sbuf[1]);

					sprintf(sfmt, "/%%s%%%ds %%16s", pad);
					sprintf(gp->sbuf[0], sfmt, gp->sbuf[1],
							"", gp->sb.time);

					strcpy(la, gp->sbuf[0]);
					la += strlen(la) + 1;

					N++;
				}
			}
		}

		dirent_rewind(&gp->sb);

		while (dirent_read(&gp->sb) == ENT_OK) {

			if (gp->sb.ntype == ENT_TYPE_REGULAR) {

				if (N >= GP_FILE_DIR_MAX)
					break;

				sprintf(gp->sbuf[0], "%s", gp->sb.name);

				if (strlen(gp->sbuf[0]) < PLOT_STRING_MAX) {

					strcpy(gp->d_names[N], gp->sbuf[0]);

					sprintf(gp->sbuf[0], "%s/%s", gp->cwd, gp->sb.name);

					kmg = 0;

					while (gp->sb.nsize >= 1024U) {

						gp->sb.nsize /= 1024U;
						++kmg;
					}

					gpTextLeftCrop(gp->pl, gp->sbuf[1], gp->sb.name,
							gp->layout_menu_dir_margin);

					pad = len - utf8_length(gp->sbuf[1]);

					sprintf(sfmt, " %%s%%%ds %%16s %%4d %%cb", pad);
					sprintf(gp->sbuf[0], sfmt, gp->sbuf[1], "",
							gp->sb.time, (int) gp->sb.nsize,
							" KMG??" [kmg]);

					strcpy(la, gp->sbuf[0]);
					la += strlen(la) + 1;

					N++;
				}
			}
		}

		dirent_close(&gp->sb);
	}

	*la = 0;
}

static int
gpDirWalk(gp_t *gp, int dir_N, int revert)
{
	const char		*file = gp->d_names[dir_N - 2];
	char			*eol;
	int			walk = 0;

	if (dir_N == 0) {

		editRaise(gp->ed, 9, gp->la->file_name_edit,
				gp->cwd, gp->mu->box_X, gp->mu->box_Y);

		gp->stat = GP_EDIT;
	}
	else if (dir_N == 1) {

		walk = 2;
	}
	else if (dir_N == 2) {

		if (gp->cwd[0] != 0 && gp->cwd_ok != 0) {

			eol = gp->cwd + strlen(gp->cwd) - 1;

			do {
				if (*eol == '/' || *eol == '\\')
					break;

				if (gp->cwd == eol)
					break;

				eol--;
			}
			while (1);

			if ((*eol == '/' || *eol == '\\') && strcmp(eol, "/..") != 0) {

				if (gp->cwd == eol) {

					*(eol + 1) = 0;
				}
				else {
					*eol = 0;
				}
			}
			else if (gp->cwd[0] == '.' && revert == 0) {

				if (gp->cwd[0] != 0) {

					eol = gp->cwd + strlen(gp->cwd) - 1;

					if (*eol == '/' || *eol == '\\')
						*eol = 0;
				}

				strcat(gp->cwd, file);
			}
		}
		else {
			if (revert == 0) {

				strcpy(gp->cwd, ".");
			}
		}

		walk = 1;
	}
	else {
		if (file[0] == '/') {

			if (gp->cwd[0] != 0) {

				eol = gp->cwd + strlen(gp->cwd) - 1;

				if (*eol == '/' || *eol == '\\')
					*eol = 0;
			}

			strcat(gp->cwd, file);

			walk = 1;
		}
		else {
			sprintf(gp->tempfile, "%s/%s", gp->cwd, file);

			gpUnifiedFileOpen(gp, gp->tempfile, 1);

			walk = 3;
		}
	}

	return walk;
}

static void
gpInsertColumn(gp_t *gp, char **la, int dN, int cN)
{
	read_t		*rd = gp->rd;

	sprintf(gp->sbuf[0], "[%3i] %.75s", cN, rd->data[dN].label[cN]);

	if (rd->data[dN].hint[cN] == DATA_HINT_FLOAT) {

		strcat(gp->sbuf[0], " (DEC)");
	}
	else if (rd->data[dN].hint[cN] == DATA_HINT_HEX) {

		strcat(gp->sbuf[0], " (HEX)");
	}
	else if (rd->data[dN].hint[cN] == DATA_HINT_OCT) {

		strcat(gp->sbuf[0], " (OCT)");
	}
	else {
		strcat(gp->sbuf[0], "      ");
	}

	strcpy(*la, gp->sbuf[0]);
	*la += strlen(*la) + 1;
}

static void
gpInsertDataset(gp_t *gp, char **la, int dN)
{
	read_t		*rd = gp->rd;
	char		*file, *sformat;

	file = rd->data[dN].file;
	file = (file[0] == '.' && file[1] == '/')
		? file + 2 : file;

	gpTextLeftCrop(gp->pl, gp->sbuf[1], file, gp->layout_menu_dataset_margin);

	if (rd->data[dN].format == FORMAT_NONE) {

		sformat = "NONE  ";
	}
	else if (rd->data[dN].format == FORMAT_PLAIN_STDIN) {

		sformat = "STDIN ";
	}
	else if (rd->data[dN].format == FORMAT_PLAIN_TEXT) {

		sformat = "TEXT  ";
	}
	else if (rd->data[dN].format == FORMAT_BINARY_FLOAT) {

		sformat = "FLOAT ";
	}
	else if (rd->data[dN].format == FORMAT_BINARY_DOUBLE) {

		sformat = "DOUBLE";
	}
	else {
		sformat = "LEGACY";
	}

	sprintf(gp->sbuf[0], "[%2i] %6s %s", dN, sformat, gp->sbuf[1]);

	strcpy(*la, gp->sbuf[0]);
	*la += strlen(*la) + 1;
}

static void
gpMakeColumnSelectMenu(gp_t *gp, int dN)
{
	read_t		*rd = gp->rd;
	char		*la = gp->la_menu;
	int		sN, cN = 0;

	sprintf(gp->sbuf[0], "[%3i] ", -1);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	while (cN < (rd->data[dN].column_N + PLOT_SUBTRACT)) {

		if (cN < rd->data[dN].column_N) {

			gpInsertColumn(gp, &la, dN, cN);
		}
		else {
			sN = cN - rd->data[dN].column_N;

			sprintf(gp->sbuf[0], "[%3i] %c ", cN,
					" TFSEPRAXHDCBLM?" [gp->pl->data[dN].sub[sN].busy]);

			if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_TIME_MEDIAN) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %i, %i)",
						gp->pl->data[dN].sub[sN].op.median.column_1,
						gp->pl->data[dN].sub[sN].op.median.length,
						gp->pl->data[dN].sub[sN].op.median.unwrap);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_DATA_MEDIAN) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %i)",
						gp->pl->data[dN].sub[sN].op.median.column_1,
						gp->pl->data[dN].sub[sN].op.median.column_2);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_SCALE) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %.2E, %.2E)",
						gp->pl->data[dN].sub[sN].op.scale.column_1,
						gp->pl->data[dN].sub[sN].op.scale.scale,
						gp->pl->data[dN].sub[sN].op.scale.offset);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_RESAMPLE) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i) (%i, %i, %i)",
						gp->pl->data[dN].sub[sN].op.resample.column_X,
						gp->pl->data[dN].sub[sN].op.resample.in_data_N,
						gp->pl->data[dN].sub[sN].op.resample.in_column_X,
						gp->pl->data[dN].sub[sN].op.resample.in_column_Y);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_POLYFIT) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %i, %i)",
						gp->pl->data[dN].sub[sN].op.polyfit.column_X,
						gp->pl->data[dN].sub[sN].op.polyfit.column_Y,
						gp->pl->data[dN].sub[sN].op.polyfit.poly_N1);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_BINARY_SUBTRACTION
					|| gp->pl->data[dN].sub[sN].busy == SUBTRACT_BINARY_ADDITION
					|| gp->pl->data[dN].sub[sN].busy == SUBTRACT_BINARY_MULTIPLICATION
					|| gp->pl->data[dN].sub[sN].busy == SUBTRACT_BINARY_HYPOTENUSE) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %i)",
						gp->pl->data[dN].sub[sN].op.binary.column_1,
						gp->pl->data[dN].sub[sN].op.binary.column_2);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_DIFFERENCE
					|| gp->pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_CUMULATIVE) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i)",
						gp->pl->data[dN].sub[sN].op.filter.column_1);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_BITMASK) {

				int	bf[2] = { 0, (int) gp->pl->data[dN].sub[sN].op.filter.gain };

				bf[0] = bf[1] & 0xFFU;
				bf[1] = bf[1] >> 8;

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %i, %i)",
						gp->pl->data[dN].sub[sN].op.filter.column_1,
						(int) bf[0], (int) bf[1]);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_LOW_PASS) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %.2E)",
						gp->pl->data[dN].sub[sN].op.filter.column_1,
						gp->pl->data[dN].sub[sN].op.filter.gain);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_MEDIAN) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %i)",
						gp->pl->data[dN].sub[sN].op.median.column_1,
						(int) gp->pl->data[dN].sub[sN].op.median.length);
			}

			strcpy(la, gp->sbuf[0]);
			la += strlen(la) + 1;
		}

		cN += 1;
	}

	*la = 0;
}

static void
gpMakeDatasetSelectMenu(gp_t *gp)
{
	char		*la = gp->la_menu;
	int		dN = 0, len, fnlen, fnlen_max;

	len = gpScreenLength(gp->pl) - gp->layout_menu_dataset_margin;
	len = (len < gp->layout_menu_dataset_minimal)
		? gp->layout_menu_dataset_minimal : len;

	fnlen_max = 0;

	do {
		gpInsertDataset(gp, &la, dN);

		fnlen = utf8_length(gp->sbuf[0]);
		fnlen_max = (fnlen_max < fnlen) ? fnlen : fnlen_max;

		dN += 1;

		if (dN > (PLOT_DATASET_MAX - 1))
			break;
	}
	while (1);

	gpTextSepFill(gp->sbuf[1], (fnlen_max > len) ? fnlen_max : len);

	strcpy(la, gp->sbuf[1]);
	la += strlen(la) + 1;

	*la = 0;
}

static void
gpMakeDatasetMenu(gp_t *gp)
{
	plot_t		*pl = gp->pl;
	read_t		*rd = gp->rd;
	char		*la = gp->la_menu;
	int		N, cN, gN, dN, len, fnlen, unwrap, opdata;
	int		mbUSAGE, mbRAW, mbCACHE, lzPC;

	len = gpScreenLength(gp->pl) - gp->layout_menu_dataset_margin;
	len = (len < gp->layout_menu_dataset_minimal)
		? gp->layout_menu_dataset_minimal : len;

	if (len < 42) {

		len = 42;
	}

	dN = gp->data_N;

	if (rd->data[dN].format == FORMAT_NONE) {

		for (N = 0; N < PLOT_DATASET_MAX; ++N) {

			if (rd->data[N].format != FORMAT_NONE) {

				dN = N;
				break;
			}
		}

		gp->data_N = dN;
	}

	gpInsertDataset(gp, &la, dN);

	fnlen = utf8_length(gp->sbuf[0]);

	gpTextSepFill(gp->sbuf[1], (fnlen > len) ? fnlen : len);

	strcpy(la, gp->sbuf[1]);
	la += strlen(la) + 1;

	cN = readGetTimeColumn(rd, dN);

	sprintf(gp->sbuf[0], gp->la->dataset_menu[0], cN);
	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	strcpy(gp->sbuf[2], "   ");
	strcpy(gp->sbuf[3], "   ");

	unwrap = 0;
	opdata = 0;

	if (cN >= -1 && pl->data[dN].map != NULL) {

		gN = pl->data[dN].map[cN];

		if (gN >= 0 && gN < PLOT_GROUP_MAX) {

			if (pl->group[gN].op_time_median != 0) {

				sprintf(gp->sbuf[2], "%3i", pl->group[gN].length);

				unwrap = pl->group[gN].op_time_unwrap;
				opdata = pl->group[gN].op_time_opdata;
			}

			if (pl->group[gN].op_scale != 0) {

				gpTextFloat(pl, gp->sbuf[3], pl->group[gN].scale);
				gpTextFloat(pl, gp->sbuf[0], pl->group[gN].offset);

				strcat(gp->sbuf[3], " ");
				strcat(gp->sbuf[3], gp->sbuf[0]);
			}
		}
	}

	sprintf(gp->sbuf[0], gp->la->dataset_menu[1], gp->sbuf[2]);
	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	sprintf(gp->sbuf[0], gp->la->dataset_menu[2], (unwrap != 0) ? " X " : "   ");
	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	sprintf(gp->sbuf[0], gp->la->dataset_menu[3], (opdata != 0) ? " X " : "   ");
	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	sprintf(gp->sbuf[0], gp->la->dataset_menu[4], gp->sbuf[3]);
	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	mbUSAGE = plotDataMemoryUsage(pl, dN) / 1048576UL;
	mbRAW = plotDataMemoryUncompressed(pl, dN) / 1048576UL;
	mbCACHE = plotDataMemoryCached(pl, dN) / 1048576UL;

	lzPC = (mbRAW != 0) ? 100U * mbUSAGE / mbRAW : 0;

	sprintf(gp->sbuf[0], gp->la->dataset_menu[5],
			rd->data[dN].length_N, mbUSAGE, lzPC, mbCACHE);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	strcpy(la, gp->la->dataset_menu[6]);
	la += strlen(la) + 1;

	strcpy(la, gp->sbuf[1]);
	la += strlen(la) + 1;

	cN = 0;

	while (cN < rd->data[dN].column_N) {

		gpInsertColumn(gp, &la, dN, cN);

		cN += 1;
	}

	*la = 0;
}

static void
gpMakeConfigurationMenu(gp_t *gp)
{
	read_t		*rd = gp->rd;
	char		*eol, *la = gp->la_menu;
	FILE		*fd;
	int		line_N;

	fd = unified_fopen(gp->rcfile, "r");

	if (fd == NULL) {

		ERROR("fopen(\"%s\"): %s\n", gp->rcfile, strerror(errno));
	}
	else {
		gpTextLeftCrop(gp->pl, gp->sbuf[1], gp->rcfile, gp->layout_menu_page_margin);

		sprintf(gp->sbuf[0], "[%s]", gp->sbuf[1]);

		strcpy(la, gp->sbuf[0]);
		la += strlen(la) + 1;

		gpTextSepFill(gp->sbuf[1], utf8_length(gp->sbuf[0]));

		strcpy(la, gp->sbuf[1]);
		la += strlen(la) + 1;

		line_N = 0;

		while (fgets(gp->sbuf[0], sizeof(gp->sbuf[0]), fd) != NULL) {

			eol = gp->sbuf[0] + strlen(gp->sbuf[0]) - 1;

			while (		eol != gp->sbuf[0]
					&& strchr(rd->mk_config.lend, *eol) != 0) {

				*eol = 0;
				--eol;
			}

			if (strlen(gp->sbuf[0]) != 0) {

				strcpy(gp->d_names[line_N], gp->sbuf[0]);
				sprintf(gp->sbuf[1], " %s", gp->sbuf[0]);

				strcpy(la, gp->sbuf[1]);
				la += strlen(la) + 1;

				line_N++;

				if (line_N >= GP_FILE_DIR_MAX - 2)
					break;
			}
		}

		gp->d_names[line_N][0] = 0;

		strcpy(la, " ...");
		la += strlen(la) + 1;

		line_N++;
		gp->d_names[line_N][0] = 0;

		line_N++;
		gp->d_names[line_N][0] = 0;

		fclose(fd);
	}

	*la = 0;
}

static void
gpWriteNewConfiguration(gp_t *gp)
{
	FILE		*fd;
	int		line_N;

	fd = unified_fopen(gp->rcfile, "w");

	if (fd == NULL) {

		ERROR("fopen(\"%s\"): %s\n", gp->rcfile, strerror(errno));
	}
	else {
		line_N = 0;

		do {
			if (gp->d_names[line_N][0] != 0) {

				fprintf(fd, "%s\n", gp->d_names[line_N]);
			}
			else if (gp->d_names[line_N + 1][0] == 0)
				break;

			line_N++;

			if (line_N >= GP_FILE_DIR_MAX)
				break;
		}
		while (1);

		fclose(fd);
	}
}

static void
gpMakeAboutMenu(gp_t *gp)
{
	char		*la = gp->la_menu;

	strcpy(la, "Graph Plotter is a tool to analyse numerical data");
	la += strlen(la) + 1;

	gpTextSepFill(gp->sbuf[0], 54);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	strcpy(la, "License: GPLv3");
	la += strlen(la) + 1;

	strcpy(la, "Build: " __DATE__);
	la += strlen(la) + 1;

	sprintf(la, "SDL: %d.%d.%d", SDL_MAJOR_VERSION,
			SDL_MINOR_VERSION, SDL_PATCHLEVEL);
	la += strlen(la) + 1;

	sprintf(la, "SDL_ttf: %d.%d.%d", SDL_TTF_MAJOR_VERSION,
			SDL_TTF_MINOR_VERSION, SDL_TTF_PATCHLEVEL);
	la += strlen(la) + 1;

	sprintf(la, "SDL_image: %d.%d.%d", SDL_IMAGE_MAJOR_VERSION,
			SDL_IMAGE_MINOR_VERSION, SDL_IMAGE_PATCHLEVEL);
	la += strlen(la) + 1;

	gpTextSepFill(gp->sbuf[0], 54);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	strcpy(la, "* https://sourceforge.net/projects/graph-plotter");
	la += strlen(la) + 1;

	strcpy(la, "* https://github.com/rombrew/gp");
	la += strlen(la) + 1;

	*la = 0;
}

static void
gpTakeScreen(gp_t *gp)
{
	if (gp->screen_take == GP_TAKE_PNG) {

		if (IMG_SavePNG(gp->surface, gp->tempfile) == 0) {

			ERROR("Screen was saved to \"%s\"\n", gp->tempfile);
		}
	}
	else if (gp->screen_take == GP_TAKE_SVG) {

		svgClose((svg_t *) gp->surface->userdata);
		gp->surface->userdata = NULL;

		ERROR("Figure was saved to \"%s\"\n", gp->tempfile);
	}
	else if (gp->screen_take == GP_TAKE_CSV) {

		plotFigureExportCSV(gp->pl, gp->tempfile);

		ERROR("CSV table was saved to \"%s\"\n", gp->tempfile);
	}

	gp->screen_take = GP_TAKE_NONE;
}

#ifdef _WINDOWS
static void
legacy_SetClipboard(SDL_Surface *surface)
{
	long		length;
	void		*mBMP;

	SDL_RWops	*rwops;
	HGLOBAL		hDIB;

	length = surface->pitch * surface->h + 1048576UL;
	mBMP = malloc(length);

	if (mBMP == NULL) {

		ERROR("Unable to allocate BMP memory\n");
		return ;
	}

	rwops = SDL_RWFromMem(mBMP, length);
	SDL_SaveBMP_RW(surface, rwops, 0);

	SDL_RWseek(rwops, 0UL, RW_SEEK_END);
	length = SDL_RWtell(rwops);

	SDL_RWclose(rwops);

	length -= sizeof(BITMAPFILEHEADER);
	hDIB = GlobalAlloc(GMEM_MOVEABLE, length);

	memcpy(GlobalLock(hDIB), (const char *) mBMP
			+ sizeof(BITMAPFILEHEADER), length);
	GlobalUnlock(hDIB);

	free(mBMP);

	if (OpenClipboard(NULL) != 0) {

		EmptyClipboard();
		SetClipboardData(CF_DIB, hDIB);
		CloseClipboard();
	}
	else {
		GlobalFree(hDIB);
	}
}
#endif /* _WINDOWS */

static void
gpYankScreen(gp_t *gp)
{
	if (gp->screen_yank == 1) {

#ifdef _WINDOWS
		legacy_SetClipboard(gp->surface);
#endif /* _WINDOWS */
	}

	gp->screen_yank = 0;
}


static void
gpMenuHandle(gp_t *gp, int menu_N, int item_N)
{
	draw_t		*dw = gp->dw;
	plot_t		*pl = gp->pl;
	read_t		*rd = gp->rd;
	menu_t		*mu = gp->mu;
	edit_t		*ed = gp->ed;

	int		N;

	if (menu_N == 1) {

		switch (item_N) {

			case 0:
				menuRaise(mu, 101, gp->la->global_zoom_menu,
						mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 1:
				editRaise(ed, 1, gp->la->page_label_edit,
						rd->page[rd->page_N].title,
						mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 2:
				sprintf(gp->sbuf[0], "%s/g%if%i.png", rd->screenpath,
						rd->page_N, gp->pl->legend_N);

				N = 1;

				while (gpFileExist(gp->sbuf[0]) != 0) {

					if (N >= 100) {

						ERROR("Failed to find free file name\n");
						break;
					}

					sprintf(gp->sbuf[0], "%s/g%if%i_%i.png", rd->screenpath,
							rd->page_N, gp->pl->legend_N, N);

					N++;
				}

				editRaise(ed, 7, gp->la->file_name_edit,
						gp->sbuf[0], mu->box_X, mu->box_Y);

				ed->list_fmt = ".png\0" ".svg\0" ".csv\0";

				gp->stat = GP_EDIT;
				break;

			case 3:
				if (gp->fullscreen) {

					SDL_SetWindowFullscreen(gp->window, SDL_WINDOW_RESIZABLE);
					gp->fullscreen = 0;
				}
				else {
					SDL_SetWindowFullscreen(gp->window, SDL_WINDOW_FULLSCREEN_DESKTOP);
					gp->fullscreen = 1;
				}
				break;

			case 4:
				menuRaise(mu, 103, gp->la->global_appearance_menu,
						mu->box_X, mu->box_Y);

				mu->mark[0].N = 0;
				mu->mark[0].subs = (rd->colorscheme == 0) ? "Dark   "
						:  (rd->colorscheme == 1) ? "Light  " : "Graysca";

				mu->mark[1].N = 1;
				mu->mark[1].subs = (pl->layout_font_ttf == TTF_ID_ROBOTO_MONO_NORMAL) ? "Normal "
						:  (pl->layout_font_ttf == TTF_ID_ROBOTO_MONO_THIN) ? "Thin   " : "       ";

				mu->mark[2].N = 2;
				mu->mark[2].subs = (dw->antialiasing == DRAW_4X_MSAA)  ? "4x MSAA"
						:  (dw->antialiasing == DRAW_8X_MSAA)  ? "8x MSAA" : "Solid  ";

				mu->mark[3].N = 3;
				mu->mark[3].subs = (dw->blendfont != 0)  ? "Blended" : "Solid  ";

				sprintf(gp->sbuf[3], "%i      ", dw->thickness);

				mu->mark[4].N = 4;
				mu->mark[4].subs = gp->sbuf[3];

				sprintf(gp->sbuf[2], "%2i     ", pl->layout_font_pt);

				mu->mark[5].N = 5;
				mu->mark[5].subs = gp->sbuf[2];

				sprintf(gp->sbuf[2] + 40, "%2i     ", dw->gamma);

				mu->mark[6].N = 6;
				mu->mark[6].subs = gp->sbuf[2] + 40;

				sprintf(gp->sbuf[2] + 20, "%2i %2i  ", pl->mark_density, pl->mark_size);

				mu->mark[7].N = 7;
				mu->mark[7].subs = gp->sbuf[2] + 20;

				gp->stat = GP_MENU;
				break;

			case 5:
				menuRaise(mu, 102, gp->la->global_data_menu,
						mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 6:
				gpMakePageMenu(gp);

				if (strlen(gp->la_menu) > 0) {

					menuRaise(mu, 9, gp->la_menu, mu->box_X, mu->box_Y);
					menuSelect(mu, rd->page_N);

					mu->hidden_N[0] = rd->page_N - 1;
					gp->combine_on = GP_COMBINE_NONE;

					gp->stat = GP_MENU;
				}
				break;

			case 7:
				gpMakePageMenu(gp);

				if (strlen(gp->la_menu) > 0) {

					menuRaise(mu, 9, gp->la_menu, mu->box_X, mu->box_Y);
					menuSelect(mu, rd->page_N);

					mu->hidden_N[0]= rd->page_N - 1;
					gp->combine_on = GP_COMBINE_AXES_REMAP;

					gp->stat = GP_MENU;
				}
				break;

			case 8:
				gpMakePageMenu(gp);

				if (strlen(gp->la_menu) > 0) {

					menuRaise(mu, 9, gp->la_menu, mu->box_X, mu->box_Y);
					menuSelect(mu, rd->page_N);

					mu->hidden_N[0] = rd->page_N - 1;
					gp->combine_on = GP_COMBINE_NO_REMAP;

					gp->stat = GP_MENU;
				}
				break;

			case 9:
				plotFigureSubtractSwitch(pl, SUBTRACT_BINARY_SUBTRACTION);
				break;

			case 10:
				pl->mark_on = pl->mark_on ? 0 : 1;
				break;

			case 11:
				if (pl->slice_on == 0) {

					pl->slice_on = 1;
					pl->slice_axis_N = pl->hover_axis;

					plotSliceTrack(pl, gp->cur_X, gp->cur_Y);
				}
				else {
					pl->data_box_on = DATA_BOX_FREE;

					pl->slice_on = 0;
					pl->slice_mode_N = 0;
				}
				break;

			case 12:
				if (pl->axis[PLOT_AXES_MAX - 1].compact != 0) {

					for (N = 0; N < PLOT_AXES_MAX; ++N)
						pl->axis[N].compact = 0;
				}
				else {
					for (N = 0; N < PLOT_AXES_MAX; ++N)
						pl->axis[N].compact = 1;
				}
				break;

			case 13:
				if (pl->axis[0].exponential != 0) {

					for (N = 0; N < PLOT_AXES_MAX; ++N)
						pl->axis[N].exponential = 0;
				}
				else {
					for (N = 0; N < PLOT_AXES_MAX; ++N)
						pl->axis[N].exponential = 1;
				}
				break;

			case 14:
				gp->screen_yank = 1;
				break;

			case 15:
				menuRaise(mu, 104, gp->la->global_lang_menu,
						mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 16:
				gpMakeAboutMenu(gp);

				menuRaise(mu, 99, gp->la_menu, mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 17:
				gp->done = 1;
				break;
		}
	}
	else if (menu_N == 101) {

		switch (item_N) {

			case 0:
				plotAxisScaleAuto(pl, pl->on_X);
				plotAxisScaleAuto(pl, pl->on_Y);
				break;

			case 1:
				plotAxisScaleEqual(pl);
				break;

			case 2:
				plotAxisScaleGridAlign(pl);
				break;

			case 3:
				plotAxisScaleStacked(pl, 0);
				break;
		}
	}
	else if (menu_N == 102) {

		switch (item_N) {

			case 0:
				readDataReload(rd);

				if (gp->shift_on != 0) {

					plotAxisScaleLock(pl, LOCK_FREE);
				}
				else {
					plotAxisScaleLock(pl, LOCK_AUTO);
				}
				break;

			case 1:
				plotDataSubtractAlternate(rd->pl);
				break;

			case 2:
				gpMakeDirMenu(gp);

				menuRaise(mu, 1021, gp->la_menu, mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 3:
				gpMakeDatasetMenu(gp);

				menuRaise(mu, 1022, gp->la_menu, mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 4:
				menuRaise(mu, 1023, gp->la->global_config_menu, mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;
		}
	}
	else if (menu_N == 1021) {

		if (item_N != -1) {

			switch (gpDirWalk(gp, item_N, 0)) {

				case 1:
					gpMakeDirMenu(gp);

					menuRaise(mu, 1021, gp->la_menu, mu->box_X, mu->box_Y);
					gp->stat = GP_MENU;
					break;

				case 2:
					menuResume(mu);
					gp->stat = GP_MENU;
					break;

				case 3:
					for (N = 0; N < MENU_OPTION_MAX; ++N) {

						if (mu->hidden_N[N] < 0) {

							mu->hidden_N[N] = item_N;
							break;
						}
					}

					menuResume(mu);
					gp->stat = GP_MENU;
					break;

				default:
					break;
			}
		}
	}
	else if (menu_N == 1022) {

		if (item_N != -1) {

			int			gN, cN;

			switch (item_N) {

				case 0:
					gpMakeDatasetSelectMenu(gp);

					menuRaise(mu, 10221, gp->la_menu, mu->box_X, mu->box_Y);
					gp->stat = GP_MENU;
					break;

				case 2:
					gpMakeColumnSelectMenu(gp, gp->data_N);

					menuRaise(mu, 10222, gp->la_menu, mu->box_X, mu->box_Y);
					gp->stat = GP_MENU;
					break;

				case 3:
					cN = readGetTimeColumn(rd, gp->data_N);

					if (cN >= -1) {

						gN = pl->data[gp->data_N].map[cN];

						if (gN < 0) {

							gN = gp->data_N + (PLOT_GROUP_MAX - PLOT_DATASET_MAX);

							plotGroupAdd(pl, gp->data_N, gN, cN);
						}

						if (pl->group[gN].op_time_median == 0) {

							plotGroupMedian(pl, gN, 15, 0, 0);
						}

						sprintf(gp->sbuf[0], "%i", pl->group[gN].length);

						gp->grp_N = gN;

						editRaise(ed, 17, gp->la->median_unwrap_edit,
								gp->sbuf[0], mu->box_X, mu->box_Y);

						gp->stat = GP_EDIT;
					}
					else {
						gpMakeDatasetMenu(gp);

						menuResume(mu);
						menuLayout(mu);

						gp->stat = GP_MENU;
					}
					break;

				case 4:
					cN = readGetTimeColumn(rd, gp->data_N);

					if (cN >= -1) {

						gN = pl->data[gp->data_N].map[cN];

						if (gN >= 0) {

							pl->group[gN].op_time_unwrap =
								(pl->group[gN].op_time_unwrap != 0) ? 0 : 1;
						}
					}

					gpMakeDatasetMenu(gp);

					menuResume(mu);
					menuLayout(mu);

					gp->stat = GP_MENU;
					break;

				case 5:
					cN = readGetTimeColumn(rd, gp->data_N);

					if (cN >= -1) {

						gN = pl->data[gp->data_N].map[cN];

						if (gN >= 0) {

							pl->group[gN].op_time_opdata =
								(pl->group[gN].op_time_opdata != 0) ? 0 : 1;
						}
					}

					gpMakeDatasetMenu(gp);

					menuResume(mu);
					menuLayout(mu);

					gp->stat = GP_MENU;
					break;

				case 6:
					cN = readGetTimeColumn(rd, gp->data_N);

					if (cN >= -1) {

						gN = pl->data[gp->data_N].map[cN];

						if (gN < 0) {

							gN = gp->data_N + (PLOT_GROUP_MAX - PLOT_DATASET_MAX);

							plotGroupAdd(pl, gp->data_N, gN, cN);
						}

						if (pl->group[gN].op_scale == 0) {

							plotGroupScale(pl, gN, 1, 1., 0.);
						}

						gpTextFloat(pl, gp->sbuf[0], pl->group[gN].scale);
						gpTextFloat(pl, gp->sbuf[3], pl->group[gN].offset);

						strcat(gp->sbuf[0], " ");
						strcat(gp->sbuf[0], gp->sbuf[3]);

						gp->grp_N = gN;

						editRaise(ed, 11, gp->la->scale_offset_edit,
								gp->sbuf[0], mu->box_X, mu->box_Y);

						gp->stat = GP_EDIT;
					}
					else {
						gpMakeDatasetMenu(gp);

						menuResume(mu);
						menuLayout(mu);

						gp->stat = GP_MENU;
					}
					break;

				case 7:
					sprintf(gp->sbuf[0], "%i", rd->data[gp->data_N].length_N);

					editRaise(ed, 12, gp->la->length_edit,
							gp->sbuf[0], mu->box_X, mu->box_Y);

					gp->stat = GP_EDIT;
					break;

				case 8:
					readDatasetClean(rd, gp->data_N);

					gpMakeDatasetMenu(gp);

					menuResume(mu);
					menuLayout(mu);

					gp->stat = GP_MENU;
					break;

				case 1:
				case 9:
					menuResume(mu);
					gp->stat = GP_MENU;
					break;

				default:

					if (item_N >= 10) {

						readToggleHint(rd, gp->data_N, item_N - 10);
					}

					gpMakeDatasetMenu(gp);

					menuResume(mu);
					menuLayout(mu);

					gp->stat = GP_MENU;
					break;
			}
		}
	}
	else if (menu_N == 10221) {

		if (item_N != -1) {

			if (		item_N < PLOT_DATASET_MAX
					&& item_N >= 0) {

				gp->data_N = item_N;
			}

			gpMakeDatasetMenu(gp);

			menuRaise(mu, 1022, gp->la_menu, mu->box_X, mu->box_Y);
			gp->stat = GP_MENU;
		}
	}
	else if (menu_N == 10222) {

		if (item_N != -1) {

			readSetTimeColumn(rd, gp->data_N, item_N - 1);

			gpMakeDatasetMenu(gp);

			menuRaise(mu, 1022, gp->la_menu, mu->box_X, mu->box_Y);
			gp->stat = GP_MENU;
		}
	}
	else if (menu_N == 1023) {

		switch (item_N) {

			case 0:
				menuRaise(mu, 10231, gp->la->cancel_menu, mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 1:
				menuRaise(mu, 10232, gp->la->cancel_menu, mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 2:
				gpMakeConfigurationMenu(gp);

				menuRaise(mu, 10234, gp->la_menu, mu->box_X, mu->box_Y);

				mu->hidden_N[0] = 0;
				gp->stat = GP_MENU;
				break;
		}
	}
	else if (menu_N == 10231) {

		if (item_N == 1) {

			gpWriteFile(gp);
		}
	}
	else if (menu_N == 10232) {

		if (item_N == 1) {

			gpDefaultFile(gp);
		}
	}
	else if (menu_N == 10234) {

		if (item_N != -1) {

			if (item_N >= 2) {

				gp->line_N = item_N - 2;

				editRaise(ed, 10, gp->la_menu,
						gp->d_names[gp->line_N],
						mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
			}
			else {
				menuResume(mu);
				gp->stat = GP_MENU;
			}
		}
	}
	else if (menu_N == 103) {

		switch (item_N) {

			case 0:
				rd->colorscheme = (rd->colorscheme < 2)
					? rd->colorscheme + 1 : 0;

				schemeFill(gp->sch, rd->colorscheme);
				break;

			case 1:
				gpFontToggle(gp, 1, 0);
				break;

			case 2:
				rd->fastdraw = 0;
				dw->antialiasing = (dw->antialiasing < DRAW_8X_MSAA)
					? dw->antialiasing + 1 : DRAW_SOLID;
				break;

			case 3:
				dw->blendfont = (dw->blendfont != 0) ? 0 : 1;
				break;

			case 4:
				dw->thickness = (dw->thickness < 2) ? dw->thickness + 1 : 0;
				break;

			case 5:
				sprintf(gp->sbuf[0], "%i", pl->layout_font_pt);

				editRaise(ed, 15, gp->la->font_size_edit,
						gp->sbuf[0], mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 6:
				sprintf(gp->sbuf[0], "%i", dw->gamma);

				editRaise(ed, 21, gp->la->gamma_edit,
						gp->sbuf[0], mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 7:
				sprintf(gp->sbuf[0], "%i %i", pl->mark_density, pl->mark_size);

				editRaise(ed, 20, gp->la->marker_density_edit,
						gp->sbuf[0], mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;
		}

		if (mu->clicked != 0 && gp->stat != GP_EDIT) {

			mu->mark[0].subs = (rd->colorscheme == 0) ? "Dark   "
					:  (rd->colorscheme == 1) ? "Light  " : "Graysca";
			mu->mark[1].subs = (pl->layout_font_ttf == TTF_ID_ROBOTO_MONO_NORMAL) ? "Normal "
					:  (pl->layout_font_ttf == TTF_ID_ROBOTO_MONO_THIN) ? "Thin   " : "       ";
			mu->mark[2].subs = (dw->antialiasing == DRAW_4X_MSAA)  ? "4x MSAA"
					:  (dw->antialiasing == DRAW_8X_MSAA)  ? "8x MSAA" : "Solid  ";
			mu->mark[3].subs = (dw->blendfont != 0)  ? "Blended" : "Solid  ";

			sprintf(gp->sbuf[3], "%i      ", dw->thickness);

			mu->mark[4].N = 4;
			mu->mark[4].subs = gp->sbuf[3];

			menuResume(mu);
			menuLayout(mu);

			gp->stat = GP_MENU;
		}
	}
	else if (menu_N == 104) {

		if (item_N != -1) {

			rd->language = item_N;
			langFill(gp->la, rd->language);
		}
	}
	else if (menu_N == 2) {

		switch (item_N) {

			case 0:
				menuRaise(mu, 201, gp->la->axis_zoom_menu, mu->box_X, mu->box_Y);

				if (pl->axis[gp->ax_N].slave != 0) {

					mu->hidden_N[0] = 0;
					mu->hidden_N[1] = 1;
					mu->hidden_N[2] = 2;
				}
				else if (	gp->ax_N == pl->on_X
						|| gp->ax_N == pl->on_Y) {

					mu->hidden_N[0] = 2;
				}

				gp->stat = GP_MENU;
				break;

			case 1:
				if (pl->axis[gp->ax_N].slave == 0) {

					if (pl->axis[gp->ax_N].busy == AXIS_BUSY_X) {

						plotAxisSlave(pl, gp->ax_N, pl->on_X, 0., 0.,
								AXIS_SLAVE_HOLD_AS_IS);
					}
					else if (pl->axis[gp->ax_N].busy == AXIS_BUSY_Y) {

						plotAxisSlave(pl, gp->ax_N, pl->on_Y, 0., 0.,
								AXIS_SLAVE_HOLD_AS_IS);
					}
				}
				else {
					plotAxisSlave(pl, gp->ax_N, -1, 0., 0., AXIS_SLAVE_DISABLE);
				}
				break;

			case 2:
				menuRaise(mu, 202, gp->la->cancel_menu, mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 3:
				pl->slice_on = 1;
				pl->slice_axis_N = gp->ax_N;

				plotSliceTrack(pl, gp->cur_X, gp->cur_Y);
				break;

			case 4:
				pl->axis[gp->ax_N].compact = pl->axis[gp->ax_N].compact ? 0 : 1;

				if (mu->clicked != 0) {

					mu->mark[0].subs = (pl->axis[gp->ax_N].compact == 0) ? " " : "X";

					menuResume(mu);
					gp->stat = GP_MENU;
				}
				break;

			case 5:
				pl->axis[gp->ax_N].exponential = pl->axis[gp->ax_N].exponential ? 0 : 1;

				if (mu->clicked != 0) {

					mu->mark[1].subs = (pl->axis[gp->ax_N].exponential == 0) ? " " : "X";

					menuResume(mu);
					gp->stat = GP_MENU;
				}
				break;

			case 6:
				pl->axis[gp->ax_N].lock_tick = pl->axis[gp->ax_N].lock_tick ? 0 : 1;

				if (pl->axis[gp->ax_N].lock_tick != 0) {

					plotAxisScaleGridLock(pl, gp->ax_N);
				}

				if (mu->clicked != 0) {

					mu->mark[2].subs = (pl->axis[gp->ax_N].lock_tick == 0) ? " " : "X";

					menuResume(mu);
					gp->stat = GP_MENU;
				}
				break;


			case 7:
				editRaise(ed, 3, gp->la->axis_label_edit,
						pl->axis[gp->ax_N].label,
						mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;
		}
	}
	else if (menu_N == 201) {

		switch (item_N) {

			case 0:
				plotAxisScaleAuto(pl, gp->ax_N);
				break;

			case 1:
				if (pl->axis[gp->ax_N].busy == AXIS_BUSY_X) {

					plotAxisScaleAutoCond(pl, gp->ax_N, -1);
				}
				else if (pl->axis[gp->ax_N].busy == AXIS_BUSY_Y) {

					plotAxisScaleAutoCond(pl, gp->ax_N, -1);
				}
				break;

			case 2:
				if (		pl->axis[gp->ax_N].busy == AXIS_BUSY_X
						&& pl->axis[gp->ax_N].slave == 0) {

					pl->axis[gp->ax_N].scale = pl->axis[pl->on_X].scale;
					pl->axis[gp->ax_N].offset = pl->axis[pl->on_X].offset;
				}
				else if (	pl->axis[gp->ax_N].busy == AXIS_BUSY_Y
						&& pl->axis[gp->ax_N].slave == 0) {

					pl->axis[gp->ax_N].scale = pl->axis[pl->on_Y].scale;
					pl->axis[gp->ax_N].offset = pl->axis[pl->on_Y].offset;
				}
				break;

			case 3:
				gpTextFloat(pl, gp->sbuf[0], pl->axis[gp->ax_N].scale);
				gpTextFloat(pl, gp->sbuf[3], pl->axis[gp->ax_N].offset);

				strcat(gp->sbuf[0], " ");
				strcat(gp->sbuf[0], gp->sbuf[3]);

				editRaise(ed, 4, gp->la->scale_offset_edit,
						gp->sbuf[0], mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;
		}
	}
	else if (menu_N == 202) {

		if (item_N == 1) {

			plotAxisRemove(pl, gp->ax_N);
		}
	}
	else if (menu_N == 3) {

		switch (item_N) {

			case 0:
				plotFigureMoveAxes(pl, gp->fig_N);
				break;

			case 1:
				plotFigureMakeIndividualAxes(pl, gp->fig_N);
				break;

			case 2:
				menuRaise(mu, 301, gp->la->cancel_menu,
						mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 3:
				menuRaise(mu, 302, gp->la->figure_operation_menu,
						mu->box_X, mu->box_Y);

				N = plotFigureSelected(pl);

				if (N < 2) {

					mu->hidden_N[0] = 2;
					mu->hidden_N[1] = 6;
					mu->hidden_N[2] = 7;
					mu->hidden_N[3] = 8;
					mu->hidden_N[4] = 9;
				}
				else if (N != 2) {

					mu->hidden_N[0] = 6;
					mu->hidden_N[1] = 7;
					mu->hidden_N[2] = 8;
					mu->hidden_N[3] = 9;
				}

				gp->stat = GP_MENU;
				break;

			case 4:
				menuRaise(mu, 303, gp->la->figure_edit_menu,
						mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;
		}
	}
	else if (menu_N == 301) {

		if (item_N == 1) {

			plotFigureRemove(pl, gp->fig_N);
		}
	}
	else if (menu_N == 302) {

		int		config[3];

		switch (item_N) {

			case 0:
				N = plotGetFreeFigure(pl);

				if (N < 0) {

					ERROR("Unable to get free figure to duplicate\n");
					break ;
				}

				plotFigureAdd(pl, N, pl->figure[gp->fig_N].data_N,
						pl->figure[gp->fig_N].column_X,
						pl->figure[gp->fig_N].column_Y,
						pl->figure[gp->fig_N].axis_X,
						pl->figure[gp->fig_N].axis_Y,
						pl->figure[gp->fig_N].label);

				pl->figure[N].hidden = pl->figure[gp->fig_N].hidden;
				pl->figure[N].drawing = pl->figure[gp->fig_N].drawing;
				pl->figure[N].width = pl->figure[gp->fig_N].width;
				break;

			case 1:
				if (plotFigureSubtractGetMedianConfig(pl, gp->fig_N, config) >= 0) {

					sprintf(gp->sbuf[0], "%d %d %d", config[0], config[1], config[2]);
				}
				else {
					sprintf(gp->sbuf[0], "15 0 0");
				}

				editRaise(ed, 18, gp->la->median_unwrap_edit,
						gp->sbuf[0], mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 2:
				plotFigureSubtractResample(pl, gp->fig_N);
				break;

			case 3:
				editRaise(ed, 5, gp->la->scale_offset_edit,
						"1 0", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 4:
				editRaise(ed, 6, gp->la->scale_offset_edit,
						"-1 0", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 5:
				if (plotDataBoxPolyfit(pl, gp->fig_N) == 0) {

					editRaise(ed, 16, gp->la->polynomial_edit,
							"1", mu->box_X, mu->box_Y);

					gp->stat = GP_EDIT;
				}
				break;

			case 6:
				plotFigureSubtractSwitch(pl, SUBTRACT_BINARY_SUBTRACTION);
				break;

			case 7:
				plotFigureSubtractSwitch(pl, SUBTRACT_BINARY_ADDITION);
				break;

			case 8:
				plotFigureSubtractSwitch(pl, SUBTRACT_BINARY_MULTIPLICATION);
				break;

			case 9:
				plotFigureSubtractSwitch(pl, SUBTRACT_BINARY_HYPOTENUSE);
				break;

			case 10:
				plotFigureSubtractFilter(pl, gp->fig_N, SUBTRACT_FILTER_DIFFERENCE, 0.);
				break;

			case 11:
				plotFigureSubtractFilter(pl, gp->fig_N, SUBTRACT_FILTER_CUMULATIVE, 0.);
				break;

			case 12:
				editRaise(ed, 8, gp->la->bit_number_edit,
						"0", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 13:
				editRaise(ed, 13, gp->la->low_pass_edit,
						"0.1", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 14:
				editRaise(ed, 19, gp->la->median_unwrap_edit,
						"15", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;
		}
	}
	else if (menu_N == 303) {

		switch (item_N) {

			case 0:
				menuRaise(mu, 3031, gp->la->figure_edit_drawing_menu,
						mu->box_X, mu->box_Y);

				N = pl->figure[gp->fig_N].width;

				if (N == 1) { N = 0; }
				else if (N == 2) { N = 1; }
				else if (N == 4) { N = 2; }
				else if (N == 6) { N = 3; }
				else { N = -1; }

				if (N >= 0) {

					N += pl->figure[gp->fig_N].drawing * 4;
				}

				mu->hidden_N[0] = N;
				gp->stat = GP_MENU;
				break;

			case 1:
				sprintf(gp->sbuf[0], "%i", pl->figure[gp->fig_N].width);

				editRaise(ed, 14, gp->la->figure_thickness_edit,
						gp->sbuf[0], mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 2:
				menuRaise(mu, 3032, gp->la->figure_edit_color_menu,
						mu->box_X, mu->box_Y);
				mu->hidden_N[0] = gp->fig_N;
				mu->colorful = 1;
				gp->stat = GP_MENU;
				break;

			case 3:
				gpMakeColumnSelectMenu(gp, pl->figure[gp->fig_N].data_N);

				menuRaise(mu, 3033, gp->la_menu, mu->box_X, mu->box_Y);
				mu->hidden_N[0] = pl->figure[gp->fig_N].column_X + 1;
				gp->stat = GP_MENU;
				break;

			case 4:
				gpMakeColumnSelectMenu(gp, pl->figure[gp->fig_N].data_N);

				menuRaise(mu, 3034, gp->la_menu, mu->box_X, mu->box_Y);
				mu->hidden_N[0] = pl->figure[gp->fig_N].column_Y + 1;
				gp->stat = GP_MENU;
				break;

			case 5:
				editRaise(ed, 2, gp->la->figure_label_edit,
						pl->figure[gp->fig_N].label,
						mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;
		}
	}
	else if (menu_N == 3031) {

		if (item_N != -1) {

			N = item_N % 4;

			if (N == 0) { N = 1; }
			else if (N == 1) { N = 2; }
			else if (N == 2) { N = 4; }
			else if (N == 3) { N = 6; }

			pl->figure[gp->fig_N].drawing = item_N / 4;
			pl->figure[gp->fig_N].width = N;
		}
	}
	else if (menu_N == 3032) {

		if (item_N != -1) {

			plotFigureExchange(pl, gp->fig_N, item_N);
		}
	}
	else if (menu_N == 3033) {

		if (item_N != -1) {

			pl->figure[gp->fig_N].column_X = item_N - 1;
		}
	}
	else if (menu_N == 3034) {

		if (item_N != -1) {

			pl->figure[gp->fig_N].column_Y = item_N - 1;
		}
	}
	else if (menu_N == 4) {

		switch (item_N) {

			case 0:
				menuRaise(mu, 401, gp->la->figure_edit_drawing_menu,
						mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 1:
				gp->fig_N = -1;

				editRaise(ed, 5, gp->la->scale_offset_edit,
						"1 0", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 2:
				gp->fig_N = -1;

				editRaise(ed, 6, gp->la->scale_offset_edit,
						"-1 0", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 3:
				sprintf(gp->sbuf[0], "%s/g%if%i.csv", rd->screenpath,
						rd->page_N, gp->pl->legend_N);

				N = 1;

				while (gpFileExist(gp->sbuf[0]) != 0) {

					if (N >= 100) {

						ERROR("Failed to find free file name\n");
						break;
					}

					sprintf(gp->sbuf[0], "%s/g%if%i_%i.csv", rd->screenpath,
							rd->page_N, gp->pl->legend_N, N);

					N++;
				}

				editRaise(ed, 7, gp->la->file_name_edit,
						gp->sbuf[0], mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 4:
				pl->transparency = pl->transparency ? 0 : 1;

				if (mu->clicked != 0) {

					mu->mark[0].subs = (pl->transparency == 0) ? " " : "X";

					menuResume(mu);
					gp->stat = GP_MENU;
				}
				break;

			case 5:
				pl->legend_hidden = pl->legend_hidden ? 0 : 1;

				if (mu->clicked != 0) {

					mu->mark[1].subs = (pl->legend_hidden == 0) ? " " : "X";

					menuResume(mu);
					gp->stat = GP_MENU;
				}
				break;
		}
	}
	else if (menu_N == 401) {

		if (item_N != -1) {

			for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

				if (pl->figure[N].busy != 0) {

					int		lN;

					lN = item_N % 4;

					if (lN == 0) { lN = 1; }
					else if (lN == 1) { lN = 2; }
					else if (lN == 2) { lN = 4; }
					else if (lN == 3) { lN = 6; }

					pl->figure[N].drawing = item_N / 4;
					pl->figure[N].width = lN;
				}
			}
		}
	}
	else if (menu_N == 5) {

		switch (item_N) {

			case 0:
				pl->data_box_on = DATA_BOX_FREE;

				pl->slice_on = 0;
				pl->slice_mode_N = 0;
				break;

			case 1:
				plotDataBoxCopyClipboard(pl);
				break;
		}
	}
	else if (menu_N == 9) {

		N = item_N + 1;

		if (gp->combine_on == GP_COMBINE_AXES_REMAP) {

			readCombinePage(rd, N, 1);

			for (N = 1; N < MENU_OPTION_MAX; ++N) {

				if (mu->hidden_N[N] < 0) {

					mu->hidden_N[N] = item_N;
					break;
				}
			}

			menuResume(mu);
			gp->stat = GP_MENU;
		}
		else if (gp->combine_on == GP_COMBINE_NO_REMAP) {

			readCombinePage(rd, N, 0);

			for (N = 1; N < MENU_OPTION_MAX; ++N) {

				if (mu->hidden_N[N] < 0) {

					mu->hidden_N[N] = item_N;
					break;
				}
			}

			menuResume(mu);
			gp->stat = GP_MENU;
		}
		else {
			readSelectPage(rd, N);
		}
	}
}

static void
gpEditHandle(gp_t *gp, int edit_N, const char *text)
{
	scheme_t	*sch = gp->sch;
	lang_t		*la = gp->la;
	draw_t		*dw = gp->dw;
	plot_t		*pl = gp->pl;
	read_t		*rd = gp->rd;
	menu_t		*mu = gp->mu;
	svg_t		*g;

	double		scale, offset, gain;
	int		len, n;

	if (edit_N == 1) {

		strcpy(rd->page[rd->page_N].title, text);
	}
	else if (edit_N == 2) {

		strcpy(pl->figure[gp->fig_N].label, text);
	}
	else if (edit_N == 3) {

		strcpy(pl->axis[gp->ax_N].label, text);
	}
	else if (edit_N == 4) {

		n = sscanf(text, "%le %le", &scale, &offset);

		if (n == 2) {

			pl->axis[gp->ax_N].scale = scale;
			pl->axis[gp->ax_N].offset = offset;
		}
	}
	else if (edit_N == 5 || edit_N == 6) {

		int		fN, aBUSY;

		n = sscanf(text, "%le %le", &scale, &offset);

		if (n != 0) {

			if (n == 1) {

				offset = 0.;
			}

			aBUSY = (edit_N == 5) ? AXIS_BUSY_X : AXIS_BUSY_Y;

			if (gp->fig_N < 0) {

				for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

					if (		pl->figure[fN].busy != 0
							&& pl->figure[fN].hidden == 0) {

						plotFigureSubtractScale(pl, fN, aBUSY,
								scale, offset);
					}
				}
			}
			else {
				plotFigureSubtractScale(pl, gp->fig_N,
						aBUSY, scale, offset);
			}
		}
	}
	else if (edit_N == 7) {

		const char	*filetype;

		strcpy(gp->tempfile, text);

		filetype = gp->tempfile + strlen(gp->tempfile);

		if (strlen(gp->tempfile) > 4) {

			filetype += - 4;
		}

		if (strcmp(filetype, ".png") == 0) {

			gp->screen_take = GP_TAKE_PNG;
		}
		else if (strcmp(filetype, ".svg") == 0) {

			g = (void *) svgOpenNew(gp->tempfile,
					gp->surface->w, gp->surface->h);

			g->font_family = "monospace";
			g->font_pt = pl->layout_font_pt;

			gp->surface->userdata = (void *) g;
			gp->screen_take = GP_TAKE_SVG;
		}
		else if (strcmp(filetype, ".csv") == 0) {

			gp->screen_take = GP_TAKE_CSV;
		}
	}
	else if (edit_N == 8) {

		int		args[2];

		n = sscanf(text, "%d %d", &args[0], &args[1]);

		if (n != 0) {

			if (n == 1) {

				args[1] = args[0];
			}

			if (		args[0] >= 0 && args[0] <= 64
					&& args[1] >= 0 && args[1] <= 64
					&& args[0] <= args[1]) {

				plotFigureSubtractFilter(pl, gp->fig_N,
						SUBTRACT_FILTER_BITMASK,
						(double) (args[0] | (args[1] << 8)));
			}
		}
	}
	else if (edit_N == 9) {

		strcpy(gp->cwd, text);

		gpMakeDirMenu(gp);

		menuRaise(mu, 1021, gp->la_menu, mu->box_X, mu->box_Y);
		gp->stat = GP_MENU;
	}
	else if (edit_N == 10) {

		sprintf(gp->sbuf[0], "%s\n", text);
		readConfigIN(rd, gp->sbuf[0], 1);

		gpFontLayout(gp);
		gpScreenLayout(gp);

		langFill(la, rd->language);
		schemeFill(sch, rd->colorscheme);

		strcpy(gp->d_names[gp->line_N], text);

		gpWriteNewConfiguration(gp);
		gpMakeConfigurationMenu(gp);

		menuRaise(mu, 10234, gp->la_menu, mu->box_X, mu->box_Y);
		menuSelect(mu, gp->line_N + 2);

		mu->hidden_N[0] = 0;
		gp->stat = GP_MENU;
	}
	else if (edit_N == 11) {

		n = sscanf(text, "%le %le", &scale, &offset);

		if (n == 2) {

			plotGroupScale(pl, gp->grp_N, 1, scale, offset);
		}
		else {
			plotGroupScale(pl, gp->grp_N, 0, 1., 0.);
		}

		gpMakeDatasetMenu(gp);

		menuRaise(mu, 1022, gp->la_menu, mu->box_X, mu->box_Y);
		gp->stat = GP_MENU;
	}
	else if (edit_N == 12) {

		n = sscanf(text, "%d", &len);

		if (n == 1) {

			if (len >= 0) {

				rd->data[gp->data_N].length_N = len;

				if (len >= 1) {

					plotDataResize(pl, gp->data_N, len);
				}
			}
		}

		gpMakeDatasetMenu(gp);

		menuRaise(mu, 1022, gp->la_menu, mu->box_X, mu->box_Y);
		gp->stat = GP_MENU;
	}
	else if (edit_N == 13) {

		n = sscanf(text, "%le", &gain);

		if (n == 1) {

			plotFigureSubtractFilter(pl, gp->fig_N,
					SUBTRACT_FILTER_LOW_PASS, gain);
		}
	}
	else if (edit_N == 14) {

		n = sscanf(text, "%i", &len);

		if (n == 1) {

			if (len >= 0 && len <= 16) {

				pl->figure[gp->fig_N].width = len;
			}
		}
	}
	else if (edit_N == 15) {

		n = sscanf(text, "%d", &len);

		if (n == 1) {

			gpFontToggle(gp, 0, len);

			sprintf(gp->sbuf[2], "%2i     ", pl->layout_font_pt);

			mu->mark[5].subs = gp->sbuf[2];

			menuResume(mu);
			menuLayout(mu);

			gp->stat = GP_MENU;
		}
	}
	else if (edit_N == 16) {

		int		args[2];

		n = sscanf(text, "%d %d", &args[0], &args[1]);

		if (n != 0) {

			if (n == 1) {

				args[1] = args[0];
				args[0] = 0;
			}

			if (		args[0] >= 0 && args[0] <= PLOT_POLYFIT_MAX
					&& args[1] >= 0 && args[1] <= PLOT_POLYFIT_MAX
					&& args[0] <= args[1]) {

				plotFigureSubtractPolyfit(pl, gp->fig_N,
						args[0], args[1]);
			}
		}
	}
	else if (edit_N == 17) {

		n = sscanf(text, "%d", &len);

		if (n != 0) {

			int		unwrap = pl->group[gp->grp_N].op_time_unwrap;
			int		opdata = pl->group[gp->grp_N].op_time_opdata;

			if (len >= 1 && len <= PLOT_MEDIAN_MAX) {

				plotGroupMedian(pl, gp->grp_N, len,
						unwrap, opdata);
			}
			else {
				plotGroupMedian(pl, gp->grp_N, 0, 0, 0);
			}
		}
		else {
			plotGroupMedian(pl, gp->grp_N, 0, 0, 0);
		}

		gpMakeDatasetMenu(gp);

		menuRaise(mu, 1022, gp->la_menu, mu->box_X, mu->box_Y);
		gp->stat = GP_MENU;
	}
	else if (edit_N == 18) {

		int		args[2];

		n = sscanf(text, "%d %d %d", &len, &args[0], &args[1]);

		if (n != 0) {

			if (len >= 0 && len <= PLOT_MEDIAN_MAX) {

				plotFigureSubtractTimeMedian(pl, gp->fig_N,
						len, args[0], args[1]);
			}
		}
	}
	else if (edit_N == 19) {

		n = sscanf(text, "%d", &len);

		if (n != 0) {

			if (len >= 3 && len <= PLOT_MEDIAN_MAX) {

				plotFigureSubtractFilter(pl, gp->fig_N,
						SUBTRACT_FILTER_MEDIAN, (double) len);
			}
		}
	}
	else if (edit_N == 20) {

		int		args[2];

		n = sscanf(text, "%d %d", &args[0], &args[1]);

		if (n == 2) {

			if (args[0] > 0 && args[0] < 100) {

				pl->mark_density = args[0];
				pl->mark_count = 0;
			}

			if (args[1] > 0 && args[1] < 100) {

				pl->mark_size = args[1];
				pl->mark_count = 0;
			}

			sprintf(gp->sbuf[2] + 20, "%2i %2i  ", pl->mark_density, pl->mark_size);

			mu->mark[7].subs = gp->sbuf[2] + 20;
		}

		menuResume(mu);
		menuLayout(mu);

		gp->stat = GP_MENU;
	}
	else if (edit_N == 21) {

		n = sscanf(text, "%d", &len);

		if (n != 0) {

			if (len > 0 && len < 1000) {

				dw->gamma = len;

				drawGamma(gp->dw);
			}

			sprintf(gp->sbuf[2] + 40, "%2i     ", dw->gamma);

			mu->mark[6].subs = gp->sbuf[2] + 40;
		}

		menuResume(mu);
		menuLayout(mu);

		gp->stat = GP_MENU;
	}
}

static void
gpEventHandle(gp_t *gp, const SDL_Event *ev)
{
	plot_t		*pl = gp->pl;
	read_t		*rd = gp->rd;
	menu_t		*mu = gp->mu;
	edit_t		*ed = gp->ed;

	double		fmin, fmax;
	int		N;

	if (ev->type == SDL_QUIT)
		gp->done = 1;

	else if (ev->type == SDL_WINDOWEVENT) {

		if (ev->window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {

			gp->fb = SDL_GetWindowSurface(gp->window);

			if (		gp->fb->w != gp->surface->w
					|| gp->fb->h != gp->surface->h) {

				SDL_FreeSurface(gp->surface);

				gp->surface = SDL_CreateRGBSurfaceWithFormat(0, gp->fb->w,
						gp->fb->h, 32, SDL_PIXELFORMAT_XRGB8888);
			}

			gpScreenLayout(gp);
		}
		else if (ev->window.event == SDL_WINDOWEVENT_CLOSE) {

			gp->done = 1;
		}
	}
	else if (ev->type == SDL_KEYDOWN) {

		if (ev->key.keysym.sym == SDLK_LCTRL || ev->key.keysym.sym == SDLK_RCTRL) {

			gp->ctrl_on = 1;
		}
		else if (ev->key.keysym.sym == SDLK_LSHIFT || ev->key.keysym.sym == SDLK_RSHIFT) {

			gp->shift_on = 1;
			pl->shift_on = 1;
		}
	}
	else if (ev->type == SDL_KEYUP) {

		if (ev->key.keysym.sym == SDLK_LCTRL || ev->key.keysym.sym == SDLK_RCTRL) {

			gp->ctrl_on = 0;
		}
		else if (ev->key.keysym.sym == SDLK_LSHIFT || ev->key.keysym.sym == SDLK_RSHIFT) {

			gp->shift_on = 0;
			pl->shift_on = 0;
		}
	}

	if (gp->stat == GP_IDLE) {

		if (ev->type == SDL_KEYDOWN) {

			if (ev->key.keysym.sym == SDLK_a) {

				if (pl->hover_axis != -1) {

					gp->ax_N = pl->hover_axis;

					gpMenuHandle(gp, 201, 0);
				}
				else {
					gpMenuHandle(gp, 101, 0);
				}
			}
			else if (ev->key.keysym.sym == SDLK_q) {

				gpMenuHandle(gp, 101, 1);
			}
			else if (ev->key.keysym.sym == SDLK_g) {

				gpMenuHandle(gp, 101, 2);
			}
			else if (ev->key.keysym.sym == SDLK_w) {

				gpMenuHandle(gp, 101, 3);
			}
			else if (ev->key.keysym.sym == SDLK_u) {

				gpMenuHandle(gp, 102, 0);
			}
			else if (ev->key.keysym.sym == SDLK_j) {

				gpMenuHandle(gp, 102, 1);
			}
			else if (ev->key.keysym.sym == SDLK_o) {

				mu->box_X = -1;
				mu->box_Y = -1;

				gpMenuHandle(gp, 102, 2);
			}
			else if (ev->key.keysym.sym == SDLK_p) {

				mu->box_X = -1;
				mu->box_Y = -1;

				gpMenuHandle(gp, 1, 2);
			}
			else if (ev->key.keysym.sym == SDLK_f) {

				gpMenuHandle(gp, 1, 3);
			}
			else if (ev->key.keysym.sym == SDLK_l) {

				mu->clicked = 0;

				gpMenuHandle(gp, 103, 0);
			}
			else if (ev->key.keysym.sym == SDLK_d) {

				mu->box_X = -1;
				mu->box_Y = -1;

				gpMenuHandle(gp, 102, 3);
			}
			else if (ev->key.keysym.sym == SDLK_i) {

				gp->i_show_fps = gp->i_show_fps ? 0 : 1;
			}
			else if (	ev->key.keysym.sym == SDLK_PAGEUP
					|| ev->key.keysym.sym == SDLK_UP) {

				readSelectPage(rd, rd->page_N - 1);
			}
			else if (	ev->key.keysym.sym == SDLK_PAGEDOWN
					|| ev->key.keysym.sym == SDLK_DOWN) {

				readSelectPage(rd, rd->page_N + 1);
			}
			else if (ev->key.keysym.sym == SDLK_RETURN) {

				mu->box_X = -1;
				mu->box_Y = -1;

				gpMenuHandle(gp, 1, 6);
			}
			else if (ev->key.keysym.sym == SDLK_c) {

				mu->box_X = -1;
				mu->box_Y = -1;

				gpMenuHandle(gp, 1, 7);
			}
			else if (ev->key.keysym.sym == SDLK_b) {

				mu->box_X = -1;
				mu->box_Y = -1;

				gpMenuHandle(gp, 1, 8);
			}
			else if (ev->key.keysym.sym == SDLK_s) {

				if (pl->hover_axis != -1) {

					N = pl->hover_axis;

					if (N != pl->on_X && N != pl->on_Y) {

						gp->ax_N = N;

						gpMenuHandle(gp, 2, 1);
					}
				}
			}
			else if (ev->key.keysym.sym == SDLK_x) {

				if (pl->hover_axis != -1) {

					if (		pl->hover_axis != pl->on_X
							&& pl->hover_axis != pl->on_Y) {

						mu->box_X = gp->cur_X;
						mu->box_Y = gp->cur_Y;

						gp->ax_N = pl->hover_axis;

						gpMenuHandle(gp, 2, 2);
					}
				}
				else if (pl->hover_figure != -1) {

					mu->box_X = gp->cur_X;
					mu->box_Y = gp->cur_Y;

					gp->fig_N = pl->hover_figure;

					gpMenuHandle(gp, 3, 2);
				}
			}
			else if (ev->key.keysym.sym == SDLK_r) {

				gpMenuHandle(gp, 1, 9);
			}
			else if (ev->key.keysym.sym == SDLK_m) {

				gpMenuHandle(gp, 1, 10);
			}
			else if (ev->key.keysym.sym == SDLK_t) {

				gpMenuHandle(gp, 1, 11);
			}
			else if (ev->key.keysym.sym == SDLK_k) {

				if (pl->hover_axis != -1) {

					mu->clicked = 0;
					gp->ax_N = pl->hover_axis;

					gpMenuHandle(gp, 2, 4);
				}
				else {
					gpMenuHandle(gp, 1, 12);
				}
			}
			else if (ev->key.keysym.sym == SDLK_e) {

				if (pl->hover_axis != -1) {

					mu->clicked = 0;
					gp->ax_N = pl->hover_axis;

					gpMenuHandle(gp, 2, 5);
				}
				else {
					gpMenuHandle(gp, 1, 13);
				}
			}
			else if (ev->key.keysym.sym == SDLK_y) {

				gpMenuHandle(gp, 1, 14);
			}
		}
		else if (ev->type == SDL_MOUSEBUTTONDOWN) {

			if (ev->button.button == SDL_BUTTON_LEFT) {

				gp->cur_X = ev->button.x;
				gp->cur_Y = ev->button.y;

				do {
					N = plotLegendGetByClick(pl, gp->cur_X, gp->cur_Y);

					if (N >= 0) {

						pl->figure[N].hidden = pl->figure[N].hidden ? 0 : 1;

						break;
					}

					N = plotLegendBoxGetByClick(pl, gp->cur_X, gp->cur_Y);

					if (N == 0) {

						gp->box_X = ev->button.x - pl->legend_X;
						gp->box_Y = ev->button.y - pl->legend_Y;

						gp->stat = GP_MOVING;
						gp->legend_drag = 1;
						break;
					}

					N = plotDataBoxGetByClick(pl, gp->cur_X, gp->cur_Y);

					if (N == 0) {

						gp->box_X = ev->button.x - pl->data_box_X;
						gp->box_Y = ev->button.y - pl->data_box_Y;

						gp->stat = GP_MOVING;
						gp->data_box_drag = 1;
						break;
					}

					N = plotAxisGetByClick(pl, gp->cur_X, gp->cur_Y);

					if (N >= 0) {

						if (pl->axis[N].busy == AXIS_BUSY_X
								&& pl->axis[N].slave == 0) {

							pl->on_X = N;
							gp->ax_N = N;

							gp->stat = GP_MOVING;
							break;
						}
						else if (pl->axis[N].busy == AXIS_BUSY_Y
								&& pl->axis[N].slave == 0) {

							pl->on_Y = N;
							gp->ax_N = N;

							gp->stat = GP_MOVING;
							break;
						}
					}
					else {
						gp->box_X = ev->button.x;
						gp->box_Y = ev->button.y;
						gp->ax_N = -1;

						gp->stat = GP_MOVING;
						break;
					}
				}
				while (0);
			}
			else if (ev->button.button == SDL_BUTTON_RIGHT) {

				gp->cur_X = ev->button.x;
				gp->cur_Y = ev->button.y;

				do {
					if (gp->cur_Y < gp->layout_page_box) {

						menuRaise(mu, 1, gp->la->global_menu,
								gp->cur_X, gp->cur_Y);

						N = plotFigureSelected(pl);

						if (N != 2) {

							mu->hidden_N[0] = 9;
						}

#ifndef _WINDOWS
						mu->hidden_N[1] = 14;
#endif /* _WINDOWS */

						gp->stat = GP_MENU;
						break;
					}

					N = plotLegendGetByClick(pl, gp->cur_X, gp->cur_Y);

					if (N >= 0) {

						gp->fig_N = N;

						menuRaise(mu, 3, gp->la->figure_menu,
								gp->cur_X, gp->cur_Y);

						gp->stat = GP_MENU;
						break;
					}

					N = plotLegendBoxGetByClick(pl, gp->cur_X, gp->cur_Y);

					if (N == 0) {

						menuRaise(mu, 4, gp->la->legend_menu,
								gp->cur_X, gp->cur_Y);

						mu->mark[0].N = 4;
						mu->mark[0].subs = (pl->transparency == 0) ? " " : "X";

						mu->mark[1].N = 5;
						mu->mark[1].subs = (pl->legend_hidden == 0) ? " " : "X";

						gp->stat = GP_MENU;
						break;
					}

					N = plotDataBoxGetByClick(pl, gp->cur_X, gp->cur_Y);

					if (N == 0) {

						menuRaise(mu, 5, gp->la->databox_menu,
								gp->cur_X, gp->cur_Y);

						gp->stat = GP_MENU;
						break;
					}

					N = plotAxisGetByClick(pl, gp->cur_X, gp->cur_Y);

					if (N >= 0) {

						if (N != pl->on_X && N != pl->on_Y) {

							gp->ax_N = N;

							menuRaise(mu, 2, gp->la->axis_menu,
									gp->cur_X, gp->cur_Y);

							mu->mark[0].N = 4;
							mu->mark[0].subs = (pl->axis[N].compact == 0) ? " " : "X";
							mu->mark[1].N = 5;
							mu->mark[1].subs = (pl->axis[N].exponential == 0) ? " " : "X";
							mu->mark[2].N = 6;
							mu->mark[2].subs = (pl->axis[N].lock_tick == 0) ? " " : "X";

							gp->stat = GP_MENU;
							break;
						}
						else {
							gp->ax_N = N;

							menuRaise(mu, 2, gp->la->axis_menu,
									gp->cur_X, gp->cur_Y);

							mu->hidden_N[0] = 1;
							mu->hidden_N[1] = 2;

							mu->mark[0].N = 4;
							mu->mark[0].subs = (pl->axis[N].compact == 0) ? " " : "X";
							mu->mark[1].N = 5;
							mu->mark[1].subs = (pl->axis[N].exponential == 0) ? " " : "X";
							mu->mark[2].N = 6;
							mu->mark[2].subs = (pl->axis[N].lock_tick == 0) ? " " : "X";

							gp->stat = GP_MENU;
							break;
						}
					}
					else {
						if (clipBoxTest(&pl->viewport, gp->cur_X, gp->cur_Y)) {

							gp->box_X = ev->button.x;
							gp->box_Y = ev->button.y;

							gp->stat = GP_BOX_SELECT;
							break;
						}
						break;
					}
				}
				while (0);
			}
		}
		else if (ev->type == SDL_MOUSEWHEEL) {

			if (gp->shift_on != 0) {

				fmin =	  (ev->wheel.y > 0) ? 1.01
					: (ev->wheel.y < 0) ? 1. / 1.01 : 1.;
			}
			else {
				fmin =	  (ev->wheel.y > 0) ? 1.1
					: (ev->wheel.y < 0) ? 1. / 1.1 : 1.;
			}

			SDL_GetMouseState(&gp->cur_X, &gp->cur_Y);
			N = plotAxisGetByClick(pl, gp->cur_X, gp->cur_Y);

			if (N >= 0) {

				if (pl->axis[N].busy == AXIS_BUSY_X) {

					plotAxisScaleZoom(pl, N, gp->cur_X, fmin);
				}
				else if (pl->axis[N].busy == AXIS_BUSY_Y) {

					plotAxisScaleZoom(pl, N, gp->cur_Y, fmin);
				}
			}
			else {
				plotAxisScaleZoom(pl, pl->on_X, gp->cur_X, fmin);
				plotAxisScaleZoom(pl, pl->on_Y, gp->cur_Y, fmin);
			}
		}
		else if (ev->type == SDL_MOUSEMOTION) {

			gp->cur_X = ev->motion.x;
			gp->cur_Y = ev->motion.y;

			plotLegendGetByClick(pl, gp->cur_X, gp->cur_Y);
			plotLegendBoxGetByClick(pl, gp->cur_X, gp->cur_Y);
			plotAxisGetByClick(pl, gp->cur_X, gp->cur_Y);

			if (pl->slice_on != 0) {

				plotSliceTrack(pl, gp->cur_X, gp->cur_Y);
			}

			if (pl->data_box_on != DATA_BOX_FREE) {

				plotDataBoxGetByClick(pl, gp->cur_X, gp->cur_Y);
			}

			gp->hover_box = (gp->cur_Y < gp->layout_page_box && gp->cur_Y > 0) ? 1 : 0;
		}
	}
	else if (gp->stat == GP_MOVING) {

		if (ev->type == SDL_KEYDOWN) {

			if (ev->key.keysym.sym == SDLK_ESCAPE) {

				gp->stat = GP_IDLE;
				gp->legend_drag = 0;
				gp->data_box_drag = 0;
				gp->ax_N = -1;
			}
		}
		else if (ev->type == SDL_MOUSEBUTTONUP) {

			if (ev->button.button == SDL_BUTTON_LEFT) {

				gp->stat = GP_IDLE;
				gp->legend_drag = 0;
				gp->data_box_drag = 0;
				gp->ax_N = -1;

				if (gp->box_X == ev->button.x && gp->box_Y == ev->button.y
						&& clipBoxTest(&pl->viewport, gp->box_X, gp->box_Y)) {

					if (pl->slice_on == 0) {

						gp->stat = GP_RANGE_SELECT;
					}
					else {
						plotSliceSwitch(pl);
					}
				}
			}
		}
		else if (ev->type == SDL_MOUSEMOTION) {

			if (gp->legend_drag) {

				pl->legend_X = ev->motion.x - gp->box_X;
				pl->legend_Y = ev->motion.y - gp->box_Y;

				gp->cur_X = ev->motion.x;
				gp->cur_Y = ev->motion.y;
			}
			else if (gp->data_box_drag) {

				pl->data_box_X = ev->motion.x - gp->box_X;
				pl->data_box_Y = ev->motion.y - gp->box_Y;

				gp->cur_X = ev->motion.x;
				gp->cur_Y = ev->motion.y;
			}
			else if (gp->ax_N < 0) {

				fmin = (double) (ev->motion.x - gp->cur_X);
				fmax = (double) (ev->motion.y - gp->cur_Y);

				if (gp->shift_on != 0) {

					fmin *= 0.1;
					fmax *= 0.1;
				}

				plotAxisScaleMove(pl, pl->on_X, fmin);
				plotAxisScaleMove(pl, pl->on_Y, fmax);

				gp->cur_X = ev->motion.x;
				gp->cur_Y = ev->motion.y;
			}
			else if (pl->axis[gp->ax_N].busy == AXIS_BUSY_X) {

				fmin = (double) (ev->motion.x - gp->cur_X);

				if (gp->shift_on != 0) {

					fmin *= 0.1;
				}

				plotAxisScaleMove(pl, gp->ax_N, fmin);

				gp->cur_X = ev->motion.x;
			}
			else if (pl->axis[gp->ax_N].busy == AXIS_BUSY_Y) {

				fmin = (double) (ev->motion.y - gp->cur_Y);

				if (gp->shift_on != 0) {

					fmin *= 0.1;
				}

				plotAxisScaleMove(pl, gp->ax_N, fmin);

				gp->cur_Y = ev->motion.y;
			}
		}
	}
	else if (gp->stat == GP_RANGE_SELECT) {

		if (ev->type == SDL_KEYDOWN) {

			if (ev->key.keysym.sym == SDLK_ESCAPE)
				gp->stat = GP_IDLE;
		}
		else if (ev->type == SDL_MOUSEBUTTONDOWN) {

			gp->cur_X = ev->button.x;
			gp->cur_Y = ev->button.y;

			if (ev->button.button == SDL_BUTTON_RIGHT)
				gp->stat = GP_IDLE;
			else if (ev->button.button == SDL_BUTTON_LEFT) {

				if (gp->box_X != gp->cur_X || gp->box_Y != gp->cur_Y) {

					if (gp->box_X > gp->cur_X) {

						N = gp->box_X;
						gp->box_X = gp->cur_X;
						gp->cur_X = N;
					}

					if (gp->box_Y < gp->cur_Y) {

						N = gp->box_Y;
						gp->box_Y = gp->cur_Y;
						gp->cur_Y = N;
					}

					if (abs(gp->box_X - gp->cur_X) >= abs(gp->box_Y - gp->cur_Y)) {

						fmin = plotAxisConvBackward(pl, pl->on_X, gp->box_X);
						fmax = plotAxisConvBackward(pl, pl->on_X, gp->cur_X);

						plotAxisScaleManual(pl, pl->on_X, fmin, fmax);

						if (gp->shift_on == 0) {

							plotAxisScaleAutoCond(pl, pl->on_Y, pl->on_X);
						}
					}
					else {
						fmin = plotAxisConvBackward(pl, pl->on_Y, gp->box_Y);
						fmax = plotAxisConvBackward(pl, pl->on_Y, gp->cur_Y);

						plotAxisScaleManual(pl, pl->on_Y, fmin, fmax);

						if (gp->shift_on == 0) {

							plotAxisScaleAutoCond(pl, pl->on_X, pl->on_Y);
						}
					}

					pl->axis[pl->on_X].lock_scale = LOCK_FREE;
					pl->axis[pl->on_Y].lock_scale = LOCK_FREE;
				}

				gp->stat = GP_IDLE;
			}
		}
		else if (ev->type == SDL_MOUSEMOTION) {

			gp->cur_X = ev->motion.x;
			gp->cur_Y = ev->motion.y;
		}
	}
	else if (gp->stat == GP_BOX_SELECT) {

		if (ev->type == SDL_KEYDOWN) {

			if (ev->key.keysym.sym == SDLK_ESCAPE)
				gp->stat = GP_IDLE;
		}
		else if (ev->type == SDL_MOUSEBUTTONDOWN) {

			if (ev->button.button == SDL_BUTTON_RIGHT)
				gp->stat = GP_IDLE;
		}
		else if (ev->type == SDL_MOUSEBUTTONUP) {

			if (ev->button.button == SDL_BUTTON_RIGHT) {

				if (gp->box_X != gp->cur_X && gp->box_Y != gp->cur_Y) {

					if (gp->box_X > gp->cur_X) {

						N = gp->box_X;
						gp->box_X = gp->cur_X;
						gp->cur_X = N;
					}

					if (gp->box_Y < gp->cur_Y) {

						N = gp->box_Y;
						gp->box_Y = gp->cur_Y;
						gp->cur_Y = N;
					}

					fmin = plotAxisConvBackward(pl, pl->on_X, gp->box_X);
					fmax = plotAxisConvBackward(pl, pl->on_X, gp->cur_X);

					plotAxisScaleManual(pl, pl->on_X, fmin, fmax);

					fmin = plotAxisConvBackward(pl, pl->on_Y, gp->box_Y);
					fmax = plotAxisConvBackward(pl, pl->on_Y, gp->cur_Y);

					plotAxisScaleManual(pl, pl->on_Y, fmin, fmax);

					pl->axis[pl->on_X].lock_scale = LOCK_FREE;
					pl->axis[pl->on_Y].lock_scale = LOCK_FREE;
				}
				else {
					gpMenuHandle(gp, 101, 0);
				}

				gp->stat = GP_IDLE;
			}
		}
		else if (ev->type == SDL_MOUSEMOTION) {

			gp->cur_X = ev->motion.x;
			gp->cur_Y = ev->motion.y;
		}
	}
	else if (gp->stat == GP_MENU) {

		SDL_StartTextInput();

		if (ev->type == SDL_KEYDOWN) {

			if (ev->key.keysym.sym == SDLK_ESCAPE) {

				gp->stat = GP_IDLE;

				menuHalt(mu);
				SDL_StopTextInput();
			}
			else if (ev->key.keysym.sym == SDLK_UP) {

				menuEvent(mu, MENU_EVNO_ARROW_UP, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_DOWN) {

				menuEvent(mu, MENU_EVNO_ARROW_DOWN, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_PAGEUP) {

				menuEvent(mu, MENU_EVNO_PAGE_UP, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_PAGEDOWN) {

				menuEvent(mu, MENU_EVNO_PAGE_DOWN, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_HOME) {

				menuEvent(mu, MENU_EVNO_HOME, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_END) {

				menuEvent(mu, MENU_EVNO_END, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_RETURN) {

				menuEvent(mu, MENU_EVNO_RETURN, gp->cur_X, gp->cur_Y);

				if (mu->clicked) {

					gp->stat = GP_IDLE;

					menuHalt(mu);
					SDL_StopTextInput();

					if (mu->clicked_N != -1) {

						gpMenuHandle(gp, mu->id, mu->clicked_N);
					}
				}
			}
			else if (ev->key.keysym.sym == SDLK_BACKSPACE) {

				menuEvent(mu, MENU_EVNO_BACKSPACE, gp->cur_X, gp->cur_Y);
			}
		}
		else if (ev->type == SDL_TEXTINPUT) {

			menuEventText(mu, ev->text.text);
		}
		else if (ev->type == SDL_MOUSEBUTTONDOWN) {

			if (ev->button.button == SDL_BUTTON_LEFT) {

				gp->cur_X = ev->button.x;
				gp->cur_Y = ev->button.y;

				menuEvent(mu, MENU_EVNO_CLICK, gp->cur_X, gp->cur_Y);

				if (mu->clicked) {

					gp->stat = GP_IDLE;

					menuHalt(mu);
					SDL_StopTextInput();

					if (mu->clicked_N != -1) {

						gpMenuHandle(gp, mu->id, mu->clicked_N);
					}
				}
			}
			else if (ev->button.button == SDL_BUTTON_RIGHT) {

				gp->stat = GP_IDLE;

				menuHalt(mu);
				SDL_StopTextInput();
			}
		}
		else if (ev->type == SDL_MOUSEBUTTONUP) {

			if (ev->button.button == SDL_BUTTON_LEFT) {

				gp->cur_X = ev->button.x;
				gp->cur_Y = ev->button.y;

				menuEvent(mu, MENU_EVNO_UNCLICK, gp->cur_X, gp->cur_Y);
			}
		}
		else if (ev->type == SDL_MOUSEMOTION) {

			gp->cur_X = ev->motion.x;
			gp->cur_Y = ev->motion.y;

			menuEvent(mu, MENU_EVNO_MOTION, gp->cur_X, gp->cur_Y);
		}
		else if (ev->type == SDL_MOUSEWHEEL) {

			if (ev->wheel.y > 0) {

				menuEvent(mu, MENU_EVNO_SCROLL_UP, gp->cur_X, gp->cur_Y);
			}
			else if (ev->wheel.y < 0) {

				menuEvent(mu, MENU_EVNO_SCROLL_DOWN, gp->cur_X, gp->cur_Y);
			}
		}
	}
	else if (gp->stat == GP_EDIT) {

		SDL_StartTextInput();

		if (ev->type == SDL_KEYDOWN) {

			if (ev->key.keysym.sym == SDLK_ESCAPE) {

				gp->stat = GP_IDLE;

				editHalt(ed);
				SDL_StopTextInput();
			}
			else if (ev->key.keysym.sym == SDLK_LEFT) {

				editEvent(ed, EDIT_EVNO_ARROW_LEFT, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_RIGHT) {

				editEvent(ed, EDIT_EVNO_ARROW_RIGHT, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_RETURN) {

				editEvent(ed, EDIT_EVNO_RETURN, gp->cur_X, gp->cur_Y);

				if (ed->entered) {

					gp->stat = GP_IDLE;

					editHalt(ed);
					SDL_StopTextInput();

					gpEditHandle(gp, ed->id, ed->text);
				}
			}
			else if (ev->key.keysym.sym == SDLK_BACKSPACE) {

				editEvent(ed, EDIT_EVNO_BACKSPACE, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_DELETE) {

				editEvent(ed, EDIT_EVNO_DELETE, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_x && gp->ctrl_on == 1) {

				editEvent(ed, EDIT_EVNO_CTRL_X, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_c && gp->ctrl_on == 1) {

				editEvent(ed, EDIT_EVNO_CTRL_C, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_v && gp->ctrl_on == 1) {

				editEvent(ed, EDIT_EVNO_CTRL_V, gp->cur_X, gp->cur_Y);
			}
			else if (ev->key.keysym.sym == SDLK_TAB) {

				editEvent(ed, EDIT_EVNO_TAB, gp->cur_X, gp->cur_Y);
			}
		}
		else if (ev->type == SDL_TEXTINPUT && gp->ctrl_on == 0) {

			editEventText(ed, ev->text.text);
		}
		else if (ev->type == SDL_MOUSEBUTTONDOWN) {

			if (ev->button.button == SDL_BUTTON_LEFT) {

				gp->cur_X = ev->button.x;
				gp->cur_Y = ev->button.y;

				editEvent(ed, EDIT_EVNO_CLICK, gp->cur_X, gp->cur_Y);
			}
		}
	}
}

static void
gpDrawRangeLight(SDL_Surface *surface, gp_t *gp)
{
	plot_t		*pl = gp->pl;

	SDL_LockSurface(surface);

	if (abs(gp->box_X - gp->cur_X) >= abs(gp->box_Y - gp->cur_Y)) {

		if (gp->box_X < gp->cur_X) {

			drawClipRect(surface, &pl->viewport, gp->box_X, pl->viewport.min_y,
					gp->cur_X, pl->viewport.max_y, pl->sch->plot_hidden);
		}
		else {
			drawClipRect(surface, &pl->viewport, gp->cur_X, pl->viewport.min_y,
					gp->box_X, pl->viewport.max_y, pl->sch->plot_hidden);
		}
	}
	else {
		if (gp->box_Y < gp->cur_Y) {

			drawClipRect(surface, &pl->viewport, pl->viewport.min_x, gp->box_Y,
					pl->viewport.max_x, gp->cur_Y, pl->sch->plot_hidden);
		}
		else {
			drawClipRect(surface, &pl->viewport, pl->viewport.min_x, gp->cur_Y,
					pl->viewport.max_x, gp->box_Y, pl->sch->plot_hidden);
		}
	}

	SDL_UnlockSurface(surface);
}

static void
gpDrawRangeSelect(SDL_Surface *surface, gp_t *gp)
{
	draw_t		*dw = gp->dw;
	plot_t		*pl = gp->pl;

	SDL_LockSurface(surface);

	if (abs(gp->box_X - gp->cur_X) >= abs(gp->box_Y - gp->cur_Y)) {

		drawDashReset(dw);
		drawDash(dw, surface, &pl->viewport, gp->box_X, pl->viewport.min_y,
				gp->box_X, pl->viewport.max_y, pl->sch->plot_text,
				pl->layout_fence_dash, pl->layout_fence_space);

		drawDashReset(dw);
		drawDash(dw, surface, &pl->viewport, gp->cur_X, pl->viewport.min_y,
				gp->cur_X, pl->viewport.max_y, pl->sch->plot_text,
				pl->layout_fence_dash, pl->layout_fence_space);
	}
	else {
		drawDashReset(dw);
		drawDash(dw, surface, &pl->viewport, pl->viewport.min_x, gp->box_Y,
				pl->viewport.max_x, gp->box_Y, pl->sch->plot_text,
				pl->layout_fence_dash, pl->layout_fence_space);

		drawDashReset(dw);
		drawDash(dw, surface, &pl->viewport, pl->viewport.min_x, gp->cur_Y,
				pl->viewport.max_x, gp->cur_Y, pl->sch->plot_text,
				pl->layout_fence_dash, pl->layout_fence_space);
	}

	SDL_UnlockSurface(surface);
}

static void
gpDrawBoxLight(SDL_Surface *surface, gp_t *gp)
{
	plot_t		*pl = gp->pl;
	int		min_X, min_Y, max_X, max_Y;

	SDL_LockSurface(surface);

	if (gp->box_X < gp->cur_X) {

		min_X = gp->box_X;
		max_X = gp->cur_X;
	}
	else {
		min_X = gp->cur_X;
		max_X = gp->box_X;
	}

	if (gp->box_Y < gp->cur_Y) {

		min_Y = gp->box_Y;
		max_Y = gp->cur_Y;
	}
	else {
		min_Y = gp->cur_Y;
		max_Y = gp->box_Y;
	}

	drawClipRect(surface, &pl->viewport, min_X, min_Y,
			max_X, max_Y, pl->sch->plot_hidden);

	SDL_UnlockSurface(surface);
}

static void
gpDrawBoxSelect(SDL_Surface *surface, gp_t *gp)
{
	draw_t		*dw = gp->dw;
	plot_t		*pl = gp->pl;

	SDL_LockSurface(surface);

	drawDash(dw, surface, &pl->viewport, gp->box_X, gp->box_Y,
			gp->cur_X, gp->box_Y, pl->sch->plot_text,
			pl->layout_fence_dash, pl->layout_fence_space);
	drawDash(dw, surface, &pl->viewport, gp->box_X, gp->box_Y,
			gp->box_X, gp->cur_Y, pl->sch->plot_text,
			pl->layout_fence_dash, pl->layout_fence_space);
	drawDash(dw, surface, &pl->viewport, gp->cur_X, gp->cur_Y,
			gp->cur_X, gp->box_Y, pl->sch->plot_text,
			pl->layout_fence_dash, pl->layout_fence_space);
	drawDash(dw, surface, &pl->viewport, gp->cur_X, gp->cur_Y,
			gp->box_X, gp->cur_Y, pl->sch->plot_text,
			pl->layout_fence_dash, pl->layout_fence_space);

	SDL_UnlockSurface(surface);
}

static void
gpFPSUpdate(gp_t *gp)
{
	gp->i_frames += 1;

	if (gp->i_clocked < gp->clock) {

		gp->i_FPS = gp->i_frames;
		gp->i_frames = 0;
		gp->i_clocked = gp->clock + 1000;
	}
}

gp_t *gp_Alloc()
{
	gp_t		*gp;
	scheme_t	*sch;
	lang_t		*la;
	draw_t		*dw;
	plot_t		*pl;
	read_t		*rd;
	menu_t		*mu;
	edit_t		*ed;

	gp = (gp_t *) calloc(1, sizeof(gp_t));

	sch = (scheme_t *) calloc(1, sizeof(scheme_t));
	gp->sch = sch;

	la = (lang_t *) calloc(1, sizeof(lang_t));
	gp->la = la;

	dw = (draw_t *) calloc(1, sizeof(draw_t));
	gp->dw = dw;

	dw->antialiasing = DRAW_4X_MSAA;
	dw->blendfont = 1;
	dw->thickness = 1;
	dw->gamma = 50;

	pl = plotAlloc(dw, sch);
	gp->pl = pl;

	rd = readAlloc(dw, pl);
	gp->rd = rd;
	pl->ld = rd;

	mu = menuAlloc(dw, sch);
	gp->mu = mu;

	ed = editAlloc(dw, sch);
	gp->ed = ed;

	gp->hinting = 2;

	gp->cwd[0] = '.';
	gp->cwd[1] = 0;

	gpFileGetPath(gp);

	if (gp->rcfile[0] != 0) {

		if (gpFileExist(gp->rcfile) == 0) {

			gpDefaultFile(gp);
		}

		if (gpFileExist(gp->rcfile) != 0) {

			readConfigGP(rd, gp->rcfile, 0);
		}
		else {
			gpFileGetLocal(gp);

			if (gpFileExist(gp->rcfile) == 0) {

				gpDefaultFile(gp);
			}

			if (gpFileExist(gp->rcfile) != 0) {

				readConfigGP(rd, gp->rcfile, 0);
			}
		}

		if (rd->config_version < GP_CONFIG_VERSION) {

			gpDefaultFile(gp);

			if (gpFileExist(gp->rcfile) != 0) {

				readConfigGP(rd, gp->rcfile, 0);
			}
		}
	}

	return gp;
}

void gp_Clean(gp_t *gp)
{
	scheme_t	*sch = gp->sch;
	lang_t		*la = gp->la;
	draw_t		*dw = gp->dw;
	plot_t		*pl = gp->pl;
	read_t		*rd = gp->rd;
	menu_t		*mu = gp->mu;
	edit_t		*ed = gp->ed;

	if (gp->surface != NULL) {

		SDL_FreeSurface(gp->surface);
	}

	if (gp->window != NULL) {

		SDL_DestroyWindow(gp->window);
	}

	plotClean(pl);
	readClean(rd);
	menuClean(mu);
	editClean(ed);

	free(sch);
	free(la);
	free(dw);
	free(gp);
}

void gp_TakeConfig(gp_t *gp, const char *config)
{
	readConfigIN(gp->rd, config, 0);
}

int gp_OpenWindow(gp_t *gp)
{
	scheme_t	*sch = gp->sch;
	lang_t		*la = gp->la;
	read_t		*rd = gp->rd;

	if (gp->window_ID != 0) {

		return gp->window_ID;
	}

	readConfigVerify(rd);

	gp->window = SDL_CreateWindow("GP", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
			rd->window_size_x, rd->window_size_y, SDL_WINDOW_RESIZABLE);

	if (gp->window == NULL) {

		ERROR("SDL_CreateWindow: %s\n", SDL_GetError());
	}

	gp->window_ID = SDL_GetWindowID(gp->window);

	if (gp->window_ID == 0) {

		ERROR("SDL_GetWindowID: %s\n", SDL_GetError());
	}

	SDL_SetWindowMinimumSize(gp->window, GP_MIN_SIZE_X, GP_MIN_SIZE_Y);
	SDL_StopTextInput();

	gp->fb = SDL_GetWindowSurface(gp->window);

	if (gp->fb == NULL) {

		ERROR("SDL_GetWindowSurface: %s\n", SDL_GetError());
	}

	gp->surface = SDL_CreateRGBSurfaceWithFormat(0, gp->fb->w,
			gp->fb->h, 32, SDL_PIXELFORMAT_XRGB8888);

	if (gp->surface == NULL) {

		ERROR("SDL_CreateRGBSurfaceWithFormat: %s\n", SDL_GetError());
	}

	gpFontHinting(gp);

	gpFontLayout(gp);
	gpScreenLayout(gp);

	gp->stat = GP_IDLE;
	gp->active = 1;

	langFill(la, rd->language);
	schemeFill(sch, rd->colorscheme);

	readSelectPage(rd, 1);

	drawGamma(gp->dw);

	return gp->window_ID;
}

void gp_TakeEvent(gp_t *gp, const SDL_Event *ev)
{
	if (ev->window.windowID == gp->window_ID) {

		gpEventHandle(gp, ev);

		gp->active = 1;
	}
}

int gp_IsQuit(gp_t *gp)
{
	return gp->done;
}

int gp_Draw(gp_t *gp)
{
	scheme_t	*sch = gp->sch;
	draw_t		*dw = gp->dw;
	plot_t		*pl = gp->pl;
	read_t		*rd = gp->rd;
	menu_t		*mu = gp->mu;
	edit_t		*ed = gp->ed;

	gp->clock = SDL_GetTicks();
	gp->drawn = 0;

	if (rd->files_N != 0) {

		if (readUpdate(rd) != 0) {

			gp->active = 1;
		}
	}
	else {
		plotAxisScaleLock(pl, LOCK_FREE);
	}

	if (gp->i_show_fps != 0) {

		gp->active = 1;
	}

	if (gp->active != 0) {

		gp->idled = 0;
	}

	if (gp->updated + 250 < gp->clock) {

		gp->idled += 1;
		gp->active = (gp->idled < 20) ? 1 : 0;
	}

	if (gp->active != 0) {

		gp->unfinished = 1;

		gp->updated = gp->clock;
		gp->active = 0;
	}

	if (gp->unfinished != 0) {

		int	t0, t1;

		SDL_LockSurface(gp->surface);

		drawClearSurface(gp->dw, gp->surface, pl->sch->plot_background);

		SDL_UnlockSurface(gp->surface);

		plotLayout(pl);
		plotAxisScaleDefault(pl);

		if (gp->stat == GP_RANGE_SELECT) {

			if (gp->shift_on == 0) {

				gpDrawRangeLight(gp->surface, gp);
			}
		}
		else if (gp->stat == GP_BOX_SELECT) {

			gpDrawBoxLight(gp->surface, gp);
		}

		if (		rd->fastdraw != 0
				&& dw->antialiasing != DRAW_SOLID) {

			t0 = SDL_GetTicks();
		}

		plotDraw(pl, gp->surface);

		if (		rd->fastdraw != 0
				&& dw->antialiasing != DRAW_SOLID) {

			t1 = SDL_GetTicks();

			gp->level += (t1 - t0 > rd->fastdraw) ? 1
				: (gp->level > 0) ? - 1 : 0;

			if (gp->level > 4) {

				dw->antialiasing = DRAW_SOLID;
				gp->level = 0;
			}
		}

		if (gp->stat == GP_RANGE_SELECT) {

			gpDrawRangeSelect(gp->surface, gp);
		}
		else if (gp->stat == GP_BOX_SELECT) {

			gpDrawBoxSelect(gp->surface, gp);
		}

		if (gp->hover_box) {

			drawFillRect(gp->surface, pl->screen.min_x,
					pl->screen.min_y - gp->layout_page_box,
					pl->screen.max_x, pl->screen.min_y,
					pl->sch->plot_hovered);
		}

		gpTextLeftCrop(pl, gp->sbuf[1], rd->page[rd->page_N].title,
				gp->layout_menu_page_margin);

		sprintf(gp->sbuf[0], "%3d %s", rd->page_N, gp->sbuf[1]);

		drawText(gp->dw, gp->surface, pl->font, (pl->screen.min_x + pl->screen.max_x) / 2,
				pl->screen.min_y + gp->layout_page_title_offset, gp->sbuf[0],
				TEXT_CENTERED, sch->plot_text);

		menuDraw(mu, gp->surface);
		editDraw(ed, gp->surface);

		if (gp->i_show_fps != 0) {

			int		len, jam;

			len = plotGetSketchLength(pl);

			sprintf(gp->sbuf[0], "L %4d FPS %2d", len, gp->i_FPS);

			TTF_SizeUTF8(pl->font, gp->sbuf[0], &len, &jam);

			drawFillRect(gp->surface, pl->screen.max_x - (len + 12),
					pl->screen.min_y - gp->layout_page_box,
					pl->screen.max_x, pl->screen.min_y,
					pl->sch->plot_background);

			drawText(gp->dw, gp->surface, pl->font, pl->screen.max_x - (len + 6),
					pl->screen.min_y + gp->layout_page_title_offset,
					gp->sbuf[0], TEXT_CENTERED_ON_Y, 0xFF2222);
		}

		SDL_BlitSurface(gp->surface, NULL, gp->fb, NULL);
		SDL_UpdateWindowSurface(gp->window);

		gpFPSUpdate(gp);

		if (pl->draw_in_progress == 0) {

			gp->unfinished = 0;
		}

		gp->drawn = 1;
	}

	gpTakeScreen(gp);
	gpYankScreen(gp);

	return gp->drawn;
}

#ifndef _EMBED_GP
static void
gpGetOPT(gp_t *gp, char *argv[])
{
	read_t		*rd = gp->rd;

	char		*subarg;
	int		argi, op = 1;

	while (argv[op] != NULL) {

		if (argv[op][0] == '-') {

			if (argv[op][1] == 'h') {

				printf(	"Usage: gp [options] file ...\n"
					"  -i        open stdin text stream\n"
					"  -k <n>    chunk size in bytes\n"
					"  -u <n>    waiting timeout in msec\n"
					"  -l <n>    data length to allocate\n"
					"  -t <n>    time column default\n"
					);

				exit(0);
			}
			else if (argv[op][1] == 'i') {

				sprintf(gp->sbuf[0],	"load 0 0 stdin\n"
							"mkpages -2\n");

				readConfigIN(rd, gp->sbuf[0], 0);
			}
			else if (argv[op][1] == 'k') {

				subarg = &argv[op][2];

				if (*subarg == 0) {

					op++;

					if (argv[op] == NULL)
						break;

					subarg = argv[op];
				}

				if (stoi(&rd->mk_config, &argi, subarg) != NULL) {

					if (argi > 0) {

						rd->chunk = argi;
					}
				}
			}
			else if (argv[op][1] == 'u') {

				subarg = &argv[op][2];

				if (*subarg == 0) {

					op++;

					if (argv[op] == NULL)
						break;

					subarg = argv[op];
				}

				if (stoi(&rd->mk_config, &argi, subarg) != NULL) {

					if (argi >= 0) {

						rd->timeout = argi;
					}
				}
			}
			else if (argv[op][1] == 'l') {

				subarg = &argv[op][2];

				if (*subarg == 0) {

					op++;

					if (argv[op] == NULL)
						break;

					subarg = argv[op];
				}

				if (stoi(&rd->mk_config, &argi, subarg) != NULL) {

					if (argi > 0) {

						rd->length_N = argi;
					}
				}
			}
			else if (argv[op][1] == 't') {

				subarg = &argv[op][2];

				if (*subarg == 0) {

					op++;

					if (argv[op] == NULL)
						break;

					subarg = argv[op];
				}

				if (stoi(&rd->mk_config, &argi, subarg) != NULL) {

					if (argi >= -1 && argi < READ_COLUMN_MAX) {

						rd->timecol = argi;
					}
				}
			}
			else {
				ERROR("Unknown option \"%s\"\n", argv[op]);
			}
		}
		else {
			if (strlen(argv[op]) >= READ_FILE_PATH_MAX) {

				ERROR("Too long input file names\n");
				break;
			}
#ifdef _WINDOWS
			legacy_ACP_to_UTF8(gp->tempfile, argv[op], READ_FILE_PATH_MAX);
#else /* _WINDOWS */
			strcpy(gp->tempfile, argv[op]);
#endif
			gpUnifiedFileOpen(gp, gp->tempfile, 0);
		}

		op++;
	}
}

int main(int argn, char *argv[])
{
	gp_t		*gp;

	setlocale(LC_NUMERIC, "C");

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) < 0) {

		ERROR("SDL_Init: %s\n", SDL_GetError());

		return 1;
	}

	if (TTF_Init() < 0) {

		ERROR("TTF_Init: %s\n", SDL_GetError());

		return 1;
	}

	IMG_Init(IMG_INIT_PNG);

	gp = gp_Alloc();

	if (argn >= 2) {

		gpGetOPT(gp, argv);
	}
	else {
		gpMakeHello(gp);
	}

	gp_OpenWindow(gp);

	while (gp_IsQuit(gp) == 0) {

		SDL_Event		ev;

		while (SDL_PollEvent(&ev) != 0) {

			gp_TakeEvent(gp, &ev);
		}

		if (gp_Draw(gp) == 0) {

			SDL_Delay(10);
		}
	}

	gp_Clean(gp);

	SDL_Quit();

	return 0;
}
#endif /* _EMBED_GP */

