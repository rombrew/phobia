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

int utf8_length(const char *s);
const char *utf8_skip(const char *s, int n);

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
				"timeout 1000\n"
				"windowsize 1200 900\n"
				"language 0\n"
				"colorscheme 0\n"
				"antialiasing 1\n"
				"solidfont 0\n"
				"thickness 1\n"
				"drawing line 2\n"
				"timecol -1\n"
				"shortfilename 1\n"
				"drawboost 200\n"
				"interpolation 1\n"
				"precision 9\n"
				"lz4_compress 1\n");

#ifdef _WINDOWS
		fprintf(fd,	"legacy_label_enc 0\n");
#endif /* _WINDOWS */

		fclose(fd);
	}
}

static int
gpFileExist(const char *file)
{
	unsigned long long		sb;

	return (fstatsize(file, &sb) == 0) ? 1 : 0;
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
	gp->layout_menu_dir_margin = 12;
	gp->layout_menu_dataset_margin = 16;
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

	style |= (gp->shift_on == 1) ? TTF_STYLE_BOLD : 0;
	style |= (gp->ctrl_on == 1) ? TTF_STYLE_ITALIC : 0;

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

#ifdef _WINDOWS
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

						legacy_readConfigGRM(gp->rd, path, argv[0], argv[1], fromUI);
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
#endif /* _WINDOWS */

static void
gpUnifiedFileOpen(gp_t *gp, const char *file, int fromUI)
{
	read_t		*rd = gp->rd;

	if (gpFileIsGP(gp, file) != 0) {

		readConfigGP(rd, file, fromUI);
	}

#ifdef _WINDOWS
	else if (legacy_FileIsBAT(gp, file) != 0) {

		legacy_FileOpenBAT(gp, file, fromUI);
	}
#endif /* _WINDOWS */

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
	DIR			*dir;
	struct dirent 		*en;
	unsigned long long	sb;
	char			*la = gp->la_menu;
	int			menulen, pad, eN, kmg;

	menulen = gpScreenLength(gp->pl) - gp->layout_menu_dir_margin;

	dir = opendir(gp->cwd);

	if (dir == NULL) {

		gpDirWalk(gp, 2, 1);

		dir = opendir(gp->cwd);
	}

	gpTextLeftCrop(gp->pl, gp->sbuf[1], gp->cwd, gp->layout_menu_dir_margin);

	sprintf(gp->sbuf[0], "[%s]", gp->sbuf[1]);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	gpTextSepFill(gp->sbuf[0], menulen + 7);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	strcpy(la, "/..");
	la += strlen(la) + 1;

	strcpy(gp->d_names[0], "/..");
	eN = 1;

	if (dir != NULL) {

		while ((en = readdir(dir)) != NULL) {

			if (en->d_type == DT_DIR) {

				if (strcmp(en->d_name, ".") != 0 && strcmp(en->d_name, "..") != 0) {

					if (eN >= GP_FILE_DIR_MAX)
						break;

					sprintf(gp->sbuf[0], "/%s", en->d_name);

					if (strlen(gp->sbuf[0]) < PLOT_STRING_MAX) {

						strcpy(gp->d_names[eN], gp->sbuf[0]);

						gpTextLeftCrop(gp->pl, gp->sbuf[1], en->d_name,
								gp->layout_menu_dir_margin);

						pad = menulen - utf8_length(gp->sbuf[1]);

						sprintf(sfmt, "/%%s%%%ds", pad);
						sprintf(gp->sbuf[0], sfmt, gp->sbuf[1], "");

						strcpy(la, gp->sbuf[0]);
						la += strlen(la) + 1;

						eN++;
					}
				}
			}
		}

		rewinddir(dir);

		while ((en = readdir(dir)) != NULL) {

			if (en->d_type != DT_DIR) {

				if (eN >= GP_FILE_DIR_MAX)
					break;

				sprintf(gp->sbuf[0], "%s", en->d_name);

				if (strlen(gp->sbuf[0]) < PLOT_STRING_MAX) {

					strcpy(gp->d_names[eN], gp->sbuf[0]);

					sprintf(gp->sbuf[0], "%s/%s", gp->cwd, en->d_name);

					kmg = 0;

					if (fstatsize(gp->sbuf[0], &sb) == 0) {

						while (sb >= 1024U) {

							sb /= 1024U;
							++kmg;
						}
					}
					else {
						sb = 0U;
					}

					gpTextLeftCrop(gp->pl, gp->sbuf[1], en->d_name,
							gp->layout_menu_dir_margin);

					pad = menulen - utf8_length(gp->sbuf[1]);

					sprintf(sfmt, " %%s%%%ds %%4d %%cb", pad);
					sprintf(gp->sbuf[0], sfmt, gp->sbuf[1], "",
							(int) sb, " KMG??" [kmg]);

					strcpy(la, gp->sbuf[0]);
					la += strlen(la) + 1;

					eN++;
				}
			}
		}

		closedir(dir);
	}

	gp->cwd_ok = (dir != NULL) ? 1 : 0;

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

		if (*gp->cwd != 0 && gp->cwd_ok != 0) {

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

				if (*gp->cwd != 0) {

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

			if (*gp->cwd != 0) {

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

			sprintf(gp->sbuf[0], "* [%1i] %c ", sN,
					" USRAMHDCBLTP??" [gp->pl->data[dN].sub[sN].busy]);

			if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_TIME_UNWRAP) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i)",
						gp->pl->data[dN].sub[sN].op.time.column_1);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_SCALE) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %.2E, %.2E)",
						gp->pl->data[dN].sub[sN].op.scale.column_1,
						gp->pl->data[dN].sub[sN].op.scale.scale,
						gp->pl->data[dN].sub[sN].op.scale.offset);
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

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %i, %i)",
						gp->pl->data[dN].sub[sN].op.filter.column_1,
						(int) gp->pl->data[dN].sub[sN].op.filter.arg_1,
						(int) gp->pl->data[dN].sub[sN].op.filter.arg_2);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_FILTER_LOW_PASS) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %.2E)",
						gp->pl->data[dN].sub[sN].op.filter.column_1,
						gp->pl->data[dN].sub[sN].op.filter.arg_1);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_RESAMPLE) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i) (%i, %i, %i)",
						gp->pl->data[dN].sub[sN].op.resample.column_X,
						gp->pl->data[dN].sub[sN].op.resample.column_in_X,
						gp->pl->data[dN].sub[sN].op.resample.column_in_Y,
						gp->pl->data[dN].sub[sN].op.resample.in_data_N);
			}
			else if (gp->pl->data[dN].sub[sN].busy == SUBTRACT_POLYFIT) {

				sprintf(gp->sbuf[0] + strlen(gp->sbuf[0]), "(%i, %i)",
						gp->pl->data[dN].sub[sN].op.polyfit.column_X,
						gp->pl->data[dN].sub[sN].op.polyfit.poly_N2);
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
	int		dN = 0, menulen, fnlen, fnlen_max;

	menulen = gpScreenLength(gp->pl) - gp->layout_menu_dataset_margin;
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

	gpTextSepFill(gp->sbuf[1], (fnlen_max > menulen) ? fnlen_max : menulen);

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
	int		N, cN, gN, dN, mbUSAGE, mbRAW, mbCACHE, lzPC, menulen, fnlen;

	menulen = gpScreenLength(gp->pl) - gp->layout_menu_dataset_margin;

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

	gpTextSepFill(gp->sbuf[1], (fnlen > menulen) ? fnlen : menulen);

	strcpy(la, gp->sbuf[1]);
	la += strlen(la) + 1;

	cN = readGetTimeColumn(rd, dN);

	sprintf(gp->sbuf[0], gp->la->dataset_menu[0], cN);
	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	strcpy(gp->sbuf[2], " ");
	strcpy(gp->sbuf[3], "   ");

	if (cN >= -1 && pl->data[dN].map != NULL) {

		gN = pl->data[dN].map[cN];

		if (gN >= 0 && gN < PLOT_GROUP_MAX) {

			if (pl->group[gN].op_time_unwrap != 0) {

				strcpy(gp->sbuf[2], "X");
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

	sprintf(gp->sbuf[0], gp->la->dataset_menu[2], gp->sbuf[3]);
	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	mbUSAGE = plotDataMemoryUsage(pl, dN) / 1048576UL;
	mbRAW = plotDataMemoryUncompressed(pl, dN) / 1048576UL;
	mbCACHE = plotDataMemoryCached(pl, dN) / 1048576UL;

	lzPC = (mbRAW != 0) ? 100U * mbUSAGE / mbRAW : 0;

	sprintf(gp->sbuf[0], gp->la->dataset_menu[3],
			rd->data[dN].length_N, mbUSAGE, lzPC, mbCACHE);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	strcpy(la, gp->la->dataset_menu[4]);
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
	char		sbuf[READ_FILE_PATH_MAX], *eol;
	read_t		*rd = gp->rd;
	char		*la = gp->la_menu;
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

		while (fgets(sbuf, sizeof(sbuf), fd) != NULL) {

			eol = sbuf + strlen(sbuf) - 1;

			while (eol != sbuf && strchr(rd->mk_config.lend, *eol) != 0) {

				*eol = 0;
				--eol;
			}

			if (strlen(sbuf) != 0) {

				strcpy(gp->d_names[line_N], sbuf);
				sprintf(gp->sbuf[0], " %s", sbuf);

				strcpy(la, gp->sbuf[0]);
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
gpWriteConfiguration(gp_t *gp)
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

	strcpy(la, "Graph Plotter (GP) for numerical data analysis");
	la += strlen(la) + 1;

	gpTextSepFill(gp->sbuf[0], 54);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	strcpy(la, "License: GPLv3");
	la += strlen(la) + 1;

	strcpy(la, "Build: " __DATE__);
	la += strlen(la) + 1;

	gpTextSepFill(gp->sbuf[0], 54);

	strcpy(la, gp->sbuf[0]);
	la += strlen(la) + 1;

	strcpy(la, "[0] https://sourceforge.net/projects/graph-plotter/");
	la += strlen(la) + 1;

	strcpy(la, "[1] https://github.com/rombrew/gp");
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
void legacy_SetClipboard(SDL_Surface *surface)
{
	long		length = 9437184UL;
	void		*mBMP;

	SDL_RWops	*rwops;
	HGLOBAL		hDIB;

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

				ed->list_fmt = ".png\0.svg\0.csv\0\0";

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
				mu->mark[3].subs = (dw->solidfont != 0)  ? "Solid  " : "Blended";

				sprintf(gp->sbuf[3], "%i      ", dw->thickness);

				mu->mark[4].N = 4;
				mu->mark[4].subs = gp->sbuf[3];

				mu->mark[5].N = 5;
				mu->mark[5].subs = (gp->hinting == 1) ? "Light  "
						:  (gp->hinting == 2) ? "Normal " : "       ";

				sprintf(gp->sbuf[2], "%2i     ", pl->layout_font_pt);

				mu->mark[6].N = 6;
				mu->mark[6].subs = gp->sbuf[2];

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
					pl->slice_range_on = 0;
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
				gpMakeDirMenu(gp);

				menuRaise(mu, 1021, gp->la_menu, mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 2:
				gpMakeDatasetMenu(gp);

				menuRaise(mu, 1022, gp->la_menu, mu->box_X, mu->box_Y);
				gp->stat = GP_MENU;
				break;

			case 3:
				gpMakeConfigurationMenu(gp);

				menuRaise(mu, 1023, gp->la_menu, mu->box_X, mu->box_Y);
				mu->hidden_N[0] = 0;
				gp->stat = GP_MENU;
				break;

			case 4:
				menuRaise(mu, 1024, gp->la->cancel_menu, mu->box_X, mu->box_Y);
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

			int			gN, cN, unwrap;

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

						if (gN != -1) {

							unwrap = (pl->group[gN].op_time_unwrap != 0) ? 0 : 1;
							plotGroupTimeUnwrap(pl, gN, unwrap);
						}
						else {
							gN = gp->data_N + (PLOT_GROUP_MAX - PLOT_DATASET_MAX);

							plotGroupAdd(pl, gp->data_N, gN, cN);
							plotGroupTimeUnwrap(pl, gN, 1);
						}
					}

					gpMakeDatasetMenu(gp);

					menuResume(mu);
					menuLayout(mu);

					gp->stat = GP_MENU;
					break;

				case 4:
					cN = readGetTimeColumn(rd, gp->data_N);

					if (cN >= -1) {

						gN = pl->data[gp->data_N].map[cN];

						if (gN < 0) {

							/* NOTE: last group numbers are occupied by time scale.
							 * */
							gN = gp->data_N + (PLOT_GROUP_MAX - PLOT_DATASET_MAX);

							plotGroupAdd(pl, gp->data_N, gN, cN);
						}

						if (pl->group[gN].op_scale == 0) {

							plotGroupScale(pl, gN, 1., 0.);
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

				case 5:
					sprintf(gp->sbuf[0], "%i", rd->data[gp->data_N].length_N);

					editRaise(ed, 12, gp->la->length_edit,
							gp->sbuf[0], mu->box_X, mu->box_Y);

					gp->stat = GP_EDIT;
					break;

				case 6:
					readDatasetClean(rd, gp->data_N);

					gpMakeDatasetMenu(gp);

					menuResume(mu);
					menuLayout(mu);

					gp->stat = GP_MENU;
					break;

				case 1:
				case 7:
					menuResume(mu);
					gp->stat = GP_MENU;
					break;

				default:

					if (item_N >= 8) {

						readToggleHint(rd, gp->data_N, item_N - 8);
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

		if (item_N != -1) {

			gp->line_N = item_N - 2;

			editRaise(ed, 10, gp->la_menu,
					gp->d_names[gp->line_N],
					mu->box_X, mu->box_Y);

			gp->stat = GP_EDIT;
		}
	}
	else if (menu_N == 1024) {

		if (item_N == 1) {

			gpDefaultFile(gp);
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
				rd->drawboost = 0;
				dw->antialiasing = (dw->antialiasing < DRAW_8X_MSAA)
					? dw->antialiasing + 1 : DRAW_SOLID;
				break;

			case 3:
				dw->solidfont = (dw->solidfont != 0) ? 0 : 1;
				break;

			case 4:
				dw->thickness = (dw->thickness < 2) ? dw->thickness + 1 : 0;
				break;

			case 5:
				gp->hinting = (gp->hinting < 2) ? gp->hinting + 1 : 0;
				gpFontHinting(gp);
				break;

			case 6:
				sprintf(gp->sbuf[0], "%i", pl->layout_font_pt);

				editRaise(ed, 15, gp->la->font_size_edit,
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
			mu->mark[3].subs = (dw->solidfont != 0)  ? "Solid  " : "Blended";

			sprintf(gp->sbuf[3], "%i      ", dw->thickness);

			mu->mark[4].N = 4;
			mu->mark[4].subs = gp->sbuf[3];

			mu->mark[5].subs = (gp->hinting == 1) ? "Light  "
					:  (gp->hinting == 2) ? "Normal " : "       ";

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
					mu->hidden_N[1] = 5;
					mu->hidden_N[2] = 6;
					mu->hidden_N[3] = 7;
					mu->hidden_N[4] = 8;
				}
				else if (N != 2) {

					mu->hidden_N[0] = 5;
					mu->hidden_N[1] = 6;
					mu->hidden_N[2] = 7;
					mu->hidden_N[3] = 8;
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
				plotFigureSubtractTimeUnwrap(pl, gp->fig_N);
				break;

			case 2:
				plotFigureSubtractResample(pl, gp->fig_N);
				break;

			case 3:
				editRaise(ed, 5, gp->la->scale_offset_edit,
						"1", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 4:
				editRaise(ed, 6, gp->la->scale_offset_edit,
						"-1", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 5:
				plotFigureSubtractSwitch(pl, SUBTRACT_BINARY_SUBTRACTION);
				break;

			case 6:
				plotFigureSubtractSwitch(pl, SUBTRACT_BINARY_ADDITION);
				break;

			case 7:
				plotFigureSubtractSwitch(pl, SUBTRACT_BINARY_MULTIPLICATION);
				break;

			case 8:
				plotFigureSubtractSwitch(pl, SUBTRACT_BINARY_HYPOTENUSE);
				break;

			case 9:
				plotFigureSubtractFilter(pl, gp->fig_N, SUBTRACT_FILTER_DIFFERENCE, 0., 0.);
				break;

			case 10:
				plotFigureSubtractFilter(pl, gp->fig_N, SUBTRACT_FILTER_CUMULATIVE, 0., 0.);
				break;

			case 11:
				editRaise(ed, 8, gp->la->bit_number_edit,
						"0", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 12:
				editRaise(ed, 13, gp->la->low_pass_edit,
						"0.1", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 13:
				if (plotDataBoxPolyfit(pl, gp->fig_N) == 0) {

					editRaise(ed, 16, gp->la->polynomial_edit,
							"1", mu->box_X, mu->box_Y);

					gp->stat = GP_EDIT;
				}
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
						"1", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 2:
				gp->fig_N = -1;

				editRaise(ed, 6, gp->la->scale_offset_edit,
						"-1", mu->box_X, mu->box_Y);

				gp->stat = GP_EDIT;
				break;

			case 3:
				pl->transparency = pl->transparency ? 0 : 1;

				if (mu->clicked != 0) {

					mu->mark[0].subs = (pl->transparency == 0) ? " " : "X";

					menuResume(mu);
					gp->stat = GP_MENU;
				}
				break;

			case 4:
				pl->legend_compact = pl->legend_compact ? 0 : 1;

				if (mu->clicked != 0) {

					mu->mark[1].subs = (pl->legend_compact == 0) ? " " : "X";

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
	plot_t		*pl = gp->pl;
	read_t		*rd = gp->rd;
	menu_t		*mu = gp->mu;
	svg_t		*g;

	double		scale, offset;
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

		int		fN, axis_1;

		offset = 0.;

		n = sscanf(text, "%le %le", &scale, &offset);

		if (n != 0) {

			axis_1 = (edit_N == 5) ? AXIS_BUSY_X : AXIS_BUSY_Y;

			if (gp->fig_N < 0) {

				for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

					if (		pl->figure[fN].busy != 0
							&& pl->figure[fN].hidden == 0) {

						plotFigureSubtractScale(pl, fN, axis_1,
								scale, offset);
					}
				}
			}
			else {
				plotFigureSubtractScale(pl, gp->fig_N, axis_1, scale, offset);
			}
		}
	}
	else if (edit_N == 7) {

		const char	*file;

		strcpy(gp->tempfile, text);

		file = gp->tempfile + strlen(gp->tempfile);

		if (strlen(gp->tempfile) > 4) {

			file += - 4;
		}

		if (strcmp(file, ".png") == 0) {

			gp->screen_take = GP_TAKE_PNG;
		}
		else if (strcmp(file, ".svg") == 0) {

			g = (void *) svgOpenNew(gp->tempfile,
					gp->surface->w, gp->surface->h);

			g->font_family = "monospace";
			g->font_pt = pl->layout_font_pt;

			gp->surface->userdata = (void *) g;
			gp->screen_take = GP_TAKE_SVG;
		}
		else if (strcmp(file, ".csv") == 0) {

			gp->screen_take = GP_TAKE_CSV;
		}
	}
	else if (edit_N == 8) {

		int		arg_1, arg_2;

		n = sscanf(text, "%d %d", &arg_1, &arg_2);

		if (n != 0) {

			arg_2 = (n == 1) ? arg_1 : arg_2;
			arg_2 = (arg_2 < arg_1) ? arg_1 : arg_2;

			plotFigureSubtractFilter(pl, gp->fig_N, SUBTRACT_FILTER_BITMASK,
					(double) arg_1, (double) arg_2);
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

		gpWriteConfiguration(gp);
		gpMakeConfigurationMenu(gp);

		menuRaise(mu, 1023, gp->la_menu, mu->box_X, mu->box_Y);
		menuSelect(mu, gp->line_N + 2);

		mu->hidden_N[0] = 0;
		gp->stat = GP_MENU;
	}
	else if (edit_N == 11) {

		n = sscanf(text, "%le %le", &scale, &offset);

		if (n == 2) {

			pl->group[gp->grp_N].scale = scale;
			pl->group[gp->grp_N].offset = offset;
		}
		else {
			pl->group[gp->grp_N].op_scale = 0;
		}

		gpMakeDatasetMenu(gp);

		menuRaise(mu, 1022, gp->la_menu, mu->box_X, mu->box_Y);
		gp->stat = GP_MENU;
	}
	else if (edit_N == 12) {

		n = sscanf(text, "%i", &len);

		if (n == 1) {

			rd->data[gp->data_N].length_N = len;

			if (rd->data[gp->data_N].length_N >= 1) {

				plotDataResize(pl, gp->data_N, rd->data[gp->data_N].length_N + 1);
			}
		}

		gpMakeDatasetMenu(gp);

		menuRaise(mu, 1022, gp->la_menu, mu->box_X, mu->box_Y);
		gp->stat = GP_MENU;
	}
	else if (edit_N == 13) {

		n = sscanf(text, "%le", &scale);

		if (n == 1) {

			plotFigureSubtractFilter(pl, gp->fig_N, SUBTRACT_FILTER_LOW_PASS, scale, 0.);
		}
	}
	else if (edit_N == 14) {

		n = sscanf(text, "%i", &len);

		if (n == 1) {

			len = (len < 0) ? 0 : (len > 16) ? 16 : len;

			pl->figure[gp->fig_N].width = len;
		}
	}
	else if (edit_N == 15) {

		n = sscanf(text, "%i", &len);

		if (n == 1) {

			gpFontToggle(gp, 0, len);

			sprintf(gp->sbuf[2], "%2i     ", pl->layout_font_pt);

			mu->mark[6].subs = gp->sbuf[2];

			menuResume(mu);
			menuLayout(mu);

			gp->stat = GP_MENU;
		}
	}
	else if (edit_N == 16) {

		int		arg_1, arg_2;

		n = sscanf(text, "%d %d", &arg_1, &arg_2);

		if (n != 0) {

			if (n == 1) {

				arg_2 = arg_1;
				arg_1 = 0;
			}

			plotFigureSubtractPolyfit(pl, gp->fig_N, arg_1, arg_2);
		}
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
			else if (ev->key.keysym.sym == SDLK_o) {

				mu->box_X = -1;
				mu->box_Y = -1;

				gpMenuHandle(gp, 102, 1);
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

						pl->figure[N].hidden = pl->figure[N].hidden
							? 0 : 1;

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

							gp->stat = GP_MOVING;
							pl->on_X = N;
							gp->ax_N = N;
							break;
						}
						else if (pl->axis[N].busy == AXIS_BUSY_Y
								&& pl->axis[N].slave == 0) {

							gp->stat = GP_MOVING;
							pl->on_Y = N;
							gp->ax_N = N;
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

						mu->mark[0].N = 3;
						mu->mark[0].subs = (pl->transparency == 0) ? " " : "X";

						mu->mark[1].N = 4;
						mu->mark[1].subs = (pl->legend_compact == 0) ? " " : "X";

						gp->stat = GP_MENU;
						break;
					}

					N = plotDataBoxGetByClick(pl, gp->cur_X, gp->cur_Y);

					if (N == 0) {

						pl->data_box_X = pl->viewport.max_x;
						pl->data_box_Y = 0;
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

		drawLineDashed(dw, surface, &pl->viewport, gp->box_X, pl->viewport.min_y,
				gp->box_X, pl->viewport.max_y, pl->sch->plot_text,
				pl->layout_fence_dash, pl->layout_fence_space);
		drawLineDashed(dw, surface, &pl->viewport, gp->cur_X, pl->viewport.min_y,
				gp->cur_X, pl->viewport.max_y, pl->sch->plot_text,
				pl->layout_fence_dash, pl->layout_fence_space);
	}
	else {
		drawLineDashed(dw, surface, &pl->viewport, pl->viewport.min_x, gp->box_Y,
				pl->viewport.max_x, gp->box_Y, pl->sch->plot_text,
				pl->layout_fence_dash, pl->layout_fence_space);
		drawLineDashed(dw, surface, &pl->viewport, pl->viewport.min_x, gp->cur_Y,
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

	drawLineDashed(dw, surface, &pl->viewport, gp->box_X, gp->box_Y,
			gp->cur_X, gp->box_Y, pl->sch->plot_text,
			pl->layout_fence_dash, pl->layout_fence_space);
	drawLineDashed(dw, surface, &pl->viewport, gp->box_X, gp->box_Y,
			gp->box_X, gp->cur_Y, pl->sch->plot_text,
			pl->layout_fence_dash, pl->layout_fence_space);
	drawLineDashed(dw, surface, &pl->viewport, gp->cur_X, gp->cur_Y,
			gp->cur_X, gp->box_Y, pl->sch->plot_text,
			pl->layout_fence_dash, pl->layout_fence_space);
	drawLineDashed(dw, surface, &pl->viewport, gp->cur_X, gp->cur_Y,
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
	dw->solidfont = 0;
	dw->thickness = 1;

	pl = plotAlloc(dw, sch);
	gp->pl = pl;

	rd = readAlloc(dw, pl);
	gp->rd = rd;

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

		if (		rd->drawboost != 0
				&& dw->antialiasing != DRAW_SOLID) {

			t0 = SDL_GetTicks();
		}

		plotDraw(pl, gp->surface);

		if (		rd->drawboost != 0
				&& dw->antialiasing != DRAW_SOLID) {

			t1 = SDL_GetTicks();

			gp->level += (t1 - t0 > rd->drawboost) ? 1
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

		sprintf(gp->sbuf[0], "%3d. %s", rd->page_N, gp->sbuf[1]);

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

#ifdef _WINDOWS
	if (argn >= 3) {

		legacy_ACP_to_UTF8(gp->sbuf[0], argv[1], READ_FILE_PATH_MAX);
		legacy_ACP_to_UTF8(gp->sbuf[1], argv[2], READ_FILE_PATH_MAX);

		legacy_readConfigGRM(gp->rd, NULL, gp->sbuf[0], gp->sbuf[1], 0);
	}
	else
#endif /* _WINDOWS */

	if (argn == 2) {

#ifdef _WINDOWS
		legacy_ACP_to_UTF8(gp->tempfile, argv[1], READ_FILE_PATH_MAX);
#else /* _WINDOWS */

		if (strlen(argv[1]) < READ_FILE_PATH_MAX) {

			strcpy(gp->tempfile, argv[1]);
		}
		else {
			ERROR("Too long file name\n");

			return 1;
		}
#endif

		gpUnifiedFileOpen(gp, gp->tempfile, 0);
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

