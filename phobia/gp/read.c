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

#include <SDL2/SDL.h>

#ifdef _WINDOWS
#include <windows.h>
#endif /* _WINDOWS */

#include "async.h"
#include "dirent.h"
#include "draw.h"
#include "edit.h"
#include "lang.h"
#include "plot.h"
#include "read.h"

char *stoi(const markup_t *mk, int *x, char *s)
{
	int		n, d, i;

	if (*s == '-') { n = - 1; s++; }
	else if (*s == '+') { n = 1; s++; }
	else { n = 1; }

	d = 0;
	i = 0;

	while (*s >= '0' && *s <= '9') {

		i = 10 * i + (*s++ - '0') * n;
		d += 1;
	}

	if (d == 0 || d > 9) { return NULL; }

	if (*s == 0 || strchr(mk->space, *s) != NULL
			|| strchr(mk->lend, *s) != NULL) {

		*x = i;
	}
	else { return NULL; }

	return s;
}

char *htoi(const markup_t *mk, int *x, char *s)
{
	int		i, d, h;

	d = 0;
	h = 0;

	if (*s == '0' && *(s + 1) == 'x') { s += 2; }

	do {
		if (*s >= '0' && *s <= '9') { i = *s++ - '0'; }
		else if (*s >= 'A' && *s <= 'F') { i = 10 + *s++ - 'A'; }
		else if (*s >= 'a' && *s <= 'f') { i = 10 + *s++ - 'a'; }
		else break;

		h = 16 * h + i;
		d += 1;
	}
	while (1);

	if (d == 0 || d > 8) { return NULL; }

	if (*s == 0 || strchr(mk->space, *s) != NULL
			|| strchr(mk->lend, *s) != NULL) {

		*x = h;
	}
	else { return NULL; }

	return s;
}

char *otoi(const markup_t *mk, int *x, char *s)
{
	int		i, d, h;

	d = 0;
	h = 0;

	do {
		if (*s >= '0' && *s <= '7') { i = *s++ - '0'; }
		else break;

		h = 8 * h + i;
		d += 1;
	}
	while (1);

	if (d == 0 || d > 11) { return NULL; }

	if (*s == 0 || strchr(mk->space, *s) != NULL
			|| strchr(mk->lend, *s) != NULL) {

		*x = h;
	}
	else { return NULL; }

	return s;
}

char *stod(const markup_t *mk, double *x, char *s)
{
	int		n, d, v, e;
	double		f;

	if (*s == '-') { n = - 1; s++; }
	else if (*s == '+') { n = 1; s++; }
	else { n = 1; }

	d = 0;
	v = 0;
	f = 0.;

	while (*s >= '0' && *s <= '9') {

		f = 10. * f + (*s++ - '0') * n;
		d += 1;
	}

	if (*s == mk->delim) {

		s++;

		while (*s >= '0' && *s <= '9') {

			f = 10. * f + (*s++ - '0') * n;
			d += 1; v -= 1;
		}
	}

	if (d == 0) { return NULL; }

	if (*s == 'n') { v += - 9; s++; }
	else if (*s == 'u') { v += - 6; s++; }
	else if (*s == 'm') { v += - 3; s++; }
	else if (*s == 'K') { v += 3; s++; }
	else if (*s == 'M') { v += 6; s++; }
	else if (*s == 'G') { v += 9; s++; }
	else if (*s == 'e' || *s == 'E') {

		s = stoi(mk, &e, s + 1);

		if (s != NULL) { v += e; }
		else { return NULL; }
	}

	if (*s == 0 || strchr(mk->space, *s) != NULL
			|| strchr(mk->lend, *s) != NULL) {

		while (v < 0) { f /= 10.; v += 1; }
		while (v > 0) { f *= 10.; v -= 1; }

		*x = f;
	}
	else { return NULL; }

	return s;
}

read_t *readAlloc(draw_t *dw, plot_t *pl)
{
	read_t		*rd;

	rd = calloc(1, sizeof(read_t));

	rd->dw = dw;
	rd->pl = pl;

	strcpy(rd->screenpath, ".");

	rd->window_size_x = GP_MIN_SIZE_X;
	rd->window_size_y = GP_MIN_SIZE_Y;
	rd->timecol = -1;
	rd->shortfilename = 1;
	rd->fastdraw = 200;

	rd->mk_config.delim = '.';
	strcpy(rd->mk_config.space, " \t;");
	strcpy(rd->mk_config.lend, "\r\n");

	rd->mk_text.delim = rd->mk_config.delim;
	strcpy(rd->mk_text.space, rd->mk_config.space);
	strcpy(rd->mk_text.lend, rd->mk_config.lend);

#ifdef _WINDOWS
	rd->legacy_label = 1;
#endif /* _WINDOWS */

	rd->preload = 8388608;
	rd->chunk = 4096;
	rd->timeout = 5000;
	rd->length_N = 1000;

	rd->bind_N = -1;
	rd->page_N = -1;
	rd->figure_N = -1;

	return rd;
}

void readClean(read_t *rd)
{
	free(rd);
}

static void
readCutLabel(char *tbuf, const char *text, int allowed)
{
	int		length;

	length = strlen(text);

	if (length > (allowed - 1)) {

		text = utf8_skip_b(text, length - (allowed - 2));

		strcpy(tbuf, "~");
		strcat(tbuf, text);
	}
	else {
		strcpy(tbuf, text);
	}
}

static void
readCutFile(read_t *rd, char *tbuf, const char *file, int allowed)
{
	const char	*eol;
	int		length, ndir;

	file = (file[0] == '.' && file[1] == '/')
		? file + 2 : file;

	if (allowed < 25 || rd->shortfilename != 0) {

		eol = file + strlen(file);
		ndir = rd->shortfilename;

		do {
			if (*eol == '/' || *eol == '\\') {

				if (ndir > 1) ndir--;
				else break;
			}

			if (file == eol)
				break;

			eol--;
		}
		while (1);
	}
	else {
		eol = file;
	}

	length = utf8_length(eol);

	if (length > (allowed - 1)) {

		eol = utf8_skip(eol, length - (allowed - 1));

		strcpy(tbuf, "~");
		strcat(tbuf, eol);
	}
	else {
		if (file == eol) {

			strcpy(tbuf, eol);
		}
		else {
			strcpy(tbuf, "~");
			strcat(tbuf, eol);
		}
	}
}

#ifdef _WINDOWS
void legacy_ACP_to_UTF8(char *us, const char *text, int n)
{
	wchar_t			wbuf[READ_TOKEN_MAX * READ_COLUMN_MAX];

	MultiByteToWideChar(CP_ACP, 0, text, -1, wbuf, sizeof(wbuf) / sizeof(wchar_t));
	WideCharToMultiByte(CP_UTF8, 0, wbuf, -1, us, n, NULL, NULL);
}

void legacy_OEM_to_UTF8(char *us, const char *text, int n)
{
	wchar_t			wbuf[READ_TOKEN_MAX * READ_COLUMN_MAX];

	MultiByteToWideChar(CP_OEMCP, 0, text, -1, wbuf, sizeof(wbuf) / sizeof(wchar_t));
	WideCharToMultiByte(CP_UTF8, 0, wbuf, -1, us, n, NULL, NULL);
}

void legacy_UTF8_to_ACP(char *text, const char *us, int n)
{
	wchar_t			wbuf[READ_TOKEN_MAX * READ_COLUMN_MAX];

	MultiByteToWideChar(CP_UTF8, 0, us, -1, wbuf, sizeof(wbuf) / sizeof(wchar_t));
	WideCharToMultiByte(CP_ACP, 0, wbuf, -1, text, n, NULL, NULL);
}

void legacy_UTF8_to_OEM(char *text, const char *us, int n)
{
	wchar_t			wbuf[READ_TOKEN_MAX * READ_COLUMN_MAX];

	MultiByteToWideChar(CP_UTF8, 0, us, -1, wbuf, sizeof(wbuf) / sizeof(wchar_t));
	WideCharToMultiByte(CP_OEMCP, 0, wbuf, -1, text, n, NULL, NULL);
}

static FILE *
legacy_fopen_from_UTF8(const char *file, const char *mode)
{
	wchar_t			wfile[READ_FILE_PATH_MAX];
	wchar_t			wmode[READ_TOKEN_MAX];

	MultiByteToWideChar(CP_UTF8, 0, file, -1, wfile, READ_FILE_PATH_MAX);
	MultiByteToWideChar(CP_UTF8, 0, mode, -1, wmode, READ_TOKEN_MAX);

	return _wfopen(wfile, wmode);
}
#endif /* _WINDOWS */

#ifdef _LEGACY
static int
legacy_GetFreeData(read_t *rd)
{
	int		N, dN = -1;

	for (N = 0; N < PLOT_DATASET_MAX; ++N) {

		if (		rd->data[N].format == FORMAT_NONE
				&& rd->data[N].file[0] == 0) {

			dN = N;
			break;
		}
	}

	return dN;
}

static int
legacy_LabelExtract(char *s, char *label[9])
{
	char		*cur;
	int		m, N;

	cur = s;
	m = 0;
	N = 0;

	while (*s != 0) {

		if (strchr(" \t,\'", *s) != NULL) {

			if (m != 0) {

				if (N != 0 || *s == ',') {

					label[N++] = cur;

					*s = 0;

					if (N >= 9)
						break;
				}

				m = 0;
			}
		}
		else {
			if (m == 0) {

				cur = s;
				m = 1;
			}
		}

		++s;
	}

	if (m != 0 && N > 0 && N < 9) {

		label[N++] = cur;
	}

	return N;
}

static char *
legacy_TextTrim(read_t *rd, char *s)
{
	char		*eol;

	while (*s != 0) {

		if (strchr(" \t\'", *s) == NULL)
			break;

		s++;
	}

	if (strlen(s) > 1) {

		eol = s + strlen(s) - 1;

		if (strchr("\r\n", *eol) != NULL) {

			*eol = 0;
		}
	}

	return s;
}

void legacy_ConfigGRM(read_t *rd, const char *path, const char *confile,
		const char *file, int fromUI)
{
	FILE		*fd;

	char 		pbuf[READ_FILE_PATH_MAX];
	char 		xbuf[READ_FILE_PATH_MAX];
	char		lbuf[PLOT_STRING_MAX];
	char		nbuf[PLOT_STRING_MAX];

	char		*tbuf, *label[9];
	const char	*pfile, *ptext;

	int		cX, cY, cYm, N;
	double		scale, offset;

	int		dN, cN, pN, fN, line_N, lbN, stub, len;
	float		fpN;

	dN = legacy_GetFreeData(rd);

	if (dN < 0) {

		ERROR("Unable to get free dataset\n");
		return ;
	}

	pfile = file;

	if (path != NULL) {

		sprintf(pbuf, "%s/%s", path, file);
		pfile = pbuf;
	}

#ifdef _WINDOWS
	fd = legacy_fopen_from_UTF8(pfile, "rb");
#else /* _WINDOWS */
	fd = fopen(pfile, "rb");
#endif

	if (fd == NULL) {

		ERROR("fopen(\"%s\"): %s\n", file, strerror(errno));
		return ;
	}
	else {
		len = fread(&stub, 2, 1, fd);
		len += fread(&fpN, 4, 1, fd);

		cN = (int) fpN;

		if (		len != 0 && fpN == (float) cN
				&& cN > 1 && cN < READ_COLUMN_MAX) {

			fclose(fd);

			readOpenUnified(rd, dN, cN + 1, 0, pfile, FORMAT_BINARY_LEGACY_V1);
		}
		else {
			fseek(fd, 0UL, SEEK_SET);
			len = fread(&fpN, 4, 1, fd);

			cN = (int) fpN;

			if (		len != 0 && fpN == (float) cN
					&& cN > 1 && cN < READ_COLUMN_MAX) {

				fclose(fd);

				readOpenUnified(rd, dN, cN + 1, 0, pfile, FORMAT_BINARY_LEGACY_V2);
			}
			else {
				fclose(fd);

				ERROR("Unable to load legacy file \"%s\"\n", file);
				return ;
			}
		}
	}

	pfile = confile;

	if (path != NULL) {

		sprintf(pbuf, "%s/%s", path, confile);
		pfile = pbuf;
	}

#ifdef _WINDOWS
	fd = legacy_fopen_from_UTF8(pfile, "r");
#else /* _WINDOWS */
	fd = fopen(pfile, "r");
#endif

	if (fd == NULL) {

		ERROR("fopen(\"%s\"): %s\n", confile, strerror(errno));
		return ;
	}
	else {
		tbuf = rd->data[dN].buf;
		line_N = 0;

		pN = rd->page_N;

		while (fgets(tbuf, sizeof(rd->data[0].buf), fd) != NULL) {

			line_N++;

			if (strncmp(tbuf, "LI", 2) == 0) {

				if (fgets(tbuf, sizeof(rd->data[0].buf), fd) == NULL)
					break;

				line_N++;
#ifdef _WINDOWS
				legacy_OEM_to_UTF8(tbuf, tbuf, sizeof(rd->data[0].buf));
#endif /* _WINDOWS */
				if (pN < 0) {

					pN = 1;
				}

				while (rd->page[pN].busy != 0) {

					pN++;

					if (pN >= READ_PAGE_MAX)
						break;
				}

				if (pN >= READ_PAGE_MAX)
					break;

				rd->page[pN].busy = 1;

				strcpy(lbuf, legacy_TextTrim(rd, tbuf));
				ptext = lbuf;

				if (fromUI != 0) {

					readCutFile(rd, nbuf, pfile, 95);

					sprintf(xbuf, "%s: %.95s", nbuf, lbuf);
					ptext = xbuf;
				}

				readCutLabel(rd->page[pN].title, ptext, PLOT_STRING_MAX);

				lbN = legacy_LabelExtract(lbuf, label);

				if (fgets(tbuf, sizeof(rd->data[0].buf), fd) == NULL)
					break;

				line_N++;

				N = sscanf(tbuf, "%i", &cX);

				if (N != 1) {

					cX = 0;
				}

				if (cX < -1 || cX >= rd->pl->data[dN].column_N) {

					ERROR("%s:%i: page %i column number %i is out of range\n",
						confile, line_N, pN, cX);
					cX = 0;
				}

				if (fgets(tbuf, sizeof(rd->data[0].buf), fd) == NULL)
					break;

				line_N++;
#ifdef _WINDOWS
				legacy_OEM_to_UTF8(tbuf, tbuf, sizeof(rd->data[0].buf));
#endif /* _WINDOWS */
				sprintf(rd->page[pN].ax[0].label, "%.20s", legacy_TextTrim(rd, tbuf));

				fN = 0;

				do {
					if (fgets(tbuf, sizeof(rd->data[0].buf), fd) == NULL)
						break;

					line_N++;

					if (strlen(tbuf) < 5)
						break;

					N = sscanf(tbuf, "%i %i %i %le %le",
							&stub, &cY, &cYm, &scale, &offset);

					if (N != 5) {

						ERROR("%s:%i: page %i figure %i invalid format\n",
							confile, line_N, pN, fN);
						break;
					}

					if (cY < -1 || cY >= rd->pl->data[dN].column_N) {

						ERROR("%s:%i: page %i column number %i is out of range\n",
							confile, line_N, pN, cY);
						break;
					}

					if (fN < PLOT_FIGURE_MAX) {

						rd->page[pN].fig[fN].busy = 1;
						rd->page[pN].fig[fN].drawing = -1;
						rd->page[pN].fig[fN].dN = dN;
						rd->page[pN].fig[fN].cX = cX;
						rd->page[pN].fig[fN].cY = cY;
						rd->page[pN].fig[fN].aX = 0;
						rd->page[pN].fig[fN].aY = 1;

						sprintf(rd->page[pN].fig[fN].label, "fig.%i.%i", fN, cY);

						N = 0;

						if (cY != cYm) {

							rd->page[pN].fig[fN].bY[N].busy = SUBTRACT_BINARY_SUBTRACTION;
							rd->page[pN].fig[fN].bY[N].column_2 = cYm;

							N++;
						}

						if (scale != 1. || offset != 0.) {

							rd->page[pN].fig[fN].bY[N].busy = SUBTRACT_SCALE;
							rd->page[pN].fig[fN].bY[N].args[0] = scale;
							rd->page[pN].fig[fN].bY[N].args[1] = offset;
						}

						fN++;
					}
					else {
						ERROR("%s:%i: too many figures on page %i\n",
							confile, line_N, pN);
						break;
					}

					if (strstr(tbuf, "'END") != NULL)
						break;
				}
				while (1);

				if (lbN >= fN) {

					for (N = 0; N < fN; ++N) {

						sprintf(rd->page[pN].fig[N].label, "%.75s.%i",
								label[N], rd->page[pN].fig[N].cY);
					}

					if (lbN - 1 >= fN) {

						sprintf(rd->page[pN].ax[1].label, "%.20s", label[lbN - 1]);
					}
				}
			}
		}

		fclose(fd);

		if (fromUI == 0) {

			rd->page_N = pN;
			rd->figure_N = -1;
		}
	}
}
#endif /* _LEGACY */

FILE *unified_fopen(const char *file, const char *mode)
{
#ifdef _WINDOWS
	return legacy_fopen_from_UTF8(file, mode);
#else /* _WINDOWS */
	return fopen(file, mode);
#endif
}

static char *
readTimeGetBuf(char *s, int len, FILE *fd, int timeout)
{
	int		c, eol, nq, waiting;

	eol = 0;
	nq = 0;
	waiting = 0;

	do {
		c = fgetc(fd);

		if (c != EOF) {

			if (c == '\r' || c == '\n') {

				eol = (nq > 0) ? 1 : 0;
			}
			else if (eol == 1) {

				ungetc(c, fd);
				break;
			}
			else if (nq < len - 1) {

				*s++ = (char) c;
				nq++;
			}
		}
		else {
			if (feof(fd) || ferror(fd)) {

				if (waiting < timeout) {

					clearerr(fd);

					SDL_Delay(10);

					waiting += 10;
				}
				else {
					eol = 1;
					break;
				}
			}
		}
	}
	while (1);

	if (eol != 0) {

		*s = 0;

		return s;
	}
	else {
		return NULL;
	}
}

static int
readTEXTGetRow(read_t *rd, int dN)
{
	fval_t 		*row = rd->data[dN].row;
	int		*hint = rd->data[dN].hint;
	char 		*r, *s = rd->data[dN].buf;

	int		hex, m, N;
	double		val;

	m = 0;
	N = 0;

	while (*s != 0) {

		if (		strchr(rd->mk_text.space, *s) != NULL
				|| strchr(rd->mk_text.lend, *s) != NULL) {

			m = 0;
		}
		else {
			if (m == 0) {

				m = 1;

				if (hint[N] == DATA_HINT_FLOAT) {

					r = stod(&rd->mk_text, &val, s);

					if (r != NULL) {

						*row++ = (fval_t) val;
					}
					else {
						*row++ = (fval_t) FP_NAN;
					}
				}
				else if (hint[N] == DATA_HINT_HEX) {

					r = htoi(&rd->mk_text, &hex, s);

					if (r != NULL) {

						*row++ = (fval_t) hex;
					}
					else {
						*row++ = (fval_t) FP_NAN;
					}
				}
				else if (hint[N] == DATA_HINT_OCT) {

					r = otoi(&rd->mk_text, &hex, s);

					if (r != NULL) {

						*row++ = (fval_t) hex;
					}
					else {
						*row++ = (fval_t) FP_NAN;
					}
				}
				else {
					r = stod(&rd->mk_text, &val, s);

					if (r != NULL) {

						*row++ = (fval_t) val;
					}
					else {
						r = htoi(&rd->mk_text, &hex, s);

						if (r != NULL) {

							if (hint[N] == DATA_HINT_NONE) {

								hint[N] = DATA_HINT_HEX;
							}

							*row++ = (fval_t) hex;
						}
						else {
							*row++ = (fval_t) FP_NAN;
						}
					}
				}

				N++;

				if (N >= READ_COLUMN_MAX)
					break;
			}
		}

		s++;
	}

	return N;
}

static int
readTEXTGetLabel(read_t *rd, int dN)
{
	char		*label, *s = rd->data[dN].buf;
	int		m, N;

#ifdef _WINDOWS
	if (rd->legacy_label == 1) {

		legacy_ACP_to_UTF8(s, rd->data[dN].buf, sizeof(rd->data[0].buf));
	}
	else if (rd->legacy_label == 2) {

		legacy_OEM_to_UTF8(s, rd->data[dN].buf, sizeof(rd->data[0].buf));
	}
#endif /* _WINDOWS */

	m = 0;
	N = 0;

	while (*s != 0) {

		if (		strchr(rd->mk_text.space, *s) != NULL
				|| strchr(rd->mk_text.lend, *s) != NULL) {

			if (m != 0) {

				*label = 0;
				m = 0;
			}
		}
		else {
			if (m == 0) {

				label = rd->data[dN].label[N++];

				if (N >= READ_COLUMN_MAX)
					break;
			}

			if (m < READ_TOKEN_MAX - 1) {

				*label++ = *s;
				m++;
			}
		}

		s++;
	}

	if (m != 0) {

		*label = 0;
	}

	return N;
}

static int
readTEXTSkipBOM(read_t *rd, FILE *fd)
{
	char		tbuf[8];
	int		len, bom = 0;

	len = fread(tbuf, 4, 1, fd);

	fseek(fd, 0UL, SEEK_SET);

	if (len != 0) {

		if (		   memcmp(tbuf, "\x00\x00\xFE\xFF", 4) == 0
				|| memcmp(tbuf, "\xFF\xFE\x00\x00", 4) == 0) {

			ERROR("UTF-32 signature detected\n");
			bom = 1;
		}
		else if (	   memcmp(tbuf, "\xFE\xFF", 2) == 0
				|| memcmp(tbuf, "\xFF\xFE", 2) == 0) {

			ERROR("UTF-16 signature detected\n");
			bom = 1;
		}
		else if (memcmp(tbuf, "\xEF\xBB\xBF", 3) == 0) {

			fseek(fd, 3UL, SEEK_SET);
		}
	}

	return bom;
}

static int
readTEXTGetCN(read_t *rd, int dN, FILE *fd, fval_t *rbuf, int *rbuf_N)
{
	int		label_cN, fixed_N, total_N;
	int		N, cN, timeout;
	char		*r;

	label_cN = 0;
	cN = 0;

	fixed_N = 0;
	total_N = 0;

	timeout = 100;

	do {
		r = readTimeGetBuf(rd->data[dN].buf, sizeof(rd->data[0].buf), fd, timeout);

		total_N++;

		if (r == NULL)
			break;

		if (label_cN < 1) {

			label_cN = readTEXTGetLabel(rd, dN);
		}
		else {
			cN = readTEXTGetRow(rd, dN);

			if (cN != 0) {

				if (cN > label_cN) {

					label_cN = readTEXTGetLabel(rd, dN);
					fixed_N = 0;
				}
				else {
					for (N = 0; N < cN; ++N) {

						rbuf[fixed_N * READ_COLUMN_MAX + N] = rd->data[dN].row[N];
					}

					fixed_N++;

					if (fixed_N >= 3) {

						rd->data[dN].line_N = total_N + 1;
						break;
					}
				}
			}
		}

		if (total_N >= READ_TEXT_HEADER_MAX) {

			cN = 0;
			break;
		}
	}
	while (1);

	*rbuf_N = fixed_N;

	return cN;
}

static void
readClose(read_t *rd, int dN)
{
	async_close(rd->data[dN].afd);

	if (rd->data[dN].fd != stdin) {

		fclose(rd->data[dN].fd);
	}

	rd->data[dN].fd = NULL;
	rd->data[dN].afd = NULL;

	rd->files_N -= 1;
}

void readOpenUnified(read_t *rd, int dN, int cN, int lN, const char *file, int fmt)
{
	fval_t		rbuf[READ_COLUMN_MAX * 3];
	int		N, rbuf_N;

	FILE			*fd;
	unsigned long long	bF = 0U;

	if (rd->data[dN].fd != NULL) {

		readClose(rd, dN);
	}

	if (fmt == FORMAT_PLAIN_STDIN) {

		fd = stdin;
	}
	else {
		fd = unified_fopen(file, "rb");
	}

	if (fd == NULL) {

		ERROR("fopen(\"%s\"): %s\n", file, strerror(errno));
	}
	else {
		if (fmt != FORMAT_PLAIN_STDIN) {

			file_stat(file, &bF);
		}

		rd->data[dN].length_N = lN;

		if (		fmt == FORMAT_PLAIN_STDIN
				|| fmt == FORMAT_PLAIN_TEXT) {

			if (fmt != FORMAT_PLAIN_STDIN) {

				if (readTEXTSkipBOM(rd, fd) != 0) {

					fclose(fd);
					return ;
				}
			}

			cN = readTEXTGetCN(rd, dN, fd, rbuf, &rbuf_N);

			if (cN < 1) {

				ERROR("No correct data in file \"%s\"\n", file);
				fclose(fd);
				return ;
			}

			if (bF != 0 && lN < 1) {

				/* We do not use the file size to guess the
				 * length since there is an incremental dataset
				 * memory allocation.
				 * */
				lN = 1000;
			}
			else if (lN < 1) {

				lN = rd->length_N;
			}
		}
		else if (fmt == FORMAT_BINARY_FLOAT) {

			lN = (lN < 1) ? bF / (cN * sizeof(float)) : lN;
			rd->data[dN].line_N = 1;
		}
		else if (fmt == FORMAT_BINARY_DOUBLE) {

			lN = (lN < 1) ? bF / (cN * sizeof(double)) : lN;
			rd->data[dN].line_N = 1;
		}

#ifdef _LEGACY
		else if (fmt == FORMAT_BINARY_LEGACY_V1) {

			lN = (lN < 1) ? (bF - 6) / (cN * 6) : lN;
			rd->data[dN].line_N = 1;

			fseek(fd, 6UL, SEEK_SET);
		}
		else if (fmt == FORMAT_BINARY_LEGACY_V2) {

			lN = (lN < 1) ? (bF - 4) / (cN * 4) : lN;
			rd->data[dN].line_N = 1;

			fseek(fd, 4UL, SEEK_SET);
		}
#endif /* _LEGACY */

		plotDataAlloc(rd->pl, dN, cN, lN + 1);

		if (		fmt == FORMAT_PLAIN_STDIN
				|| fmt == FORMAT_PLAIN_TEXT) {

			for (N = 0; N < rbuf_N; ++N) {

				plotDataInsert(rd->pl, dN, rbuf + READ_COLUMN_MAX * N);
			}
		}

		rd->data[dN].format = fmt;
		rd->data[dN].column_N = cN;

		strcpy(rd->data[dN].file, file);

		rd->data[dN].fd = fd;
		rd->data[dN].afd = async_open(fd, rd->preload, rd->chunk, rd->timeout);

		rd->files_N += 1;
		rd->bind_N = dN;
	}
}

void readOpenStub(read_t *rd, int dN, int cN, int lN, const char *file, int fmt)
{
	rd->data[dN].length_N = lN;

	lN = (lN < 1) ? 10 : lN;

	plotDataAlloc(rd->pl, dN, cN, lN + 1);

	rd->data[dN].format = fmt;
	rd->data[dN].column_N = cN;

	strcpy(rd->data[dN].file, file);

	rd->bind_N = dN;
}

void readToggleHint(read_t *rd, int dN, int cN)
{
	if (rd->data[dN].format == FORMAT_NONE) {

		ERROR("Dataset number %i was not allocated\n", dN);
		return ;
	}

	if (cN < 0 || cN >= rd->data[dN].column_N) {

		return ;
	}

	if (rd->data[dN].hint[cN] == DATA_HINT_NONE) {

		rd->data[dN].hint[cN] = DATA_HINT_FLOAT;
	}
	else if (rd->data[dN].hint[cN] == DATA_HINT_FLOAT) {

		rd->data[dN].hint[cN] = DATA_HINT_HEX;
	}
	else if (rd->data[dN].hint[cN] == DATA_HINT_HEX) {

		rd->data[dN].hint[cN] = DATA_HINT_OCT;
	}
	else if (rd->data[dN].hint[cN] == DATA_HINT_OCT) {

		rd->data[dN].hint[cN] = DATA_HINT_NONE;
	}
}

static int
readTEXTCSV(read_t *rd, int dN)
{
	int		r, cN;

	r = async_gets(rd->data[dN].afd, rd->data[dN].buf, sizeof(rd->data[0].buf));

	if (r == ASYNC_OK) {

		cN = readTEXTGetRow(rd, dN);

		if (cN == rd->pl->data[dN].column_N) {

			plotDataInsert(rd->pl, dN, rd->data[dN].row);
		}

		return 1;
	}
	else if (r == ASYNC_END_OF_FILE) {

		readClose(rd, dN);
	}

	return 0;
}

static int
readFLOAT(read_t *rd, int dN)
{
	float		*fb = (float *) rd->data[dN].buf;
	int		r, N, cN = rd->pl->data[dN].column_N;

	r = async_read(rd->data[dN].afd, (void *) fb, cN * sizeof(float));

	if (r == ASYNC_OK) {

		for (N = 0; N < cN; ++N)
			rd->data[dN].row[N] = (fval_t) fb[N];

		plotDataInsert(rd->pl, dN, rd->data[dN].row);

		return 1;
	}
	else if (r == ASYNC_END_OF_FILE) {

		readClose(rd, dN);
	}

	return 0;
}

static int
readDOUBLE(read_t *rd, int dN)
{
	double		*fb = (double *) rd->data[dN].buf;
	int		r, N, cN = rd->pl->data[dN].column_N;

	r = async_read(rd->data[dN].afd, (void *) fb, cN * sizeof(double));

	if (r == ASYNC_OK) {

		for (N = 0; N < cN; ++N)
			rd->data[dN].row[N] = (fval_t) fb[N];

		plotDataInsert(rd->pl, dN, rd->data[dN].row);

		return 1;
	}
	else if (r == ASYNC_END_OF_FILE) {

		readClose(rd, dN);
	}

	return 0;
}

#ifdef _LEGACY
static int
readLEGACY(read_t *rd, int dN)
{
	char		*fb = (char *) rd->data[dN].buf;
	int		r, N, cN = rd->pl->data[dN].column_N;

	if (rd->data[dN].format == FORMAT_BINARY_LEGACY_V1) {

		r = async_read(rd->data[dN].afd, (void *) fb, cN * 6);
	}
	else {
		r = async_read(rd->data[dN].afd, (void *) fb, cN * 4);
	}

	if (r == ASYNC_OK) {

		if (rd->data[dN].format == FORMAT_BINARY_LEGACY_V1) {

			for (N = 0; N < cN; ++N)
				rd->data[dN].row[N] = * (float *) (fb + N * 6 + 2);
		}
		else {
			for (N = 0; N < cN; ++N)
				rd->data[dN].row[N] = * (float *) (fb + N * 4);
		}

		plotDataInsert(rd->pl, dN, rd->data[dN].row);

		return 1;
	}
	else if (r == ASYNC_END_OF_FILE) {

		readClose(rd, dN);
	}

	return 0;
}
#endif /* _LEGACY */

int readUpdate(read_t *rd)
{
	FILE		*fd;
	int		dN, bN, tTOP, file_N = 0, ulN = 0;

	for (dN = 0; dN < PLOT_DATASET_MAX; ++dN) {

		fd = rd->data[dN].fd;

		if (fd != NULL) {

			bN = 0;
			file_N += 1;

			tTOP = SDL_GetTicks() + 20;

			do {
				if (		rd->data[dN].format == FORMAT_PLAIN_STDIN
						|| rd->data[dN].format == FORMAT_PLAIN_TEXT) {

					if (readTEXTCSV(rd, dN) != 0) {

						ulN += 1;
					}
					else {
						break;
					}
				}
				else if (rd->data[dN].format == FORMAT_BINARY_FLOAT) {

					if (readFLOAT(rd, dN) != 0) {

						ulN += 1;
					}
					else {
						break;
					}
				}
				else if (rd->data[dN].format == FORMAT_BINARY_DOUBLE) {

					if (readDOUBLE(rd, dN) != 0) {

						ulN += 1;
					}
					else {
						break;
					}
				}

#ifdef _LEGACY
				else if (rd->data[dN].format == FORMAT_BINARY_LEGACY_V1
					|| rd->data[dN].format == FORMAT_BINARY_LEGACY_V2) {

					if (readLEGACY(rd, dN) != 0) {

						ulN += 1;
					}
					else {
						break;
					}
				}
#endif /* _LEGACY */

				rd->data[dN].line_N++;
				bN++;

				if (		rd->data[dN].length_N < 1
						&& plotDataSpaceLeft(rd->pl, dN) < 10) {

					plotDataGrowUp(rd->pl, dN);
				}
			}
			while (SDL_GetTicks() < tTOP);

			plotDataSubtractResidual(rd->pl, dN);
		}
	}

	rd->files_N = (file_N < rd->files_N) ? file_N : rd->files_N;

	return ulN;
}

static int
configGetC(parse_t *pa)
{
	int		rc;

	if (pa->unchar < 0) {

		if (pa->fd != NULL) {

			rc = fgetc(pa->fd);
			rc = (rc == EOF) ? -1 : rc;
		}
		else if (pa->in != NULL) {

			rc = *pa->in++;
			rc = (rc == 0) ? -1 : rc;
		}
		else {
			rc = -1;
		}
	}
	else {
		rc = pa->unchar;
		pa->unchar = -1;
	}

	return rc;
}

static int
configUngetC(parse_t *pa, int c)
{
	pa->unchar = c;

	return c;
}

static int
configToken(read_t *rd, parse_t *pa)
{
	char		*p = pa->tbuf;
	int		c, n, rc = 0;

	do { c = configGetC(pa); }
	while (strchr(rd->mk_config.space, c) != NULL);

	if (c < 0) {

		rc = -1;
	}
	else if (c == '"') {

		c = configGetC(pa);
		n = 0;

		while (c != -1 && c != '"' && strchr(rd->mk_config.lend, c) == NULL) {

			if (n < READ_FILE_PATH_MAX - 1) {

				*p++ = c;
				n++;
			}

			c = configGetC(pa);
		}

		if (strchr(rd->mk_config.lend, c) != NULL)
			configUngetC(pa, c);

		*p = 0;
	}
	else if (strchr(rd->mk_config.lend, c) != NULL) {

		pa->line_N++;
		pa->newline = 1;

		rc = 1;
	}
	else {
		n = 0;

		do {
			if (n < READ_TOKEN_MAX) {

				*p++ = c;
				n++;
			}

			c = configGetC(pa);
		}
		while (c != -1  && strchr(rd->mk_config.space, c) == NULL
				&& strchr(rd->mk_config.lend, c) == NULL);

		if (strchr(rd->mk_config.lend, c) != NULL)
			configUngetC(pa, c);

		*p = 0;
	}

	return rc;
}

static void
configDataMap(read_t *rd, parse_t *pa)
{
	int		dN, N = 0;

	for (dN = 0; dN < PLOT_DATASET_MAX; ++dN)
		pa->dmap[dN] = -1;

	for (dN = 0; dN < PLOT_DATASET_MAX; ++dN) {

		if (		rd->data[dN].format == FORMAT_NONE
				&& rd->data[dN].file[0] == 0) {

			pa->dmap[N++] = dN;
		}
	}
}

static int
configGetSubtract(read_t *rd, int pN, int fN, int axis)
{
	int		N, sN = -1;

	for (N = 0; N < READ_SUBTRACT_MAX; ++N) {

		if (axis == 0) {

			if (rd->page[pN].fig[fN].bX[N].busy == SUBTRACT_FREE) {

				sN = N;
				break;
			}
		}
		else if (axis == 1) {

			if (rd->page[pN].fig[fN].bY[N].busy == SUBTRACT_FREE) {

				sN = N;
				break;
			}
		}
	}

	return sN;
}

static void
configParseFSM(read_t *rd, parse_t *pa)
{
	char		msg_tbuf[READ_FILE_PATH_MAX];
	char 		lpath[READ_FILE_PATH_MAX];
	char 		lname[READ_FILE_PATH_MAX];

	char		*lbuf, *tbuf = pa->tbuf;
	int		r, failed, N;

	double		argd[2];
	int		argi[4];

	FILE		*fd;
	parse_t		rpa;

	configDataMap(rd, pa);

	failed = 0;

	do {
		r = configToken(rd, pa);

		if (r < 0) break;
		else if (r == 1) {

			failed = 0;
			continue;
		}
		else if (r == 0 && pa->newline != 0) {

			pa->newline = 0;

			sprintf(msg_tbuf, "unable to parse \"%.80s\"", tbuf);

			if (tbuf[0] == '#') {

				failed = 0;
				while (configToken(rd, pa) == 0) ;
			}
			else if (strcmp(tbuf, "include") == 0) {

				r = configToken(rd, pa);

				if (r == 0) {

					lbuf = tbuf;

					if (pa->path != NULL && tbuf[0] != '/') {

						sprintf(lpath, "%s/%s", pa->path, tbuf);
						lbuf = lpath;
					}

					fd = unified_fopen(lbuf, "r");

					if (fd == NULL) {

						ERROR("fopen(\"%s\"): %s\n", tbuf, strerror(errno));
						failed = 1;
					}
					else {
						strcpy(rpa.file, tbuf);

						rpa.path = pa->path;
						rpa.fd = fd;
						rpa.in = NULL;

						rpa.unchar = -1;
						rpa.line_N = 1;
						rpa.newline = 1;
						rpa.fromUI = pa->fromUI;

						configParseFSM(rd, &rpa);

						fclose(fd);
					}
				}
				else {
					failed = 1;
				}
			}
			else if (strcmp(tbuf, "gpconfig") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] > 0) {

						failed = 0;
						rd->config_version = argi[0];
					}
					else {
						sprintf(msg_tbuf, "unknown config version %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "font") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0) {

						if (strcmp(tbuf, "normal") == 0) {

							plotFontDefault(rd->pl, TTF_ID_ROBOTO_MONO_NORMAL,
									argi[0], TTF_STYLE_NORMAL);
						}
						else if (strcmp(tbuf, "normal-bold") == 0) {

							plotFontDefault(rd->pl, TTF_ID_ROBOTO_MONO_NORMAL,
									argi[0], TTF_STYLE_BOLD);
						}
						else if (strcmp(tbuf, "normal-italic") == 0) {

							plotFontDefault(rd->pl, TTF_ID_ROBOTO_MONO_NORMAL,
									argi[0], TTF_STYLE_ITALIC);
						}
						else if (strcmp(tbuf, "thin") == 0) {

							plotFontDefault(rd->pl, TTF_ID_ROBOTO_MONO_THIN,
									argi[0], TTF_STYLE_NORMAL);
						}
						else if (strcmp(tbuf, "thin-bold") == 0) {

							plotFontDefault(rd->pl, TTF_ID_ROBOTO_MONO_THIN,
									argi[0], TTF_STYLE_BOLD);
						}
						else if (strcmp(tbuf, "thin-italic") == 0) {

							plotFontDefault(rd->pl, TTF_ID_ROBOTO_MONO_THIN,
									argi[0], TTF_STYLE_ITALIC);
						}
						else {
							plotFontOpen(rd->pl, tbuf, argi[0], TTF_STYLE_NORMAL);
						}

						if (rd->pl->font != NULL) {

							failed = 0;
						}
					}
				}
				while (0);
			}

#ifdef _WINDOWS
			else if (strcmp(tbuf, "legacy_label") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					failed = 0;
					rd->legacy_label = argi[0];
				}
				while (0);
			}
#endif /* _WINDOWS */

			else if (strcmp(tbuf, "preload") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] > sizeof(rd->data[0].buf)) {

						failed = 0;
						rd->preload = argi[0];
					}
					else {
						sprintf(msg_tbuf, "preload size %i is too small", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "chunk") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] > 0) {

						failed = 0;
						rd->chunk = argi[0];
					}
					else {
						sprintf(msg_tbuf, "chunk size %i must be positive", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "timeout") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0) {

						failed = 0;
						rd->timeout = argi[0];
					}
					else {
						sprintf(msg_tbuf, "timeout %i must be non-negative", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "length") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] > 0) {

						failed = 0;
						rd->length_N = argi[0];
					}
					else {
						sprintf(msg_tbuf, "data length %i must be positive", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "screenpath") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0) {

						failed = 0;
						strcpy(rd->screenpath, tbuf);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "windowsize") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[1], tbuf) != NULL) ;
					else break;

					if (argi[0] >= GP_MIN_SIZE_X && argi[1] >= GP_MIN_SIZE_Y) {

						failed = 0;
						rd->window_size_x = argi[0];
						rd->window_size_y = argi[1];
					}
					else {
						sprintf(msg_tbuf, "too small window sizes %i %i", argi[0], argi[1]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "language") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= LANG_EN && argi[0] < LANG_END_OF_LIST) {

						failed = 0;
						rd->language = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid language number %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "colorscheme") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0 && argi[0] <= 2) {

						failed = 0;
						rd->colorscheme = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid colorscheme number %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "antialiasing") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0 && argi[0] < 3) {

						failed = 0;
						rd->dw->antialiasing = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid antialiasing %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "blendfont") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0 && argi[0] <= 1) {

						failed = 0;
						rd->dw->blendfont = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid blendfont %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "thickness") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0 && argi[0] < 3) {

						failed = 0;
						rd->dw->thickness = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid thickness %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "gamma") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] > 0 && argi[0] < 1000) {

						failed = 0;
						rd->dw->gamma = argi[0];

						drawGamma(rd->dw);
					}
					else {
						sprintf(msg_tbuf, "invalid gamma %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "timecol") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= -1 && argi[0] < READ_COLUMN_MAX) {

						failed = 0;
						rd->timecol = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid column number %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "shortfilename") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0) {

						failed = 0;
						rd->shortfilename = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid number of dirs %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "fastdraw") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0) {

						failed = 0;
						rd->fastdraw = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid fastdraw %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "interpolation") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0 && argi[0] <= 1) {

						failed = 0;
						rd->pl->interpolation = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid interpolation %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "defungap") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0.) {

						failed = 0;
						rd->pl->defungap = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid defungap %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "precision") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 1 && argi[0] <= 16) {

						failed = 0;
						rd->pl->fprecision = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid precision %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "delim") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0) {

						failed = 0;
						rd->mk_text.delim = tbuf[0];
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "space") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0) {

						failed = 0;
						strcpy(rd->mk_text.space, rd->mk_config.space);
						strcat(rd->mk_text.space, tbuf);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "lz4_compress") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (rd->bind_N != -1) {

						if (pa->fromUI != 0) {

							failed = 0;
							break;
						}

						sprintf(msg_tbuf, "unable if dataset was already opened");
						break;
					}

					if (argi[0] >= 0 && argi[0] < 2) {

						failed = 0;
						rd->pl->lz4_compress = argi[0];
					}
					else {
						sprintf(msg_tbuf, "invalid lz4_compress %i", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "load") == 0) {

				int		flag_stub = 0;

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] < 0 || argi[0] >= PLOT_DATASET_MAX) {

						sprintf(msg_tbuf, "dataset number %i is out of range", argi[0]);
						break;
					}

					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[1], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0) {

						if (strcmp(tbuf, "stdin") == 0) {

							argi[2] = FORMAT_PLAIN_STDIN;
						}
						else if (strcmp(tbuf, "text") == 0) {

							argi[2] = FORMAT_PLAIN_TEXT;
						}
						else if (strcmp(tbuf, "float") == 0) {

							argi[2] = FORMAT_BINARY_FLOAT;
						}
						else if (strcmp(tbuf, "double") == 0) {

							argi[2] = FORMAT_BINARY_DOUBLE;
						}
						else {
							sprintf(msg_tbuf, "invalid file format \"%.80s\"", tbuf);
							break;
						}
					}
					else break;

					if (argi[2] == FORMAT_PLAIN_STDIN) {

						int		dN_remap;

						dN_remap = pa->dmap[argi[0]];

						if (dN_remap < 0) {

							sprintf(msg_tbuf, "no free dataset to remap %i", argi[0]);
							break;
						}

						readOpenUnified(rd, dN_remap, argi[3], argi[1], "", argi[2]);

						failed = 0;
						break;
					}
					else if (	argi[2] == FORMAT_BINARY_FLOAT
							|| argi[2] == FORMAT_BINARY_DOUBLE) {

						r = configToken(rd, pa);

						if (r == 0 && stoi(&rd->mk_config, &argi[3], tbuf) != NULL) ;
						else break;

						flag_stub = 1;
					}

					r = configToken(rd, pa);

					if (r == 0) {

						int		dN_remap;

						dN_remap = pa->dmap[argi[0]];

						if (dN_remap < 0) {

							sprintf(msg_tbuf, "no free dataset to remap %i", argi[0]);
							break;
						}

						lbuf = tbuf;

						if (pa->path != NULL && tbuf[0] != '/') {

							sprintf(lpath, "%s/%s", pa->path, tbuf);
							lbuf = lpath;
						}

						readOpenUnified(rd, dN_remap, argi[3], argi[1], lbuf, argi[2]);

						if (		rd->data[dN_remap].fd == NULL
								&& flag_stub != 0) {

							readOpenStub(rd, dN_remap, argi[3], argi[1], lbuf, argi[2]);
						}

						failed = 0;
					}
					else break;
				}
				while (0);
			}
			else if (strcmp(tbuf, "bind") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0 && argi[0] < PLOT_DATASET_MAX) {

						int		dN_remap;

						dN_remap = pa->dmap[argi[0]];

						if (		dN_remap >= 0 &&
								rd->data[dN_remap].format != FORMAT_NONE) {

							failed = 0;
							rd->bind_N = dN_remap;
						}
						else {
							sprintf(msg_tbuf, "no dataset has a number %i", argi[0]);
						}
					}
					else {
						sprintf(msg_tbuf, "dataset number %i is out of range", argi[0]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "group") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (argi[0] < 0 || argi[0] >= PLOT_GROUP_MAX) {

						sprintf(msg_tbuf, "group number %i is out of range", argi[0]);
						break;
					}

					if (rd->bind_N < 0) {

						sprintf(msg_tbuf, "no dataset selected");
						break;
					}

					do {
						r = configToken(rd, pa);

						if (r != 0) {

							failed = 0;
							break;
						}

						if (stoi(&rd->mk_config, &argi[1], tbuf) != NULL) ;
						else break;

						if (argi[1] >= -1 && argi[1] < rd->pl->data[rd->bind_N].column_N) {

							failed = 0;

							plotGroupAdd(rd->pl, rd->bind_N, argi[0], argi[1]);
						}
						else {
							sprintf(msg_tbuf, "column number %i is out of range", argi[1]);
							break;
						}
					}
					while (1);
				}
				while (0);
			}
			else if (strcmp(tbuf, "deflabel") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (argi[0] >= 0 && argi[0] < PLOT_GROUP_MAX) {

						failed = 0;

						plotGroupLabel(rd->pl, argi[0], tbuf);
					}
					else {
						sprintf(msg_tbuf, "group number %i is out of range", argi[0]);
						break;
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "defmedian") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[1], tbuf) != NULL) ;
					else break;

					if (argi[1] < 1 || argi[1] > PLOT_MEDIAN_MAX) {

						sprintf(msg_tbuf, "median length %i is out of range", argi[1]);
						break;
					}

					argi[2] = 0;
					argi[3] = 0;

					r = configToken(rd, pa);

					if (r == 0) {

						stoi(&rd->mk_config, &argi[2], tbuf);

						r = configToken(rd, pa);

						if (r == 0) {

							stoi(&rd->mk_config, &argi[3], tbuf);
						}
					}

					if (argi[0] >= 0 && argi[0] < PLOT_GROUP_MAX) {

						failed = 0;

						plotGroupMedian(rd->pl, argi[0], argi[1], argi[2], argi[3]);
					}
					else {
						sprintf(msg_tbuf, "group number %i is out of range", argi[0]);
						break;
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "defscale") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stod(&rd->mk_config, &argd[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stod(&rd->mk_config, &argd[1], tbuf) != NULL) ;
					else break;

					if (argi[0] >= 0 && argi[0] < PLOT_GROUP_MAX) {

						failed = 0;

						plotGroupScale(rd->pl, argi[0], 1, argd[0], argd[1]);
					}
					else {
						sprintf(msg_tbuf, "group number %i is out of range", argi[0]);
						break;
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "page") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (rd->bind_N < 0) {

						sprintf(msg_tbuf, "no dataset selected");
						break;
					}

					if (r == 0) {

						failed = 0;

						if (rd->page_N < 0) {

							rd->page_N = 1;
						}

						while (rd->page[rd->page_N].busy != 0) {

							rd->page_N++;

							if (rd->page_N >= READ_PAGE_MAX)
								break;
						}

						if (rd->page_N >= READ_PAGE_MAX) {

							sprintf(msg_tbuf, "no free pages to remap");
							break;
						}

						rd->figure_N = -1;
						rd->page[rd->page_N].busy = 1;

						lbuf = tbuf;

						if (pa->fromUI != 0) {

							readCutFile(rd, lpath, pa->file, 95);

							sprintf(lname, "%s: %.95s", lpath, tbuf);
							lbuf = lname;
						}

						readCutLabel(rd->page[rd->page_N].title, lbuf, PLOT_STRING_MAX);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "mkpages") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					if (rd->bind_N < 0) {

						sprintf(msg_tbuf, "no dataset selected");
						break;
					}

					if (argi[0] >= -2 && argi[0] < rd->pl->data[rd->bind_N].column_N) {

						failed = 0;

						readMakePages(rd, rd->bind_N, argi[0], pa->fromUI);
					}
					else {
						sprintf(msg_tbuf, "column number %i is out of range", argi[0]);
						break;
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "label") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (rd->page_N < 0) {

						sprintf(msg_tbuf, "no page selected");
						break;
					}

					if (r == 0) {

						if (argi[0] >= 0 && argi[0] < PLOT_AXES_MAX) {

							failed = 0;
							strcpy(rd->page[rd->page_N].ax[argi[0]].label, tbuf);
						}
						else {
							sprintf(msg_tbuf, "axis number %i is out of range", argi[0]);
						}
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "slave") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[1], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stod(&rd->mk_config, &argd[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stod(&rd->mk_config, &argd[1], tbuf) != NULL) ;
					else break;

					if (rd->page_N < 0) {

						sprintf(msg_tbuf, "no page selected");
						break;
					}

					if (argi[0] >= 0 && argi[0] < PLOT_AXES_MAX
						&& argi[1] >= 0 && argi[1] < PLOT_AXES_MAX) {

						failed = 0;
						rd->page[rd->page_N].ax[argi[0]].slave = 1;
						rd->page[rd->page_N].ax[argi[0]].slave_N = argi[1];
						rd->page[rd->page_N].ax[argi[0]].scale = argd[0];
						rd->page[rd->page_N].ax[argi[0]].offset = argd[1];
					}
					else {
						sprintf(msg_tbuf, "axes numbers %i %i are out of range", argi[0], argi[1]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "figure") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[1], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (rd->page_N < 0) {

						sprintf(msg_tbuf, "no page selected");
						break;
					}

					if (rd->bind_N < 0) {

						sprintf(msg_tbuf, "no dataset selected");
						break;
					}

					if (argi[0] < -1 || argi[0] >= rd->pl->data[rd->bind_N].column_N
						|| argi[1] < -1 || argi[1] >= rd->pl->data[rd->bind_N].column_N) {

						sprintf(msg_tbuf, "column numbers %i %i are out of range", argi[0], argi[1]);
						break;
					}

					if (r == 0) {

						if (rd->figure_N < PLOT_FIGURE_MAX - 1) {

							failed = 0;
							rd->figure_N++;

							rd->page[rd->page_N].fig[rd->figure_N].busy = 1;
							rd->page[rd->page_N].fig[rd->figure_N].drawing = -1;
							rd->page[rd->page_N].fig[rd->figure_N].dN = rd->bind_N;
							rd->page[rd->page_N].fig[rd->figure_N].cX = argi[0];
							rd->page[rd->page_N].fig[rd->figure_N].cY = argi[1];
							rd->page[rd->page_N].fig[rd->figure_N].aX = 0;
							rd->page[rd->page_N].fig[rd->figure_N].aY = 1;

							strcpy(rd->page[rd->page_N].fig[rd->figure_N].label, tbuf);
						}
						else {
							sprintf(msg_tbuf, "too many figures on page %i", rd->page_N);
						}
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "map") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[1], tbuf) != NULL) ;
					else break;

					if (rd->figure_N < 0) {

						sprintf(msg_tbuf, "no figure selected");
						break;
					}

					if (argi[0] >= 0 && argi[0] < PLOT_AXES_MAX
						&& argi[1] >= 0 && argi[1] < PLOT_AXES_MAX) {

						failed = 0;
						rd->page[rd->page_N].fig[rd->figure_N].aX = argi[0];
						rd->page[rd->page_N].fig[rd->figure_N].aY = argi[1];
					}
					else {
						sprintf(msg_tbuf, "axes numbers %i %i are out of range", argi[0], argi[1]);
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "xscale") == 0 || strcmp(tbuf, "yscale") == 0) {

				failed = 1;

				do {
					argi[0] = (int) tbuf[0];

					r = configToken(rd, pa);

					if (r == 0 && stod(&rd->mk_config, &argd[0], tbuf) != NULL) ;
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stod(&rd->mk_config, &argd[1], tbuf) != NULL) ;
					else break;

					if (rd->figure_N < 0) {

						sprintf(msg_tbuf, "no figure selected");
						break;
					}

					if (argi[0] == 'x') {

						N = configGetSubtract(rd, rd->page_N, rd->figure_N, 0);

						if (N < 0) {

							sprintf(msg_tbuf, "no free subtract found");
						}
						else {
							failed = 0;

							rd->page[rd->page_N].fig[rd->figure_N].bX[N].busy = SUBTRACT_SCALE;
							rd->page[rd->page_N].fig[rd->figure_N].bX[N].args[0] = argd[0];
							rd->page[rd->page_N].fig[rd->figure_N].bX[N].args[1] = argd[1];
						}
					}
					else if (argi[0] == 'y') {

						N = configGetSubtract(rd, rd->page_N, rd->figure_N, 1);

						if (N < 0) {

							sprintf(msg_tbuf, "no free subtract found");
						}
						else {
							failed = 0;

							rd->page[rd->page_N].fig[rd->figure_N].bY[N].busy = SUBTRACT_SCALE;
							rd->page[rd->page_N].fig[rd->figure_N].bY[N].args[0] = argd[0];
							rd->page[rd->page_N].fig[rd->figure_N].bY[N].args[1] = argd[1];
						}
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "xsubtract") == 0 || strcmp(tbuf, "ysubtract") == 0) {

				failed = 1;

				do {
					argi[0] = (int) tbuf[0];

					r = configToken(rd, pa);

					if (r == 0) {

						if (strcmp(tbuf, "sub") == 0) {

							argi[1] = SUBTRACT_BINARY_SUBTRACTION;
						}
						else if (strcmp(tbuf, "add") == 0) {

							argi[1] = SUBTRACT_BINARY_ADDITION;
						}
						else if (strcmp(tbuf, "mul") == 0) {

							argi[1] = SUBTRACT_BINARY_MULTIPLICATION;
						}
						else if (strcmp(tbuf, "hyp") == 0) {

							argi[1] = SUBTRACT_BINARY_HYPOTENUSE;
						}
						else {
							sprintf(msg_tbuf, "invalid subtract operation \"%.80s\"", tbuf);
							break;
						}
					}

					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[2], tbuf) != NULL) ;
					else break;

					if (rd->figure_N < 0) {

						sprintf(msg_tbuf, "no figure selected");
						break;
					}

					if (argi[0] == 'x') {

						N = configGetSubtract(rd, rd->page_N, rd->figure_N, 0);

						if (N < 0) {

							sprintf(msg_tbuf, "no free subtract found");
						}
						else {
							failed = 0;

							rd->page[rd->page_N].fig[rd->figure_N].bX[N].busy = argi[1];
							rd->page[rd->page_N].fig[rd->figure_N].bX[N].column_2 = argi[2];
						}
					}
					else if (argi[0] == 'y') {

						N = configGetSubtract(rd, rd->page_N, rd->figure_N, 1);

						if (N < 0) {

							sprintf(msg_tbuf, "no free subtract found");
						}
						else {
							failed = 0;

							rd->page[rd->page_N].fig[rd->figure_N].bY[N].busy = argi[1];
							rd->page[rd->page_N].fig[rd->figure_N].bY[N].column_2 = argi[2];
						}
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "xfilter") == 0 || strcmp(tbuf, "yfilter") == 0) {

				failed = 1;

				do {
					argi[0] = (int) tbuf[0];

					r = configToken(rd, pa);

					if (r == 0) {

						if (strcmp(tbuf, "diff") == 0) {

							argi[1] = SUBTRACT_FILTER_DIFFERENCE;
						}
						else if (strcmp(tbuf, "csum") == 0) {

							argi[1] = SUBTRACT_FILTER_CUMULATIVE;
						}
						else if (strcmp(tbuf, "bf") == 0) {

							argi[1] = SUBTRACT_FILTER_BITMASK;

							r = configToken(rd, pa);

							if (r == 0 && stoi(&rd->mk_config, &argi[2], tbuf) != NULL) ;
							else break;

							r = configToken(rd, pa);

							if (r != 0) {

								if (stoi(&rd->mk_config, &argi[3], tbuf) != NULL) ;
								else break;
							}
							else {
								argi[3] = argi[2];
							}
						}
						else if (strcmp(tbuf, "low") == 0) {

							argi[1] = SUBTRACT_FILTER_LOW_PASS;

							r = configToken(rd, pa);

							if (r == 0 && stod(&rd->mk_config, &argd[0], tbuf) != NULL) ;
							else break;
						}
						else if (strcmp(tbuf, "med") == 0) {

							argi[1] = SUBTRACT_FILTER_MEDIAN;

							r = configToken(rd, pa);

							if (r == 0 && stoi(&rd->mk_config, &argi[2], tbuf) != NULL) ;
							else break;

							if (argi[2] < 3 || argi[2] > PLOT_MEDIAN_MAX) {

								sprintf(msg_tbuf, "median length %i is out of range", argi[2]);
								break;
							}
						}
						else {
							sprintf(msg_tbuf, "invalid filter operation \"%.80s\"", tbuf);
							break;
						}
					}

					if (rd->figure_N < 0) {

						sprintf(msg_tbuf, "no figure selected");
						break;
					}

					if (argi[0] == 'x') {

						N = configGetSubtract(rd, rd->page_N, rd->figure_N, 0);

						if (N < 0) {

							sprintf(msg_tbuf, "no free subtract found");
						}
						else {
							failed = 0;

							rd->page[rd->page_N].fig[rd->figure_N].bX[N].busy = argi[1];

							if (argi[1] == SUBTRACT_FILTER_BITMASK) {

								rd->page[rd->page_N].fig[rd->figure_N].bX[N].args[0] = argi[2];
								rd->page[rd->page_N].fig[rd->figure_N].bX[N].args[1] = argi[3];
							}
							else if (argi[1] == SUBTRACT_FILTER_LOW_PASS) {

								rd->page[rd->page_N].fig[rd->figure_N].bX[N].args[0] = argd[0];
							}
							else if (argi[1] == SUBTRACT_FILTER_MEDIAN) {

								rd->page[rd->page_N].fig[rd->figure_N].bX[N].args[0] = argi[2];
							}
						}
					}
					else if (argi[0] == 'y') {

						N = configGetSubtract(rd, rd->page_N, rd->figure_N, 1);

						if (N < 0) {

							sprintf(msg_tbuf, "no free subtract found");
						}
						else {
							failed = 0;

							rd->page[rd->page_N].fig[rd->figure_N].bY[N].busy = argi[1];

							if (argi[1] == SUBTRACT_FILTER_BITMASK) {

								rd->page[rd->page_N].fig[rd->figure_N].bY[N].args[0] = argi[2];
								rd->page[rd->page_N].fig[rd->figure_N].bY[N].args[1] = argi[3];
							}
							else if (argi[1] == SUBTRACT_FILTER_LOW_PASS) {

								rd->page[rd->page_N].fig[rd->figure_N].bY[N].args[0] = argd[0];
							}
							else if (argi[1] == SUBTRACT_FILTER_MEDIAN) {

								rd->page[rd->page_N].fig[rd->figure_N].bY[N].args[0] = argi[2];
							}
						}
					}
				}
				while (0);
			}
			else if (strcmp(tbuf, "drawing") == 0) {

				failed = 1;

				do {
					r = configToken(rd, pa);

					if (r == 0) {

						if (strcmp(tbuf, "line") == 0)
							argi[0] = FIGURE_DRAWING_LINE;
						else if (strcmp(tbuf, "dash") == 0)
							argi[0] = FIGURE_DRAWING_DASH;
						else if (strcmp(tbuf, "dot") == 0)
							argi[0] = FIGURE_DRAWING_DOT;
						else {
							sprintf(msg_tbuf, "invalid drawing \"%.80s\"", tbuf);
							break;
						}
					}
					else break;

					r = configToken(rd, pa);

					if (r == 0 && stoi(&rd->mk_config, &argi[1], tbuf) != NULL) ;
					else break;

					if (argi[1] >= 0 && argi[1] <= 16) {

						failed = 0;

						if (rd->figure_N < 0) {

							rd->pl->default_drawing = argi[0];
							rd->pl->default_width = argi[1];
						}
						else {
							rd->page[rd->page_N].fig[rd->figure_N].drawing = argi[0];
							rd->page[rd->page_N].fig[rd->figure_N].width = argi[1];
						}
					}
					else {
						sprintf(msg_tbuf, "figure width %i is out of range", argi[1]);
					}
				}
				while (0);
			}
			else {
				failed = 1;

				sprintf(msg_tbuf, "unknown token \"%.80s\"", tbuf);
			}

			if (failed) {

				ERROR("%s:%i: %s\n", pa->file, pa->line_N, msg_tbuf);
			}
		}
		else if (r == 0 && pa->newline == 0) {

			sprintf(msg_tbuf, "extra token \"%.80s\"", tbuf);

			ERROR("%s:%i: %s\n", pa->file, pa->line_N, msg_tbuf);
		}
	}
	while (1);
}

void readConfigIN(read_t *rd, const char *config, int fromUI)
{
	parse_t		pa;
	int		pN;

	strcpy(pa.file, "inline");

	pa.path = NULL;
	pa.fd = NULL;
	pa.in = config;

	pa.unchar = -1;
	pa.line_N = 1;
	pa.newline = 1;
	pa.fromUI = fromUI;

	pN = rd->page_N;

	configParseFSM(rd, &pa);

	if (fromUI != 0) {

		rd->page_N = pN;
	}
}

static char *
readDirName(char *path)
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

void readConfigGP(read_t *rd, const char *file, int fromUI)
{
	char 		lpath[READ_FILE_PATH_MAX];

	FILE		*fd;
	parse_t		pa;
	int		pN;

	fd = unified_fopen(file, "r");

	if (fd == NULL) {

		ERROR("fopen(\"%s\"): %s\n", file, strerror(errno));
	}
	else {
		if (readTEXTSkipBOM(rd, fd) != 0) {

			fclose(fd);
			return ;
		}

		strcpy(pa.file, file);
		strcpy(lpath, file);

		pa.path = readDirName(lpath);
		pa.fd = fd;
		pa.in = NULL;

		pa.unchar = -1;
		pa.line_N = 1;
		pa.newline = 1;
		pa.fromUI = fromUI;

		pN = rd->page_N;

		configParseFSM(rd, &pa);

		if (fromUI != 0) {

			rd->page_N = pN;
		}

		fclose(fd);
	}
}

void readConfigVerify(read_t *rd)
{
	if (rd->pl->font == NULL) {

		plotFontDefault(rd->pl, TTF_ID_ROBOTO_MONO_NORMAL, 24, TTF_STYLE_NORMAL);
	}

	rd->page_N = -1;
}

static void
readGetUnit(char *tbuf, const char *label, int allowed)
{
	const char		*text;
	int			length;

	text = strchr(label, '@');

	if (text != NULL) {

		text += 1;
		length = utf8_length(text);

		if (length > (allowed - 1)) {

			text = utf8_skip(text, length - (allowed - 1));

			strcpy(tbuf, "~");
			strcat(tbuf, text);
		}
		else {
			strcpy(tbuf, text);
		}
	}
	else {
		*tbuf = 0;
	}
}

void readMakePages(read_t *rd, int dN, int cX, int fromUI)
{
	char		tbuf[READ_FILE_PATH_MAX];
	char		sbuf[READ_FILE_PATH_MAX];
	int		N, pN;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return ;
	}

	cX = (cX < -1) ? rd->timecol : cX;
	pN = rd->page_N;

	if (cX < -1 || cX >= rd->pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Time column number %i is out of range\n", cX);
		return ;
	}

	for (N = 0; N < rd->data[dN].column_N; ++N) {

		if (pN < 0) {

			pN = 1;
		}

		while (rd->page[pN].busy != 0) {

			pN++;

			if (pN >= READ_PAGE_MAX)
				break;
		}

		if (pN >= READ_PAGE_MAX)
			break;

		rd->page[pN].busy = 2;

		readCutFile(rd, tbuf, rd->data[dN].file, 95);

		sprintf(sbuf, "%s: [%2i] %.95s", tbuf, N, rd->data[dN].label[N]);
		readCutLabel(rd->page[pN].title, sbuf, PLOT_STRING_MAX);

		rd->page[pN].fig[0].busy = 1;
		rd->page[pN].fig[0].drawing = -1;
		rd->page[pN].fig[0].dN = dN;
		rd->page[pN].fig[0].cX = cX;
		rd->page[pN].fig[0].cY = N;
		rd->page[pN].fig[0].aX = 0;
		rd->page[pN].fig[0].aY = 1;

		readCutFile(rd, tbuf, rd->data[dN].file, 20);

		sprintf(sbuf, "%s: [%2i] %.95s", tbuf, N, rd->data[dN].label[N]);
		readCutLabel(rd->page[pN].fig[0].label, sbuf, PLOT_STRING_MAX);

		if (cX >= 0) {

			readGetUnit(tbuf, rd->data[dN].label[cX], 20);
			strcpy(rd->page[pN].ax[0].label, tbuf);
		}
		else {
			rd->page[pN].ax[0].label[0] = 0;
		}

		readGetUnit(tbuf, rd->data[dN].label[N], 20);
		strcpy(rd->page[pN].ax[1].label, tbuf);
	}

	if (fromUI == 0) {

		rd->page_N = pN;
		rd->figure_N = -1;
	}
}

void readDatasetClean(read_t *rd, int dN)
{
	page_t		*pg;
	int		N, pN, pW, fN;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return ;
	}

	pN = 1;

	do {
		if (rd->page[pN].busy != 0) {

			pg = rd->page + pN;

			for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

				if (pg->fig[fN].dN == dN)
					pg->fig[fN].busy = 0;
			}

			N = 0;

			for (fN = 0; fN < PLOT_FIGURE_MAX; ++fN) {

				if (pg->fig[fN].busy != 0)
					++N;
			}

			if (N == 0) {

				memset(&rd->page[pN], 0, sizeof(rd->page[0]));
			}
		}

		pN += 1;

		if (pN >= READ_PAGE_MAX)
			break;
	}
	while (1);

	pN = 1;
	pW = 1;

	do {
		if (rd->page[pN].busy != 0) {

			if (pN != pW) {

				memcpy(&rd->page[pW], &rd->page[pN], sizeof(rd->page[0]));
				memset(&rd->page[pN], 0, sizeof(rd->page[0]));
			}

			pW += 1;
		}

		pN += 1;

		if (pN >= READ_PAGE_MAX)
			break;
	}
	while (1);

	if (rd->data[dN].fd != NULL) {

		readClose(rd, dN);
	}

	memset(&rd->data[dN], 0, sizeof(rd->data[0]));

	plotFigureGarbage(rd->pl, dN);

	plotDataRangeCacheClean(rd->pl, dN);
	plotDataClean(rd->pl, dN);
}

int readGetTimeColumn(read_t *rd, int dN)
{
	page_t		*pg;
	int		cNP, pN;

	cNP = -2;
	pN = 1;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return cNP;
	}

	do {
		if (rd->page[pN].busy == 2) {

			pg = rd->page + pN;

			if (pg->fig[0].dN == dN) {

				cNP = pg->fig[0].cX;
				break;
			}
		}

		pN += 1;

		if (pN >= READ_PAGE_MAX)
			break;
	}
	while (1);

	return cNP;
}

void readSetTimeColumn(read_t *rd, int dN, int cX)
{
	char		tbuf[READ_TOKEN_MAX];
	page_t		*pg;
	int		gN, cNP, pN;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return ;
	}

	if (cX < -1 || cX >= rd->pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Time column number %i is out of range\n", cX);
		return ;
	}

	cNP = -2;
	pN = 1;

	do {
		if (rd->page[pN].busy == 2) {

			pg = rd->page + pN;

			if (pg->fig[0].dN == dN) {

				cNP = (cNP == -2) ? pg->fig[0].cX : cNP;

				pg->fig[0].cX = cX;

				if (cX >= 0) {

					readGetUnit(tbuf, rd->data[dN].label[cX], 20);
					strcpy(rd->page[pN].ax[0].label, tbuf);
				}
				else {
					rd->page[pN].ax[0].label[0] = 0;
				}
			}
		}

		pN += 1;

		if (pN >= READ_PAGE_MAX)
			break;
	}
	while (1);

	if (cNP != -2) {

		gN = rd->pl->data[dN].map[cNP];

		if (gN != -1) {

			rd->pl->data[dN].map[cNP] = -1;
			rd->pl->data[dN].map[cX] = gN;
		}
	}
}

static tuple_t
readTimeDataMap(plot_t *pl, int dN, int cNX, int cNY)
{
	tuple_t		uMAP, uN = { cNX, cNY };
	int		gN;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return uN;
	}

	if (cNX < -1 || cNX >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Column number %i is out of range\n", cNX);
		return uN;
	}

	if (cNY < -1 || cNY >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Column number %i is out of range\n", cNY);
		return uN;
	}

	if (pl->data[dN].map == NULL) {

		ERROR("Dataset number %i was not allocated\n", dN);
		return uN;
	}

	gN = pl->data[dN].map[cNX];

	if (gN != -1) {

		if (pl->group[gN].op_time_median != 0) {

			uMAP = plotGetSubtractTimeMedian(pl, dN, cNX, cNY,
					pl->group[gN].length, pl->group[gN].op_time_unwrap,
					pl->group[gN].op_time_opdata);

			if (uMAP.X != -1) {

				pl->data[dN].map[uMAP.X] = gN;

				uN = uMAP;
			}
		}
	}

	return uN;
}

static int
readScaleDataMap(plot_t *pl, int dN, int cN, subtract_t *sb)
{
	int		N, gN, cMAP;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return cN;
	}

	if (cN < -1 || cN >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Column number %i is out of range\n", cN);
		return cN;
	}

	if (pl->data[dN].map == NULL) {

		ERROR("Dataset number %i was not allocated\n", dN);
		return cN;
	}

	gN = pl->data[dN].map[cN];

	if (gN != -1) {

		if (pl->group[gN].op_scale != 0) {

			cMAP = plotGetSubtractScale(pl, dN, cN,
					pl->group[gN].scale,
					pl->group[gN].offset);

			if (cMAP != -1) {

				pl->data[dN].map[cMAP] = gN;
				cN = cMAP;
			}
		}
	}

	for (N = 0; N < READ_SUBTRACT_MAX; ++N) {

		if (sb[N].busy == SUBTRACT_SCALE) {

			cMAP = plotGetSubtractScale(pl, dN, cN,
					sb[N].args[0], sb[N].args[1]);

			if (cMAP != -1) {

				cN = cMAP;
			}
		}
		else if (sb[N].busy == SUBTRACT_RESAMPLE) {

			/* TODO */
		}
		else if (	sb[N].busy == SUBTRACT_BINARY_SUBTRACTION
				|| sb[N].busy == SUBTRACT_BINARY_ADDITION
				|| sb[N].busy == SUBTRACT_BINARY_MULTIPLICATION
				|| sb[N].busy == SUBTRACT_BINARY_HYPOTENUSE) {

			cMAP = plotGetSubtractBinary(pl, dN, sb[N].busy,
					cN, sb[N].column_2);

			if (cMAP != -1) {

				cN = cMAP;
			}
		}
		else if (	sb[N].busy == SUBTRACT_FILTER_DIFFERENCE
				|| sb[N].busy == SUBTRACT_FILTER_CUMULATIVE
				|| sb[N].busy == SUBTRACT_FILTER_LOW_PASS) {

			cMAP = plotGetSubtractFilter(pl, dN, cN,
					sb[N].busy, sb[N].args[0]);

			if (cMAP != -1) {

				cN = cMAP;
			}
		}
		else if (sb[N].busy == SUBTRACT_FILTER_BITMASK) {

			cMAP = plotGetSubtractFilter(pl, dN, cN, sb[N].busy,
					sb[N].args[0] + sb[N].args[1] * (double) 0x100U);

			if (cMAP != -1) {

				cN = cMAP;
			}
		}
		else if (sb[N].busy == SUBTRACT_FILTER_MEDIAN) {

			cMAP = plotGetSubtractMedian(pl, dN, cN,
					sb[N].busy, sb[N].args[0]);

			if (cMAP != -1) {

				cN = cMAP;
			}
		}
	}

	return cN;
}

void readSelectPage(read_t *rd, int pN)
{
	plot_t		*pl = rd->pl;
	page_t		*pg;

	tuple_t		uN;
	int		N, cX, cY;

	if (pN < 0 || pN >= READ_PAGE_MAX) {

		ERROR("Page number is out of range\n");
		return ;
	}

	pg = rd->page + pN;

	if (pg->busy == 0)
		return;

	rd->page_N = pN;

	plotFigureClean(pl);

	plotDataRangeCacheSubtractClean(pl);
	plotDataSubtractClean(pl);
	plotDataSubtractPaused(pl);

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		if (pg->fig[N].busy != 0) {

			uN = readTimeDataMap(pl, pg->fig[N].dN, pg->fig[N].cX, pg->fig[N].cY);

			cX = uN.X;
			cY = uN.Y;

			cX = readScaleDataMap(pl, pg->fig[N].dN, cX, pg->fig[N].bX);
			cY = readScaleDataMap(pl, pg->fig[N].dN, cY, pg->fig[N].bY);

			plotFigureAdd(pl, N, pg->fig[N].dN, cX, cY, pg->fig[N].aX,
					pg->fig[N].aY, pg->fig[N].label);

			if (pg->fig[N].drawing != -1) {

				pl->figure[N].drawing = pg->fig[N].drawing;
				pl->figure[N].width = pg->fig[N].width;
			}
		}
	}

	for (N = 0; N < PLOT_AXES_MAX; ++N) {

		plotAxisLabel(pl, N, pg->ax[N].label);

		if (pg->ax[N].slave != 0) {

			plotAxisSlave(pl, N, pg->ax[N].slave_N, pg->ax[N].scale,
					pg->ax[N].offset, AXIS_SLAVE_ENABLE);
		}
	}

	plotDataSubtractAlternate(rd->pl);

	plotLayout(pl);
	plotAxisScaleDefault(pl);
}

static int
readCombineGetFreeAxis(plot_t *pl, int *map)
{
	int		N, J, sF, aN = -1;

	for (N = 0; N < PLOT_AXES_MAX; ++N) {

		if (pl->axis[N].busy == AXIS_FREE) {

			sF = 0;

			for (J = 0; J < PLOT_AXES_MAX; ++J) {

				if (map[J] == N) {

					sF = 1;
					break;
				}
			}

			if (sF == 0) {

				aN = N;
				break;
			}
		}
	}

	return aN;
}

static int
readCombineGetMappedAxis(plot_t *pl, int dN, int cN, int aBUSY)
{
	int		N, aN = -1;
	int		gN, *map;

	if (dN < 0 || dN >= PLOT_DATASET_MAX) {

		ERROR("Dataset number is out of range\n");
		return -1;
	}

	if (cN < -1 || cN >= pl->data[dN].column_N + PLOT_SUBTRACT) {

		ERROR("Column number %i is out of range\n", cN);
		return -1;
	}

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		if (pl->figure[N].busy != 0) {

			if (pl->figure[N].data_N == dN) {

				if (		aBUSY == AXIS_BUSY_X
						&& pl->figure[N].column_X == cN) {

					aN = pl->figure[N].axis_X;
					break;
				}
				else if (	aBUSY == AXIS_BUSY_Y
						&& pl->figure[N].column_Y == cN) {

					aN = pl->figure[N].axis_Y;
					break;
				}
			}

			gN = pl->data[dN].map[cN];

			if (gN != -1) {

				map = pl->data[pl->figure[N].data_N].map;

				if (		aBUSY == AXIS_BUSY_X
						&& gN == map[pl->figure[N].column_X]) {

					aN = pl->figure[N].axis_X;
					break;
				}
				else if (	aBUSY == AXIS_BUSY_Y
						&& gN == map[pl->figure[N].column_Y]) {

					aN = pl->figure[N].axis_Y;
					break;
				}
			}
		}
	}

	return aN;
}

void readCombinePage(read_t *rd, int pN, int remap)
{
	plot_t		*pl = rd->pl;
	page_t		*pg;

	tuple_t		uN;
	int		N, fN, bN, cX, cY, aX, aY;
	int		map[PLOT_AXES_MAX];

	if (pN < 0 || pN >= READ_PAGE_MAX) {

		ERROR("Page number is out of range\n");
		return ;
	}

	if (pN == rd->page_N)
		return;

	pg = rd->page + pN;

	if (pg->busy == 0)
		return;

	plotDataSubtractPaused(pl);

	for (N = 0; N < PLOT_AXES_MAX; ++N)
		map[N] = -1;

	if (remap == 0) {

		map[0] = pl->on_X;
		map[1] = pl->on_Y;
	}
	else {
		for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

			if (pg->fig[N].busy != 0) {

				if (map[pg->fig[N].aX] < 0) {

					aX = readCombineGetMappedAxis(pl, pg->fig[N].dN,
							pg->fig[N].cX, AXIS_BUSY_X);

					aX = (aX < 0) ? readCombineGetFreeAxis(pl, map) : aX;
					map[pg->fig[N].aX] = aX;
				}

				if (map[pg->fig[N].aY] < 0) {

					aY = readCombineGetMappedAxis(pl, pg->fig[N].dN,
							pg->fig[N].cY, AXIS_BUSY_Y);

					aY = (aY < 0) ? readCombineGetFreeAxis(pl, map) : aY;
					map[pg->fig[N].aY] = aY;
				}
			}
		}
	}

	for (N = 0; N < PLOT_FIGURE_MAX; ++N) {

		if (pg->fig[N].busy != 0) {

			fN = plotGetFreeFigure(pl);

			if (fN < 0) {

				ERROR("No free figure to combine\n");
				break;
			}

			aX = (map[pg->fig[N].aX] != -1) ? map[pg->fig[N].aX] : pg->fig[N].aX;
			aY = (map[pg->fig[N].aY] != -1) ? map[pg->fig[N].aY] : pg->fig[N].aY;

			uN = readTimeDataMap(pl, pg->fig[N].dN, pg->fig[N].cX, pg->fig[N].cY);

			cX = uN.X;
			cY = uN.Y;

			cX = readScaleDataMap(pl, pg->fig[N].dN, cX, pg->fig[N].bX);
			cY = readScaleDataMap(pl, pg->fig[N].dN, cY, pg->fig[N].bY);

			plotFigureAdd(pl, fN, pg->fig[N].dN, cX, cY, aX, aY, pg->fig[N].label);

			if (pg->fig[N].drawing != -1) {

				pl->figure[fN].drawing = pg->fig[N].drawing;
				pl->figure[fN].width = pg->fig[N].width;
			}
		}
	}

	for (N = 0; N < PLOT_AXES_MAX; ++N) {

		aX = (map[N] != -1) ? map[N] : N;

		if (pl->axis[aX].label[0] == 0) {

			plotAxisLabel(pl, aX, pg->ax[N].label);
		}

		if (pg->ax[N].slave != 0) {

			aY = pg->ax[N].slave_N;
			bN = (map[aY] != -1) ? map[aY] : aY;

			plotAxisSlave(pl, aX, bN, pg->ax[N].scale,
					pg->ax[N].offset, AXIS_SLAVE_ENABLE);
		}
	}

	plotDataSubtractAlternate(rd->pl);

	plotLayout(pl);
	plotAxisScaleDefault(pl);
}

void readDataReload(read_t *rd)
{
	int		dN;

	for (dN = 0; dN < PLOT_DATASET_MAX; ++dN) {

		if (		rd->data[dN].format != FORMAT_NONE
				&& rd->data[dN].file[0] != 0) {

			readOpenUnified(rd, dN, rd->data[dN].column_N,
					rd->data[dN].length_N, rd->data[dN].file,
					rd->data[dN].format);
		}
	}
}

