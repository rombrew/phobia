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
#include <string.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "edit.h"
#include "draw.h"
#include "scheme.h"

const char *utf8_go_next(const char *s)
{
	if (*s != 0) {

		++s;

		while ((*s & 0xC0) == 0x80) ++s;
	}

	return s;
}

const char *utf8_go_prev(const char *s)
{
	--s;

	while ((*s & 0xC0) == 0x80) --s;

	return s;
}

char *utf8_remove_next(char *s)
{
	char		*q, *r;

	q = (char *) utf8_go_next(s);
	r = s;

	do {
		*s = *q;

		if  (*q == 0)
			break;

		q++;
		s++;
	}
	while (1);

	return r;
}

char *utf8_remove_prev(char *s)
{
	char		*q, *r;

	q = (char *) utf8_go_prev(s);
	r = q;

	do {
		*q = *s;

		if  (*s == 0)
			break;

		q++;
		s++;
	}
	while (1);

	return r;
}

char *utf8_insert_prev(char *s, const char *i)
{
	char		*q, *r;
	int		n;

	q = s + strlen(s);

	n = strlen(i);
	r = s + n;

	do {
		*(q + n) = *q;

		if  (q == s)
			break;

		q--;
	}
	while (1);

	do {
		*s++ = *i++;
	}
	while (*i != 0);

	return r;
}

char *utf8_backspace(char *s)
{
	return utf8_remove_prev(s + strlen(s));
}

int utf8_length(const char *s)
{
	int		l = 0;

	do {
		if (*s == 0)
			break;

		l++;

		s = utf8_go_next(s);
	}
	while (1);

	return l;
}

const char *utf8_skip(const char *s, int n)
{
	do {
		if (*s == 0)
			break;

		if (n > 0) n--;
		else break;

		s = (char *) utf8_go_next(s);
	}
	while (1);

	return s;
}

const char *utf8_skip_b(const char *s, int n)
{
	char		*r;

	do {
		if (*s == 0)
			break;

		if (n > 0) ;
		else break;

		r = (char *) utf8_go_next(s);
		n += (int) (s - r);
		s = r;
	}
	while (1);

	return s;
}

static void
editBuild(edit_t *ed, int rep)
{
	int		bT, bMIN, bMAX;

	/* To make it look pretty we force the font height to be odd.
	 * */
	ed->layout_height += (ed->layout_height & 1) ? 0 : 1;

	TTF_SizeUTF8(ed->font, ed->title, &bT, &bMAX);
	TTF_SizeUTF8(ed->font, ed->text, &bMIN, &bMAX);

	bMAX = (bT > bMIN) ? bT : bMIN;

	if (rep == 0) {

		bMIN = (ed->screen.max_x - ed->screen.min_x) / 2;
		bT = (bMIN > bMAX) ? bMIN : bMAX;

		ed->size_X = bT + 2 * ed->layout_height;
		ed->size_Y = 3 * ed->layout_height;
	}
	else {
		bMAX = bMAX + 2 * ed->layout_height;
		ed->size_X = (bMAX > ed->size_X) ? bMAX : ed->size_X;
	}

	if (ed->box_X < 0) {

		ed->box_X = (ed->screen.min_x + ed->screen.max_x - ed->size_X) / 2;
	}

	if (ed->box_Y < 0) {

		ed->box_Y = (ed->screen.min_y + ed->screen.max_y - ed->size_Y) / 2;
	}

	ed->box_X = (ed->box_X > ed->screen.max_x - ed->size_X)
		? ed->screen.max_x - ed->size_X : ed->box_X;
	ed->box_Y = (ed->box_Y > ed->screen.max_y - ed->size_Y)
		? ed->screen.max_y - ed->size_Y : ed->box_Y;

	ed->box_X = (ed->box_X < ed->screen.min_x) ? ed->screen.min_x : ed->box_X;
	ed->box_Y = (ed->box_Y < ed->screen.min_y) ? ed->screen.min_y : ed->box_Y;
}

edit_t *editAlloc(draw_t *dw, scheme_t *sch)
{
	edit_t			*ed;

	ed = calloc(1, sizeof(edit_t));

	ed->dw = dw;
	ed->sch = sch;

	return ed;
}

void editClean(edit_t *ed)
{
	free(ed);
}

void editLayout(edit_t *ed)
{
	if (ed->raised != 0) {

		editBuild(ed, 1);
	}
}

void editRaise(edit_t *ed, int id, const char *title, const char *text, int sx, int sy)
{
	if (ed->raised != 0)
		return ;

	ed->raised = 1;
	ed->entered = 0;

	ed->id = id;
	ed->title = title;

	ed->cur_X = sx;
	ed->cur_Y = sy;

	ed->box_X = ed->cur_X;
	ed->box_Y = ed->cur_Y;

	strcpy(ed->text, text);
	ed->text_cur = ed->text + strlen(ed->text);

	editBuild(ed, 0);
}

void editHalt(edit_t *ed)
{
	ed->raised = 0;
}

void editEvent(edit_t *ed, int evno, int ex, int ey)
{
	char		curbuf[EDIT_STRING_MAX], *clip;
	const char	*curp, *curp_next;
	int		min_X, min_Y, max_X, max_Y;

	if (ed->raised == 0)
		return ;

	ed->cur_X = ex;
	ed->cur_Y = ey;

	if (evno == EDIT_EVNO_CLICK) {

		min_X = ed->box_X + ed->layout_height - ed->layout_height / 2;
		min_Y = ed->box_Y + ed->layout_height + ed->layout_height / 2;

		max_X = ed->box_X + ed->size_X - ed->layout_height + ed->layout_height / 2;
		max_Y = ed->box_Y + 2 * ed->layout_height + ed->layout_height / 2;

		if (ed->cur_X > min_X && ed->cur_X < max_X
				&& ed->cur_Y > min_Y && ed->cur_Y < max_Y) {

			curbuf[0] = 0;
			curp = ed->text;

			while (*curp != 0) {

				curp_next = utf8_go_next(curp);

				TTF_SizeUTF8(ed->font, curbuf, &min_X, &min_Y);

				strncat(curbuf, curp, curp_next - curp);

				TTF_SizeUTF8(ed->font, curbuf, &max_X, &min_Y);

				min_X += ed->box_X + ed->layout_height;
				max_X += ed->box_X + ed->layout_height;

				if (ed->cur_X >= min_X && ed->cur_X < max_X) {

					ed->text_cur = (char *) curp;
					break;
				}

				curp = curp_next;
			}

			if (*curp == 0) {

				TTF_SizeUTF8(ed->font, curbuf, &min_X, &min_Y);

				min_X += ed->box_X + ed->layout_height;

				if (ed->cur_X >= min_X) {

					ed->text_cur = (char *) curp;
				}
			}
		}
	}
	else if (evno == EDIT_EVNO_ARROW_LEFT) {

		if (ed->text_cur != ed->text) {

			ed->text_cur = (char *) utf8_go_prev(ed->text_cur);
		}
	}
	else if (evno == EDIT_EVNO_ARROW_RIGHT) {

		ed->text_cur = (char *) utf8_go_next(ed->text_cur);
	}
	else if (evno == EDIT_EVNO_RETURN) {

		ed->entered = 1;
	}
	else if (evno == EDIT_EVNO_BACKSPACE) {

		if (ed->text_cur != ed->text) {

			ed->text_cur = utf8_remove_prev(ed->text_cur);
		}
	}
	else if (evno == EDIT_EVNO_DELETE) {

		ed->text_cur = utf8_remove_next(ed->text_cur);
	}
	else if (evno == EDIT_EVNO_CTRL_X) {

		ed->text[0] = 0;
		ed->text_cur = ed->text;
	}
	else if (evno == EDIT_EVNO_CTRL_C) {

		SDL_SetClipboardText(ed->text);
	}
	else if (evno == EDIT_EVNO_CTRL_V) {

		if (SDL_HasClipboardText() != 0) {

			clip = SDL_GetClipboardText();

			if (clip != NULL) {

				if (strlen(ed->text) + strlen(clip) < EDIT_TEXT_SIZE) {

					ed->text_cur = utf8_insert_prev(ed->text_cur, clip);
				}

				SDL_free(clip);
			}
		}
	}

	editBuild(ed, 1);
}

void editEventText(edit_t *ed, const char *tx)
{
	if (strlen(ed->text) < EDIT_TEXT_SIZE) {

		ed->text_cur = utf8_insert_prev(ed->text_cur, tx);
		editBuild(ed, 1);
	}
}

void editDraw(edit_t *ed, SDL_Surface *surface)
{
	char		curbuf[EDIT_STRING_MAX];
	int		baseX, baseY, endX, endY;

	if (ed->raised == 0)
		return ;

	SDL_LockSurface(surface);

	drawFillRect(surface, ed->box_X, ed->box_Y, ed->box_X + ed->size_X,
			ed->box_Y + ed->size_Y, ed->sch->menu_background);

	baseX = ed->box_X + ed->layout_height - ed->layout_height / 2;
	baseY = ed->box_Y + ed->layout_height + ed->layout_height / 2;

	endX = ed->box_X + ed->size_X - ed->layout_height + ed->layout_height / 2;
	endY = ed->box_Y + 2 * ed->layout_height + ed->dw->thickness + ed->layout_height / 2;

	drawLine(ed->dw, surface, &ed->screen, baseX, baseY, endX, baseY, ed->sch->menu_scrollbar);
	drawLine(ed->dw, surface, &ed->screen, endX, baseY, endX, endY, ed->sch->menu_scrollbar);
	drawLine(ed->dw, surface, &ed->screen, baseX, baseY, baseX, endY, ed->sch->menu_scrollbar);
	drawLine(ed->dw, surface, &ed->screen, baseX, endY, endX, endY, ed->sch->menu_scrollbar);

	SDL_UnlockSurface(surface);

	baseX = ed->box_X + (ed->size_X + ed->layout_height) / 2;
	baseY = ed->box_Y + ed->layout_height / 2;

	drawText(ed->dw, surface, ed->font, baseX, baseY, ed->title,
			TEXT_CENTERED, ed->sch->menu_item_text);

	baseX = ed->box_X + ed->layout_height;
	baseY = ed->box_Y + 2 * ed->layout_height;

	drawText(ed->dw, surface, ed->font, baseX, baseY, ed->text,
			TEXT_CENTERED_ON_Y, ed->sch->menu_item_text);

	curbuf[0] = 0;
	strncat(curbuf, ed->text, ed->text_cur - ed->text);

	TTF_SizeUTF8(ed->font, curbuf, &baseX, &endX);

	baseX += ed->box_X + ed->layout_height;
	baseY += - ed->layout_height / 2;

	if (SDL_GetTicks() & 0x0100U) {

		drawFillRect(surface, baseX, baseY + 2, baseX + ed->layout_long - 1,
				baseY + ed->layout_height - 4, ed->sch->menu_item_text);
	}
}

