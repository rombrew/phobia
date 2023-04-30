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

#ifndef _H_EDIT_
#define _H_EDIT_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "draw.h"
#include "scheme.h"

#define EDIT_TEXT_SIZE			360
#define EDIT_STRING_MAX			400

typedef struct {

	int			raised;
	int			entered;
	int			id;

	const char		*title;

	int			cur_X;
	int			cur_Y;

	int			box_X;
	int			box_Y;

	int			size_X;
	int			size_Y;

	TTF_Font		*font;
	clipBox_t		screen;

	char			text[EDIT_STRING_MAX];
	char			*text_cur;

	int			layout_height;
	int			layout_long;

	const char		*list_fmt;

	draw_t			*dw;
	scheme_t		*sch;
}
edit_t;

enum {
	EDIT_EVNO_CLICK			= 1,
	EDIT_EVNO_ARROW_LEFT,
	EDIT_EVNO_ARROW_RIGHT,
	EDIT_EVNO_RETURN,
	EDIT_EVNO_BACKSPACE,
	EDIT_EVNO_DELETE,
	EDIT_EVNO_CTRL_X,
	EDIT_EVNO_CTRL_C,
	EDIT_EVNO_CTRL_V,
	EDIT_EVNO_TAB,
};

edit_t *editAlloc(draw_t *dw, scheme_t *sch);
void editClean(edit_t *ed);

void editLayout(edit_t *ed);
void editRaise(edit_t *ed, int id, const char *title, const char *text, int sx, int sy);
void editHalt(edit_t *ed);
void editEvent(edit_t *ed, int evno, int ex, int ey);
void editEventText(edit_t *ed, const char *tx);
void editDraw(edit_t *ed, SDL_Surface *surface);

#endif /* _H_EDIT_ */

