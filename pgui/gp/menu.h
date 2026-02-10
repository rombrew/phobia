/*
   Graph Plotter is a tool to analyse numerical data.
   Copyright (C) 2026 Roman Belov <romblv@gmail.com>

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

#ifndef _H_MENU_
#define _H_MENU_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "draw.h"
#include "scheme.h"

#define MENU_FUZZY_SIZE			80
#define MENU_STRING_MAX			200
#define MENU_OPTION_MAX			10

typedef struct {

	int			raised;
	int			clicked;
	int			id;

	const char		*list;
	int			hidden_N[MENU_OPTION_MAX];
	int			colorful;

	struct {

		int		N;
		const char	*subs;
	}
	mark[MENU_OPTION_MAX];

	int			cur_X;
	int			cur_Y;

	int			box_X;
	int			box_Y;

	int			size_X;
	int			size_Y;
	int			size_N;

	int			scroll_limit;
	int			scroll_shift;
	int			scroll_drag;
	int			scroll_page;

	int			hovered_N;
	int			clicked_N;

	TTF_Font		*font;
	clipBox_t		screen;

	char			fuzzy[MENU_FUZZY_SIZE];

	int			layout_height;

	draw_t			*dw;
	scheme_t		*sch;
}
menu_t;

enum {
	MENU_EVNO_CLICK			= 1,
	MENU_EVNO_UNCLICK,
	MENU_EVNO_MOTION,
	MENU_EVNO_SCROLL_UP,
	MENU_EVNO_SCROLL_DOWN,
	MENU_EVNO_ARROW_UP,
	MENU_EVNO_ARROW_DOWN,
	MENU_EVNO_PAGE_UP,
	MENU_EVNO_PAGE_DOWN,
	MENU_EVNO_HOME,
	MENU_EVNO_END,
	MENU_EVNO_RETURN,
	MENU_EVNO_BACKSPACE,
};

menu_t *menuAlloc(draw_t *dw, scheme_t *sch);
void menuClean(menu_t *mu);

void menuLayout(menu_t *mu);
void menuRaise(menu_t *mu, int id, const char *list, int sx, int sy);
void menuHalt(menu_t *mu);
void menuResume(menu_t *mu);
void menuSelect(menu_t *mu, int item_N);
void menuEvent(menu_t *mu, int evno, int ex, int ey);
void menuEventText(menu_t *mu, const char *tx);
void menuDraw(menu_t *mu, SDL_Surface *surface);

#endif /* _H_MENU_ */

