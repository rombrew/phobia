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

#ifndef _H_SCHEME_
#define _H_SCHEME_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "draw.h"

typedef struct {

	colType_t		plot_background;
	colType_t		plot_figure[8];
	colType_t		plot_axis;
	colType_t		plot_hovered;
	colType_t		plot_text;
	colType_t		plot_hidden;

	colType_t		menu_background;
	colType_t		menu_hovered;
	colType_t		menu_scrollbar;
	colType_t		menu_item_text;
	colType_t		menu_item_hidden;
	colType_t		menu_fuzzy_light;
}
scheme_t;

void schemeFill(scheme_t *sch, int nu);

#endif /* _H_SCHEME_ */

