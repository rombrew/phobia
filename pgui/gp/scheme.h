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

#ifndef _H_SCHEME_
#define _H_SCHEME_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "draw.h"

typedef struct {

	Uint32		plot_background;
	Uint32		plot_figure[10];
	Uint32		plot_axis;
	Uint32		plot_hovered;
	Uint32		plot_text;
	Uint32		plot_hidden;

	Uint32		menu_background;
	Uint32		menu_hovered;
	Uint32		menu_scrollbar;
	Uint32		menu_item_text;
	Uint32		menu_item_hidden;
	Uint32		menu_fuzzy_light;
}
scheme_t;

void schemeFill(scheme_t *sch, int nu);

#endif /* _H_SCHEME_ */

