/*
   Graph Plotter is a tool to analyse numerical data.
   Copyright (C) 2024 Roman Belov <romblv@gmail.com>

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

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "scheme.h"

void schemeFill(scheme_t *sch, int nu)
{
	if (nu == 0) {

		sch->plot_background = 0x000000;
		sch->plot_figure[0] = 0xFFFFFF;
		sch->plot_figure[1] = 0xFF0000;
		sch->plot_figure[2] = 0x00FF00;
		sch->plot_figure[3] = 0xFFFF00;
		sch->plot_figure[4] = 0x00FFFF;
		sch->plot_figure[5] = 0xFF00FF;
		sch->plot_figure[6] = 0xFF7700;
		sch->plot_figure[7] = 0x7700FF;
		sch->plot_figure[8] = 0xFF0077;
		sch->plot_figure[9] = 0x0077FF;
		sch->plot_axis = 0x00BFCF;
		sch->plot_hovered = 0x223344;
		sch->plot_text = 0xFFFFFF;
		sch->plot_hidden = 0x111119;

		sch->menu_background = 0x37373F;
		sch->menu_hovered = 0x777788;
		sch->menu_scrollbar = 0xBBBBCC;
		sch->menu_item_text = 0xFFFFFF;
		sch->menu_item_hidden = 0x555555;
		sch->menu_fuzzy_light = 0xFF3333;
	}
	else if (nu == 1) {

		sch->plot_background = 0xFFFFFF;
		sch->plot_figure[0] = 0x000000;
		sch->plot_figure[1] = 0xFF0000;
		sch->plot_figure[2] = 0x00AA00;
		sch->plot_figure[3] = 0x0000FF;
		sch->plot_figure[4] = 0x00AAAA;
		sch->plot_figure[5] = 0xFF00FF;
		sch->plot_figure[6] = 0xFF7700;
		sch->plot_figure[7] = 0x7700FF;
		sch->plot_figure[8] = 0xFF0077;
		sch->plot_figure[9] = 0x0077FF;
		sch->plot_axis = 0x005F6F;
		sch->plot_hovered = 0xCCDDEE;
		sch->plot_text = 0x000000;
		sch->plot_hidden = 0xEEEEF9;

		sch->menu_background = 0xC0C0CF;
		sch->menu_hovered = 0x888899;
		sch->menu_scrollbar = 0x555566;
		sch->menu_item_text = 0x000000;
		sch->menu_item_hidden = 0xAAAAAA;
		sch->menu_fuzzy_light = 0xFF3333;
	}
	else if (nu == 2) {

		sch->plot_background = 0xFFFFFF;
		sch->plot_figure[0] = 0x000000;
		sch->plot_figure[1] = 0x555555;
		sch->plot_figure[2] = 0x999999;
		sch->plot_figure[3] = 0x111111;
		sch->plot_figure[4] = 0x666666;
		sch->plot_figure[5] = 0x888888;
		sch->plot_figure[6] = 0x222222;
		sch->plot_figure[7] = 0x333333;
		sch->plot_figure[8] = 0x444444;
		sch->plot_figure[9] = 0x777777;
		sch->plot_axis = 0x5F5F5F;
		sch->plot_hovered = 0xDDDDDD;
		sch->plot_text = 0x000000;
		sch->plot_hidden = 0xEEEEEE;

		sch->menu_background = 0xCCCCCC;
		sch->menu_hovered = 0x888888;
		sch->menu_scrollbar = 0x555555;
		sch->menu_item_text = 0x000000;
		sch->menu_item_hidden = 0xAAAAAA;
		sch->menu_fuzzy_light = 0xFFFFFF;
	}
}

