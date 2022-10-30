/*
   Graph Plotter for numerical data analysis.
   Copyright (C) 2022 Roman Belov <romblv@gmail.com>

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

#ifndef _H_SVG_
#define _H_SVG_

#include <stdlib.h>
#include <stdio.h>

#include <SDL2/SDL.h>

typedef Uint32		svgCol_t;

typedef struct {

	FILE		*fd;

	const char	*font_family;
	int		font_pt;

	int		_line_open;
	double		_last_x;
	double		_last_y;
}
svg_t;

svg_t *svgOpenNew(const char *file, int width, int height);
void svgClose(svg_t *g);

void svgDrawLine(svg_t *g, double xs, double ys, double xe, double ye, svgCol_t col, int h, int d, int s);
void svgDrawRect(svg_t *g, double xs, double ys, double xe, double ye, svgCol_t col);
void svgDrawCircle(svg_t *g, double xs, double ys, double r, svgCol_t col);
void svgDrawText(svg_t *g, double xs, double ys, const char *text, svgCol_t col, int flags);

#endif /* _H_SVG_ */

