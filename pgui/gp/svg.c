/*
   Graph Plotter is a tool to analyse numerical data.
   Copyright (C) 2025 Roman Belov <romblv@gmail.com>

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

#include "svg.h"
#include "draw.h"
#include "plot.h"
#include "read.h"

svg_t *svgOpenNew(const char *file, int width, int height)
{
	svg_t		*g;

	g = (svg_t *) calloc(1, sizeof(svg_t));
	g->fd = unified_fopen(file, "w");

	if (g->fd == NULL) {

		ERROR("fopen(\"%s\"): %s\n", file, strerror(errno));
		free(g);

		return NULL;
	}

	fprintf(g->fd, "<svg xmlns=\"http://www.w3.org/2000/svg\" "
			"width=\"%dpx\" height=\"%dpx\"><g>\n", width, height);

	g->line_open = 0;

	return g;
}

void svgClose(svg_t *g)
{
	if (g->line_open != 0) {

		fprintf(g->fd, "\"/>\n");
		g->line_open = 0;
	}

	fprintf(g->fd, "</g></svg>\n");

	fclose(g->fd);
	free(g);
}

void svgDrawLine(svg_t *g, double xs, double ys, double xe, double ye, svgCol_t col, int h, int d, int s)
{
	if (g->line_open != 0) {

		if (xs == g->last_x && ys == g->last_y) {

			fprintf(g->fd, " %.1f,%.1f", xe, ye);
		}
		else if (xe == g->last_x && ye == g->last_y) {

			fprintf(g->fd, " %.1f,%.1f", xs, ys);
		}
		else {
			fprintf(g->fd, "\"/>\n");
			g->line_open = 0;
		}
	}

	if (g->line_open == 0) {

		if (d == 0) {

			fprintf(g->fd, "<path style=\"fill:none;stroke:#%06x;stroke-width:%.1f;"
					"stroke-linejoin:round;stroke-linecap:round\" "
					"d=\"M %.1f,%.1f %.1f,%.1f",
					(int) (col & 0xFFFFFF), (h != 0) ? h : 0.5, xs, ys, xe, ye);
		}
		else {
			fprintf(g->fd, "<path style=\"fill:none;stroke:#%06x;stroke-width:%.1f;"
					"stroke-linejoin:round;stroke-linecap:butt;"
					"stroke-dasharray:%d,%d\" "
					"d=\"M %.1f,%.1f %.1f,%.1f",
					(int) (col & 0xFFFFFF), (h != 0) ? h : 0.5, d, s,
					xs, ys, xe, ye);
		}

		g->line_open = 1;
	}

	g->last_x = xe;
	g->last_y = ye;
}

void svgDrawRect(svg_t *g, double xs, double ys, double xe, double ye, svgCol_t col)
{
	if (g->line_open != 0) {

		fprintf(g->fd, "\"/>\n");
		g->line_open = 0;
	}

	fprintf(g->fd, "<path style=\"fill:#%06x;stroke:none\" "
			"d=\"M %.1f,%.1f %.1f,%.1f %.1f,%.1f %.1f,%.1f Z\"/>\n",
			(int) (col & 0xFFFFFF), xs, ys, xe, ys, xe, ye, xs, ye);
}

void svgDrawCircle(svg_t *g, double xs, double ys, double r, svgCol_t col)
{
	if (g->line_open != 0) {

		fprintf(g->fd, "\"/>\n");
		g->line_open = 0;
	}

	fprintf(g->fd, "<circle style=\"fill:#%06x;stroke:none\" "
			"cx=\"%.1f\" cy=\"%.1f\" r=\"%.1f\"/>\n",
			(int) (col & 0xFFFFFF), xs, ys, r);
}

void svgDrawText(svg_t *g, double xs, double ys, const char *text, svgCol_t col, int flags)
{
	if (g->line_open != 0) {

		fprintf(g->fd, "\"/>\n");
		g->line_open = 0;
	}

	if (flags & TEXT_VERTICAL) {

		fprintf(g->fd, "<text style=\"font-family:%s;font-size:%dpx;fill:#%06x;stroke:none;"
				"dominant-baseline:%s;text-anchor:%s\" "
				"transform=\"rotate(-90,%.1f,%.1f)\" "
				"x=\"%.1f\" y=\"%.1f\">%s</text>\n",
				g->font_family, g->font_pt, (int) (col & 0xFFFFFF),
				(flags & TEXT_CENTERED_ON_X) ? "middle" : "text-before-edge",
				(flags & TEXT_CENTERED_ON_Y) ? "middle" : "end",
				xs, ys, xs, ys, text);
	}
	else {
		fprintf(g->fd, "<text style=\"font-family:%s;font-size:%dpx;fill:#%06x;stroke:none;"
				"dominant-baseline:%s;text-anchor:%s\" "
				"x=\"%.1f\" y=\"%.1f\">%s</text>\n",
				g->font_family, g->font_pt, (int) (col & 0xFFFFFF),
				(flags & TEXT_CENTERED_ON_Y) ? "middle" : "text-before-edge",
				(flags & TEXT_CENTERED_ON_X) ? "middle" : "start",
				xs, ys, text);
	}
}

