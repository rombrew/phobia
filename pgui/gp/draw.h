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

#ifndef _H_DRAW_
#define _H_DRAW_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

enum {
	TEXT_CENTERED_ON_X	= 1,
	TEXT_CENTERED_ON_Y	= 2,
	TEXT_CENTERED		= 3,
	TEXT_VERTICAL		= 4
};

enum {
	SHAPE_CIRCLE		= 0,
	SHAPE_STARLET,
	SHAPE_HUBCAP,
	SHAPE_RECTANGLE,
	SHAPE_SNOWFLAKE,
	SHAPE_TRIANGLE,
	SHAPE_SHARP,
	SHAPE_COLUMN,
	SHAPE_FILLING,
	SHAPE_SYMBOL,
};

enum {
	DRAW_SOLID,
	DRAW_4X_MSAA,
	DRAW_8X_MSAA,
};

typedef struct {

	int		min_x;
	int		min_y;
	int		max_x;
	int		max_y;
}
clipBox_t;

typedef struct {

	int		antialiasing;
	int		blendfont;
	int		thickness;
	int		gamma;

	int		dash_context;

	int		cached_x;
	int		cached_min_y;
	int		cached_max_y;
	int		cached_ncol;

	struct {

		int	w;
		int	h;

		int	yspan;
		int	len;

		void	*canvas;
		void	*trial;
	}
	pixmap;

	Uint32		palette[16];
	Uint8		ltgamma[256];
	Uint8		ltcomap[256];
}
draw_t;

void drawDashReset(draw_t *dw);
void drawGamma(draw_t *dw);

Uint32 drawRGBMap(draw_t *dw, Uint32 col);

void drawClearSurface(draw_t *dw, SDL_Surface *surface, Uint32 col);
void drawClearCanvas(draw_t *dw);
void drawClearTrial(draw_t *dw);

void drawPixmapAlloc(draw_t *dw, SDL_Surface *surface);
void drawPixmapClean(draw_t *dw);

int clipBoxTest(clipBox_t *cb, int x, int y);

void drawLine(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		double fxe, double fye, Uint32 col);

void drawDash(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		double fxe, double fye, Uint32 col, int dash, int space);

void drawLineCanvas(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		double fxe, double fye, int ncol, int thickness);

void drawDashCanvas(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		double fxe, double fye, int ncol, int thickness, int dash, int space);

int drawLineTrial(draw_t *dw, clipBox_t *cb, double fxs, double fys,
		double fxe, double fye, int ncol, int thickness);

void drawText(draw_t *dw, SDL_Surface *surface, TTF_Font *font, int xs, int ys,
		const char *text, int flags, Uint32 col);

void drawFillRect(SDL_Surface *surface, int xs, int ys,
		int xe, int ye, Uint32 col);

void drawClipRect(SDL_Surface *surface, clipBox_t *cb, int xs, int ys,
		int xe, int ye, Uint32 col);

void drawDotCanvas(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		int rsize, int ncol, int round);

int drawDotTrial(draw_t *dw, clipBox_t *cb, double fxs, double fys,
		int rsize, int ncol, int round);

void drawMarkCanvas(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		int rsize, int shape, int ncol, int thickness);

void drawFlushCanvas(draw_t *dw, SDL_Surface *surface, clipBox_t *cb);

#endif /* _H_DRAW_ */

