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

#include <stdlib.h>
#include <math.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "draw.h"
#include "plot.h"
#include "svg.h"

extern int fp_isfinite(double x);

void drawDashReset(draw_t *dw)
{
	dw->dash_context = 0;
	dw->cached_ncol = -1;
}

void drawGamma(draw_t *dw)
{
	int		n;

	for (n = 0; n < 256; ++n) {

		dw->ltgamma[n] = (Uint8) ceilf(powf(n / 255.f, dw->gamma / 100.f) * 255.f);
		dw->ltcomap[n] = (Uint8) ceilf(powf(n / 255.f, 100.f / dw->gamma) * 255.f);
	}
}

Uint32 drawRGBMap(draw_t *dw, Uint32 col)
{
	union {

		Uint32          l;
		Uint8           b[4];
	}
	vcol = { col };

	if (dw->antialiasing != DRAW_SOLID) {

		vcol.b[0] = dw->ltcomap[vcol.b[0] & 0xFFU];
		vcol.b[1] = dw->ltcomap[vcol.b[1] & 0xFFU];
		vcol.b[2] = dw->ltcomap[vcol.b[2] & 0xFFU];
	}

	return vcol.l;
}

void drawClearSurface(draw_t *dw, SDL_Surface *surface, Uint32 col)
{
	Uint32			*pixels = (Uint32 *) surface->pixels;

	int			pitch = surface->pitch / 4;
	int			len, n;

	len = pitch * surface->h;

	for (n = 0; n < len - 15; n += 16) {

		*pixels++ = col;
		*pixels++ = col;
		*pixels++ = col;
		*pixels++ = col;

		*pixels++ = col;
		*pixels++ = col;
		*pixels++ = col;
		*pixels++ = col;

		*pixels++ = col;
		*pixels++ = col;
		*pixels++ = col;
		*pixels++ = col;

		*pixels++ = col;
		*pixels++ = col;
		*pixels++ = col;
		*pixels++ = col;
	}

	for (; n < len; n++)
		*pixels++ = col;

	drawDashReset(dw);
}

void drawClearCanvas(draw_t *dw)
{
	int			len;

	len = dw->pixmap.yspan * dw->pixmap.h;

	if (dw->antialiasing == DRAW_4X_MSAA) {

		len *= 2;
	}
	else if (dw->antialiasing == DRAW_8X_MSAA) {

		len *= 4;
	}

	memset(dw->pixmap.canvas, 0, len);
}

void drawClearTrial(draw_t *dw)
{
	int			len;

	len = dw->pixmap.yspan * dw->pixmap.h;

	if (dw->antialiasing != DRAW_SOLID) {

		len *= 2;
	}

	memset(dw->pixmap.trial, 0, len);
}

void drawPixmapAlloc(draw_t *dw, SDL_Surface *surface)
{
	int		w, h, len;

	w = surface->w;
	h = surface->h;

	dw->pixmap.w = w;
	dw->pixmap.h = h;

	w += (w & 7UL) ? 8UL - (w & 7UL) : 0UL;
	h += (h & 7UL) ? 8UL - (h & 7UL) : 0UL;

	len = w * h;

	if (dw->antialiasing == DRAW_4X_MSAA) {

		len *= 2;
	}
	else if (dw->antialiasing == DRAW_8X_MSAA) {

		len *= 4;
	}

	dw->pixmap.yspan = w;

	if (dw->pixmap.len < len) {

		if (dw->pixmap.len != 0) {

			free(dw->pixmap.canvas);
			free(dw->pixmap.trial);
		}

		dw->pixmap.len = len + 1048576UL;
		dw->pixmap.canvas = (void *) malloc(dw->pixmap.len);

		if (dw->pixmap.canvas == NULL) {

			ERROR("Unable to allocate memory of the canvas pixmap\n");
		}

		dw->pixmap.trial = (void *) malloc(dw->pixmap.len);

		if (dw->pixmap.trial == NULL) {

			ERROR("Unable to allocate memory of the trial pixmap\n");
		}
	}
}

void drawPixmapClean(draw_t *dw)
{
	if (dw->pixmap.len != 0) {

		free(dw->pixmap.canvas);
		free(dw->pixmap.trial);
	}
}

static int
clipCode(clipBox_t *cb, double x, double y)
{
	int			cc = 0UL;

	if (x < cb->min_x)
		cc |= 0x01UL;

	else if (x > cb->max_x)
		cc |= 0x02UL;

	if (y < cb->min_y)
		cc |= 0x04UL;

	else if (y > cb->max_y)
		cc |= 0x08UL;

	return cc;
}

int clipBoxTest(clipBox_t *cb, int x, int y)
{
	return (clipCode(cb, x, y) == 0);
}

static int
clipLine(clipBox_t *cb, double *xs, double *ys, double *xe, double *ye)
{
	double			dx, dy;
	int			s_cc, e_cc;

	s_cc = clipCode(cb, *xs, *ys);
	e_cc = clipCode(cb, *xe, *ye);

	do {
		if (s_cc & e_cc)
			return -1;

		if (!(s_cc | e_cc))
			return 0;

		dx = *xe - *xs;
		dy = *ye - *ys;

		if (s_cc != 0) {

			if (s_cc & 0x01UL) {

				*ys = *ye + ((cb->min_x - *xe) * dy) / dx;
				*xs = cb->min_x;
			}
			else if (s_cc & 0x02UL) {

				*ys = *ys + ((cb->max_x - *xs) * dy) / dx;
				*xs = cb->max_x;
			}
			else if (s_cc & 0x04UL) {

				*xs = *xe + ((cb->min_y - *ye) * dx) / dy;
				*ys = cb->min_y;
			}
			else if (s_cc & 0x08UL) {

				*xs = *xs + ((cb->max_y - *ys) * dx) / dy;
				*ys = cb->max_y;
			}

			s_cc = clipCode(cb, *xs, *ys);
		}
		else if (e_cc != 0) {

			if (e_cc & 0x01UL) {

				*ye = *ys + ((cb->min_x - *xs) * dy) / dx;
				*xe = cb->min_x;
			}
			else if (e_cc & 0x02UL) {

				*ye = *ye + ((cb->max_x - *xe) * dy) / dx;
				*xe = cb->max_x;
			}
			else if (e_cc & 0x04UL) {

				*xe = *xs + ((cb->min_y - *ys) * dx) / dy;
				*ye = cb->min_y;
			}
			else if (e_cc & 0x08UL) {

				*xe = *xe + ((cb->max_y - *ye) * dx) / dy;
				*ye = cb->max_y;
			}

			e_cc = clipCode(cb, *xe, *ye);
		}
	}
	while (1);
}

static void
drawRoughLine(SDL_Surface *surface, int xs, int ys, int xe, int ye, Uint32 col)
{
	Uint32			*pixels = (Uint32 *) surface->pixels;

	int			pitch = surface->pitch / 4;
	int			dx, dy, vx, vy, i;

	if (xs < xe) {

		dx = xe - xs;
		vx = 1;
	}
	else {
		dx = xs - xe;
		vx = - 1;
	}

	if (ys < ye) {

		dy = ye - ys;
		vy = 1;
	}
	else {
		dy = ys - ye;
		vy = - 1;
	}

	pixels += pitch * ys + xs;

	i = 0;

	if (dx < dy) {

		while (ys != ye) {

			*pixels = col;

			ys += vy;
			pixels += pitch * vy;

			i += dx;

			if (i >= dy) {

				i -= dy;
				pixels += vx;
			}
		}
	}
	else if (dx > dy) {

		while (xs != xe) {

			*pixels = col;

			xs += vx;
			pixels += vx;

			i += dy;

			if (i >= dx) {

				i -= dx;
				pixels += pitch * vy;
			}
		}
	}
	else {
		while (ys != ye) {

			*pixels = col;

			ys += vy;
			pixels += vx + pitch * vy;
		}
	}

	*pixels = col;
}

static void
drawRoughDash(draw_t *dw, SDL_Surface *surface, int xs, int ys, int xe, int ye,
		Uint32 col, int dash, int space)
{
	Uint32			*pixels = (Uint32 *) surface->pixels;

	int			pitch = surface->pitch / 4;
	int			dx, dy, vx, vy, j, i;

	if (xs < xe) {

		dx = xe - xs;
		vx = 1;
	}
	else {
		dx = xs - xe;
		vx = -1;
	}

	if (ys < ye) {

		dy = ye - ys;
		vy = 1;
	}
	else {
		dy = ys - ye;
		vy = -1;
	}

	pixels += pitch * ys + xs;

	j = dw->dash_context;
	i = 0;

	if (dx < dy) {

		while (ys != ye) {

			if (j < dash) *pixels = col;
			j++; if (j >= dash + space) { j = 0; }

			ys += vy;
			pixels += pitch * vy;

			i += dx;

			if (i >= dy) {

				i -= dy;
				pixels += vx;
			}
		}
	}
	else if (dx > dy) {

		while (xs != xe) {

			if (j < dash) *pixels = col;
			j++; if (j >= dash + space) { j = 0; }

			xs += vx;
			pixels += vx;

			i += dy;

			if (i >= dx) {

				i -= dx;
				pixels += pitch * vy;
			}
		}
	}
	else {
		while (ys != ye) {

			if (j < dash) *pixels = col;
			j++; if (j >= dash + space) { j = 0; }

			ys += vy;
			pixels += vx + pitch * vy;
		}
	}

	if (j < dash) *pixels = col;

	dw->dash_context = j;
}

void drawLine(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		double fxe, double fye, Uint32 col)
{
	svg_t			*g = (svg_t *) surface->userdata;
	int			xs, ys, xe, ye, n;

	if (clipLine(cb, &fxs, &fys, &fxe, &fye) < 0)
		return ;

	if (g != NULL) {

		svgDrawLine(g, fxs, fys, fxe, fye, (svgCol_t) col,
				dw->thickness, 0, 0);
	}

	xs = (int) fxs;
	ys = (int) fys;
	xe = (int) fxe;
	ye = (int) fye;

	drawRoughLine(surface, xs, ys, xe, ye, col);

	for (n = 1; n < dw->thickness; ++n) {

		if (abs(xs - xe) < abs(ys - ye)) {

			drawRoughLine(surface, xs + n, ys, xe + n, ye, col);
		}
		else {
			drawRoughLine(surface, xs, ys - n, xe, ye - n, col);
		}
	}
}

void drawDash(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		double fxe, double fye, Uint32 col, int dash, int space)
{
	svg_t			*g = (svg_t *) surface->userdata;
	int			xs, ys, xe, ye, context, n;

	if (clipLine(cb, &fxs, &fys, &fxe, &fye) < 0)
		return ;

	if (g != NULL) {

		svgDrawLine(g, fxs, fys, fxe, fye, (svgCol_t) col,
				dw->thickness, dash + dw->thickness, space);
	}

	xs = (int) fxs;
	ys = (int) fys;
	xe = (int) fxe;
	ye = (int) fye;

	context = dw->dash_context;
	dash += dw->thickness;

	drawRoughDash(dw, surface, xs, ys, xe, ye, col, dash, space);

	for (n = 1; n < dw->thickness; ++n) {

		dw->dash_context = context;

		if (abs(xs - xe) < abs(ys - ye)) {

			drawRoughDash(dw, surface, xs + n, ys, xe + n, ye,
					col, dash, space);
		}
		else {
			drawRoughDash(dw, surface, xs, ys - n, xe, ye - n,
					col, dash, space);
		}
	}
}

void drawLineCanvas(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		double fxe, double fye, int ncol, int thickness)
{
	svg_t			*g = (svg_t *) surface->userdata;
	clipBox_t		lcb;

	int			yspan, xs, ys, xe, ye, x, y, h, l, d, r;
	int			w1, w2, w3, lw1, lw2, lw3, w1dx, w2dx, w3dx;
	int			w1dy, w2dy, w3dy, ldxs, ldys, ldxe, ldye, la;

	lcb.min_x = cb->min_x - 16;
	lcb.min_y = cb->min_y - 16;
	lcb.max_x = cb->max_x + 16;
	lcb.max_y = cb->max_y + 16;

	if (clipLine(&lcb, &fxs, &fys, &fxe, &fye) < 0)
		return ;

	if (g != NULL) {

		double		_fxs = fxs, _fys = fys, _fxe = fxe, _fye = fye;

		if (clipLine(cb, &_fxs, &_fys, &_fxe, &_fye) == 0) {

			svgDrawLine(g, _fxs, _fys, _fxe, _fye,
					(svgCol_t) dw->palette[ncol],
					thickness, 0, 0);
		}
	}

	if (fys < fye) {

		xs = (int) (fxe * 16.);
		ys = (int) (fye * 16.);
		xe = (int) (fxs * 16.);
		ye = (int) (fys * 16.);
	}
	else {
		xs = (int) (fxs * 16.);
		ys = (int) (fys * 16.);
		xe = (int) (fxe * 16.);
		ye = (int) (fye * 16.);
	}

	h = (thickness > 0) ? thickness * 8 : 5;
	h += (dw->antialiasing != DRAW_SOLID) ? 12 : 0;

	if (xs < xe) {

		lcb.min_x = (xs - h) / 16;
		lcb.max_x = (xe + h) / 16;
	}
	else {
		lcb.min_x = (xe - h) / 16;
		lcb.max_x = (xs + h) / 16;
	}

	if (ys < ye) {

		lcb.min_y = (ys - h) / 16;
		lcb.max_y = (ye + h) / 16;
	}
	else {
		lcb.min_y = (ye - h) / 16;
		lcb.max_y = (ys + h) / 16;
	}

	lcb.min_x = (lcb.min_x < cb->min_x) ? cb->min_x : lcb.min_x;
	lcb.min_y = (lcb.min_y < cb->min_y) ? cb->min_y : lcb.min_y;
	lcb.max_x = (lcb.max_x > cb->max_x) ? cb->max_x : lcb.max_x;
	lcb.max_y = (lcb.max_y > cb->max_y) ? cb->max_y : lcb.max_y;

	l = (xs - xe) * (xs - xe) + (ys - ye) * (ys - ye);
	d = (int) sqrtf((float) l);

	l = d * h;
	r = h * h;

	w1 = (ys - ye) * (lcb.min_x * 16 - xe + 8) - (xs - xe) * (lcb.min_y * 16 - ye + 8);
	w2 = (xe - xs) * (lcb.min_x * 16 - xs + 8) + (ye - ys) * (lcb.min_y * 16 - ys + 8);
	w3 = (xs - xe) * (lcb.min_x * 16 - xe + 8) + (ys - ye) * (lcb.min_y * 16 - ye + 8);

	w1dx = (ys - ye) * 16;
	w2dx = (xe - xs) * 16;
	w3dx = (xs - xe) * 16;

	w1dy = - (xs - xe) * 16;
	w2dy = (ye - ys) * 16;
	w3dy = (ys - ye) * 16;

	if (dw->antialiasing == DRAW_SOLID) {

		Uint8		*canvas = (Uint8 *) dw->pixmap.canvas;

		yspan = dw->pixmap.yspan;
		canvas += lcb.min_y * yspan;

		for (y = lcb.min_y; y <= lcb.max_y; ++y) {

			x = (w1dx > 0) ? - (l + w1) / w1dx : 0;
			x = (x < 0) ? 0 : x;

			lw1 = w1 + w1dx * x;
			lw2 = w2 + w2dx * x;
			lw3 = w3 + w3dx * x;

			for (x += lcb.min_x; x <= lcb.max_x; ++x) {

				if (lw2 <= 0) {

					ldxs = x * 16 - xs + 8;
					ldys = y * 16 - ys + 8;

					la = ldxs * ldxs + ldys * ldys;

					if (la <= r) {

						*(canvas + x) = ncol;
					}
				}
				else if (lw3 <= 0) {

					ldxe = x * 16 - xe + 8;
					ldye = y * 16 - ye + 8;

					la = ldxe * ldxe + ldye * ldye;

					if (la <= r) {

						*(canvas + x) = ncol;
					}
				}
				else if (lw1 < l) {

					if (lw1 >= - l) {

						*(canvas + x) = ncol;
					}
				}
				else {
					break;
				}

				lw1 += w1dx;
				lw2 += w2dx;
				lw3 += w3dx;
			}

			w1 += w1dy;
			w2 += w2dy;
			w3 += w3dy;

			canvas += yspan;
		}
	}
	else if (dw->antialiasing == DRAW_4X_MSAA) {

		Uint16		*canvas = (Uint16 *) dw->pixmap.canvas;

		int		lk, rk, dk, nw1, nw2, nw3, u1dx, u2dx;
		int		u3dx, u1dy, u2dy, u3dy, touch;

		lk = d * (h - 12);
		rk = (h - 12) * (h - 12);
		dk = d * 12;

		u1dx = (ys - ye);
		u2dx = (xe - xs);
		u3dx = (xs - xe);

		u1dy = - (xs - xe);
		u2dy = (ye - ys);
		u3dy = (ys - ye);

		yspan = dw->pixmap.yspan;
		canvas += lcb.min_y * yspan;

		for (y = lcb.min_y; y <= lcb.max_y; ++y) {

			x = (w1dx > 0) ? - (l + w1) / w1dx : 0;
			x = (x < 0) ? 0 : x;

			lw1 = w1 + w1dx * x;
			lw2 = w2 + w2dx * x;
			lw3 = w3 + w3dx * x;

			for (x += lcb.min_x; x <= lcb.max_x; ++x) {

				touch = 0;

				if (lw2 <= 0) {

					ldxs = x * 16 - xs + 8;
					ldys = y * 16 - ys + 8;

					la = ldxs * ldxs + ldys * ldys;

					if (la <= r) {

						touch = 1;
					}
				}
				else if (lw3 <= 0) {

					ldxe = x * 16 - xe + 8;
					ldye = y * 16 - ye + 8;

					la = ldxe * ldxe + ldye * ldye;

					if (la <= r) {

						touch = 1;
					}
				}
				else if (lw1 < l) {

					if (lw1 >= - l) {

						touch = 2;
					}
				}
				else {
					break;
				}

				if (		touch == 2
						&& lw2 > dk
						&& lw3 > dk) {

					Uint16		nb = *(canvas + x);

					nw1 = lw1 - u1dx * 5 + u1dy * 2;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFFF0U) | (ncol << 0);
					}

					nw1 += u1dx * 3 - u1dy * 7;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFF0FU) | (ncol << 4);
					}

					nw1 += u1dx * 4 + u1dy * 10;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xF0FFU) | (ncol << 8);
					}

					nw1 += u1dx * 3 - u1dy * 7;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0x0FFFU) | (ncol << 12);
					}

					*(canvas + x) = nb;
				}
				else if (touch != 0) {

					Uint16		nb = *(canvas + x);

					nw1 = lw1 - u1dx * 5 + u1dy * 2;
					nw2 = lw2 - u2dx * 5 + u2dy * 2;
					nw3 = lw3 - u3dx * 5 + u3dy * 2;

					ldxs = x * 16 - xs + 3;
					ldys = y * 16 - ys + 10;
					ldxe = x * 16 - xe + 3;
					ldye = y * 16 - ye + 10;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFFF0U) | (ncol << 0);
					}

					nw1 += u1dx * 3 - u1dy * 7;
					nw2 += u2dx * 3 - u2dy * 7;
					nw3 += u3dx * 3 - u3dy * 7;

					ldxs += 3;
					ldys += - 7;
					ldxe += 3;
					ldye += - 7;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFF0FU) | (ncol << 4);
					}

					nw1 += u1dx * 4 + u1dy * 10;
					nw2 += u2dx * 4 + u2dy * 10;
					nw3 += u3dx * 4 + u3dy * 10;

					ldxs += 4;
					ldys += 10;
					ldxe += 4;
					ldye += 10;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xF0FFU) | (ncol << 8);
					}

					nw1 += u1dx * 3 - u1dy * 7;
					nw2 += u2dx * 3 - u2dy * 7;
					nw3 += u3dx * 3 - u3dy * 7;

					ldxs += 3;
					ldys += - 7;
					ldxe += 3;
					ldye += - 7;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0x0FFFU) | (ncol << 12);
					}

					*(canvas + x) = nb;
				}

				lw1 += w1dx;
				lw2 += w2dx;
				lw3 += w3dx;
			}

			w1 += w1dy;
			w2 += w2dy;
			w3 += w3dy;

			canvas += yspan;
		}
	}
	else if (dw->antialiasing == DRAW_8X_MSAA) {

		Uint16		*canvas = (Uint16 *) dw->pixmap.canvas;

		int		lk, rk, dk, nw1, nw2, nw3, u1dx, u2dx;
		int		u3dx, u1dy, u2dy, u3dy, touch;

		lk = d * (h - 12);
		rk = (h - 12) * (h - 12);
		dk = d * 12;

		u1dx = (ys - ye);
		u2dx = (xe - xs);
		u3dx = (xs - xe);

		u1dy = - (xs - xe);
		u2dy = (ye - ys);
		u3dy = (ys - ye);

		yspan = dw->pixmap.yspan * 2;
		canvas += lcb.min_y * yspan;

		for (y = lcb.min_y; y <= lcb.max_y; ++y) {

			x = (w1dx > 0) ? - (l + w1) / w1dx : 0;
			x = (x < 0) ? 0 : x;

			lw1 = w1 + w1dx * x;
			lw2 = w2 + w2dx * x;
			lw3 = w3 + w3dx * x;

			for (x += lcb.min_x; x <= lcb.max_x; ++x) {

				touch = 0;

				if (lw2 <= 0) {

					ldxs = x * 16 - xs + 8;
					ldys = y * 16 - ys + 8;

					la = ldxs * ldxs + ldys * ldys;

					if (la <= r) {

						touch = 1;
					}
				}
				else if (lw3 <= 0) {

					ldxe = x * 16 - xe + 8;
					ldye = y * 16 - ye + 8;

					la = ldxe * ldxe + ldye * ldye;

					if (la <= r) {

						touch = 1;
					}
				}
				else if (lw1 < l) {

					if (lw1 >= - l) {

						touch = 2;
					}
				}
				else {
					break;
				}

				if (		touch == 2
						&& lw2 > dk
						&& lw3 > dk) {

					Uint16		nb = *(canvas + x * 2);

					nw1 = lw1 - u1dx * 7 - u1dy;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFFF0U) | (ncol << 0);
					}

					nw1 += u1dx * 2 - u1dy * 6;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFF0FU) | (ncol << 4);
					}

					nw1 += u1dx + u1dy * 11;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xF0FFU) | (ncol << 8);
					}

					nw1 += u1dx * 2 - u1dy * 7;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0x0FFFU) | (ncol << 12);
					}

					*(canvas + x * 2) = nb;

					nb = *(canvas + x * 2 + 1);

					nw1 += u1dx * 2 + u1dy * 5;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFFF0U) | (ncol << 0);
					}

					nw1 += u1dx * 3 + u1dy * 4;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFF0FU) | (ncol << 4);
					}

					nw1 += u1dx - u1dy * 12;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xF0FFU) | (ncol << 8);
					}

					nw1 += u1dx + u1dy * 6;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0x0FFFU) | (ncol << 12);
					}

					*(canvas + x * 2 + 1) = nb;
				}
				else if (touch != 0) {

					Uint16		nb = *(canvas + x * 2);

					nw1 = lw1 - u1dx * 7 - u1dy;
					nw2 = lw2 - u2dx * 7 - u2dy;
					nw3 = lw3 - u3dx * 7 - u3dy;

					ldxs = x * 16 - xs + 1;
					ldys = y * 16 - ys + 7;
					ldxe = x * 16 - xe + 1;
					ldye = y * 16 - ye + 7;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFFF0U) | (ncol << 0);
					}

					nw1 += u1dx * 2 - u1dy * 6;
					nw2 += u2dx * 2 - u2dy * 6;
					nw3 += u3dx * 2 - u3dy * 6;

					ldxs += 2;
					ldys += - 6;
					ldxe += 2;
					ldye += - 6;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFF0FU) | (ncol << 4);
					}

					nw1 += u1dx + u1dy * 11;
					nw2 += u2dx + u2dy * 11;
					nw3 += u3dx + u3dy * 11;

					ldxs += 1;
					ldys += 11;
					ldxe += 1;
					ldye += 11;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xF0FFU) | (ncol << 8);
					}

					nw1 += u1dx * 2 - u1dy * 7;
					nw2 += u2dx * 2 - u2dy * 7;
					nw3 += u3dx * 2 - u3dy * 7;

					ldxs += 2;
					ldys += - 7;
					ldxe += 2;
					ldye += - 7;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0x0FFFU) | (ncol << 12);
					}

					*(canvas + x * 2) = nb;

					nb = *(canvas + x * 2 + 1);

					nw1 += u1dx * 2 + u1dy * 5;
					nw2 += u2dx * 2 + u2dy * 5;
					nw3 += u2dx * 2 + u3dy * 5;

					ldxs += 2;
					ldys += 5;
					ldxe += 2;
					ldye += 5;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFFF0U) | (ncol << 0);
					}

					nw1 += u1dx * 3 + u1dy * 4;
					nw2 += u2dx * 3 + u2dy * 4;
					nw3 += u3dx * 3 + u3dy * 4;

					ldxs += 3;
					ldys += 4;
					ldxe += 3;
					ldye += 4;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFF0FU) | (ncol << 4);
					}

					nw1 += u1dx - u1dy * 12;
					nw2 += u2dx - u2dy * 12;
					nw3 += u3dx - u3dy * 12;

					ldxs += 1;
					ldys += - 12;
					ldxe += 1;
					ldye += - 12;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xF0FFU) | (ncol << 8);
					}

					nw1 += u1dx + u1dy * 6;
					nw2 += u2dx + u2dy * 6;
					nw3 += u3dx + u3dy * 6;

					ldxs += 1;
					ldys += 6;
					ldxe += 1;
					ldye += 6;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0x0FFFU) | (ncol << 12);
					}

					*(canvas + x * 2 + 1) = nb;
				}

				lw1 += w1dx;
				lw2 += w2dx;
				lw3 += w3dx;
			}

			w1 += w1dy;
			w2 += w2dy;
			w3 += w3dy;

			canvas += yspan;
		}
	}
}

void drawDashCanvas(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		double fxe, double fye, int ncol, int thickness, int dash, int space)
{
	svg_t			*g = (svg_t *) surface->userdata;
	clipBox_t		lcb;

	int			yspan, xs, ys, xe, ye, x, y, h, m, f, e, l, d, r;
	int			w1, w2, lw1, lw2, ww2, nw1, nw2, lk, context;
	int			w1dx, w2dx, w1dy, w2dy, u1dx, u2dx, u1dy, u2dy;

	lcb.min_x = cb->min_x - 16;
	lcb.min_y = cb->min_y - 16;
	lcb.max_x = cb->max_x + 16;
	lcb.max_y = cb->max_y + 16;

	if (clipLine(&lcb, &fxs, &fys, &fxe, &fye) < 0)
		return ;

	if (g != NULL) {

		double		_fxs = fxs, _fys = fys, _fxe = fxe, _fye = fye;

		if (clipLine(cb, &_fxs, &_fys, &_fxe, &_fye) == 0) {

			svgDrawLine(g, _fxs, _fys, _fxe, _fye,
					(svgCol_t) dw->palette[ncol],
					thickness, dash + thickness, space);
		}
	}

	if (fys < fye) {

		xs = (int) (fxe * 16.);
		ys = (int) (fye * 16.);
		xe = (int) (fxs * 16.);
		ye = (int) (fys * 16.);
	}
	else {
		xs = (int) (fxs * 16.);
		ys = (int) (fys * 16.);
		xe = (int) (fxe * 16.);
		ye = (int) (fye * 16.);
	}

	h = (thickness > 0) ? thickness * 8 : 5;
	h += (dw->antialiasing != DRAW_SOLID) ? 12 : 0;

	m = (dash + thickness) * 16;
	f = m + space * 16;

	if (xs < xe) {

		lcb.min_x = (xs - h) / 16;
		lcb.max_x = (xe + h) / 16;
	}
	else {
		lcb.min_x = (xe - h) / 16;
		lcb.max_x = (xs + h) / 16;
	}

	if (ys < ye) {

		lcb.min_y = (ys - h) / 16;
		lcb.max_y = (ye + h) / 16;
	}
	else {
		lcb.min_y = (ye - h) / 16;
		lcb.max_y = (ys + h) / 16;
	}

	lcb.min_x = (lcb.min_x < cb->min_x) ? cb->min_x : lcb.min_x;
	lcb.min_y = (lcb.min_y < cb->min_y) ? cb->min_y : lcb.min_y;
	lcb.max_x = (lcb.max_x > cb->max_x) ? cb->max_x : lcb.max_x;
	lcb.max_y = (lcb.max_y > cb->max_y) ? cb->max_y : lcb.max_y;

	e = (xs - xe) * (xs - xe) + (ys - ye) * (ys - ye);
	d = (int) sqrtf((float) e);

	l = d * h;

	if (fys < fye) {

		w1 = (ys - ye) * (lcb.min_x * 16 - xe + 8) - (xs - xe) * (lcb.min_y * 16 - ye + 8);
		w2 = (xs - xe) * (lcb.min_x * 16 - xe + 8) + (ys - ye) * (lcb.min_y * 16 - ye + 8);

		w1dx = (ys - ye) * 16;
		w2dx = (xs - xe) * 16;

		w1dy = - (xs - xe) * 16;
		w2dy = (ys - ye) * 16;
	}
	else {
		w1 = (ys - ye) * (lcb.min_x * 16 - xe + 8) - (xs - xe) * (lcb.min_y * 16 - ye + 8);
		w2 = (xe - xs) * (lcb.min_x * 16 - xs + 8) + (ye - ys) * (lcb.min_y * 16 - ys + 8);

		w1dx = (ys - ye) * 16;
		w2dx = (xe - xs) * 16;

		w1dy = - (xs - xe) * 16;
		w2dy = (ye - ys) * 16;
	}

	context = dw->dash_context;

	ww2 = context * d;
	context = (context + d) % f;

	dw->dash_context = context;

	if (dw->antialiasing == DRAW_SOLID) {

		Uint8		*canvas = (Uint8 *) dw->pixmap.canvas;

		yspan = dw->pixmap.yspan;
		canvas += lcb.min_y * yspan;

		for (y = lcb.min_y; y <= lcb.max_y; ++y) {

			x = (w1dx > 0) ? - (l + w1) / w1dx : 0;
			x = (x < 0) ? 0 : x;

			lw1 = w1 + w1dx * x;
			lw2 = w2 + w2dx * x;

			for (x += lcb.min_x; x <= lcb.max_x; ++x) {

				if (		lw1 < l
						&& lw1 >= - l) {

					if (		lw2 >= 0
							&& lw2 < e) {

						r = (lw2 + ww2) / d;

						if ((r % f) < m) {

							*(canvas + x) = ncol;
						}
					}
				}
				else if (lw1 > 0) {

					break;
				}

				lw1 += w1dx;
				lw2 += w2dx;
			}

			w1 += w1dy;
			w2 += w2dy;

			canvas += yspan;
		}
	}
	else if (dw->antialiasing == DRAW_4X_MSAA) {

		Uint16		*canvas = (Uint16 *) dw->pixmap.canvas;

		lk = d * (h - 12);

		u1dx = (ys - ye);
		u2dx = (xe - xs);

		u1dy = - (xs - xe);
		u2dy = (ye - ys);

		yspan = dw->pixmap.yspan;
		canvas += lcb.min_y * yspan;

		for (y = lcb.min_y; y <= lcb.max_y; ++y) {

			x = (w1dx > 0) ? - (l + w1) / w1dx : 0;
			x = (x < 0) ? 0 : x;

			lw1 = w1 + w1dx * x;
			lw2 = w2 + w2dx * x;

			for (x += lcb.min_x; x <= lcb.max_x; ++x) {

				if (		lw1 < l
						&& lw1 >= - l) {

					Uint16          nb = *(canvas + x);

					nw1 = lw1 - u1dx * 5 + u1dy * 2;
					nw2 = lw2 - u2dx * 5 + u2dy * 2;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}

					nw1 += u1dx * 3 - u1dy * 7;
					nw2 += u2dx * 3 - u2dy * 7;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}

					nw1 += u1dx * 4 + u1dy * 10;
					nw2 += u2dx * 4 + u2dy * 10;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}

					nw1 += u1dx * 3 - u1dy * 7;
					nw2 += u2dx * 3 - u2dy * 7;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}

					*(canvas + x) = nb;
				}
				else if (lw1 > 0) {

					break;
				}

				lw1 += w1dx;
				lw2 += w2dx;
			}

			w1 += w1dy;
			w2 += w2dy;

			canvas += yspan;
		}
	}
	else if (dw->antialiasing == DRAW_8X_MSAA) {

		Uint16		*canvas = (Uint16 *) dw->pixmap.canvas;

		lk = d * (h - 12);

		u1dx = (ys - ye);
		u2dx = (xe - xs);

		u1dy = - (xs - xe);
		u2dy = (ye - ys);

		yspan = dw->pixmap.yspan * 2;
		canvas += lcb.min_y * yspan;

		for (y = lcb.min_y; y <= lcb.max_y; ++y) {

			x = (w1dx > 0) ? - (l + w1) / w1dx : 0;
			x = (x < 0) ? 0 : x;

			lw1 = w1 + w1dx * x;
			lw2 = w2 + w2dx * x;

			for (x += lcb.min_x; x <= lcb.max_x; ++x) {

				if (		lw1 < l
						&& lw1 >= - l) {

					Uint16          nb = *(canvas + x * 2);

					nw1 = lw1 - u1dx * 7 - u1dy;
					nw2 = lw2 - u2dx * 7 - u2dy;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}

					nw1 += u1dx * 2 - u1dy * 6;
					nw2 += u2dx * 2 - u2dy * 6;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}

					nw1 += u1dx + u1dy * 11;
					nw2 += u2dx + u2dy * 11;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}

					nw1 += u1dx * 2 - u1dy * 7;
					nw2 += u2dx * 2 - u2dy * 7;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}

					*(canvas + x * 2) = nb;

					nb = *(canvas + x * 2 + 1);

					nw1 += u1dx * 2 + u1dy * 5;
					nw2 += u2dx * 2 + u2dy * 5;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}

					nw1 += u1dx * 3 + u1dy * 4;
					nw2 += u2dx * 3 + u2dy * 4;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}

					nw1 += u1dx - u1dy * 12;
					nw2 += u2dx - u2dy * 12;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}

					nw1 += u1dx + u1dy * 6;
					nw2 += u2dx + u2dy * 6;

					if (		nw1 < lk
							&& nw1 >= - lk
							&& nw2 >= 0
							&& nw2 < e) {

						r = (nw2 + ww2) / d;

						if ((r % f) < m) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}

					*(canvas + x * 2 + 1) = nb;
				}
				else if (lw1 > 0) {

					break;
				}

				lw1 += w1dx;
				lw2 += w2dx;
			}

			w1 += w1dy;
			w2 += w2dy;

			canvas += yspan;
		}
	}
}

int drawLineTrial(draw_t *dw, clipBox_t *cb, double fxs, double fys,
		double fxe, double fye, int ncol, int thickness)
{
	clipBox_t		lcb;

	int			yspan, xs, ys, xe, ye, x, y, h, l, d, r;
	int			w1, w2, w3, lw1, lw2, lw3, w1dx, w2dx, w3dx;
	int			w1dy, w2dy, w3dy, ldxs, ldys, ldxe, ldye, la, fill;

	lcb.min_x = cb->min_x - 16;
	lcb.min_y = cb->min_y - 16;
	lcb.max_x = cb->max_x + 16;
	lcb.max_y = cb->max_y + 16;

	if (clipLine(&lcb, &fxs, &fys, &fxe, &fye) < 0)
		return 0;

	h =	  (dw->antialiasing == DRAW_4X_MSAA) ? 2
		: (dw->antialiasing == DRAW_8X_MSAA) ? 3 : 1;

	xs = (int) (fxs * (double) h + .5);
	ys = (int) (fys * (double) h + .5);
	xe = (int) (fxe * (double) h + .5);
	ye = (int) (fye * (double) h + .5);

	if (		ncol == dw->cached_ncol
			&& xs == xe
			&& xe == dw->cached_x) {

		if (ys < ye) {

			if (ys >= dw->cached_min_y && ye <= dw->cached_max_y)
				return 0;

			dw->cached_min_y = (ys < dw->cached_min_y) ? ys : dw->cached_min_y;
			dw->cached_max_y = (ye > dw->cached_max_y) ? ye : dw->cached_max_y;
		}
		else {
			if (ye >= dw->cached_min_y && ys <= dw->cached_max_y)
				return 0;

			dw->cached_min_y = (ye < dw->cached_min_y) ? ye : dw->cached_min_y;
			dw->cached_max_y = (ys > dw->cached_max_y) ? ys : dw->cached_max_y;
		}
	}
	else {
		dw->cached_x = xe;
		dw->cached_min_y = lcb.max_y * h;
		dw->cached_max_y = lcb.min_y * h;
		dw->cached_ncol = ncol;
	}

	if (fys < fye) {

		xs = (int) (fxe * 16.);
		ys = (int) (fye * 16.);
		xe = (int) (fxs * 16.);
		ye = (int) (fys * 16.);
	}
	else {
		xs = (int) (fxs * 16.);
		ys = (int) (fys * 16.);
		xe = (int) (fxe * 16.);
		ye = (int) (fye * 16.);
	}


	h = (thickness > 0) ? thickness * 8 : 5;
	h += (dw->antialiasing != DRAW_SOLID) ? 12 : 0;

	if (xs < xe) {

		lcb.min_x = (xs - h) / 16;
		lcb.max_x = (xe + h) / 16;
	}
	else {
		lcb.min_x = (xe - h) / 16;
		lcb.max_x = (xs + h) / 16;
	}

	if (ys < ye) {

		lcb.min_y = (ys - h) / 16;
		lcb.max_y = (ye + h) / 16;
	}
	else {
		lcb.min_y = (ye - h) / 16;
		lcb.max_y = (ys + h) / 16;
	}

	lcb.min_x = (lcb.min_x < cb->min_x) ? cb->min_x : lcb.min_x;
	lcb.min_y = (lcb.min_y < cb->min_y) ? cb->min_y : lcb.min_y;
	lcb.max_x = (lcb.max_x > cb->max_x) ? cb->max_x : lcb.max_x;
	lcb.max_y = (lcb.max_y > cb->max_y) ? cb->max_y : lcb.max_y;

	l = (xs - xe) * (xs - xe) + (ys - ye) * (ys - ye);
	d = (int) sqrtf((float) l);

	l = d * h;
	r = h * h;

	w1 = (ys - ye) * (lcb.min_x * 16 - xe + 8) - (xs - xe) * (lcb.min_y * 16 - ye + 8);
	w2 = (xe - xs) * (lcb.min_x * 16 - xs + 8) + (ye - ys) * (lcb.min_y * 16 - ys + 8);
	w3 = (xs - xe) * (lcb.min_x * 16 - xe + 8) + (ys - ye) * (lcb.min_y * 16 - ye + 8);

	w1dx = (ys - ye) * 16;
	w2dx = (xe - xs) * 16;
	w3dx = (xs - xe) * 16;

	w1dy = - (xs - xe) * 16;
	w2dy = (ye - ys) * 16;
	w3dy = (ys - ye) * 16;

	fill = 0;

	if (dw->antialiasing == DRAW_SOLID) {

		Uint8		*trial = (Uint8 *) dw->pixmap.trial;

		yspan = dw->pixmap.yspan;
		trial += lcb.min_y * yspan;

		for (y = lcb.min_y; y <= lcb.max_y; ++y) {

			x = (w1dx > 0) ? - (l + w1) / w1dx : 0;
			x = (x < 0) ? 0 : x;

			lw1 = w1 + w1dx * x;
			lw2 = w2 + w2dx * x;
			lw3 = w3 + w3dx * x;

			for (x += lcb.min_x; x <= lcb.max_x; ++x) {

				if (lw2 <= 0) {

					ldxs = x * 16 - xs + 8;
					ldys = y * 16 - ys + 8;

					la = ldxs * ldxs + ldys * ldys;

					if (la <= r) {

						if (*(trial + x) != ncol) {
							*(trial + x) = ncol;
							fill++; }
					}
				}
				else if (lw3 <= 0) {

					ldxe = x * 16 - xe + 8;
					ldye = y * 16 - ye + 8;

					la = ldxe * ldxe + ldye * ldye;

					if (la <= r) {

						if (*(trial + x) != ncol) {
							*(trial + x) = ncol;
							fill++; }
					}
				}
				else if (lw1 < l) {

					if (lw1 >= - l) {

						if (*(trial + x) != ncol) {
							*(trial + x) = ncol;
							fill++; }
					}
				}
				else {
					break;
				}

				lw1 += w1dx;
				lw2 += w2dx;
				lw3 += w3dx;
			}

			w1 += w1dy;
			w2 += w2dy;
			w3 += w3dy;

			trial += yspan;
		}
	}
	else {
		Uint16		*trial = (Uint16 *) dw->pixmap.trial;

		int		lk, rk, dk, nw1, nw2, nw3, u1dx, u2dx;
		int		u3dx, u1dy, u2dy, u3dy, touch;

		lk = d * (h - 12);
		rk = (h - 12) * (h - 12);
		dk = d * 12;

		u1dx = (ys - ye) * 4;
		u2dx = (xe - xs) * 4;
		u3dx = (xs - xe) * 4;

		u1dy = - (xs - xe) * 4;
		u2dy = (ye - ys) * 4;
		u3dy = (ys - ye) * 4;

		yspan = dw->pixmap.yspan;
		trial += lcb.min_y * yspan;

		for (y = lcb.min_y; y <= lcb.max_y; ++y) {

			x = (w1dx > 0) ? - (l + w1) / w1dx : 0;
			x = (x < 0) ? 0 : x;

			lw1 = w1 + w1dx * x;
			lw2 = w2 + w2dx * x;
			lw3 = w3 + w3dx * x;

			for (x += lcb.min_x; x <= lcb.max_x; ++x) {

				touch = 0;

				if (lw2 <= 0) {

					ldxs = x * 16 - xs + 8;
					ldys = y * 16 - ys + 8;

					la = ldxs * ldxs + ldys * ldys;

					if (la <= r) {

						touch = 1;
					}
				}
				else if (lw3 <= 0) {

					ldxe = x * 16 - xe + 8;
					ldye = y * 16 - ye + 8;

					la = ldxe * ldxe + ldye * ldye;

					if (la <= r) {

						touch = 1;
					}
				}
				else if (lw1 < l) {

					if (lw1 >= - l) {

						touch = 2;
					}
				}
				else {
					break;
				}

				if (		touch == 2
						&& lw2 > dk
						&& lw3 > dk) {

					Uint16		nb = *(trial + x);
					Uint16		bg = nb;

					nw1 = lw1 - u1dx * 5 + u1dy * 2;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFFF0U) | (ncol << 0);
					}

					nw1 += u1dx * 3 - u1dy * 7;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFF0FU) | (ncol << 4);
					}

					nw1 += u1dx * 4 + u1dy * 10;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xF0FFU) | (ncol << 8);
					}

					nw1 += u1dx * 3 - u1dy * 7;

					if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0x0FFFU) | (ncol << 12);
					}

					if (bg != nb) {
						*(trial + x) = nb;
						fill++; }
				}
				else if (touch != 0) {

					Uint16		nb = *(trial + x);
					Uint16		bg = nb;

					nw1 = lw1 - u1dx * 5 + u1dy * 2;
					nw2 = lw2 - u2dx * 5 + u2dy * 2;
					nw3 = lw3 - u3dx * 5 + u3dy * 2;

					ldxs = x * 16 - xs + 3;
					ldys = y * 16 - ys + 10;
					ldxe = x * 16 - xe + 3;
					ldye = y * 16 - ye + 10;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFFF0U) | (ncol << 0);
					}

					nw1 += u1dx * 3 - u1dy * 7;
					nw2 += u2dx * 3 - u2dy * 7;
					nw3 += u3dx * 3 - u3dy * 7;

					ldxs += 3;
					ldys += - 7;
					ldxe += 3;
					ldye += - 7;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xFF0FU) | (ncol << 4);
					}

					nw1 += u1dx * 4 + u1dy * 10;
					nw2 += u2dx * 4 + u2dy * 10;
					nw3 += u3dx * 4 + u3dy * 10;

					ldxs += 4;
					ldys += 10;
					ldxe += 4;
					ldye += 10;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0xF0FFU) | (ncol << 8);
					}

					nw1 += u1dx * 3 - u1dy * 7;
					nw2 += u2dx * 3 - u2dy * 7;
					nw3 += u3dx * 3 - u3dy * 7;

					ldxs += 3;
					ldys += - 7;
					ldxe += 3;
					ldye += - 7;

					if (nw2 <= 0) {

						la = ldxs * ldxs + ldys * ldys;

						if (la <= rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}
					else if (nw3 <= 0) {

						la = ldxe * ldxe + ldye * ldye;

						if (la <= rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}
					}
					else if (nw1 < lk && nw1 >= - lk) {

						nb = (nb & 0x0FFFU) | (ncol << 12);
					}

					if (bg != nb) {
						*(trial + x) = nb;
						fill++; }
				}

				lw1 += w1dx;
				lw2 += w2dx;
				lw3 += w3dx;
			}

			w1 += w1dy;
			w2 += w2dy;
			w3 += w3dy;

			trial += yspan;
		}
	}

	return fill;
}

void drawText(draw_t *dw, SDL_Surface *surface, TTF_Font *font, int xs, int ys,
		const char *text, int flags, Uint32 col)
{
	svg_t			*g = (svg_t *) surface->userdata;
	SDL_Surface		*textSurface, *surfaceCopy;
	SDL_Rect		textRect;
	SDL_Color		textColor;

	int			pitch, i, j;

	if (font == NULL)
		return ;

	if (text[0] == 0)
		return ;

	if (g != NULL) {

		svgDrawText(g, xs, ys, text, (svgCol_t) col, flags);
	}

	textColor.a = (Uint8) 0;
	textColor.r = (Uint8) ((col & 0x00FF0000UL) >> 16);
	textColor.g = (Uint8) ((col & 0x0000FF00UL) >> 8);
	textColor.b = (Uint8) ((col & 0x000000FFUL) >> 0);

	if (dw->blendfont != 0) {

		textSurface = TTF_RenderUTF8_Blended(font, text, textColor);
	}
	else {
		textSurface = TTF_RenderUTF8_Solid(font, text, textColor);
	}

	if (textSurface == NULL)
		return ;

	if (flags & TEXT_VERTICAL) {

		surfaceCopy = SDL_CreateRGBSurfaceWithFormat(0, textSurface->h, textSurface->w,
				textSurface->format->BitsPerPixel, textSurface->format->format);

		if (textSurface->format->BitsPerPixel == 8) {

			Uint8		*pixels, *pixelsCopy;

			SDL_SetSurfacePalette(surfaceCopy, textSurface->format->palette);
			SDL_SetColorKey(surfaceCopy, SDL_TRUE, 0);

			pixels = (Uint8 *) textSurface->pixels;
			pixelsCopy = (Uint8 *) surfaceCopy->pixels;

			SDL_LockSurface(textSurface);
			SDL_LockSurface(surfaceCopy);

			pixels += textSurface->w - 1;
			pitch = textSurface->pitch;

			for (i = 0; i < surfaceCopy->h; ++i) {

				for (j = 0; j < surfaceCopy->w; ++j)
					pixelsCopy[j] = pixels[j * pitch];

				pixelsCopy += surfaceCopy->pitch;
				pixels += -1;
			}

			SDL_UnlockSurface(textSurface);
			SDL_UnlockSurface(surfaceCopy);
		}
		else if (textSurface->format->BitsPerPixel == 32) {

			Uint32		*pixels, *pixelsCopy;

			pixels = (Uint32 *) textSurface->pixels;
			pixelsCopy = (Uint32 *) surfaceCopy->pixels;

			SDL_LockSurface(textSurface);
			SDL_LockSurface(surfaceCopy);

			pixels += textSurface->w - 1;
			pitch = textSurface->pitch / 4;

			for (i = 0; i < surfaceCopy->h; ++i) {

				for (j = 0; j < surfaceCopy->w; ++j)
					pixelsCopy[j] = pixels[j * pitch];

				pixelsCopy += surfaceCopy->pitch / 4;
				pixels += - 1;
			}

			SDL_UnlockSurface(textSurface);
			SDL_UnlockSurface(surfaceCopy);
		}

		SDL_FreeSurface(textSurface);
		textSurface = surfaceCopy;
	}

	textRect.w = textSurface->w;
	textRect.h = textSurface->h;
	textRect.x = xs;
	textRect.y = ys;

	if (flags & TEXT_CENTERED_ON_X) {

		textRect.x += - textRect.w / 2;
	}

	if (flags & TEXT_CENTERED_ON_Y) {

		textRect.y += - textRect.h / 2;
	}

	SDL_BlitSurface(textSurface, NULL, surface, &textRect);
	SDL_FreeSurface(textSurface);
}

void drawFillRect(SDL_Surface *surface, int xs, int ys,
		int xe, int ye, Uint32 col)
{
	svg_t			*g = (svg_t *) surface->userdata;
	Uint32			*pixels = (Uint32 *) surface->pixels;
	Uint32			*px, *px_end;

	int			pitch, i;

	xs = (xs < 0) ? 0 : xs;
	xe = (xe > surface->w - 1) ? surface->w - 1 : xe;

	ys = (ys < 0) ? 0 : ys;
	ye = (ye > surface->h - 1) ? surface->h - 1 : ye;

	if (g != NULL) {

		svgDrawRect(g, xs, ys, xe, ye, (svgCol_t) col);
	}

	pitch = surface->pitch / 4;

	for (i = ys; i <= ye; i++) {

		px = pixels + pitch * i + xs;
		px_end = px + (xe - xs);

		while (px <= px_end) {

			*px++ = col;
		}
	}
}

void drawClipRect(SDL_Surface *surface, clipBox_t *cb, int xs, int ys,
		int xe, int ye, Uint32 col)
{
	svg_t			*g = (svg_t *) surface->userdata;
	Uint32			*pixels = (Uint32 *) surface->pixels;
	Uint32			*px, *px_end;

	int			pitch, i;

	if (xs > cb->max_x || xe < cb->min_x)
		return ;

	if (ys > cb->max_y || ye < cb->min_y)
		return ;

	xs = (xs < cb->min_x) ? cb->min_x : xs;
	xe = (xe > cb->max_x) ? cb->max_x : xe;

	ys = (ys < cb->min_y) ? cb->min_y : ys;
	ye = (ye > cb->max_y) ? cb->max_y : ye;

	if (g != NULL) {

		svgDrawRect(g, xs, ys, xe, ye, (svgCol_t) col);
	}

	pitch = surface->pitch / 4;

	for (i = ys; i <= ye; i++) {

		px = pixels + pitch * i + xs;
		px_end = px + (xe - xs);

		while (px <= px_end) {

			*px++ = col;
		}
	}
}

void drawDotCanvas(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		int rsize, int ncol, int round)
{
	svg_t			*g = (svg_t *) surface->userdata;
	clipBox_t		lcb;

	int			yspan, xs, ys, x, y, h, r;
	int			w1, w2, lw1, nw1, nw2, lw1a, lw2a;
	int			ldx, ldy, rk, hk, la;

	if (fxs > cb->max_x + 16 || fxs < cb->min_x - 16)
		return ;

	if (fys > cb->max_y + 16 || fys < cb->min_y - 16)
		return ;

	if (g != NULL) {

		if (		   fxs > cb->min_x && fxs < cb->max_x
				&& fys > cb->min_y && fys < cb->max_y) {

			if (round == 0) {

				svgDrawRect(g, fxs - rsize * 0.5, fys - rsize * 0.5,
						fxs + rsize * 0.5, fys + rsize * 0.5,
						(svgCol_t) dw->palette[ncol]);
			}
			else {
				svgDrawCircle(g, fxs, fys, rsize * 0.5,
						(svgCol_t) dw->palette[ncol]);
			}
		}
	}

	xs = (int) (fxs * 16.);
	ys = (int) (fys * 16.);

	h = (rsize > 0) ? rsize * 8 : 5;
	h += (dw->antialiasing != DRAW_SOLID) ? 12 : 0;

	lcb.min_x = (xs - h) / 16;
	lcb.max_x = (xs + h) / 16;
	lcb.min_y = (ys - h) / 16;
	lcb.max_y = (ys + h) / 16;

	lcb.min_x = (lcb.min_x < cb->min_x) ? cb->min_x : lcb.min_x;
	lcb.min_y = (lcb.min_y < cb->min_y) ? cb->min_y : lcb.min_y;
	lcb.max_x = (lcb.max_x > cb->max_x) ? cb->max_x : lcb.max_x;
	lcb.max_y = (lcb.max_y > cb->max_y) ? cb->max_y : lcb.max_y;

	if (round == 0) {

		w1 = lcb.min_x * 16 - xs + 8;
		w2 = lcb.min_y * 16 - ys + 8;

		if (dw->antialiasing == DRAW_SOLID) {

			Uint8		*canvas = (Uint8 *) dw->pixmap.canvas;

			yspan = dw->pixmap.yspan;
			canvas += lcb.min_y * yspan;

			for (y = lcb.min_y; y <= lcb.max_y; ++y) {

				lw1 = w1;

				for (x = lcb.min_x; x <= lcb.max_x; ++x) {

					lw1a = (lw1 < 0) ? - lw1 : lw1;
					lw2a = (w2 < 0) ? - w2 : w2;

					if (lw1a < h && lw2a < h) {

						*(canvas + x) = ncol;
					}

					lw1 += 16;
				}

				w2 += 16;

				canvas += yspan;
			}
		}
		else if (dw->antialiasing == DRAW_4X_MSAA) {

			Uint16		*canvas = (Uint16 *) dw->pixmap.canvas;

			hk = h - 12;

			yspan = dw->pixmap.yspan;
			canvas += lcb.min_y * yspan;

			for (y = lcb.min_y; y <= lcb.max_y; ++y) {

				lw1 = w1;

				for (x = lcb.min_x; x <= lcb.max_x; ++x) {

					lw1a = (lw1 < 0) ? - lw1 : lw1;
					lw2a = (w2 < 0) ? - w2 : w2;

					if (lw1a < h && lw2a < h) {

						Uint16		nb = *(canvas + x);

						nw1 = lw1 - 5;
						nw2 = w2 + 2;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}

						nw1 += 3;
						nw2 += - 7;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}

						nw1 += 4;
						nw2 += 10;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}

						nw1 += 3;
						nw2 += - 7;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}

						*(canvas + x) = nb;
					}

					lw1 += 16;
				}

				w2 += 16;

				canvas += yspan;
			}
		}
		else if (dw->antialiasing == DRAW_8X_MSAA) {

			Uint16		*canvas = (Uint16 *) dw->pixmap.canvas;

			hk = h - 12;

			yspan = dw->pixmap.yspan * 2;
			canvas += lcb.min_y * yspan;

			for (y = lcb.min_y; y <= lcb.max_y; ++y) {

				lw1 = w1;

				for (x = lcb.min_x; x <= lcb.max_x; ++x) {

					lw1a = (lw1 < 0) ? - lw1 : lw1;
					lw2a = (w2 < 0) ? - w2 : w2;

					if (lw1a < h && lw2a < h) {

						Uint16		nb = *(canvas + x * 2);

						nw1 = lw1 - 7;
						nw2 = w2 - 1;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}

						nw1 += 2;
						nw2 += - 6;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}

						nw1 += 1;
						nw2 += 11;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}

						nw1 += 2;
						nw2 += - 7;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}

						*(canvas + x * 2) = nb;

						nb = *(canvas + x * 2 + 1);

						nw1 += 2;
						nw2 += 5;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}

						nw1 += 3;
						nw2 += 4;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}

						nw1 += 1;
						nw2 += - 12;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}

						nw1 += 1;
						nw2 += 6;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}

						*(canvas + x * 2 + 1) = nb;
					}

					lw1 += 16;
				}

				w2 += 16;

				canvas += yspan;
			}
		}
	}
	else {
		r = h * h;

		if (dw->antialiasing == DRAW_SOLID) {

			Uint8		*canvas = (Uint8 *) dw->pixmap.canvas;

			yspan = dw->pixmap.yspan;
			canvas += lcb.min_y * yspan;

			for (y = lcb.min_y; y <= lcb.max_y; ++y) {

				for (x = lcb.min_x; x <= lcb.max_x; ++x) {

					ldx = x * 16 - xs + 8;
					ldy = y * 16 - ys + 8;

					la = ldx * ldx + ldy * ldy;

					if (la < r) {

						*(canvas + x) = ncol;
					}
				}

				canvas += yspan;
			}
		}
		else if (dw->antialiasing == DRAW_4X_MSAA) {

			Uint16		*canvas = (Uint16 *) dw->pixmap.canvas;

			rk = (h - 12) * (h - 12);

			yspan = dw->pixmap.yspan;
			canvas += lcb.min_y * yspan;

			for (y = lcb.min_y; y <= lcb.max_y; ++y) {

				for (x = lcb.min_x; x <= lcb.max_x; ++x) {

					ldx = x * 16 - xs + 8;
					ldy = y * 16 - ys + 8;

					la = ldx * ldx + ldy * ldy;

					if (la < r) {

						Uint16		nb = *(canvas + x);

						ldx = x * 16 - xs + 3;
						ldy = y * 16 - ys + 10;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}

						ldx += 3;
						ldy += - 7;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}

						ldx += 4;
						ldy += 10;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}

						ldx += 3;
						ldy += - 7;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}

						*(canvas + x) = nb;
					}
				}

				canvas += yspan;
			}
		}
		else if (dw->antialiasing == DRAW_8X_MSAA) {

			Uint16		*canvas = (Uint16 *) dw->pixmap.canvas;

			rk = (h - 12) * (h - 12);

			yspan = dw->pixmap.yspan * 2;
			canvas += lcb.min_y * yspan;

			for (y = lcb.min_y; y <= lcb.max_y; ++y) {

				for (x = lcb.min_x; x <= lcb.max_x; ++x) {

					ldx = x * 16 - xs + 8;
					ldy = y * 16 - ys + 8;

					la = ldx * ldx + ldy * ldy;

					if (la < r) {

						Uint16		nb = *(canvas + x * 2);

						ldx = x * 16 - xs + 1;
						ldy = y * 16 - ys + 7;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}

						ldx += 2;
						ldy += - 6;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}

						ldx += 1;
						ldy += 11;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}

						ldx += 2;
						ldy += - 7;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}

						*(canvas + x * 2) = nb;

						nb = *(canvas + x * 2 + 1);

						ldx += 2;
						ldy += 5;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}

						ldx += 3;
						ldy += 4;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}

						ldx += 1;
						ldy += - 12;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}

						ldx += 1;
						ldy += 6;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}

						*(canvas + x * 2 + 1) = nb;
					}
				}

				canvas += yspan;
			}
		}
	}
}

int drawDotTrial(draw_t *dw, clipBox_t *cb, double fxs, double fys,
		int rsize, int ncol, int round)
{
	clipBox_t		lcb;

	int			yspan, xs, ys, x, y, h, r;
	int			w1, w2, lw1, nw1, nw2, lw1a, lw2a;
	int			ldx, ldy, rk, hk, la, fill;

	if (fxs > cb->max_x + 16 || fxs < cb->min_x - 16)
		return 0;

	if (fys > cb->max_y + 16 || fys < cb->min_y - 16)
		return 0;

	xs = (int) (fxs * 16.);
	ys = (int) (fys * 16.);

	h = (rsize > 0) ? rsize * 8 : 5;
	h += (dw->antialiasing != DRAW_SOLID) ? 12 : 0;

	lcb.min_x = (xs - h) / 16;
	lcb.max_x = (xs + h) / 16;
	lcb.min_y = (ys - h) / 16;
	lcb.max_y = (ys + h) / 16;

	lcb.min_x = (lcb.min_x < cb->min_x) ? cb->min_x : lcb.min_x;
	lcb.min_y = (lcb.min_y < cb->min_y) ? cb->min_y : lcb.min_y;
	lcb.max_x = (lcb.max_x > cb->max_x) ? cb->max_x : lcb.max_x;
	lcb.max_y = (lcb.max_y > cb->max_y) ? cb->max_y : lcb.max_y;

	fill = 0;

	if (round == 0) {

		w1 = lcb.min_x * 16 - xs + 8;
		w2 = lcb.min_y * 16 - ys + 8;

		if (dw->antialiasing == DRAW_SOLID) {

			Uint8		*trial = (Uint8 *) dw->pixmap.trial;

			yspan = dw->pixmap.yspan;
			trial += lcb.min_y * yspan;

			for (y = lcb.min_y; y <= lcb.max_y; ++y) {

				lw2a = (w2 < 0) ? - w2 : w2;

				lw1 = w1;

				for (x = lcb.min_x; x <= lcb.max_x; ++x) {

					lw1a = (lw1 < 0) ? - lw1 : lw1;

					if (lw1a < h && lw2a < h) {

						if (*(trial + x) != ncol) {
							*(trial + x) = ncol;
							fill++; }
					}

					lw1 += 16;
				}

				w2 += 16;

				trial += yspan;
			}
		}
		else {
			Uint16		*trial = (Uint16 *) dw->pixmap.trial;

			hk = h - 12;

			yspan = dw->pixmap.yspan;
			trial += lcb.min_y * yspan;

			for (y = lcb.min_y; y <= lcb.max_y; ++y) {

				lw2a = (w2 < 0) ? - w2 : w2;

				lw1 = w1;

				for (x = lcb.min_x; x <= lcb.max_x; ++x) {

					lw1a = (lw1 < 0) ? - lw1 : lw1;

					if (lw1a < h && lw2a < h) {

						Uint16		nb = *(trial + x);
						Uint16		bg = nb;

						nw1 = lw1 - 5;
						nw2 = w2 + 2;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}

						nw1 += 3;
						nw2 += - 7;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}

						nw1 += 4;
						nw2 += 10;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}

						nw1 += 3;
						nw2 += - 7;

						lw1a = (nw1 < 0) ? - nw1 : nw1;
						lw2a = (nw2 < 0) ? - nw2 : nw2;

						if (lw1a < hk && lw2a < hk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}

						if (bg != nb) {
							*(trial + x) = nb;
							fill++; }
					}

					lw1 += 16;
				}

				w2 += 16;

				trial += yspan;
			}
		}
	}
	else {
		r = h * h;
		rk = (h - 12) * (h - 12);

		if (dw->antialiasing == DRAW_SOLID) {

			Uint8		*trial = (Uint8 *) dw->pixmap.trial;

			yspan = dw->pixmap.yspan;
			trial += lcb.min_y * yspan;

			for (y = lcb.min_y; y <= lcb.max_y; ++y) {

				for (x = lcb.min_x; x <= lcb.max_x; ++x) {

					ldx = x * 16 - xs + 8;
					ldy = y * 16 - ys + 8;

					la = ldx * ldx + ldy * ldy;

					if (la < r) {

						if (*(trial + x) != ncol) {
							*(trial + x) = ncol;
							fill++; }
					}
				}

				trial += yspan;
			}
		}
		else {
			Uint16		*trial = (Uint16 *) dw->pixmap.trial;

			yspan = dw->pixmap.yspan;
			trial += lcb.min_y * yspan;

			for (y = lcb.min_y; y <= lcb.max_y; ++y) {

				for (x = lcb.min_x; x <= lcb.max_x; ++x) {

					ldx = x * 16 - xs + 8;
					ldy = y * 16 - ys + 8;

					la = ldx * ldx + ldy * ldy;

					if (la < r) {

						Uint16		nb = *(trial + x);
						Uint16		bg = nb;

						ldx = x * 16 - xs + 3;
						ldy = y * 16 - ys + 10;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xFFF0U) | (ncol << 0);
						}

						ldx += 3;
						ldy += - 7;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xFF0FU) | (ncol << 4);
						}

						ldx += 4;
						ldy += 10;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0xF0FFU) | (ncol << 8);
						}

						ldx += 3;
						ldy += - 7;

						la = ldx * ldx + ldy * ldy;

						if (la < rk) {

							nb = (nb & 0x0FFFU) | (ncol << 12);
						}

						if (bg != nb) {
							*(trial + x) = nb;
							fill++; }
					}
				}

				trial += yspan;
			}
		}
	}

	return fill;
}

void drawMarkCanvas(draw_t *dw, SDL_Surface *surface, clipBox_t *cb, double fxs, double fys,
		int rsize, int shape, int ncol, int thickness)
{
	double		r2, r6, r3, r5, r8, r9;

	if (shape == SHAPE_CIRCLE) {

		r2 = (double) rsize * 0.5;
		r6 = (double) rsize * 0.8660;

		drawLineCanvas(dw, surface, cb, fxs - rsize, fys, fxs - r6, fys - r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r6, fys - r2, fxs - r2, fys - r6, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys - r6, fxs, fys - rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + rsize, fys, fxs + r6, fys - r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r6, fys - r2, fxs + r2, fys - r6, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r2, fys - r6, fxs, fys - rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + rsize, fys, fxs + r6, fys + r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r6, fys + r2, fxs + r2, fys + r6, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r2, fys + r6, fxs, fys + rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - rsize, fys, fxs - r6, fys + r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r6, fys + r2, fxs - r2, fys + r6, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys + r6, fxs, fys + rsize, ncol, thickness);
	}
	else if (shape == SHAPE_STARLET) {

		r2 = (double) 0.4;
		r6 = (double) rsize * 1.2;

		r3 = (double) rsize * 0.3708;
		r5 = (double) rsize * 0.7054;
		r8 = (double) rsize * 0.9708;
		r9 = (double) rsize * 1.1413;

		drawLineCanvas(dw, surface, cb, fxs, fys + r6*r2, fxs + r5, fys + r8, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r5, fys + r8, fxs + r9*r2, fys + r3*r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r9*r2, fys + r3*r2, fxs + r9, fys - r3, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r9, fys - r3, fxs + r5*r2, fys - r8*r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r5*r2, fys - r8*r2, fxs, fys - r6, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs, fys - r6, fxs - r5*r2, fys - r8*r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r5*r2, fys - r8*r2, fxs - r9, fys - r3, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r9, fys - r3, fxs - r9*r2, fys + r3*r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r9*r2, fys + r3*r2, fxs - r5, fys + r8, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r5, fys + r8, fxs, fys + r6*r2, ncol, thickness);
	}
	else if (shape == SHAPE_HUBCAP) {

		r2 = (double) rsize * 0.7;

		drawLineCanvas(dw, surface, cb, fxs - r2, fys + rsize, fxs + r2, fys + rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r2, fys + rsize, fxs, fys - rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs, fys - rsize, fxs - r2, fys + rsize, ncol, thickness);
	}
	else if (shape == SHAPE_RECTANGLE) {

		r2 = (double) rsize * 0.9;

		drawLineCanvas(dw, surface, cb, fxs - r2, fys - r2, fxs + r2, fys - r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys + r2, fxs + r2, fys + r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys - r2, fxs - r2, fys + r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r2, fys - r2, fxs + r2, fys + r2, ncol, thickness);
	}
	else if (shape == SHAPE_SNOWFLAKE) {

		r2 = (double) rsize * 0.7;

		drawLineCanvas(dw, surface, cb, fxs - rsize, fys, fxs + rsize, fys, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs, fys - rsize, fxs, fys + rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys - r2, fxs + r2, fys + r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys + r2, fxs + r2, fys - r2, ncol, thickness);
	}
	else if (shape == SHAPE_TRIANGLE) {

		drawLineCanvas(dw, surface, cb, fxs - rsize, fys - rsize, fxs + rsize, fys - rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + rsize, fys - rsize, fxs, fys + rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs, fys + rsize, fxs - rsize, fys - rsize, ncol, thickness);
	}
	else if (shape == SHAPE_SHARP) {

		r2 = (double) rsize * 0.5;

		drawLineCanvas(dw, surface, cb, fxs - rsize, fys - r2, fxs + rsize, fys - r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - rsize, fys + r2, fxs + rsize, fys + r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys - rsize, fxs - r2, fys + rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r2, fys - rsize, fxs + r2, fys + rsize, ncol, thickness);
	}
	else if (shape == SHAPE_COLUMN) {

		r2 = (double) rsize * 0.5;

		drawLineCanvas(dw, surface, cb, fxs - r2, fys - rsize, fxs + r2, fys - rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys + rsize, fxs + r2, fys + rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys - rsize, fxs - r2, fys + rsize, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs + r2, fys - rsize, fxs + r2, fys + rsize, ncol, thickness);
	}
	else if (shape == SHAPE_FILLING) {

		drawDotCanvas(dw, surface, cb, fxs, fys, (int) (rsize * 1.8), ncol, 0);
	}
	else if (shape == SHAPE_SYMBOL) {

		r2 = (double) rsize * 0.9;

		drawLineCanvas(dw, surface, cb, fxs, fys - r2, fxs, fys + r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys + r2, fxs + r2, fys + r2, ncol, thickness);
		drawLineCanvas(dw, surface, cb, fxs - r2, fys - r2, fxs + r2, fys - r2, ncol, thickness);
	}
}

void drawFlushCanvas(draw_t *dw, SDL_Surface *surface, clipBox_t *cb)
{
	Uint32			*pixels = (Uint32 *) surface->pixels;
	Uint32			*palette = dw->palette;
	Uint8			*ltgamma = dw->ltgamma;

	int			pitch, x, y;
	int			yspan, ncol, blend[3];

	pitch = surface->pitch / 4;
	pixels += cb->min_y * pitch;

	if (dw->antialiasing == DRAW_SOLID) {

		Uint8		nb, *canvas = (Uint8 *) dw->pixmap.canvas;

		yspan = dw->pixmap.yspan;
		canvas += cb->min_y * yspan;

		for (y = cb->min_y; y <= cb->max_y; ++y) {

			for (x = cb->min_x; x <= cb->max_x; ++x) {

				nb = *(canvas + x);

				if (nb != 0) {

					*(pixels + x) = palette[nb];
				}
			}

			pixels += pitch;
			canvas += yspan;
		}
	}
	else if (dw->antialiasing == DRAW_4X_MSAA) {

		Uint16		nb, *canvas = (Uint16 *) dw->pixmap.canvas;

		union {

			Uint32          l;
			Uint8           b[4];
		}
		vcol;

		yspan = dw->pixmap.yspan;
		canvas += cb->min_y * yspan;

		for (y = cb->min_y; y <= cb->max_y; ++y) {

			for (x = cb->min_x; x <= cb->max_x; ++x) {

				nb = *(canvas + x);

				if (nb != 0) {

					palette[0] = *(pixels + x);

					ncol = (nb & 0x000FU) >> 0;
					vcol.l = palette[ncol];

					blend[0] = vcol.b[0];
					blend[1] = vcol.b[1];
					blend[2] = vcol.b[2];

					ncol = (nb & 0x00F0U) >> 4;
					vcol.l = palette[ncol];

					blend[0] += vcol.b[0];
					blend[1] += vcol.b[1];
					blend[2] += vcol.b[2];

					ncol = (nb & 0x0F00U) >> 8;
					vcol.l = palette[ncol];

					blend[0] += vcol.b[0];
					blend[1] += vcol.b[1];
					blend[2] += vcol.b[2];

					ncol = (nb & 0xF000U) >> 12;
					vcol.l = palette[ncol];

					blend[0] += vcol.b[0];
					blend[1] += vcol.b[1];
					blend[2] += vcol.b[2];

					vcol.b[0] = ltgamma[(blend[0] >> 2) & 0xFFU];
					vcol.b[1] = ltgamma[(blend[1] >> 2) & 0xFFU];
					vcol.b[2] = ltgamma[(blend[2] >> 2) & 0xFFU];
					vcol.b[3] = 0;

					*(pixels + x) = vcol.l;
				}
			}

			pixels += pitch;
			canvas += yspan;
		}
	}
	else if (dw->antialiasing == DRAW_8X_MSAA) {

		Uint16		nb[2], *canvas = (Uint16 *) dw->pixmap.canvas;

		union {

			Uint32          l;
			Uint8           b[4];
		}
		vcol;

		yspan = dw->pixmap.yspan * 2;
		canvas += cb->min_y * yspan;

		for (y = cb->min_y; y <= cb->max_y; ++y) {

			for (x = cb->min_x; x <= cb->max_x; ++x) {

				nb[0] = *(canvas + x * 2 + 0);
				nb[1] = *(canvas + x * 2 + 1);

				if (		   nb[0] != 0
						|| nb[1] != 0) {

					palette[0] = *(pixels + x);

					ncol = (nb[0] & 0x000FU) >> 0;
					vcol.l = palette[ncol];

					blend[0] = vcol.b[0];
					blend[1] = vcol.b[1];
					blend[2] = vcol.b[2];

					ncol = (nb[0] & 0x00F0U) >> 4;
					vcol.l = palette[ncol];

					blend[0] += vcol.b[0];
					blend[1] += vcol.b[1];
					blend[2] += vcol.b[2];

					ncol = (nb[0] & 0x0F00U) >> 8;
					vcol.l = palette[ncol];

					blend[0] += vcol.b[0];
					blend[1] += vcol.b[1];
					blend[2] += vcol.b[2];

					ncol = (nb[0] & 0xF000U) >> 12;
					vcol.l = palette[ncol];

					blend[0] += vcol.b[0];
					blend[1] += vcol.b[1];
					blend[2] += vcol.b[2];

					ncol = (nb[1] & 0x000FU) >> 0;
					vcol.l = palette[ncol];

					blend[0] += vcol.b[0];
					blend[1] += vcol.b[1];
					blend[2] += vcol.b[2];

					ncol = (nb[1] & 0x00F0U) >> 4;
					vcol.l = palette[ncol];

					blend[0] += vcol.b[0];
					blend[1] += vcol.b[1];
					blend[2] += vcol.b[2];

					ncol = (nb[1] & 0x0F00U) >> 8;
					vcol.l = palette[ncol];

					blend[0] += vcol.b[0];
					blend[1] += vcol.b[1];
					blend[2] += vcol.b[2];

					ncol = (nb[1] & 0xF000U) >> 12;
					vcol.l = palette[ncol];

					blend[0] += vcol.b[0];
					blend[1] += vcol.b[1];
					blend[2] += vcol.b[2];

					vcol.b[0] = ltgamma[(blend[0] >> 3) & 0xFFU];
					vcol.b[1] = ltgamma[(blend[1] >> 3) & 0xFFU];
					vcol.b[2] = ltgamma[(blend[2] >> 3) & 0xFFU];
					vcol.b[3] = 0;

					*(pixels + x) = vcol.l;
				}
			}

			pixels += pitch;
			canvas += yspan;
		}
	}
}

