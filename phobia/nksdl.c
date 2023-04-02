/*
 * MIT License
 *
 * Copyright (c) 2016-2017 Patrick Rudolph <siro@das-labor.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

/* Adapted from sdl2surface_rawfb.h for use in Phobia by Roman Belov.
 * */

#include <stdlib.h>
#include <string.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#define NK_MEMSET	memset
#define NK_MEMCPY	memcpy
#define NK_SQRT		sqrtf
#define NK_SIN		sinf
#define NK_COS		cosf

#include "nksdl.h"

#define NK_IMPLEMENTATION
#include "nuklear.h"

static const char *
nk_sdl_nullstr(const char *text, int len)
{
	static char		nbuf[1024];

	len = NK_MIN(len, sizeof(nbuf));

	memcpy(nbuf, text, len);

	nbuf[len] = 0;

	return (const char *) nbuf;
}

NK_API void nk_sdl_input_event(struct nk_sdl *nk, SDL_Event *ev)
{
	struct nk_vec2		wheel;
	int			key, button;

	switch (ev->type)
	{
		case SDL_QUIT:
			nk->onquit = 1;
			break;

		case SDL_WINDOWEVENT:
			if (ev->window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {

				nk->fb = SDL_GetWindowSurface(nk->window);

				if (		nk->fb->w != nk->surface->w
						|| nk->fb->h != nk->surface->h) {

					SDL_FreeSurface(nk->surface);

					nk->surface = SDL_CreateRGBSurfaceWithFormat(0, nk->fb->w,
							nk->fb->h, 32, SDL_PIXELFORMAT_XRGB8888);
				}
			}
			else if (ev->window.event == SDL_WINDOWEVENT_CLOSE) {

				nk->onquit = 1;
			}
			break;

		case SDL_TEXTINPUT:
			nk_input_glyph(&nk->ctx, ev->text.text);
			break;

		case SDL_KEYDOWN:
		case SDL_KEYUP:
			switch (ev->key.keysym.sym) {

				case SDLK_ESCAPE:
					key = NK_KEY_NONE;
					SDL_StopTextInput();
					break;

				case SDLK_LSHIFT:
				case SDLK_RSHIFT:
					key = NK_KEY_SHIFT;
					break;

				case SDLK_LCTRL:
				case SDLK_RCTRL:
					key = NK_KEY_CTRL;
					nk->keyctrl = (ev->type == SDL_KEYDOWN) ? 1 : 0;
					break;

				case SDLK_DELETE:
					key = NK_KEY_DEL;
					break;

				case SDLK_RETURN:
					key = NK_KEY_ENTER;
					break;

				case SDLK_TAB:
					key = NK_KEY_TAB;
					break;

				case SDLK_BACKSPACE:
					key = NK_KEY_BACKSPACE;
					break;

				case SDLK_a:
					key = (nk->keyctrl != 0) ? NK_KEY_TEXT_SELECT_ALL : NK_KEY_NONE;
					break;

				case SDLK_c:
					key = (nk->keyctrl != 0) ? NK_KEY_COPY : NK_KEY_NONE;
					break;

				case SDLK_x:
					key = (nk->keyctrl != 0) ? NK_KEY_CUT : NK_KEY_NONE;
					break;

				case SDLK_v:
					key = (nk->keyctrl != 0) ? NK_KEY_PASTE : NK_KEY_NONE;
					break;

				case SDLK_UP:
					key = NK_KEY_UP;
					break;

				case SDLK_DOWN:
					key = NK_KEY_DOWN;
					break;

				case SDLK_LEFT:
					key = NK_KEY_LEFT;
					break;

				case SDLK_RIGHT:
					key = NK_KEY_RIGHT;
					break;

				default:
					key = NK_KEY_NONE;
					break;
			}
			nk_input_key(&nk->ctx, key, (ev->type == SDL_KEYDOWN) ? nk_true : nk_false);
			break;

		case SDL_MOUSEMOTION:
			nk_input_motion(&nk->ctx, ev->motion.x, ev->motion.y);
			break;

		case SDL_MOUSEBUTTONDOWN:
		case SDL_MOUSEBUTTONUP:
			switch (ev->button.button) {

				case SDL_BUTTON_LEFT:
					SDL_StartTextInput();
					button = NK_BUTTON_LEFT;
					break;

				case SDL_BUTTON_MIDDLE:
					button = NK_BUTTON_MIDDLE;
					break;

				default:
				case SDL_BUTTON_RIGHT:
					button = NK_BUTTON_RIGHT;
					break;
			}
			nk_input_button(&nk->ctx, button, ev->button.x, ev->button.y,
					(ev->type == SDL_MOUSEBUTTONDOWN) ? nk_true : nk_false);
			break;

		case SDL_MOUSEWHEEL:
			wheel.x = ev->wheel.x;
			wheel.y = ev->wheel.y;
			nk_input_scroll(&nk->ctx, wheel);
			break;

		default:
			break;
	}
}

NK_API void nk_sdl_clipboard_paste(nk_handle userdata, struct nk_text_edit* edit)
{
	char				*clip;

	if (SDL_HasClipboardText() != 0) {

		clip = SDL_GetClipboardText();

		if (clip != NULL) {

			nk_textedit_paste(edit, clip, strlen(clip));

			SDL_free(clip);
		}
	}
}

NK_API void nk_sdl_clipboard_copy(nk_handle userdata, const char *text, int len)
{
	SDL_SetClipboardText(nk_sdl_nullstr(text, len));
}

NK_API void nk_sdl_style_custom(struct nk_sdl *nk, int padding)
{
	struct nk_context		*ctx = &nk->ctx;
	struct nk_style			*style;

	nk->table[NK_COLOR_TEXT] = nk_rgba(190, 210, 210, 255);
	nk->table[NK_COLOR_WINDOW] = nk_rgba(30, 30, 35, 255);
	nk->table[NK_COLOR_HEADER] = nk_rgba(60, 60, 65, 255);
	nk->table[NK_COLOR_BORDER] = nk_rgba(10, 10, 15, 255);
	nk->table[NK_COLOR_BUTTON] = nk_rgba(35, 65, 95, 255);
	nk->table[NK_COLOR_BUTTON_HOVER] = nk_rgba(65, 95, 125, 255);
	nk->table[NK_COLOR_BUTTON_ACTIVE] = nk_rgba(40, 40, 45, 255);
	nk->table[NK_COLOR_TOGGLE] = nk_rgba(50, 50, 55, 255);
	nk->table[NK_COLOR_TOGGLE_HOVER] = nk_rgba(60, 60, 65, 255);
	nk->table[NK_COLOR_TOGGLE_CURSOR] = nk_rgba(210, 120, 60, 255);
	nk->table[NK_COLOR_SELECT] = nk_rgba(50, 50, 55, 255);
	nk->table[NK_COLOR_SELECT_ACTIVE] = nk_rgba(45, 125, 55, 255);
	nk->table[NK_COLOR_SLIDER] = nk_rgba(50, 55, 65, 255);
	nk->table[NK_COLOR_SLIDER_CURSOR] = nk_rgba(45, 85, 115, 245);
	nk->table[NK_COLOR_SLIDER_CURSOR_HOVER] = nk_rgba(55, 85, 115, 255);
	nk->table[NK_COLOR_SLIDER_CURSOR_ACTIVE] = nk_rgba(55, 95, 125, 255);
	nk->table[NK_COLOR_PROPERTY] = nk_rgba(50, 55, 65, 255);
	nk->table[NK_COLOR_EDIT] = nk_rgba(50, 50, 55, 225);
	nk->table[NK_COLOR_EDIT_CURSOR] = nk_rgba(190, 210, 210, 255);
	nk->table[NK_COLOR_COMBO] = nk_rgba(50, 50, 55, 255);
	nk->table[NK_COLOR_CHART] = nk_rgba(50, 55, 65, 255);
	nk->table[NK_COLOR_CHART_COLOR] = nk_rgba(45, 85, 115, 255);
	nk->table[NK_COLOR_CHART_COLOR_HIGHLIGHT] = nk_rgba(210, 0, 0, 255);
	nk->table[NK_COLOR_SCROLLBAR] = nk_rgba(50, 50, 55, 255);
	nk->table[NK_COLOR_SCROLLBAR_CURSOR] = nk_rgba(35, 65, 95, 255);
	nk->table[NK_COLOR_SCROLLBAR_CURSOR_HOVER] = nk_rgba(45, 75, 105, 255);
	nk->table[NK_COLOR_SCROLLBAR_CURSOR_ACTIVE] = nk_rgba(65, 95, 125, 255);
	nk->table[NK_COLOR_TAB_HEADER] = nk_rgba(45, 85, 115, 255);

	nk->table[NK_COLOR_HIDDEN] = nk_rgba(50, 50, 55, 255);
	nk->table[NK_COLOR_CONFIG] = nk_rgba(190, 210, 120, 255);
	nk->table[NK_COLOR_FLICKER_LIGHT] = nk_rgba(75, 75, 85, 255);
	nk->table[NK_COLOR_FLICKER_ALERT] = nk_rgba(175, 45, 55, 255);
	nk->table[NK_COLOR_ENABLED] = nk_rgba(45, 155, 55, 255);
	nk->table[NK_COLOR_ORANGE_BUTTON] = nk_rgba(155, 65, 35, 255);
	nk->table[NK_COLOR_ORANGE_BUTTON_HOVER] = nk_rgba(185, 95, 65, 255);
	nk->table[NK_COLOR_EDIT_NUMBER] = nk_rgba(210, 110, 55, 255);
	nk->table[NK_COLOR_COMBO_HOVER] = nk_rgba(60, 60, 65, 255);
	nk->table[NK_COLOR_ACTIVE_HOVER] = nk_rgba(55, 135, 65, 255);
	nk->table[NK_COLOR_DRAWING_PEN] = nk_rgba(90, 90, 95, 255);

	nk_style_from_table(ctx, nk->table);

	style = &ctx->style;

	style->button.text_active = nk->table[NK_COLOR_EDIT_NUMBER];
	style->button.padding = nk_vec2(padding, padding);
	style->button.rounding = 4.0f;

	style->selectable.hover = nk_style_item_color(nk->table[NK_COLOR_COMBO_HOVER]);
	style->selectable.hover_active = nk_style_item_color(nk->table[NK_COLOR_ACTIVE_HOVER]);
	style->selectable.padding = nk_vec2(padding, padding);
	style->selectable.rounding = 2.0f;

	style->checkbox.padding = nk_vec2(4.0f, 4.0f);
	style->checkbox.spacing = padding;

	style->combo.hover = nk_style_item_color(nk->table[NK_COLOR_COMBO_HOVER]);
	style->combo.button.hover = style->combo.hover;
	style->combo.content_padding = nk_vec2(padding, padding);
	style->combo.button_padding = nk_vec2(0.0f, 8.0f + padding / 5);
	style->combo.rounding = 2.0f;

	style->edit.normal = nk_style_item_color(nk->table[NK_COLOR_EDIT]);
	style->edit.hover = nk_style_item_color(nk->table[NK_COLOR_EDIT]);
	style->edit.active = nk_style_item_color(nk->table[NK_COLOR_EDIT]);
	style->edit.cursor_normal = nk->table[NK_COLOR_EDIT_CURSOR];
	style->edit.cursor_hover = nk->table[NK_COLOR_EDIT_CURSOR];
	style->edit.cursor_text_normal = nk->table[NK_COLOR_EDIT];
	style->edit.cursor_text_hover = nk->table[NK_COLOR_EDIT];
	style->edit.text_normal = nk->table[NK_COLOR_EDIT_NUMBER];
	style->edit.text_hover = nk->table[NK_COLOR_EDIT_NUMBER];
	style->edit.text_active = nk->table[NK_COLOR_EDIT_NUMBER];
	style->edit.selected_normal = nk->table[NK_COLOR_EDIT_NUMBER];
	style->edit.selected_hover = nk->table[NK_COLOR_EDIT_NUMBER];
	style->edit.selected_text_normal = nk->table[NK_COLOR_EDIT];
	style->edit.selected_text_hover = nk->table[NK_COLOR_EDIT];
	style->edit.padding = nk_vec2(padding, padding);
	style->edit.rounding = 2.0f;

	ctx->clip.paste = &nk_sdl_clipboard_paste;
	ctx->clip.copy = &nk_sdl_clipboard_copy;

	nk_style_hide_cursor(ctx);
}

NK_API float nk_sdl_text_width(nk_handle font, float height, const char *text, int len)
{
	TTF_Font	*ttf_font = (TTF_Font *) font.ptr;
	int		text_w = 0, text_h = 0;

	TTF_SizeUTF8(ttf_font, nk_sdl_nullstr(text, len), &text_w, &text_h);

	return (float) text_w;
}

static Uint32
nk_sdl_color_packed(const struct nk_color col)
{
	Uint32		q;

	q  = (Uint32) col.a << 24;
	q |= (Uint32) col.r << 16;
	q |= (Uint32) col.g << 8;
	q |= (Uint32) col.b;

	return q;
}

static void
nk_sdl_scissor(struct nk_sdl *nk, const struct nk_command_scissor *s)
{
	nk->scissor.x = NK_MIN(NK_MAX(s->x, 0), nk->surface->w);
	nk->scissor.y = NK_MIN(NK_MAX(s->y, 0), nk->surface->h);
	nk->scissor.w = NK_MIN(NK_MAX(s->w + s->x + 1, 0), nk->surface->w);
	nk->scissor.h = NK_MIN(NK_MAX(s->h + s->y + 1, 0), nk->surface->h);
}

static void
nk_sdl_line_horizontal(struct nk_sdl *nk, int x0, int y0, int x1, Uint32 qcol)
{
	Uint32			*pixels = nk->surface->pixels;

	if (y0 >= nk->scissor.h || y0 < nk->scissor.y)
		return ;

	if (x1 < x0) {

		int	temp;

		temp = x1;
		x1 = x0;
		x0 = temp;
	}

	if (x0 >= nk->scissor.w || x1 < nk->scissor.x)
		return ;

	x0 = NK_MAX(nk->scissor.x, x0);
	x1 = NK_MIN(nk->scissor.w - 1, x1);

	pixels += x0 + y0 * (nk->surface->pitch / 4);

	while (x1 - x0 >= 7) {

		*pixels++ = qcol;
		*pixels++ = qcol;
		*pixels++ = qcol;
		*pixels++ = qcol;
		*pixels++ = qcol;
		*pixels++ = qcol;
		*pixels++ = qcol;
		*pixels++ = qcol;

		x0 += 8;
	}

	while (x1 >= x0) {

		*pixels++ = qcol;

		x0 += 1;
	}
}

static void
nk_sdl_setpixel(struct nk_sdl *nk, int x0, int y0, Uint32 qcol)
{
	Uint32			*pixels = nk->surface->pixels;

	if (		y0 >= nk->scissor.y && y0 < nk->scissor.h
			&& x0 >= nk->scissor.x && x0 < nk->scissor.w) {

		*(pixels + x0 + y0 * (nk->surface->pitch / 4)) = qcol;
	}
}

static void
nk_sdl_stroke_line(struct nk_sdl *nk, int x0, int y0, int x1, int y1,
		int thickness, const struct nk_color col)
{
	int			dy, dx, stepx, stepy;
	Uint32			qcol;

	qcol = nk_sdl_color_packed(col);

	dy = y1 - y0;
	dx = x1 - x0;

	if (dy == 0) {

		nk_sdl_line_horizontal(nk, x0, y0, x1, qcol);

		return ;
	}

	if (dy < 0) {

		dy = - dy;
		stepy = -1;
	}
	else {
		stepy = 1;
	}

	if (dx < 0) {

		dx = - dx;
		stepx = -1;
	}
	else {
		stepx = 1;
	}

	dy <<= 1;
	dx <<= 1;

	nk_sdl_setpixel(nk, x0, y0, qcol);

	if (dx > dy) {

		int	fraction = dy - (dx >> 1);

		while (x0 != x1) {

			if (fraction >= 0) {

				y0 += stepy;
				fraction -= dx;
			}

			x0 += stepx;
			fraction += dy;

			nk_sdl_setpixel(nk, x0, y0, qcol);
		}
	}
	else {
		int	fraction = dx - (dy >> 1);

		while (y0 != y1) {

			if (fraction >= 0) {

				x0 += stepx;
				fraction -= dy;
			}

			y0 += stepy;
			fraction += dx;

			nk_sdl_setpixel(nk, x0, y0, qcol);
		}
	}
}

static void
nk_sdl_fill_polygon(struct nk_sdl *nk, const struct nk_vec2i *pnts,
		int count, const struct nk_color col)
{
	int			left, top, bottom, right, swap, i, j;
	int			nodes, nodeX[80], pixelY;
	Uint32			qcol;

	if (count == 0)
		return ;

	count = NK_MIN(count, NK_LEN(nodeX));

	qcol = nk_sdl_color_packed(col);

	left = pnts[0].x;
	right = pnts[0].x;
	top = pnts[0].y;
	bottom = pnts[0].y;

	for (i = 1; i < count; i++) {

		left = NK_MIN(left, pnts[i].x);
		right = NK_MAX(right, pnts[i].x);
		top = NK_MIN(top, pnts[i].y);
		bottom = NK_MAX(bottom, pnts[i].y);
	}

	bottom++;
	right++;

	for (pixelY = top; pixelY < bottom; pixelY++) {

		nodes = 0;
		j = count - 1;

		for (i = 0; i < count; i++) {

			if (		((pnts[i].y < pixelY) && (pnts[j].y >= pixelY))
					|| ((pnts[j].y < pixelY) && (pnts[i].y >= pixelY))) {

				nodeX[nodes++] = (int) ((float) pnts[i].x
						+ ((float) pixelY - (float) pnts[i].y)
						/ ((float) pnts[j].y - (float) pnts[i].y)
						* ((float) pnts[j].x - (float) pnts[i].x));
			}

			j = i;
		}

		i = 0;

		while (i < nodes - 1) {

			if (nodeX[i] > nodeX[i+1]) {

				swap = nodeX[i];
				nodeX[i] = nodeX[i+1];
				nodeX[i+1] = swap;

				if (i) i--;
			}
			else i++;
		}

		for (i = 0; i < nodes; i += 2) {

			if (nodeX[i+0] >= right) break;
			if (nodeX[i+1] > left) {

				if (nodeX[i+0] < left) nodeX[i+0] = left;
				if (nodeX[i+1] > right) nodeX[i+1] = right;

				nk_sdl_line_horizontal(nk, nodeX[i], pixelY,
						nodeX[i+1], qcol);
			}
		}
	}
}

static void
nk_sdl_line(struct nk_sdl *nk, const struct nk_command_line *l)
{
	nk_sdl_stroke_line(nk, l->begin.x, l->begin.y, l->end.x,
			l->end.y, l->line_thickness, l->color);
}

static void
nk_sdl_curve(struct nk_sdl *nk, const struct nk_command_curve *c)
{
	struct nk_vec2i		last = c->begin;
	float			t_step;
	int			i_step, segments;

	segments = 24;
	t_step = 1.f / (float) segments;

	for (i_step = 1; i_step <= segments; ++i_step) {

		float	t = t_step * (float) i_step;
		float	u = 1.f - t;
		float	w1 = u*u*u;
		float	w2 = 3.f*u*u*t;
		float	w3 = 3.f*u*t*t;
		float	w4 = t*t*t;

		float	x = w1 * c->begin.x + w2 * c->ctrl[0].x
				+ w3 * c->ctrl[1].x + w4 * c->end.x;

		float	y = w1 * c->begin.y + w2 * c->ctrl[0].y
				+ w3 * c->ctrl[1].y + w4 * c->end.y;

		nk_sdl_stroke_line(nk, last.x, last.y, (int) x, (int) y,
				c->line_thickness, c->color);

		last.x = (int) x;
		last.y = (int) y;
	}
}

static void
nk_sdl_rect(struct nk_sdl *nk, const struct nk_command_rect *r)
{
	int		x, y, w, h, b, tc;

	x = r->x;
	y = r->y;
	w = r->w;
	h = r->h;

	b = r->rounding;
	tc = r->line_thickness;

	nk_sdl_stroke_line(nk, x + b, y, x + w - b, y, tc, r->color);
	nk_sdl_stroke_line(nk, x + b, y + h, x + w - b, y + h, tc, r->color);
	nk_sdl_stroke_line(nk, x, y + b, x, y + h - b, tc, r->color);
	nk_sdl_stroke_line(nk, x + w, y + b, x + w, y + h - b, tc, r->color);

	nk_sdl_stroke_line(nk, x + b, y, x, y + b, tc, r->color);
	nk_sdl_stroke_line(nk, x + w - b, y, x + w, y + b, tc, r->color);
	nk_sdl_stroke_line(nk, x, y + h - b, x + b, y + h, tc, r->color);
	nk_sdl_stroke_line(nk, x + w - b, y + h, x + w, y + h - b, tc, r->color);
}

static void
nk_sdl_rect_filled(struct nk_sdl *nk, const struct nk_command_rect_filled *r)
{
	int		x, y, w, h, i, b;

	Uint32 		qcol;

	x = r->x;
	y = r->y;
	w = r->w;
	h = r->h;

	qcol = nk_sdl_color_packed(r->color);

	for (i = 0; i < r->rounding; i++) {

		b = r->rounding - i;

		nk_sdl_line_horizontal(nk, x + b, y + i, x + w - b, qcol);
	}

	for (i = r->rounding; i < h - r->rounding; i++) {

		nk_sdl_line_horizontal(nk, x, y + i, x + w, qcol);
	}

	for (i = h - r->rounding; i <= h; i++) {

		b = i - (h - r->rounding);

		nk_sdl_line_horizontal(nk, x + b, y + i, x + w - b, qcol);
	}
}

static void
nk_sdl_rect_multi_color(struct nk_sdl *nk, const struct nk_command_rect_multi_color *r)
{
	/* TODO */
}

static void
nk_sdl_circle(struct nk_sdl *nk, const struct nk_command_circle *c)
{
	int		x0, y0, w, h, x, y, sigma;
	int		a2, b2, fa2, fb2;

	Uint32 		qcol;

	qcol = nk_sdl_color_packed(c->color);

	x0 = c->x;
	y0 = c->y;
	w = c->w;
	h = c->h;

	a2 = (w * w) / 4;
	b2 = (h * h) / 4;
	fa2 = 4 * a2;
	fb2 = 4 * b2;

	h = (h + 1) / 2;
	w = (w + 1) / 2;

	x0 += w;
	y0 += h;

	for (x = 0, y = h, sigma = 2*b2+a2*(1-2*h); b2*x <= a2*y; x++) {

		nk_sdl_setpixel(nk, x0 + x, y0 + y, qcol);
		nk_sdl_setpixel(nk, x0 - x, y0 + y, qcol);
		nk_sdl_setpixel(nk, x0 + x, y0 - y, qcol);
		nk_sdl_setpixel(nk, x0 - x, y0 - y, qcol);

		if (sigma >= 0) {

			sigma += fa2 * (1 - y);
			y--;
		}

		sigma += b2 * ((4 * x) + 6);
	}

	for (x = w, y = 0, sigma = 2*a2+b2*(1-2*w); a2*y <= b2*x; y++) {

		nk_sdl_setpixel(nk, x0 + x, y0 + y, qcol);
		nk_sdl_setpixel(nk, x0 - x, y0 + y, qcol);
		nk_sdl_setpixel(nk, x0 + x, y0 - y, qcol);
		nk_sdl_setpixel(nk, x0 - x, y0 - y, qcol);

		if (sigma >= 0) {

			sigma += fb2 * (1 - x);
			x--;
		}

		sigma += a2 * ((4 * y) + 6);
	}
}

static void
nk_sdl_circle_filled(struct nk_sdl *nk, const struct nk_command_circle_filled *c)
{
	int		x0, y0, w, h, x, y, sigma;
	int		a2, b2, fa2, fb2;
	Uint32 		qcol;

	qcol = nk_sdl_color_packed(c->color);

	x0 = c->x;
	y0 = c->y;
	w = c->w;
	h = c->h;

	a2 = (w * w) / 4;
	b2 = (h * h) / 4;
	fa2 = 4 * a2;
	fb2 = 4 * b2;

	h = (h + 1) / 2;
	w = (w + 1) / 2;

	x0 += w;
	y0 += h;

	for (x = 0, y = h, sigma = 2*b2+a2*(1-2*h); b2*x <= a2*y; x++) {

		nk_sdl_line_horizontal(nk, x0 - x, y0 + y, x0 + x, qcol);
		nk_sdl_line_horizontal(nk, x0 - x, y0 - y, x0 + x, qcol);

		if (sigma >= 0) {

			sigma += fa2 * (1 - y);
			y--;
		}

		sigma += b2 * ((4 * x) + 6);
	}

	for (x = w, y = 0, sigma = 2*a2+b2*(1-2*w); a2*y <= b2*x; y++) {

		nk_sdl_line_horizontal(nk, x0 - x, y0 + y, x0 + x, qcol);
		nk_sdl_line_horizontal(nk, x0 - x, y0 - y, x0 + x, qcol);

		if (sigma >= 0) {

			sigma += fb2 * (1 - x);
			x--;
		}

		sigma += a2 * ((4 * y) + 6);
	}
}

static void
nk_sdl_arc(struct nk_sdl *nk, const struct nk_command_arc *a)
{
	/* TODO */
}

static void
nk_sdl_arc_filled(struct nk_sdl *nk, const struct nk_command_arc_filled *a)
{
	/* TODO */
}

static void
nk_sdl_triangle(struct nk_sdl *nk, const struct nk_command_triangle *t)
{
	int		tc = t->line_thickness;

	nk_sdl_stroke_line(nk, t->a.x, t->a.y, t->b.x, t->b.y, tc, t->color);
	nk_sdl_stroke_line(nk, t->b.x, t->b.y, t->c.x, t->c.y, tc, t->color);
	nk_sdl_stroke_line(nk, t->c.x, t->c.y, t->a.x, t->a.y, tc, t->color);
}

static void
nk_sdl_triangle_filled(struct nk_sdl *nk, const struct nk_command_triangle_filled *t)
{
	struct nk_vec2i		pnts[3];

	pnts[0].x = t->a.x;
	pnts[0].y = t->a.y;
	pnts[1].x = t->b.x;
	pnts[1].y = t->b.y;
	pnts[2].x = t->c.x;
	pnts[2].y = t->c.y;

	nk_sdl_fill_polygon(nk, pnts, 3, t->color);
}

static void
nk_sdl_polygon(struct nk_sdl *nk, const struct nk_command_polygon *p)
{
	int		i;

	for (i = 1; i < p->point_count; ++i) {

		nk_sdl_stroke_line(nk, p->points[i-1].x, p->points[i-1].y, p->points[i].x,
				p->points[i].y, p->line_thickness, p->color);
	}

	nk_sdl_stroke_line(nk, p->points[p->point_count-1].x, p->points[p->point_count-1].y,
			p->points[0].x, p->points[0].y, p->line_thickness, p->color);
}

static void
nk_sdl_polygon_filled(struct nk_sdl *nk, const struct nk_command_polygon_filled *p)
{
	nk_sdl_fill_polygon(nk, p->points, p->point_count, p->color);
}

static void
nk_sdl_polyline(struct nk_sdl *nk, const struct nk_command_polyline *l)
{
	int		i;

	for (i = 0; i < l->point_count - 1; ++i) {

		nk_sdl_stroke_line(nk, l->points[i].x, l->points[i].y,
				l->points[i+1].x, l->points[i+1].y,
				l->line_thickness, l->color);
	}
}

static void
nk_sdl_text(struct nk_sdl *nk, const struct nk_command_text *t)
{
	TTF_Font	*ttf_font = (TTF_Font *) t->font->userdata.ptr;
	SDL_Surface	*text_surface;
	SDL_Rect	clip_rect, text_rect;
	SDL_Color	fg;

	fg.r = t->foreground.r;
	fg.g = t->foreground.g;
	fg.b = t->foreground.b;
	fg.a = t->foreground.a;

	text_surface = TTF_RenderUTF8_Blended(ttf_font,
			nk_sdl_nullstr(t->string, t->length), fg);

	if (text_surface != NULL) {

		clip_rect.x = nk->scissor.x;
		clip_rect.y = nk->scissor.y;
		clip_rect.w = nk->scissor.w - nk->scissor.x;
		clip_rect.h = nk->scissor.h - nk->scissor.y;

		text_rect.x = t->x;
		text_rect.y = t->y;
		text_rect.w = t->w;
		text_rect.h = t->h;

		SDL_SetClipRect(nk->surface, &clip_rect);

		SDL_BlitSurface(text_surface, NULL, nk->surface, &text_rect);
		SDL_FreeSurface(text_surface);

		SDL_SetClipRect(nk->surface, NULL);
	}
}

static void
nk_sdl_image(struct nk_sdl *nk, const struct nk_command_image *i)
{
	/* TODO */
}

NK_API void nk_sdl_render(struct nk_sdl *nk)
{
	const struct nk_command		*cmd;

	nk->scissor.x = 0;
	nk->scissor.y = 0;
	nk->scissor.w = nk->surface->w;
	nk->scissor.h = nk->surface->h;

	nk_foreach(cmd, (struct nk_context *) &nk->ctx) {

		switch (cmd->type) {

			case NK_COMMAND_NOP:
				break;

			case NK_COMMAND_SCISSOR:
				nk_sdl_scissor(nk, (void *) cmd);
				break;

			case NK_COMMAND_LINE:
				nk_sdl_line(nk, (void *) cmd);
				break;

			case NK_COMMAND_CURVE:
				nk_sdl_curve(nk, (void *) cmd);
				break;

			case NK_COMMAND_RECT:
				nk_sdl_rect(nk, (void *) cmd);
				break;

			case NK_COMMAND_RECT_FILLED:
				nk_sdl_rect_filled(nk, (void *) cmd);
				break;

			case NK_COMMAND_RECT_MULTI_COLOR:
				nk_sdl_rect_multi_color(nk, (void *) cmd);
				break;

			case NK_COMMAND_CIRCLE:
				nk_sdl_circle(nk, (void *) cmd);
				break;

			case NK_COMMAND_CIRCLE_FILLED:
				nk_sdl_circle_filled(nk, (void *) cmd);
				break;

			case NK_COMMAND_ARC:
				nk_sdl_arc(nk, (void *) cmd);
				break;

			case NK_COMMAND_ARC_FILLED:
				nk_sdl_arc_filled(nk, (void *) cmd);
				break;

			case NK_COMMAND_TRIANGLE:
				nk_sdl_triangle(nk, (void *) cmd);
				break;

			case NK_COMMAND_TRIANGLE_FILLED:
				nk_sdl_triangle_filled(nk, (void *) cmd);
				break;

			case NK_COMMAND_POLYGON:
				nk_sdl_polygon(nk, (void *) cmd);
				break;

			case NK_COMMAND_POLYGON_FILLED:
				nk_sdl_polygon_filled(nk, (void *) cmd);
				break;

			case NK_COMMAND_POLYLINE:
				nk_sdl_polyline(nk, (void *) cmd);
				break;

			case NK_COMMAND_TEXT:
				nk_sdl_text(nk, (void *) cmd);
				break;

			case NK_COMMAND_IMAGE:
				nk_sdl_image(nk, (void *) cmd);
				break;

			case NK_COMMAND_CUSTOM:
				break;
		}
	}

	nk_clear((struct nk_context*) &nk->ctx);
}

