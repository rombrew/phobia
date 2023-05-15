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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "menu.h"
#include "draw.h"
#include "scheme.h"

const char *utf8_go_next(const char *s);
char *utf8_backspace(char *s);

static int
menuFuzzyMatch(menu_t *mu, const char *s, const char *fu, int light[])
{
	const char	*b, *fue;
	char		ubuf[8], sbuf[MENU_STRING_MAX + 1];
	int		bX, bY, rc, lk = 0;

	sbuf[0] = 0;

	b = s;
	s = strstr(s, fu);

	if (s == NULL) {

		s = b;

		while (*fu != 0) {

			ubuf[0] = 0;

			fue = utf8_go_next(fu);
			strncat(ubuf, fu, fue - fu);
			fu = fue;

			b = s;
			s = strstr(s, ubuf);

			if (s == NULL)
				break;
			else {
				if (light != NULL) {

					strncat(sbuf, b, s - b);
					TTF_SizeUTF8(mu->font, sbuf, &bX, &bY);
					light[lk++] = bX;

					b = s;
					s = utf8_go_next(s);

					strncat(sbuf, b, s - b);
					TTF_SizeUTF8(mu->font, sbuf, &bX, &bY);
					light[lk++] = bX;
				}
				else {
					s = utf8_go_next(s);
				}
			}
		}
	}
	else {
		if (light != NULL) {

			while (*fu != 0) {

				strncat(sbuf, b, s - b);
				TTF_SizeUTF8(mu->font, sbuf, &bX, &bY);
				light[lk++] = bX;

				b = s;
				s = utf8_go_next(s);

				strncat(sbuf, b, s - b);
				TTF_SizeUTF8(mu->font, sbuf, &bX, &bY);
				light[lk++] = bX;

				b = s;
				fu = utf8_go_next(fu);
			}
		}
	}

	rc = (s != NULL) ? 1 : 0;

	return rc;
}

static void
menuBuild(menu_t *mu, int rep)
{
	int			bX, bY, size_X, size_Y;
	const char		*s;

	s = mu->list;

	size_X = 0;
	mu->size_N = 0;

	while (*s != 0) {

		TTF_SizeUTF8(mu->font, s, &bX, &bY);

		if (menuFuzzyMatch(mu, s, mu->fuzzy, NULL))
			mu->size_N++;

		size_X = (size_X < bX) ? bX : size_X;

		while (*s != 0) ++s;
		++s;
	}

	mu->size_X = size_X + mu->layout_height + mu->layout_height / 4;
	mu->size_Y = mu->size_N * mu->layout_height;
	size_Y = mu->screen.max_y / mu->layout_height;
	size_Y = (mu->fuzzy[0] != 0) ? size_Y - 1 : size_Y;

	if (size_Y < mu->size_N) {

		mu->scroll_limit = mu->size_N - size_Y;
		mu->scroll_shift = (rep == 0) ? 0 :
			(mu->scroll_shift > mu->scroll_limit)
			? mu->scroll_limit : mu->scroll_shift;

		mu->size_Y -= mu->scroll_limit * mu->layout_height;
	}
	else {
		mu->scroll_limit = 0;
		mu->scroll_shift = 0;
	}

	mu->scroll_page = size_Y - 1;

	if (mu->box_X < 0) {

		mu->box_X = (mu->screen.min_x + mu->screen.max_x - mu->size_X) / 2;
	}

	if (mu->box_Y < 0) {

		mu->box_Y = (mu->screen.min_y + mu->screen.max_y - mu->size_Y) / 2;
	}

	mu->box_X = (mu->box_X > mu->screen.max_x - mu->size_X)
		? mu->screen.max_x - mu->size_X : mu->box_X;
	mu->box_Y = (mu->box_Y > mu->screen.max_y - mu->size_Y)
		? mu->screen.max_y - mu->size_Y : mu->box_Y;

	mu->box_X = (mu->box_X < mu->screen.min_x) ? mu->screen.min_x : mu->box_X;
	mu->box_Y = (mu->box_Y < mu->screen.min_y) ? mu->screen.min_y : mu->box_Y;

	if (rep != 0) {

		mu->hovered_N = (mu->hovered_N < 0) ? 0 : mu->hovered_N;
		mu->hovered_N = (mu->hovered_N > mu->size_N - 1)
			? mu->size_N - 1 : mu->hovered_N;
	}
}

menu_t *menuAlloc(draw_t *dw, scheme_t *sch)
{
	menu_t			*mu;

	mu = calloc(1, sizeof(menu_t));

	mu->dw = dw;
	mu->sch = sch;

	return mu;
}

void menuClean(menu_t *mu)
{
	free(mu);
}

void menuLayout(menu_t *mu)
{
	if (mu->raised != 0) {

		menuBuild(mu, 1);
	}
}

void menuRaise(menu_t *mu, int id, const char *list, int sx, int sy)
{
	int		hN;

	if (mu->raised != 0)
		return ;

	mu->raised = 1;
	mu->clicked = 0;

	mu->id = id;
	mu->list = list;

	for (hN = 0; hN < MENU_OPTION_MAX; ++hN) {

		mu->hidden_N[hN] = -1;
		mu->mark[hN].N = -1;
	}

	mu->colorful = 0;

	mu->cur_X = sx;
	mu->cur_Y = sy;

	mu->box_X = mu->cur_X;
	mu->box_Y = mu->cur_Y;

	mu->scroll_drag = 0;

	mu->hovered_N = -1;
	mu->clicked_N = -1;

	mu->fuzzy[0] = 0;

	menuBuild(mu, 0);
}

void menuHalt(menu_t *mu)
{
	mu->raised = 0;
}

void menuResume(menu_t *mu)
{
	if (mu->raised != 0)
		return ;

	mu->raised = 1;
	mu->clicked = 0;

	mu->clicked_N = -1;
}

void menuSelect(menu_t *mu, int item_N)
{
	int			N;

	mu->hovered_N = -1;

	for (N = 0; N < item_N + 1; ++N) {

		mu->hovered_N += +1;
		mu->hovered_N = (mu->hovered_N > mu->size_N - 1)
			? mu->size_N - 1 : mu->hovered_N;

		if (mu->hovered_N > (mu->size_N - 1)
				+ (mu->scroll_shift - mu->scroll_limit)) {

			mu->scroll_shift += +1;
			mu->scroll_shift = (mu->scroll_shift > mu->scroll_limit)
				? mu->scroll_limit : mu->scroll_shift;
		}
	}
}

static int
menuItemHover(menu_t *mu)
{
	const char		*s = mu->list;
	int			N = 0, N_visible, rN = -1;
	int			topY, botY, baseX;

	baseX = (mu->scroll_limit != 0) ? mu->size_X - mu->layout_height : mu->size_X;
	N_visible = mu->size_Y / mu->layout_height;

	while (*s != 0) {

		if (menuFuzzyMatch(mu, s, mu->fuzzy, NULL)) {

			topY = mu->box_Y + N * mu->layout_height;
			botY = topY + mu->layout_height;

			if (N > N_visible - 1) {

				break;
			}

			if (mu->cur_X > mu->box_X && mu->cur_X < mu->box_X + baseX
					&& mu->cur_Y > topY && mu->cur_Y < botY) {

				rN = N + mu->scroll_shift;
				break;
			}

			N++;
		}

		while (*s != 0) ++s;
		++s;
	}

	return rN;
}

static int
menuConvFuzzy(menu_t *mu, int fuzzy_N)
{
	const char		*s = mu->list;
	int			N = 0, K = 0, rN = -1;

	if (fuzzy_N < 0) {

	}
	else if (mu->fuzzy[0] != 0) {

		while (*s != 0) {

			if (menuFuzzyMatch(mu, s, mu->fuzzy, NULL)) {

				if (K == fuzzy_N) {

					rN = N;
				}

				K++;
			}

			N++;

			while (*s != 0) ++s;
			++s;
		}
	}
	else {
		rN = fuzzy_N;
	}

	return rN;
}

static void
menuScrollClick(menu_t *mu)
{
	int			topY, baseX, baseY;

	if (mu->scroll_limit != 0 && mu->clicked_N == -1) {

		baseX = mu->box_X + mu->size_X - mu->layout_height;
		baseY = mu->box_Y;

		if ((mu->cur_X > baseX && mu->cur_X < baseX + mu->layout_height)
				|| mu->scroll_drag != 0) {

			if (mu->cur_Y > baseY && mu->cur_Y < baseY + mu->size_Y) {

				topY = mu->cur_Y - mu->layout_height / 2 - mu->box_Y;
				baseY = mu->size_Y - mu->layout_height;
				mu->scroll_shift = (topY * mu->scroll_limit + baseY / 2) / baseY;
				mu->scroll_shift = (mu->scroll_shift < 0) ? 0 : mu->scroll_shift;
				mu->scroll_shift = (mu->scroll_shift > mu->scroll_limit)
					? mu->scroll_limit : mu->scroll_shift;

				mu->scroll_drag = 1;
				mu->clicked = 0;
			}
		}
	}
}

void menuEvent(menu_t *mu, int evno, int ex, int ey)
{
	int		hN;

	if (mu->raised == 0)
		return ;

	mu->cur_X = ex;
	mu->cur_Y = ey;

	if (evno == MENU_EVNO_CLICK) {

		mu->hovered_N = menuItemHover(mu);

		mu->clicked = 1;
		mu->clicked_N = menuConvFuzzy(mu, mu->hovered_N);

		for (hN = 0; hN < MENU_OPTION_MAX; ++hN) {

			mu->clicked_N = (mu->clicked_N == mu->hidden_N[hN])
				? -1 : mu->clicked_N;
		}

		menuScrollClick(mu);
	}
	else if (evno == MENU_EVNO_UNCLICK) {

		mu->scroll_drag = 0;
	}
	else if (evno == MENU_EVNO_MOTION) {

		if (mu->scroll_drag != 0) {

			menuScrollClick(mu);
		}
		else {
			mu->hovered_N = menuItemHover(mu);
		}
	}
	else if (evno == MENU_EVNO_SCROLL_UP) {

		mu->scroll_shift += -1;
		mu->scroll_shift = (mu->scroll_shift < 0) ? 0 : mu->scroll_shift;

		mu->hovered_N = menuItemHover(mu);
	}
	else if (evno == MENU_EVNO_SCROLL_DOWN) {

		mu->scroll_shift += +1;
		mu->scroll_shift = (mu->scroll_shift > mu->scroll_limit)
			? mu->scroll_limit : mu->scroll_shift;

		mu->hovered_N = menuItemHover(mu);
	}
	else if (evno == MENU_EVNO_BACKSPACE) {

		utf8_backspace(mu->fuzzy);
		menuBuild(mu, 1);
	}
	else if (evno == MENU_EVNO_ARROW_UP) {

		mu->hovered_N += -1;
		mu->hovered_N = (mu->hovered_N < 0) ? 0 : mu->hovered_N;
		mu->hovered_N = (mu->hovered_N > mu->size_N - 1)
			? mu->size_N - 1 : mu->hovered_N;

		if (mu->hovered_N < mu->scroll_shift) {

			mu->scroll_shift += -1;
			mu->scroll_shift = (mu->scroll_shift < 0) ? 0 : mu->scroll_shift;
		}
	}
	else if (evno == MENU_EVNO_ARROW_DOWN) {

		mu->hovered_N += +1;
		mu->hovered_N = (mu->hovered_N > mu->size_N - 1)
			? mu->size_N - 1 : mu->hovered_N;

		if (mu->hovered_N > (mu->size_N - 1) + (mu->scroll_shift - mu->scroll_limit)) {

			mu->scroll_shift += +1;
			mu->scroll_shift = (mu->scroll_shift > mu->scroll_limit)
				? mu->scroll_limit : mu->scroll_shift;
		}
	}
	else if (evno == MENU_EVNO_PAGE_UP) {

		mu->hovered_N += - mu->scroll_page;
		mu->hovered_N = (mu->hovered_N < 0) ? 0 : mu->hovered_N;
		mu->hovered_N = (mu->hovered_N > mu->size_N - 1)
			? mu->size_N - 1 : mu->hovered_N;

		if (mu->hovered_N < mu->scroll_shift) {

			mu->scroll_shift += - mu->scroll_page;
			mu->scroll_shift = (mu->scroll_shift < 0) ? 0 : mu->scroll_shift;
		}
	}
	else if (evno == MENU_EVNO_PAGE_DOWN) {

		mu->hovered_N += + mu->scroll_page;
		mu->hovered_N = (mu->hovered_N > mu->size_N - 1)
			? mu->size_N - 1 : mu->hovered_N;

		if (mu->hovered_N > (mu->size_N - 1) + (mu->scroll_shift - mu->scroll_limit)) {

			mu->scroll_shift += + mu->scroll_page;
			mu->scroll_shift = (mu->scroll_shift > mu->scroll_limit)
				? mu->scroll_limit : mu->scroll_shift;
		}
	}
	else if (evno == MENU_EVNO_HOME) {

		mu->hovered_N = 0;
		mu->scroll_shift = 0;
	}
	else if (evno == MENU_EVNO_END) {

		mu->hovered_N = mu->size_N - 1;
		mu->scroll_shift = mu->scroll_limit;
	}
	else if (evno == MENU_EVNO_RETURN) {

		mu->clicked = 1;
		mu->clicked_N = menuConvFuzzy(mu, mu->hovered_N);

		for (hN = 0; hN < MENU_OPTION_MAX; ++hN) {

			mu->clicked_N = (mu->clicked_N == mu->hidden_N[hN])
				? -1 : mu->clicked_N;
		}
	}
}

void menuEventText(menu_t *mu, const char *tx)
{
	int			bX, bY;

	if (mu->colorful != 0)
		return ;

	TTF_SizeUTF8(mu->font, mu->fuzzy, &bX, &bY);
	bX += mu->layout_height;

	if (strlen(mu->fuzzy) < MENU_FUZZY_SIZE - 7
			&& bX < mu->size_X) {

		strcat(mu->fuzzy, tx);
		menuBuild(mu, 1);
	}
}

void menuDraw(menu_t *mu, SDL_Surface *surface)
{
	const char		*fu, *s = mu->list;
	int			light[MENU_FUZZY_SIZE * 2], N_fu;
	int			topY, baseX, baseY, margin, side;
	int			N = 0, N_visible, N_conv, hN;

	colType_t		iCol;

	if (mu->raised == 0)
		return ;

	SDL_LockSurface(surface);

	baseY = mu->box_Y + mu->size_Y;
	baseY += (mu->fuzzy[0] != 0) ? mu->layout_height : 0;

	drawFillRect(surface, mu->box_X, mu->box_Y, mu->box_X + mu->size_X,
			baseY, mu->sch->menu_background);

	if (mu->hovered_N != -1) {

		topY = mu->box_Y + (mu->hovered_N - mu->scroll_shift) * mu->layout_height;
		baseX = (mu->scroll_limit != 0) ? mu->size_X - mu->layout_height : mu->size_X;

		drawFillRect(surface, mu->box_X, topY, mu->box_X + baseX,
				topY + mu->layout_height, mu->sch->menu_hovered);
	}

	if (mu->scroll_limit != 0) {

		margin = mu->layout_height / 4;
		side = mu->layout_height - margin;

		baseX = mu->box_X + mu->size_X - mu->layout_height;
		baseY = mu->box_Y + (mu->size_Y - mu->layout_height)
			* mu->scroll_shift / mu->scroll_limit;

		drawFillRect(surface, baseX + margin, baseY + margin, baseX + side,
				baseY + side, mu->sch->menu_scrollbar);

		baseX = mu->box_X + mu->size_X - mu->layout_height + margin;
		baseY = mu->box_Y + margin;

		margin = side - margin + 1;
		side = mu->size_Y - mu->layout_height + margin;

		drawLine(mu->dw, surface, &mu->screen, baseX, baseY,
				baseX + margin, baseY, mu->sch->menu_scrollbar);

		drawLine(mu->dw, surface, &mu->screen, baseX, baseY + side,
				baseX + margin, baseY + side, mu->sch->menu_scrollbar);

		drawLine(mu->dw, surface, &mu->screen, baseX, baseY,
				baseX, baseY + side, mu->sch->menu_scrollbar);

		drawLine(mu->dw, surface, &mu->screen, baseX + margin, baseY,
				baseX + margin, baseY + side, mu->sch->menu_scrollbar);
	}

	while (*s != 0) {

		if (menuFuzzyMatch(mu, s, mu->fuzzy, NULL)) {

			if (N >= mu->scroll_shift)
				break;

			N++;
		}

		while (*s != 0) ++s;
		++s;
	}

	SDL_UnlockSurface(surface);

	N = 0;
	N_visible = mu->size_Y / mu->layout_height;

	margin = mu->layout_height / 4;
	side = mu->layout_height / 2;

	while (*s != 0) {

		if (menuFuzzyMatch(mu, s, mu->fuzzy, light)) {

			topY = mu->box_Y + N * mu->layout_height;

			if (N > N_visible - 1) {

				break;
			}

			if (mu->fuzzy[0] != 0) {

				SDL_LockSurface(surface);

				fu = mu->fuzzy;
				N_fu = 0;

				while (*fu != 0) {

					baseX = mu->box_X + margin;

					drawFillRect(surface, baseX + light[N_fu + 0],
							topY, baseX + light[N_fu + 1],
							topY + mu->layout_height,
							mu->sch->menu_fuzzy_light);

					fu = utf8_go_next(fu);
					N_fu += 2;
				}

				SDL_UnlockSurface(surface);
			}

			N_conv = menuConvFuzzy(mu, N) + mu->scroll_shift;

			if (mu->colorful != 0 && N_conv != -1) {

				iCol = mu->sch->plot_figure[N_conv];

				for (hN = 0; hN < MENU_OPTION_MAX; ++hN) {

					iCol = (N_conv == mu->hidden_N[hN])
						? mu->sch->menu_background : iCol;
				}

				baseX = (mu->scroll_limit != 0)
					? mu->size_X - mu->layout_height
					: mu->size_X;

				drawFillRect(surface, mu->box_X + side,
						topY + margin, mu->box_X + baseX - side,
						topY + mu->layout_height - margin, iCol);
			}
			else {
				const char	*text = s;
				char		sbuf[MENU_STRING_MAX + 1];

				if (strncmp(s, "---", 3) != 0) {

					iCol = mu->sch->menu_item_text;
				}
				else {
					iCol = mu->sch->menu_item_hidden;
				}

				for (hN = 0; hN < MENU_OPTION_MAX; ++hN) {

					if (N_conv == mu->hidden_N[hN]) {

						iCol = mu->sch->menu_item_hidden;
					}
					else if (N_conv == mu->mark[hN].N) {

						strcpy(sbuf, s);
						sprintf(sbuf, s, mu->mark[hN].subs);

						text = sbuf;
					}
				}

				drawText(mu->dw, surface, mu->font, mu->box_X + margin,
						topY + side, text, TEXT_CENTERED_ON_Y, iCol);
			}

			N++;
		}

		while (*s != 0) ++s;
		++s;
	}

	if (mu->fuzzy[0] != 0) {

		topY = mu->box_Y + mu->size_Y + side;

		drawText(mu->dw, surface, mu->font, mu->box_X + margin, topY, mu->fuzzy,
				TEXT_CENTERED_ON_Y, mu->sch->menu_fuzzy_light);
	}
}

