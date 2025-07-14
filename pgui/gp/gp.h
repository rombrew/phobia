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

#ifndef _H_GP_
#define _H_GP_

#include <SDL2/SDL.h>

struct gp_context;

typedef struct gp_context gpcon_t;

enum {
	GP_PAGE_SELECT		= 0,
	GP_PAGE_COMBINE,
	GP_PAGE_NO_REMAP
};

gpcon_t *gp_Alloc();
void gp_Clean(gpcon_t *gp);

void gp_TakeConfig(gpcon_t *gp, const char *config);
void gp_TakeEvent(gpcon_t *gp, const SDL_Event *ev);

SDL_Surface *gp_GetSurface(gpcon_t *gp);
Uint32 gp_OpenWindow(gpcon_t *gp);

int gp_DataAdd(gpcon_t *gp, int dN, const double *payload);
void gp_FileReload(gpcon_t *gp);
void gp_PageCombine(gpcon_t *gp, int pN, int remap);
int gp_PageSafe(gpcon_t *gp);
void gp_AxisRange(gpcon_t *gp, int aN, double min, double max);

int gp_IsQuit(gpcon_t *gp);
int gp_Draw(gpcon_t *gp);

void gp_SavePNG(gpcon_t *gp, const char *file);
void gp_SaveSVG(gpcon_t *gp, const char *file);

#endif /* _H_GP_ */

