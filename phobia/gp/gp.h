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

#ifndef _H_GP_
#define _H_GP_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

struct gp_struct;

typedef struct gp_struct gp_t;

gp_t *gp_Alloc();
void gp_Clean(gp_t *gp);

void gp_TakeConfig(gp_t *gp, const char *config);
int gp_OpenWindow(gp_t *gp);

void gp_TakeEvent(gp_t *gp, const SDL_Event *ev);
int gp_IsQuit(gp_t *gp);
int gp_Draw(gp_t *gp);

#endif /* _H_GP_ */

