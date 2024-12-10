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

#ifndef _H_ASYNC_
#define _H_ASYNC_

#include <stdlib.h>
#include <stdio.h>

#include <SDL2/SDL.h>

enum {
	ASYNC_OK		= 0,
	ASYNC_NO_FREE_SPACE,
	ASYNC_NO_DATA_READY,
	ASYNC_END_OF_FILE
};

typedef struct {

	FILE		*fd;
	SDL_Thread	*thread;

	Uint32		clock;

	int		preload;
	int		chunk;
	int		timeout;
	int		cached;
	int		waiting;

	char		*stream;

	SDL_atomic_t	rp;
	SDL_atomic_t	wp;

	SDL_atomic_t	flag_eof;
	SDL_atomic_t	flag_break;
}
async_FILE;

async_FILE *async_open(FILE *fd, int preload, int chunk, int timeout);
async_FILE *async_stub(int preload, int chunk, int timeout);
void async_close(async_FILE *afd);

int async_write(async_FILE *afd, const char *raw, int n);
int async_read(async_FILE *afd, char *raw, int n);
int async_gets(async_FILE *afd, char *raw, int n);

#endif /* _H_ASYNC_ */

