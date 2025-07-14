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

#include "async.h"
#include "plot.h"

static int
async_READ(async_FILE *afd)
{
	int		rp, wp, nw, rc;

	wp = SDL_AtomicGet(&afd->wp);

	do {
		rp = SDL_AtomicGet(&afd->rp);

		nw = rp - (wp + 1);
		nw += (nw < 0) ? afd->preload : 0;

		if (nw >= afd->chunk) {

			nw = afd->preload - wp;
			nw = (nw > afd->chunk) ? afd->chunk : nw;

			rc = (int) fread(afd->stream + wp, 1, nw, afd->fd);

			if (rc != 0) {

				wp += rc;
				wp -= (wp >= afd->preload) ? afd->preload : 0;

				SDL_AtomicSet(&afd->wp, wp);

				afd->waiting = 0;
			}

			if (rc != nw) {

				if (feof(afd->fd) || ferror(afd->fd)) {

					if (afd->waiting < afd->timeout) {

						clearerr(afd->fd);

						SDL_Delay(10);

						afd->waiting += 10;
					}
					else {
						break;
					}
				}
			}
		}
		else {
			SDL_Delay(1);
		}
	}
	while (SDL_AtomicGet(&afd->flag_break) == 0);

	SDL_AtomicSet(&afd->flag_eof, 1);

	return 0;
}

async_FILE *async_open(FILE *fd, int preload, int chunk, int timeout)
{
	async_FILE		*afd;

	afd = (async_FILE *) calloc(1, sizeof(async_FILE));

	afd->preload = preload;
	afd->chunk = chunk;
	afd->timeout = timeout;

	afd->stream = (char *) malloc(afd->preload);

	if (afd->stream == NULL) {

		ERROR("No memory allocated for async preload\n");
		return NULL;
	}

	afd->fd = fd;
	afd->thread = SDL_CreateThread((int (*) (void *)) &async_READ, "async_READ", afd);

	return afd;
}

async_FILE *async_stub(int preload, int chunk, int timeout)
{
	async_FILE		*afd;

	afd = (async_FILE *) calloc(1, sizeof(async_FILE));

	afd->preload = preload;
	afd->chunk = chunk;
	afd->timeout = timeout;

	afd->stream = (char *) malloc(afd->preload);

	if (afd->stream == NULL) {

		ERROR("No memory allocated for async preload\n");
		return NULL;
	}

	return afd;
}

void async_close(async_FILE *afd)
{
	int		tN = 0;

	SDL_AtomicSet(&afd->flag_break, 1);
	SDL_DetachThread(afd->thread);

	do {
		SDL_Delay(10);

		if (SDL_AtomicGet(&afd->flag_eof) != 0) {

			free(afd->stream);
			free(afd);
			break;
		}

		tN += 1;

		if (tN >= 100) {

			ERROR("Unable to terminate async_READ (memory leak)\n");
			break;
		}
	}
	while (1);
}

int async_write(async_FILE *afd, const char *raw, int n)
{
	int		rp, wp, nw;

	rp = SDL_AtomicGet(&afd->rp);
	wp = SDL_AtomicGet(&afd->wp);

	nw = (wp < rp) ? rp - wp : rp + afd->preload - wp;

	if (nw > n) {

		if (wp + n >= afd->preload) {

			nw = afd->preload - wp;

			memcpy(afd->stream + wp, raw, nw);
			memcpy(afd->stream, raw + nw, n - nw);

			wp += n - afd->preload;
		}
		else {
			memcpy(afd->stream + wp, raw, n);

			wp += n;
		}

		afd->clock = SDL_GetTicks();

		SDL_AtomicSet(&afd->wp, wp);

		return ASYNC_OK;
	}
	else {
		return ASYNC_NO_FREE_SPACE;
	}
}

int async_read(async_FILE *afd, char *raw, int n)
{
	int		rp, wp, nr;

	rp = SDL_AtomicGet(&afd->rp);
	wp = SDL_AtomicGet(&afd->wp);

	nr = (wp < rp) ? wp + afd->preload - rp : wp - rp;

	if (nr >= n) {

		if (rp + n >= afd->preload) {

			nr = afd->preload - rp;

			memcpy(raw, afd->stream + rp, nr);
			memcpy(raw + nr, afd->stream, n - nr);

			rp += n - afd->preload;
		}
		else {
			memcpy(raw, afd->stream + rp, n);

			rp += n;
		}

		SDL_AtomicSet(&afd->rp, rp);

		return ASYNC_OK;
	}
	else if (SDL_AtomicGet(&afd->flag_eof) != 0) {

		return ASYNC_END_OF_FILE;
	}
	else {
		return ASYNC_NO_DATA_READY;
	}
}

int async_gets(async_FILE *afd, char *raw, int n)
{
	int		rp, wp, eol, nq;
	char		c;

	rp = SDL_AtomicGet(&afd->rp);
	wp = SDL_AtomicGet(&afd->wp);

	if (wp != afd->cached) {

		eol = 0;
		nq = 0;

		do {
			if (rp == wp)
				break;

			c = (int) afd->stream[rp];

			if (c == '\r' || c == '\n') {

				eol = (nq > 0) ? 1 : 0;
			}
			else if (eol != 0) {

				break;
			}
			else if (nq < n - 1) {

				*raw++ = (char) c;
				nq++;
			}

			rp = (rp < afd->preload - 1) ? rp + 1 : 0;
		}
		while (1);

		afd->cached = rp;

		if (eol != 0) {

			*raw = 0;

			SDL_AtomicSet(&afd->rp, rp);

			return ASYNC_OK;
		}

		return ASYNC_NO_DATA_READY;
	}
	else if (SDL_AtomicGet(&afd->flag_eof) != 0) {

		if (rp != wp) {

			nq = 0;

			do {
				if (rp == wp)
					break;

				c = (int) afd->stream[rp];

				if (nq < n - 1) {

					*raw++ = (char) c;
					nq++;
				}

				rp = (rp < afd->preload - 1) ? rp + 1 : 0;
			}
			while (1);

			*raw = 0;

			SDL_AtomicSet(&afd->rp, rp);

			return ASYNC_OK;
		}

		return ASYNC_END_OF_FILE;
	}
	else {
		return ASYNC_NO_DATA_READY;
	}
}

