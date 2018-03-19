/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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

#include <stddef.h>

#include "shell.h"
#include "lib.h"

#define SH_CLINE_SZ			80
#define SH_HISTORY_SZ			440

#define FIFO_INC(I, SZ)                 (((I) < ((SZ) - 1)) ? (I) + 1 : 0)
#define FIFO_DEC(I, SZ)                 (((I) > 0) ? (I) - 1 : (SZ) - 1)

static const char
SH_PROMPT[] = "(pmc) ",
SH_BACKSPACE[] = "\b \b";

enum {
	DIR_UP,
	DIR_DOWN
};

typedef struct {

	/* Base SH data.
	 * */
	char		cline[SH_CLINE_SZ];
	int		neol, xesc;
	char		*parg;

	/* Completion block.
	 * */
	int		cmd, ceon, cit;

	/* History block.
	 * */
	char		chist[SH_HISTORY_SZ];
	int		hmd, hhead, htail, hit;
}
sh_t;

static sh_t			sh;
extern const sh_cmd_t		cmlist[];

static inline char
isdigit(char c)
{
	return (c >= '0') && (c <= '9');
}

static inline char
ischar(char c)
{
	return ((c >= 'a') && (c <= 'z'))
		|| ((c >= 'A') && (c <= 'Z'));
}

static void
sh_erase(int n)
{
	while (n > 0) {

		puts(SH_BACKSPACE);
		--n;
	}
}

static void
sh_exact_match()
{
	const sh_cmd_t		*cmd;
	const char		*id;

	cmd = cmlist;

	do {
		id = cmd->sym;

		if (id == NULL)
			break;

		if (!strcmp(sh.cline, id)) {

			/* Call the function.
			 * */
			cmd->proc(sh.parg);

			break;
		}

		++cmd;
	}
	while (1);
}

static void
sh_cyclic_match(int xdir)
{
	const sh_cmd_t		*cmd;
	const char		*id;
	int			n = 0;

	cmd = cmlist + sh.cit;
	sh.cline[sh.ceon] = 0;

	cmd += (xdir == DIR_UP) ? 1 : - 1;

	do {
		if (cmd < cmlist) {

			/* Jump to the end.
			 * */
			while (cmd->sym != NULL) ++cmd;
			--cmd;

			if (n++)
				break;
		}

		id = cmd->sym;

		if (id == NULL) {

			/* Jump to the begin.
			 * */
			cmd = cmlist;

			if (n++)
				break;
		}

		if (!strpcmp(sh.cline, id)) {

			/* Copy the command name.
			 * */
			strcpy(sh.cline, id);

			break;
		}

		if (xdir == DIR_UP)
			++cmd;
		else
			--cmd;
	}
	while (1);

	sh.cit = cmd - cmlist;
}

static void
sh_common_match()
{
	const sh_cmd_t		*cmd;
	const char		*id, *ilast;
	int			n;

	ilast = NULL;
	cmd = cmlist;

	do {
		id = cmd->sym;

		if (id == NULL)
			break;

		if (!strpcmp(sh.cline, id)) {

			if (ilast != NULL)
				n = strspl(ilast, id, n);
			else
				n = strlen(id);

			ilast = id;
		}

		++cmd;
	}
	while (1);

	if (ilast != NULL) {

		strncpy(sh.cline, ilast, n);
		sh.ceon = n;
	}
	else
		sh.ceon = 0;
}

static int
sh_history_move(int xit, int xdir)
{
	if (xdir == DIR_UP) {

		if (xit != sh.hhead) {

			/* Get previous line.
			 * */
			xit = FIFO_DEC(xit, SH_HISTORY_SZ);

			do {
				xit = FIFO_DEC(xit, SH_HISTORY_SZ);

				if (sh.chist[xit] == 0)
					break;
			}
			while (1);

			xit = FIFO_INC(xit, SH_HISTORY_SZ);
		}
	}
	else {
		if (xit != sh.htail) {

			/* Get next line.
			 * */
			do {
				xit = FIFO_INC(xit, SH_HISTORY_SZ);

				if (sh.chist[xit] == 0)
					break;
			}
			while (1);

			xit = FIFO_INC(xit, SH_HISTORY_SZ);
		}
	}

	return xit;
}

static void
sh_history_put(const char *xs)
{
	int			xit, r;
	const char		*xp = xs;

	if (sh.hhead != sh.htail) {

		xit = sh_history_move(sh.htail, DIR_UP);

		do {
			r = sh.chist[xit] - *xp;

			if (r || !*xp)
				break;

			xit = FIFO_INC(xit, SH_HISTORY_SZ);
			xp++;
		}
		while (1);

		if (r == 0)

			/* Do not put the same line again.
			 * */
			return ;
	}

	do {
		sh.chist[sh.htail] = *xs;
		sh.htail = FIFO_INC(sh.htail, SH_HISTORY_SZ);

		if (sh.htail == sh.hhead) {

			/* Forget old lines.
			 * */
			do {
				sh.hhead = FIFO_INC(sh.hhead, SH_HISTORY_SZ);

				if (sh.chist[sh.hhead] == 0)
					break;
			}
			while (1);

			sh.hhead = FIFO_INC(sh.hhead, SH_HISTORY_SZ);
		}

		if (*xs == 0)
			break;
		else
			xs++;
	}
	while (1);
}

static void
sh_eval()
{
	char			*pc;

	pc = sh.cline;

	if (*pc != 0) {

		/* Put the line in history.
		 * */
		sh_history_put(pc);

		/* Parse the command line.
		 * */
		while (*pc && *pc != ' ') ++pc;
		while (*pc && *pc == ' ') *pc++ = 0;
		sh.parg = pc;

		/* Search for specific command to execute.
		 * */
		sh_exact_match();
	}
}

static void
sh_complete(int xdir)
{
	char			*pc;

	if (sh.cmd == 0) {

		pc = sh.cline;

		while (*pc) {

			/* Do not complete with trailing spaces.
			 * */
			if (*pc == ' ')
				return ;

			++pc;
		}

		/* Complete to the common substring.
		 * */
		sh_common_match();
		puts(sh.cline + sh.neol);

		if (sh.neol <= sh.ceon) {

			/* Enter completion mode.
			 * */
			sh.cmd = 1;
			sh.cit = (xdir == DIR_UP) ? - 1 : 0;

			if (sh.neol != sh.ceon)
				sh.neol = sh.ceon;
			else
				sh_complete(xdir);
		}
	}
	else {
		/* Search for the next match.
		 * */
		sh_cyclic_match(xdir);

		/* Update the command line.
		 * */
		sh_erase(sh.neol - sh.ceon);
		sh.neol = strlen(sh.cline);
		puts(sh.cline + sh.ceon);
	}

	sh.hmd = 0;
}

static void
sh_history(int xdir)
{
	int			xit;
	char			*xd;

	if (sh.hmd == 0) {

		/* Enter history mode.
		 * */
		sh.hit = sh.htail;
		sh.hmd = 1;
		xit = sh.htail;

		/* Save current line.
		 * */
		sh_history_put(sh.cline);
		sh.htail = xit;
	}

	if (xdir == DIR_UP)

		xit = sh_history_move(sh.hit, DIR_UP);
	else
		xit = sh_history_move(sh.hit, DIR_DOWN);

	if (xit != sh.hit) {

		sh.hit = xit;
		xd = sh.cline;

		do {
			if (!(*xd = sh.chist[xit]))
				break;

			xd++;
			xit = FIFO_INC(xit, SH_HISTORY_SZ);
		}
		while (1);

		/* Update the command line.
		 * */
		sh_erase(sh.neol);
		sh.neol = strlen(sh.cline);
		puts(sh.cline);
	}

	sh.cmd = 0;
}

static void
sh_line_putc(char c)
{
	if (sh.neol < (SH_CLINE_SZ - 1)) {

		sh.cline[sh.neol++] = c;
		sh.cline[sh.neol] = 0;

		/* Echo.
		 * */
		iodef->putc(c);

		sh.cmd = 0;
		sh.hmd = 0;
	}
}

static void
sh_line_bs()
{
	if (sh.neol > 0) {

		sh.cline[--sh.neol] = 0;

		/* Echo.
		 * */
		puts(SH_BACKSPACE);

		sh.cmd = 0;
		sh.hmd = 0;
	}
}

static void
sh_line_null()
{
	sh.cline[sh.neol = 0] = 0;

	/* Prompt.
	 * */
	puts(SH_PROMPT);

	sh.cmd = 0;
	sh.hmd = 0;
}

void taskSH(void *pData)
{
	int		c;

	do {
		c = iodef->getc();

		if (sh.xesc == 0) {

			if (ischar(c) || isdigit(c)
					|| (c == ' ')
					|| (c == '_')
					|| (c == '.')
					|| (c == '-')
					|| (c == '+')
					|| (c == '%')) {

				sh_line_putc(c);
			}
			else if (c == '\r') {

				/* Echo.
				 * */
				puts(EOL);

				sh_eval();
				sh_line_null();
			}
			else if (c == '\b') {

				sh_line_bs();
			}
			else if (c == '\t') {

				sh_complete(DIR_UP);
			}
			else if (c == K_ETX || c == K_EOT) {

				puts(EOL);
				sh_line_null();
			}
			else if (c == K_DLE) {

				sh_history(DIR_UP);
			}
			else if (c == K_SO) {

				sh_history(DIR_DOWN);
			}
			else if (c == K_ESC) {

				sh.xesc = 1;
			}
		}
		else {
			if (sh.xesc == 1) {

				sh.xesc = (c == '[') ? 2 : 0;
			}
			else {
				if (c == 'A') {

					sh_history(DIR_UP);
				}
				else if (c == 'B') {

					sh_history(DIR_DOWN);
				}
				else if (c == 'Z') {

					sh_complete(DIR_DOWN);
				}

				sh.xesc = 0;
			}
		}
	}
	while (1);
}

#include "shell.gen_h"

const sh_cmd_t		cmlist[] = {

#include "shell.gen_list"

	{NULL, NULL}
};

