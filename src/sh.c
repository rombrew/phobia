/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

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

#include "sh.h"
#include "task.h"
#include "lib.h"

#define SH_RXBUF_SZ			40
#define SH_TXBUF_SZ			80
#define SH_CLINE_SZ			80
#define SH_HISTORY_SZ			440

#define K_SO				0x0E
#define K_DLE				0x10
#define K_ESC				0x1B

#define FIFO_INC(I, SZ)			(((I) < ((SZ) - 1)) ? (I) + 1 : 0)
#define FIFO_DEC(I, SZ)			(((I) > 0) ? (I) - 1 : (SZ) - 1)

static const char
SH_PROMPT[] = "# ",
SH_BACKSPACE[] = "\b \b";

enum {
	DIR_UP,
	DIR_DOWN
};

typedef struct {

	/* Incoming FIFO.
	 * */
	char		rBuf[SH_RXBUF_SZ];
	int		rR, rT;

	/* Outgoing FIFO.
	 * */
	char		tBuf[SH_TXBUF_SZ];
	int		tR, tT;

	/* Base SH data.
	 * */
	char		cLine[SH_CLINE_SZ];
	int		nEOL, xESC;
	char		*pARG;

	/* Completion block.
	 * */
	int		cMD, cEON, cIT;

	/* History block.
	 * */
	char		cHist[SH_HISTORY_SZ];
	int		hMD, hHEAD, hTAIL, hIT;
}
shTASK_t;

static shTASK_t		sh;
extern shCMD_t		cmList[];

int shRecv()
{
	int		xC;

	if (sh.rR != sh.rT) {

		xC = sh.rBuf[sh.rR];
		sh.rR = FIFO_INC(sh.rR, SH_RXBUF_SZ);
	}
	else
		xC = -1;

	return xC;
}

void shSend(int xC)
{
	int		tT;

	tT = FIFO_INC(sh.tT, SH_TXBUF_SZ);

	while (sh.tR == tT)
		taskIOMUX();

	sh.tBuf[sh.tT] = (char) xC;
	sh.tT = tT;
}

int shExRecv()
{
	int		xC;

	if (sh.tR != sh.tT) {

		xC = sh.tBuf[sh.tR];
		sh.tR = FIFO_INC(sh.tR, SH_TXBUF_SZ);
	}
	else
		xC = -1;

	return xC;
}

int shExPoll()
{
	return (sh.rT < sh.rR) ? sh.rR - sh.rT - 1
		: sh.rR - sh.rT - 1 + SH_RXBUF_SZ;
}

void shExPush(int xC)
{
	sh.rBuf[sh.rT] = (char) xC;
	sh.rT = FIFO_INC(sh.rT, SH_RXBUF_SZ);
}

static inline char
isDigit(char xC)
{
	return (xC >= '0') && (xC <= '9');
}

static inline char
isChar(char xC)
{
	return ((xC >= 'a') && (xC <= 'z'))
		|| ((xC >= 'A') && (xC <= 'Z'));
}

static void
shErase(int N)
{
	while (N) {

		puts(SH_BACKSPACE);
		--N;
	}
}

static void
shExactMatch()
{
	const shCMD_t		*pCMD;
	const char		*iD;

	pCMD = cmList;

	do {
		iD = pCMD->iD;

		if (iD == NULL)
			break;

		if (!strcmp(sh.cLine, iD)) {

			/* Call the function.
			 * */
			pCMD->pF(sh.pARG);

			break;
		}

		++pCMD;
	}
	while (1);
}

static void
shCyclicMatch()
{
	const shCMD_t		*pCMD;
	const char		*iD;
	int			N = 0;

	pCMD = cmList + sh.cIT;
	sh.cLine[sh.cEON] = 0;

	do {
		iD = pCMD->iD;

		if (iD == NULL) {

			pCMD = cmList;

			if (N++)
				break;
			else
				continue;
		}

		if (!strpcmp(sh.cLine, iD)) {

			/* Copy command name.
			 * */
			strcpy(sh.cLine, iD);

			break;
		}

		++pCMD;
	}
	while (1);

	sh.cIT = (pCMD - cmList) + 1;
}

static int
shHistoryMove(int xIT, int xDIR)
{
	if (xDIR == DIR_UP) {

		if (xIT != sh.hHEAD) {

			/* Get previous line.
			 * */
			xIT = FIFO_DEC(xIT, SH_HISTORY_SZ);

			do {
				xIT = FIFO_DEC(xIT, SH_HISTORY_SZ);

				if (sh.cHist[xIT] == 0)
					break;
			}
			while (1);

			xIT = FIFO_INC(xIT, SH_HISTORY_SZ);
		}
	}
	else {
		if (xIT != sh.hTAIL) {

			/* Get next line.
			 * */
			do {
				xIT = FIFO_INC(xIT, SH_HISTORY_SZ);

				if (sh.cHist[xIT] == 0)
					break;
			}
			while (1);

			xIT = FIFO_INC(xIT, SH_HISTORY_SZ);
		}
	}

	return xIT;
}

static void
shHistoryPut(const char *xS)
{
	int			xIT, R;
	const char		*xP = xS;

	if (sh.hHEAD != sh.hTAIL) {

		xIT = shHistoryMove(sh.hTAIL, DIR_UP);

		do {
			R = sh.cHist[xIT] - *xP;

			if (R || !*xP)
				break;

			xIT = FIFO_INC(xIT, SH_HISTORY_SZ);
			xP++;
		}
		while (1);

		if (R == 0)

			/* Do not put the same line again.
			 * */
			return ;
	}

	do {
		sh.cHist[sh.hTAIL] = *xS;
		sh.hTAIL = FIFO_INC(sh.hTAIL, SH_HISTORY_SZ);

		if (sh.hTAIL == sh.hHEAD) {

			/* Forget old lines.
			 * */
			do {
				sh.hHEAD = FIFO_INC(sh.hHEAD, SH_HISTORY_SZ);

				if (sh.cHist[sh.hHEAD] == 0)
					break;
			}
			while (1);

			sh.hHEAD = FIFO_INC(sh.hHEAD, SH_HISTORY_SZ);
		}

		if (*xS == 0)
			break;
		else
			xS++;
	}
	while (1);
}

static void
shEval()
{
	char			*pC;

	pC = sh.cLine;

	if (*pC != 0) {

		/* Put the line in history.
		 * */
		shHistoryPut(pC);

		/* Parse the command line.
		 * */
		while (*pC && *pC != ' ') ++pC;
		while (*pC && *pC == ' ') *pC++ = 0;
		sh.pARG = pC;

		/* Search for specific command to execute.
		 * */
		shExactMatch();
	}
}

static void
shComplete()
{
	char			*pC;
	int			N;

	if (sh.cMD == 0) {

		pC = sh.cLine;

		while (*pC) {

			/* Do not complete with trailing spaces.
			 * */
			if (*pC == ' ')
				return ;

			++pC;
		}

		N = sh.nEOL;

		/* Enter completion mode.
		 * */
		sh.cMD = 1;
		sh.cEON = N;
		sh.cIT = 0;
	}
	else {
		N = sh.cEON;
	}

	/* Search for the next match.
	 * */
	shCyclicMatch();

	/* Update the command line.
	 * */
	shErase(sh.nEOL - N);
	sh.nEOL = strlen(sh.cLine);
	puts(sh.cLine + N);

	sh.hMD = 0;
}

static void
shHistory(int xDIR)
{
	int			xIT;
	char			*xD;

	if (sh.hMD == 0) {

		/* Enter history mode.
		 * */
		sh.hIT = sh.hTAIL;
		sh.hMD = 1;
		xIT = sh.hTAIL;

		/* Save current line.
		 * */
		shHistoryPut(sh.cLine);
		sh.hTAIL = xIT;
	}

	if (xDIR == DIR_UP)

		xIT = shHistoryMove(sh.hIT, DIR_UP);
	else
		xIT = shHistoryMove(sh.hIT, DIR_DOWN);

	if (xIT != sh.hIT) {

		sh.hIT = xIT;
		xD = sh.cLine;

		do {
			if (!(*xD = sh.cHist[xIT]))
				break;

			xD++;
			xIT = FIFO_INC(xIT, SH_HISTORY_SZ);
		}
		while (1);

		/* Update the command line.
		 * */
		shErase(sh.nEOL);
		sh.nEOL = strlen(sh.cLine);
		puts(sh.cLine);
	}

	sh.cMD = 0;
}

static void
shLinePutC(char xC)
{
	if (sh.nEOL < (SH_CLINE_SZ - 1)) {

		sh.cLine[sh.nEOL++] = xC;
		sh.cLine[sh.nEOL] = 0;

		/* Echo.
		 * */
		putc(xC);

		sh.cMD = 0;
		sh.hMD = 0;
	}
}

static void
shLineBS()
{
	if (sh.nEOL > 0) {

		sh.cLine[--sh.nEOL] = 0;

		/* Echo.
		 * */
		puts(SH_BACKSPACE);

		sh.cMD = 0;
		sh.hMD = 0;
	}
}

static void
shLineNULL()
{
	sh.cLine[sh.nEOL = 0] = 0;

	/* Prompt.
	 * */
	puts(SH_PROMPT);

	sh.cMD = 0;
	sh.hMD = 0;
}

void shTask()
{
	int		xC;

	while ((xC = shRecv()) >= 0) {

		if (sh.xESC == 0) {

			if (isChar(xC) || isDigit(xC)
					|| (xC == ' ')
					|| (xC == '_')
					|| (xC == '.')
					|| (xC == '-')
					|| (xC == '+')
					|| (xC == '%')) {

				shLinePutC(xC);
			}
			else if (xC == '\r') {

				/* Echo.
				 * */
				puts(EOL);

				shEval();
				shLineNULL();
			}
			else if (xC == '\b') {

				shLineBS();
			}
			else if (xC == '\t') {

				shComplete();
			}
			else if (xC == K_DLE) {

				shHistory(DIR_UP);
			}
			else if (xC == K_SO) {

				shHistory(DIR_DOWN);
			}
			else if (xC == K_ESC) {

				sh.xESC = 1;
			}
		}
		else {
			if (sh.xESC == 1) {

				sh.xESC = (xC == '[') ? 2 : 0;
			}
			else {
				if (xC == 'A') {

					shHistory(DIR_UP);
				}
				else if (xC == 'B') {

					shHistory(DIR_DOWN);
				}

				sh.xESC = 0;
			}
		}
	}
}

