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

#ifndef _H_DIRENT_
#define _H_DIRENT_

#define DIRENT_PATH_MAX			272

#ifdef _WINDOWS
#define DIRSEP 			"\\"
#else /* _WINDOWS */
#define DIRSEP			"/"
#endif

enum {
	ENT_TYPE_UNKNOWN		= 0,
	ENT_TYPE_REGULAR,
	ENT_TYPE_DIRECTORY
};

enum {
	ENT_OK				= 0,
	ENT_END_OF_DIR,
	ENT_ERROR_UNKNOWN
};

struct dirent_priv;

struct dirent_stat {

	unsigned long long	nsize;
	int			ntype;

	char			name[DIRENT_PATH_MAX];
	char			time[24];

	struct dirent_priv	*priv;
};

int dirent_open(struct dirent_stat *sb, const char *path);
int dirent_rewind(struct dirent_stat *sb);
int dirent_read(struct dirent_stat *sb);
void dirent_close(struct dirent_stat *sb);

int file_stat(const char *file, unsigned long long *nsize);
int file_remove(const char *file);

#endif /* _H_DIRENT_ */

