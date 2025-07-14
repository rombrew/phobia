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
#include <time.h>

#include "dirent.h"

#ifdef _WINDOWS
#include <windows.h>

struct dirent_priv {

	wchar_t			wpath[DIRENT_PATH_MAX];

	HANDLE			hDIR;
	WIN32_FIND_DATAW	fDATA;

	int			first;
};

int dirent_open(struct dirent_stat *sb, const char *path)
{
	struct dirent_priv	*p;

	if (sb->priv != NULL) {

		memset(sb->priv, 0, sizeof(struct dirent_priv));
	}
	else {
		sb->priv = (struct dirent_priv *) calloc(1, sizeof(struct dirent_priv));
	}

	p = sb->priv;

	MultiByteToWideChar(CP_UTF8, 0, path, -1, p->wpath, DIRENT_PATH_MAX);

	wcscat(p->wpath, L"\\*");

	p->hDIR = FindFirstFileW(p->wpath, &p->fDATA);

	if (p->hDIR == INVALID_HANDLE_VALUE)
		return ENT_ERROR_UNKNOWN;

	p->first = 1;

	return ENT_OK;
}

int dirent_rewind(struct dirent_stat *sb)
{
	struct dirent_priv	*p = sb->priv;

	if (p == NULL)
		return ENT_ERROR_UNKNOWN;

	FindClose(p->hDIR);

	p->hDIR = FindFirstFileW(p->wpath, &p->fDATA);

	if (p->hDIR == INVALID_HANDLE_VALUE)
		return ENT_ERROR_UNKNOWN;

	p->first = 1;

	return ENT_OK;
}

int dirent_read(struct dirent_stat *sb)
{
	struct dirent_priv	*p = sb->priv;

	SYSTEMTIME		tUTC, tLOC;

	union {
		unsigned long long	l;
		unsigned int		w[2];
	}
	len;

	if (p == NULL)
		return ENT_ERROR_UNKNOWN;

	if (p->first == 0) {

		if (FindNextFileW(p->hDIR, &p->fDATA) == 0)
			return ENT_END_OF_DIR;
	}
	else {
		p->first = 0;
	}

	if (p->fDATA.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {

		sb->ntype = ENT_TYPE_DIRECTORY;
	}
	else {
		sb->ntype = ENT_TYPE_REGULAR;
	}

	len.w[1] = p->fDATA.nFileSizeHigh;
	len.w[0] = p->fDATA.nFileSizeLow;

	sb->nsize = len.l;

	WideCharToMultiByte(CP_UTF8, 0, p->fDATA.cFileName, -1,
				sb->name, DIRENT_PATH_MAX, NULL, NULL);

	FileTimeToSystemTime(&p->fDATA.ftLastWriteTime, &tUTC);
	SystemTimeToTzSpecificLocalTime(NULL, &tUTC, &tLOC);

	sprintf(sb->time, "%04d-%02d-%02d %02d:%02d",
			tLOC.wYear, tLOC.wMonth, tLOC.wDay,
			tLOC.wHour, tLOC.wMinute);

	return ENT_OK;
}

void dirent_close(struct dirent_stat *sb)
{
	struct dirent_priv	*p = sb->priv;

	if (p != NULL) {

		FindClose(p->hDIR);
	}
}

int file_stat(const char *file, unsigned long long *nsize)
{
	wchar_t			wfile[DIRENT_PATH_MAX];
	HANDLE			hFile;
	LARGE_INTEGER		nSize = { 0 } ;
	BOOL			bSize;

	MultiByteToWideChar(CP_UTF8, 0, file, -1, wfile, DIRENT_PATH_MAX);

	hFile = CreateFileW(wfile, 0, 0, NULL, OPEN_EXISTING, 0, NULL);

	if (hFile == INVALID_HANDLE_VALUE) {

		return ENT_ERROR_UNKNOWN;
	}

	bSize = GetFileSizeEx(hFile, &nSize);

	CloseHandle(hFile);

	if (bSize == 0) {

		return ENT_ERROR_UNKNOWN;
	}

	*nsize = nSize.QuadPart;

	return ENT_OK;
}

int file_remove(const char *file)
{
	wchar_t			wfile[DIRENT_PATH_MAX];

	MultiByteToWideChar(CP_UTF8, 0, file, -1, wfile, DIRENT_PATH_MAX);

	return (DeleteFileW(wfile) != 0) ? ENT_OK : ENT_ERROR_UNKNOWN;
}

#else /* _WINDOWS */
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <dirent.h>
#include <unistd.h>

struct dirent_priv {

	char			path[DIRENT_PATH_MAX];
	int			len;

	DIR			*dir;
	struct stat		sb;
};

int dirent_open(struct dirent_stat *sb, const char *path)
{
	struct dirent_priv	*p;

	if (sb->priv != NULL) {

		memset(sb->priv, 0, sizeof(struct dirent_priv));
	}
	else {
		sb->priv = (struct dirent_priv *) calloc(1, sizeof(struct dirent_priv));
	}

	p = sb->priv;

	p->dir = opendir(path);

	if (p->dir == NULL)
		return ENT_ERROR_UNKNOWN;

	strcpy(p->path, path);
	p->len = strlen(p->path);

	return ENT_OK;
}

int dirent_rewind(struct dirent_stat *sb)
{
	struct dirent_priv	*p = sb->priv;

	if (p == NULL)
		return ENT_ERROR_UNKNOWN;

	rewinddir(p->dir);

	return ENT_OK;
}

int dirent_read(struct dirent_stat *sb)
{
	struct dirent_priv	*p = sb->priv;

	struct dirent 		*en;
	struct tm		*loc;

	if (p == NULL)
		return ENT_ERROR_UNKNOWN;

	en = readdir(p->dir);

	if (en == NULL)
		return ENT_ERROR_UNKNOWN;

	p->path[p->len] = 0;

	strcat(p->path, DIRSEP);
	strcat(p->path, en->d_name);

	if (stat(p->path, &p->sb) == 0) {

		if (p->sb.st_mode & S_IFDIR) {

			sb->ntype = ENT_TYPE_DIRECTORY;
		}
		else {
			sb->ntype = ENT_TYPE_REGULAR;
		}

		sb->nsize = p->sb.st_size;

		loc = localtime(&p->sb.st_mtime);

		sprintf(sb->time, "%04d-%02d-%02d %02d:%02d",
				loc->tm_year + 1900, loc->tm_mon + 1, loc->tm_mday,
				loc->tm_hour, loc->tm_min);

		strcpy(sb->name, en->d_name);

		return ENT_OK;
	}
	else {
		return ENT_ERROR_UNKNOWN;
	}
}

void dirent_close(struct dirent_stat *sb)
{
	struct dirent_priv	*p = sb->priv;

	if (p != NULL) {

		closedir(p->dir);
	}
}

int file_stat(const char *file, unsigned long long *nsize)
{
	struct stat		sb;
	int			rc;

	rc = stat(file, &sb);

	if (rc == 0) {

		*nsize = sb.st_size;
	}

	return (rc == 0) ? ENT_OK : ENT_ERROR_UNKNOWN;
}

int file_remove(const char *file)
{
	return (remove(file) == 0) ? ENT_OK : ENT_ERROR_UNKNOWN;
}

#endif /* _WINDOWS */

