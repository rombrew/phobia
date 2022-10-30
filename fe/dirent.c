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

	int			firstfile;
};

void winapi_ACP_to_UTF8(char *lputf, const char *lpacp, int len)
{
	wchar_t			wbuf[DIRENT_PATH_MAX];

	MultiByteToWideChar(CP_ACP, 0, lpacp, -1, wbuf, DIRENT_PATH_MAX);
	WideCharToMultiByte(CP_UTF8, 0, wbuf, -1, lputf, len, NULL, NULL);
}

int dirent_open(struct dirent_stat *sb, const char *path)
{
	struct dirent_priv	*p;

	if (sb->priv != NULL) {

		memset(sb->priv, 0, sizeof(struct dirent_priv));
	}
	else {
		sb->priv = calloc(1, sizeof(struct dirent_priv));
	}

	p = sb->priv;

	MultiByteToWideChar(CP_UTF8, 0, path, -1, p->wpath, DIRENT_PATH_MAX);

	wcscat(p->wpath, L"/*");

	p->hDIR = FindFirstFileW(p->wpath, &p->fDATA);

	if (p->hDIR == INVALID_HANDLE_VALUE)
		return ENT_ERROR_UNKNOWN;

	p->firstfile = 1;

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

	if (p->firstfile == 0) {

		if (FindNextFileW(p->hDIR, &p->fDATA) == 0)
			return ENT_END_OF_DIR;
	}
	else {
		p->firstfile = 0;
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

void file_remove(const char *file)
{
	wchar_t			wfile[DIRENT_PATH_MAX];

	MultiByteToWideChar(CP_UTF8, 0, file, -1, wfile, DIRENT_PATH_MAX);

	DeleteFileW(wfile);
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
		sb->priv = calloc(1, sizeof(struct dirent_priv));
	}

	p = sb->priv;

	p->dir = opendir(path);

	if (p->dir == NULL)
		return ENT_ERROR_UNKNOWN;

	strcpy(p->path, path);
	p->len = strlen(p->path);

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

void file_remove(const char *file)
{
	remove(file);
}
#endif /* _WINDOWS */

FILE *fopen_from_UTF8(const char *file, const char *mode)
{
#ifdef _WINDOWS
	wchar_t			wfile[DIRENT_PATH_MAX];
	wchar_t			wmode[16];

	MultiByteToWideChar(CP_UTF8, 0, file, -1, wfile, DIRENT_PATH_MAX);
	MultiByteToWideChar(CP_UTF8, 0, mode, -1, wmode, 16);

	return _wfopen(wfile, wmode);
#else /* _WINDOWS */
	return fopen(file, mode);
#endif
}

