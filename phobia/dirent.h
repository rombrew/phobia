#ifndef _H_DIRENT_
#define _H_DIRENT_

#include <stdlib.h>
#include <stdio.h>

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

#ifdef _WINDOWS
void windows_ACP_to_UTF8(char *lputf, const char *lpacp, int len);
#endif /* _WINDOWS */

int dirent_open(struct dirent_stat *sb, const char *path);
int dirent_read(struct dirent_stat *sb);
void dirent_close(struct dirent_stat *sb);

void file_remove(const char *file);

FILE *fopen_from_UTF8(const char *file, const char *mode);

#endif /* _H_DIRENT_ */

