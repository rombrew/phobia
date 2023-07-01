#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _WINDOWS
#include <windows.h>
#endif /* _WINDOWS */

#include "gp/dirent.h"
#include "config.h"

#ifdef _WINDOWS
#define FILE_HIDDEN_CONFIG	"_" FILE_HOME_CONFIG
#else /* _WINDOWS */
#define FILE_HIDDEN_CONFIG	"." FILE_HOME_CONFIG
#endif

#ifdef _WINDOWS
static void
config_ACP_to_UTF8(char *lputf, const char *lpacp, int len)
{
	wchar_t			wbuf[DIRENT_PATH_MAX];

	MultiByteToWideChar(CP_ACP, 0, lpacp, -1, wbuf, DIRENT_PATH_MAX);
	WideCharToMultiByte(CP_UTF8, 0, wbuf, -1, lputf, len, NULL, NULL);
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

static int
config_rcfiletry(struct config_phobia *fe)
{
	FILE			*fd;
	int			rc = 0;

	fd = fopen_from_UTF8(fe->rcfile, "r");

	if (fd != NULL) {

		rc = 1;

		fclose(fd);
	}

	return rc;
}

void config_read(struct config_phobia *fe)
{
	FILE		*fd;
	char		line[PHOBIA_PATH_MAX];

	const char	*name, *value;
	const char	*sep = " \t\r\n";

	fd = fopen_from_UTF8(fe->rcfile, "r");

	if (fd != NULL) {

		while (fgets(line, sizeof(line), fd) != NULL) {

			name = strtok(line, sep);
			value = strtok(NULL, sep);

			if (		   name == NULL
					|| value == NULL) {

				/* Skip empty lines */
			}
			else if (strcmp(name, "serialport") == 0) {

				strcpy(fe->serialport, value);
			}
			else if (strcmp(name, "windowsize") == 0) {

				fe->windowsize = strtol(value, NULL, 10);
			}
			else if (strcmp(name, "storage") == 0) {

				strcpy(fe->storage, value);
			}
			else if (strcmp(name, "fuzzy") == 0) {

				strcpy(fe->fuzzy, value);
			}
			else if (strcmp(name, "tlmrate") == 0) {

				fe->tlmrate = strtol(value, NULL, 10);
				sprintf(fe->tlmrate_lbuf, "%i", fe->tlmrate);
			}
			else if (strcmp(name, "regmaxn") == 0) {

				fe->regmaxn = strtol(value, NULL, 10);
			}
		}

		fclose(fd);
	}
}

void config_open(struct config_phobia *fe)
{
	char		*path_HOME;

	config_default(fe);

#ifdef _WINDOWS
	path_HOME = getenv("APPDATA");
#else /* _WINDOWS */
	path_HOME = getenv("HOME");
#endif

	if (path_HOME == NULL) {

		fe->local = 1;
	}

	if (fe->local == 0) {

#ifdef _WINDOWS
		config_ACP_to_UTF8(fe->rcfile, path_HOME, sizeof(fe->rcfile));
#else /* _WINDOWS */
		strcpy(fe->rcfile, path_HOME);
#endif
		strcat(fe->rcfile, DIRSEP FILE_HIDDEN_CONFIG);

		if (config_rcfiletry(fe) == 0) {

			config_write(fe);
		}

		if (config_rcfiletry(fe) != 0) {

			config_read(fe);
		}
		else {
			fe->local = 1;
		}
	}

	if (fe->local != 0) {

		strcpy(fe->rcfile, FILE_HIDDEN_CONFIG);

		config_default(fe);

		if (config_rcfiletry(fe) == 0) {

			config_write(fe);
		}

		if (config_rcfiletry(fe) != 0) {

			config_read(fe);
		}
	}
}

void config_write(struct config_phobia *fe)
{
	FILE		*fd;

	fd = fopen_from_UTF8(fe->rcfile, "w");

	if (fd != NULL) {

		fprintf(fd, "serialport %s\n", fe->serialport);
		fprintf(fd, "windowsize %i\n", fe->windowsize);
		fprintf(fd, "storage %s\n", fe->storage);
		fprintf(fd, "fuzzy %s\n", fe->fuzzy);
		fprintf(fd, "tlmrate %i\n", fe->tlmrate);
		fprintf(fd, "regmaxn %i\n", fe->regmaxn);

		fclose(fd);
	}
}

void config_default(struct config_phobia *fe)
{
#ifdef _WINDOWS
	char		lptemp[PHOBIA_PATH_MAX];
#endif /* _WINDOWS */

	fe->serialport[0] = 0;
	fe->windowsize = 1;

#ifdef _WINDOWS
	if (fe->local == 0) {

		GetTempPathA(sizeof(lptemp), lptemp);
		config_ACP_to_UTF8(fe->storage, lptemp, sizeof(fe->storage));
	}
	else {
		fe->storage[0] = 0;
	}
#else /* _WINDOWS */
	strcpy(fe->storage, "/tmp");
#endif

	strcpy(fe->fuzzy, "setpoint");

	fe->tlmrate = 1000;
	fe->regmaxn = 467;
}

