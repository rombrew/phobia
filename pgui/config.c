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

			if (name == NULL)
				goto config_read_SKIP;

			if (value == NULL)
				value = "";

			if (strcmp(name, "version") == 0) {

				fe->version = strtol(value, NULL, 10);
			}
			else if (strcmp(name, "serialport") == 0) {

				strcpy(fe->serialport, value);
			}
			else if (strcmp(name, "baudrate") == 0) {

				fe->baudrate = strtol(value, NULL, 10);
			}
			else if (strcmp(name, "parity") == 0) {

				fe->parity = strtol(value, NULL, 10);
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
			else if (strcmp(name, "regfile") == 0) {

				fe->regfile = strtol(value, NULL, 10);
			}
config_read_SKIP:

		}

		fclose(fd);
	}
}

void config_open(struct config_phobia *fe)
{
	char		*path_HOME;

#ifdef _WINDOWS
	path_HOME = getenv("APPDATA");
#else /* _WINDOWS */
	path_HOME = getenv("HOME");
#endif

	if (path_HOME == NULL) {

		fe->local = 1;
	}

#ifdef _LOCAL_PGUI
	fe->local = 1;
#endif /* _LOCAL_PGUI */

	if (fe->local == 0) {

#ifdef _WINDOWS
		config_ACP_to_UTF8(fe->rcfile, path_HOME, sizeof(fe->rcfile));
#else /* _WINDOWS */
		strcpy(fe->rcfile, path_HOME);
#endif
		strcat(fe->rcfile, DIRSEP FILE_HIDDEN_CONFIG);

		config_default(fe);

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

	if (fe->version != CONFIG_VERSION) {

		if (config_rcfiletry(fe) != 0) {

			fe->version = CONFIG_VERSION;

			config_write(fe);
			config_read(fe);
		}
	}
}

void config_write(struct config_phobia *fe)
{
	FILE		*fd;

	fd = fopen_from_UTF8(fe->rcfile, "w");

	if (fd != NULL) {

		fprintf(fd, "version %i\n", fe->version);
		fprintf(fd, "serialport %s\n", fe->serialport);
		fprintf(fd, "baudrate %i\n", fe->baudrate);
		fprintf(fd, "parity %i\n", fe->parity);
		fprintf(fd, "windowsize %i\n", fe->windowsize);
		fprintf(fd, "storage %s\n", fe->storage);
		fprintf(fd, "fuzzy %s\n", fe->fuzzy);
		fprintf(fd, "regfile %i\n", fe->regfile);

		fclose(fd);
	}
}

void config_default(struct config_phobia *fe)
{
#ifdef _WINDOWS
	char		lptemp[PHOBIA_PATH_MAX];
#endif /* _WINDOWS */

	fe->version = CONFIG_VERSION;

	strcpy(fe->serialport, "none");

	fe->baudrate = 2;
	fe->parity = 1;

	fe->windowsize = 1;

	if (fe->local == 0) {
#ifdef _WINDOWS
		GetTempPathA(sizeof(lptemp), lptemp);
		config_ACP_to_UTF8(fe->storage, lptemp, sizeof(fe->storage));
#else /* _WINDOWS */
		strcpy(fe->storage, "/tmp");
#endif
	}
	else {
		fe->storage[0] = 0;
	}

	strcpy(fe->fuzzy, "setpoint");

	fe->regfile = 500;
}

void config_storage_path(struct config_phobia *fe, char *lbuf, const char *file)
{
	lbuf[0] = 0;

	if (fe->storage[0] != 0) {

		strcat(lbuf, fe->storage);
		strcat(lbuf, DIRSEP);
	}

	strcat(lbuf, file);
}

