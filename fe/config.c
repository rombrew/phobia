#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _WINDOWS
#include <windows.h>
#endif /* _WINDOWS */

#include "config.h"
#include "dirent.h"

#ifdef _WINDOWS
#define FILE_HIDDEN_CONFIG	"_" FILE_HOME_CONFIG
#else /* _WINDOWS */
#define FILE_HIDDEN_CONFIG	"." FILE_HOME_CONFIG
#endif

static int
config_rcfiletry(struct config_pmcfe *fe)
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

void config_read(struct config_pmcfe *fe)
{
	FILE		*fd;
	char		line[PMCFE_PATH_MAX];

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
			else if (strcmp(name, "windowsize") == 0) {

				fe->windowsize = strtol(value, NULL, 10);
			}
			else if (strcmp(name, "storage") == 0) {

				strcpy(fe->storage, value);
			}
			else if (strcmp(name, "fuzzy") == 0) {

				strcpy(fe->fuzzy, value);
			}
			else if (strcmp(name, "gpcmd") == 0) {

				strcpy(fe->gpcmd, value);
			}
		}

		fclose(fd);
	}
}

void config_open(struct config_pmcfe *fe)
{
	char		*path_HOME;

	config_default(fe);

#ifdef _WINDOWS
	path_HOME = getenv("APPDATA");
#else /* _WINDOWS */
	path_HOME = getenv("HOME");
#endif

	if (path_HOME == NULL) {

		path_HOME = ".";
	}

#ifdef _WINDOWS
	legacy_ACP_to_UTF8(fe->rcfile, path_HOME, sizeof(fe->rcfile));
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
		strcpy(fe->rcfile, FILE_HIDDEN_CONFIG);

		if (config_rcfiletry(fe) == 0) {

			config_write(fe);
		}

		if (config_rcfiletry(fe) != 0) {

			config_read(fe);
		}
	}
}

void config_write(struct config_pmcfe *fe)
{
	FILE		*fd;

	fd = fopen_from_UTF8(fe->rcfile, "w");

	if (fd != NULL) {

		fprintf(fd, "windowsize %i\n", fe->windowsize);
		fprintf(fd, "storage %s\n", fe->storage);
		fprintf(fd, "fuzzy %s\n", fe->fuzzy);
		fprintf(fd, "gpcmd %s\n", fe->gpcmd);

		fclose(fd);
	}
}

void config_default(struct config_pmcfe *fe)
{
#ifdef _WINDOWS
	char		lptemp[PMCFE_PATH_MAX];
#endif /* _WINDOWS */

	fe->windowsize = 1;

#ifdef _WINDOWS
	GetTempPathA(sizeof(lptemp), lptemp);
	legacy_ACP_to_UTF8(fe->storage, lptemp, sizeof(fe->storage));
#else /* _WINDOWS */
	strcpy(fe->storage, "/tmp");
#endif

	strcpy(fe->fuzzy, "setpoint");
	strcpy(fe->gpcmd, "gp");
}

