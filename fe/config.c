#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "config.h"

#ifdef _WINDOWS
#define RC_FILE_PMCFE		"_pmcfe"
#else /* _WINDOWS */
#define RC_FILE_PMCFE		".pmcfe"
#endif

static int
config_rcfiletry(struct config_pmcfe *fe)
{
	FILE			*fd;
	int			rc = 0;

	fd = fopen(fe->rcfile, "r");

	if (fd != NULL) {

		rc = 1;

		fclose(fd);
	}

	return rc;
}

void config_read(struct config_pmcfe *fe)
{
	FILE		*fd;
	char		line[1024];

	const char	*name, *str;
	const char	*sep = " \t";

	fd = fopen(fe->rcfile, "r");

	if (fd != NULL) {

		while (fgets(line, sizeof(line), fd) != NULL) {

			name = strtok(line, sep);
			str = strtok(NULL, sep);

			if (		   name == NULL
					|| str == NULL) {

				/* Skip empty lines */
			}
			else if (strcmp(name, "windowsize") == 0) {

				fe->windowsize = strtol(str, NULL, 10);
			}
			else if (strcmp(name, "path") == 0) {

				strcpy(fe->path, str);
			}
		}

		fclose(fd);
	}
}

void config_open(struct config_pmcfe *fe)
{
	char		*path_HOME;

#ifdef _WINDOWS
	path_HOME = getenv("APPDATA");
#else /* _WINDOWS */
	path_HOME = getenv("HOME");
#endif

	if (path_HOME == NULL) {

		path_HOME = ".";
	}

	strcpy(fe->rcfile, path_HOME);
	strcat(fe->rcfile, "/" RC_FILE_PMCFE);

	if (config_rcfiletry(fe) == 0) {

		config_default(fe);
		config_write(fe);
	}

	if (config_rcfiletry(fe) != 0) {

		config_read(fe);
	}
	else {
		strcpy(fe->rcfile, RC_FILE_PMCFE);

		if (config_rcfiletry(fe) == 0) {

			config_default(fe);
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

	fd = fopen(fe->rcfile, "w");

	if (fd != NULL) {

		fprintf(fd, "windowsize %i\n", fe->windowsize);
		fprintf(fd, "path %s\n", fe->path);

		fclose(fd);
	}
}

void config_default(struct config_pmcfe *fe)
{
	fe->windowsize = 1;

#ifdef _WINDOWS
	strcpy(fe->path, "./");
#else /* _WINDOWS */
	strcpy(fe->path, "/tmp");
#endif
}

