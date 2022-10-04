#ifndef _H_CONFIG_
#define _H_CONFIG_

#include <stdlib.h>
#include <stdio.h>

#define FILE_HOME_CONFIG	"pmcfe"
#define FILE_LINK_LOG		"pmcfe.log"
#define FILE_GRAB_FORMAT	"grab_%d.csv"
#define FILE_GRAB_SUFFIX	".csv"

#define PMCFE_PATH_MAX		270
#define PMCFE_NAME_MAX		80

struct config_pmcfe {

	char			rcfile[PMCFE_PATH_MAX];

	int			windowsize;
	char			storage[PMCFE_PATH_MAX];
	char			fuzzy[PMCFE_NAME_MAX];
	char			gpcmd[PMCFE_NAME_MAX];
};

FILE *fopen_from_UTF8(const char *file, const char *mode);

void config_open(struct config_pmcfe *fe);
void config_write(struct config_pmcfe *fe);
void config_default(struct config_pmcfe *fe);

#endif /* _H_CONFIG_ */

