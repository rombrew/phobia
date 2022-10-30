#ifndef _H_CONFIG_
#define _H_CONFIG_

#include <stdlib.h>
#include <stdio.h>

#define FILE_HOME_CONFIG	"pmcfe"
#define FILE_LINK_LOG		"pmclink.log"
#define FILE_TELEMETRY_GRAB	"tlmgrab.csv"
#define FILE_TELEMETRY_FLT	".csv"
#define FILE_CONFIG_DEFAULT	"config.txt"
#define FILE_CONFIG_FLT		".txt"

#define PMCFE_PATH_MAX		400
#define PMCFE_NAME_MAX		80

struct config_pmcfe {

	char			rcfile[PMCFE_PATH_MAX];

	int			windowsize;
	char			storage[PMCFE_PATH_MAX];
	char			fuzzy[PMCFE_NAME_MAX];
};

FILE *fopen_from_UTF8(const char *file, const char *mode);

void config_open(struct config_pmcfe *fe);
void config_write(struct config_pmcfe *fe);
void config_default(struct config_pmcfe *fe);

#endif /* _H_CONFIG_ */

