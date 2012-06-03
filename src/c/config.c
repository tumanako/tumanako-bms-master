/*
 Copyright 2012 Tom Parker

 This file is part of the Tumanako EVD5 BMS.

 The Tumanako EVD5 BMS is free software: you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public License as
 published by the Free Software Foundation, either version 3 of the License,
 or (at your option) any later version.

 The Tumanako EVD5 BMS is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with the Tumanako EVD5 BMS.  If not, see
 <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <confuse.h>

#include "config.h"
#include "monitor.h"

/**
 * Load Configuration from config file
 *
 * @return null if configuration was not loaded
 */
struct config_t *getConfig() {
	cfg_opt_t opts[] = {
			CFG_INT_LIST("cells", 0, CFGF_NODEFAULT),
			CFG_END()
	};

	cfg_t *cfg = cfg_init(opts, CFGF_NONE);

	if (cfg_parse(cfg, "cells.conf") == CFG_PARSE_ERROR) {
		printf("error loading configuration\n");
		exit(1);
		return NULL;
	}

	if (cfg_size(cfg, "cells") > 512) {
		fprintf(stderr, "Found %d cells, maximum supported is %d", cfg_size(cfg, "cells"), 512);
		return NULL;
	}
	struct config_t *result = malloc(sizeof(struct config_t));
	if (!result) {
		return NULL;
	}
	result->cellCount = cfg_size(cfg, "cells");
	result->cellIds = malloc(sizeof(unsigned short) * result->cellCount);
	for (int i = 0; i < result->cellCount; i++) {
		long int cellId = cfg_getnint(cfg, "cells", i);
		if (cellId > 0xffffffff) {
			fprintf(stderr, "cellId '%d' at index %d too large must be smaller than %d\n", i, cellId, 0xffffffff);
			return NULL;
		}
		result->cellIds[i] = cellId;
	}
	printf("Loaded config got %d cells\n", result->cellCount);
	return result;
}
