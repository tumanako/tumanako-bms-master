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
 * @return 0 if configuration successfully loaded, 1 otherwise
 */
unsigned char initConfig() {
	cfg_opt_t opts[] = {
			CFG_INT_LIST("cells", 0, CFGF_NODEFAULT),
			CFG_END()
	};

	cfg_t *cfg = cfg_init(opts, CFGF_NONE);

	if (cfg_parse(cfg, "cells.conf") == CFG_PARSE_ERROR) {
		printf("error loading configuration\n");
		exit(1);
		return 1;
	}

	if (cfg_size(cfg, "cells") > 512) {
		fprintf(stderr, "Found %d cells, maximum supported is %d", cfg_size(cfg, "cells"), 512);
		return 1;
	}
	cellCount = cfg_size(cfg, "cells");
	cells = calloc(sizeof(struct status_t), cellCount);
	for (int i = 0; i < cellCount; i++) {
		long int cellId = cfg_getnint(cfg, "cells", i);
		if (cellId > 0xffffffff) {
			fprintf(stderr, "cellId '%d' at index %d too large must be smaller than %d\n", i, cellId, 0xffffffff);
			return 0;
		}
		cells[i].cellId = cellId;
	}
	printf("Loaded config got %d cells\n", cfg_size(cfg, "cells"));
	return 0;
}
