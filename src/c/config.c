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

unsigned char parseCell(cfg_t *cfg, struct config_battery_t *battery);

/**
 * Load Configuration from config file
 *
 * @return null if configuration was not loaded
 */
struct config_t *getConfig() {
	cfg_opt_t battery_opts[] = {
			CFG_INT_LIST("cells", 0, CFGF_NODEFAULT),
			CFG_END()
	};
	cfg_opt_t opts[] = {
			CFG_SEC("battery", battery_opts, CFGF_TITLE | CFGF_MULTI),
			CFG_END()
	};

	cfg_t *cfg = cfg_init(opts, CFGF_NONE);

	if (cfg_parse(cfg, "cells.conf") == CFG_PARSE_ERROR) {
		printf("error loading configuration\n");
		exit(1);
		return NULL;
	}

	if (cfg_size(cfg, "battery") > MAX_BATTERIES) {
		fprintf(stderr, "Found %d batteries, maximum supported is %d", cfg_size(cfg, "battery"), MAX_BATTERIES);
		return NULL;
	}

	struct config_t *result = malloc(sizeof(struct config_t));

	result->batteryCount = cfg_size(cfg, "battery");
	result->batteries = malloc(sizeof(struct config_battery_t) * result->batteryCount);
	for (int i = 0; i < cfg_size(cfg, "battery"); i++) {
		if (!parseCell(cfg_getnsec(cfg, "battery", i), &result->batteries[i])) {
			free(result->batteries);
			free(result);
			return NULL;
		}
	}
	return result;
}

unsigned char parseCell(cfg_t *cfg, struct config_battery_t *battery) {
	battery->name = cfg_title(cfg);
	if (cfg_size(cfg, "cells") > MAX_CELLS) {
		fprintf(stderr, "Found %d cells, maximum supported is %d", cfg_size(cfg, "cells"), MAX_CELLS);
		return 0;
	}
	if (!battery) {
		return 0;
	}
	battery->cellCount = cfg_size(cfg, "cells");
	battery->cellIds = malloc(sizeof(unsigned short) * battery->cellCount);
	for (int i = 0; i < battery->cellCount; i++) {
		long int cellId = cfg_getnint(cfg, "cells", i);
		if (cellId > MAX_CELL_ID) {
			fprintf(stderr, "cellId '%ld' at index %d too large must be smaller than %d\n", cellId, i, MAX_CELL_ID);
			return 0;
		}
		battery->cellIds[i] = cellId;
	}
	printf("Battery '%s' has %d cells\n", battery->name, battery->cellCount);
	return 1;
}
