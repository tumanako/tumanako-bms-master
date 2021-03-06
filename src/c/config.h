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

#ifndef CONFIG_H
#define CONFIG_H

#define MAX_BATTERIES 10
#define MAX_CELLS 1024
#define MAX_CELL_ID 0xffff

struct config_battery_t {
	const char *name;
	unsigned short cellCount;
	unsigned short *cellIds;
};

struct config_t {
	const char *serialPort;
	unsigned short loopDelay;
	unsigned short minVoltageSocRelevant;
	unsigned short voltageDeadband;
	unsigned short minShuntCurrent;
	unsigned short maxBootTemperature;
	unsigned short maxCellTemperature;
	unsigned char batteryCount;
	struct config_battery_t *batteries;
};

extern struct config_t *getConfig();

#endif
