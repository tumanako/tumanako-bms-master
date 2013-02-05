/*
 Copyright 2013 Tom Parker

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

#ifndef SHUNT_ALGORITHM_H
#define SHUNT_ALGORITHM_H

#include "linux/types.h"
#include "monitor.h"

extern __u8 shuntAlgorithm_shouldCellShunt(struct status_t *cell, __u16 minVoltage, double current,
		monitor_mode_t mode, __u8 isChargerOn);

// TODO do something better about unittesting
extern void testIsCellVoltageRelevant();
#endif
