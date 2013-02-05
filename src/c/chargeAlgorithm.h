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

#ifndef CHARGE_ALGORITHM_H
#define CHARGE_ALGORITHM_H

#include <linux/types.h>
#include "config.h"


typedef enum {
	UNDEFINED,
	SOC_ERROR,
	CONSECUTIVE_ERRORS,
	DATA_TIMEOUT,
	OVER_VOLTAGE,
	OVER_BOOT_TEMPERATURE,
	OVER_SHUNT_TEMPERATURE,
	END_OF_CHARGE,
	CHARGE_CURRENT_TOO_LOW,
	SHUNT_WAIT,
	NEED_COMPLETE_DATA_FOR_RESTART,
	LOW_VOLTAGE

} chargerStateChangeReason_t;

void chargeAlgorithm_init(struct config_t *_config);

__u8 chargeAlgorithm_isChargerOn();

const char *chargeAlgorithm_getStateChangeReasonString(chargerStateChangeReason_t reason);

#endif
