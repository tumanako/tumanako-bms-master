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

#include <stdio.h>
#include <time.h>

#include "soc.h"
#include "canEventListener.h"
#include "monitor_can.h"
#include "chargercontrol.h"
#include "chargeAlgorithm.h"

#define CHARGER_ON_VOLTAGE 3450
#define CHARGER_OFF_VOLTAGE 3650
#define END_OF_CHARGE_VOLTAGE 3450

#define FALSE 0
#define TRUE 1

static struct config_t *config;

static unsigned short minVoltage = 0xffff;
static unsigned short maxVoltage = 0;
static unsigned short maxShuntTemperature = 0;

static time_t whenChargerOn = 0;
static time_t whenLastValid;
static time_t whenTurnedOff = 0;
static time_t whenLastShunting = 0;
static char chargerShutdown = 0;
static char chargerState = 0;
static chargerStateChangeReason_t chargerStateChangeReason = UNDEFINED;
static unsigned short validCount;
static unsigned short invalidCount;
static char errorLastTime = 0;

#define CHARGER_CONTROL_BATTERY_INDEX 1

static void doChargerControl() {
	// do error checking stuff
	if (soc_getError()) {
		fprintf(stderr, "State of Charge error?");
		chargerShutdown = TRUE;
		chargerStateChangeReason = SOC_ERROR;
	}
	if (config->maxBootTemperature) {
		printf("boo");
	}
	if (soc_getT1() > config->maxBootTemperature || soc_getT2() > config->maxBootTemperature) {
		chargerShutdown = TRUE;
		chargerStateChangeReason = OVER_BOOT_TEMPERATURE;
	}
	if (maxShuntTemperature / 1000 > config->maxCellTemperature) {
		printf("max temp %d %d\n", maxShuntTemperature, config->maxCellTemperature);
		chargerShutdown = TRUE;
		chargerStateChangeReason = OVER_SHUNT_TEMPERATURE;
	}
	unsigned short expectedCount = config->batteries[CHARGER_CONTROL_BATTERY_INDEX].cellCount;
	if (validCount + invalidCount != expectedCount) {
		fprintf(stderr, "got %d + %d = %d expected %d\n", validCount, invalidCount, validCount + invalidCount, config->batteries[2].cellCount);
		if (errorLastTime) {
			chargerShutdown = TRUE;
			chargerStateChangeReason = CONSECUTIVE_ERRORS;
		} else {
			errorLastTime = TRUE;
		}
	} else {
		errorLastTime = FALSE;
	}
	time_t now;
	time(&now);
	if (validCount == expectedCount) {
		whenLastValid = now;
	} else if (now - whenLastValid > 150) {
		fprintf(stderr, "no valid data for %ld seconds", now - whenLastValid);
		chargerShutdown = TRUE;
		chargerStateChangeReason = DATA_TIMEOUT;
	}

	__u16 shuntingDelay = 0;
	// do charger control stuff
	if (chargerShutdown) {
		// charging is finished or we had an error
		chargercontrol_setCharger(FALSE);
		chargerState = 0;
	} else if (chargerState == 1) {
		// charger is on, find a reason to turn it off
		chargercontrol_setCharger(TRUE);
		if (maxVoltage > CHARGER_OFF_VOLTAGE) {
			// over voltage, turn off the charger
			chargercontrol_setCharger(FALSE);
			chargerState = 0;
			chargerStateChangeReason = OVER_VOLTAGE;
			whenTurnedOff = now;
		} else if (invalidCount == 0 && minVoltage > END_OF_CHARGE_VOLTAGE && soc_getCurrent() > -4) {
			// charging is finished
			chargercontrol_setCharger(FALSE);
			chargerState = 0;
			chargerStateChangeReason = END_OF_CHARGE;
			chargerShutdown = TRUE;
		} else if (
				(soc_getCurrent() > -3 && maxVoltage > 3500) ||
				(soc_getCurrent() > -2 && maxVoltage > 3450) ||
				(soc_getCurrent() > -1 && maxVoltage > 3400)
				) {
			// charging current is too low
			// did we just just turn it on?
			if (now - whenChargerOn > 10) {
				chargercontrol_setCharger(FALSE);
				chargerState = 0;
				chargerStateChangeReason = CHARGE_CURRENT_TOO_LOW;
				whenTurnedOff = now;
			}
		}
	} else {
		// charger is off, find a reason to turn it on
		chargercontrol_setCharger(FALSE);
		fprintf(stderr, "shunt delay %ld %ld %d %ld\n",whenTurnedOff, now, whenTurnedOff + 10 * 60 > now, whenTurnedOff + 10 * 60 - now);
		if (whenTurnedOff == 0) {
			shuntingDelay = 0;
		} else if (whenTurnedOff + 30 * 60 > now) {
			shuntingDelay = whenTurnedOff + 30 * 60 - now;
		} else {
			shuntingDelay = 0;
		}
		if (shuntingDelay > 0) {
			chargerStateChangeReason = SHUNT_WAIT;
		} else if (invalidCount != 0) {
			// we didn't get voltages for every cell, can't use this pass to turn on the charger
			chargerStateChangeReason = NEED_COMPLETE_DATA_FOR_RESTART;
		} else {
			if (maxVoltage < CHARGER_ON_VOLTAGE) {
				// battery voltage is low, charger should switch on
				chargercontrol_setCharger(TRUE);
				chargerState = 1;
				chargerStateChangeReason = LOW_VOLTAGE;
				whenChargerOn = now;
			}
		}
	}

	// decide whether to go to sleep
	if (chargerShutdown) {
		if (whenLastShunting == 0) {
			// first time through this code, initialise
			whenLastShunting = now;
		}
		if (now - whenLastShunting > 120) {
			config->loopDelay = 300;
		}
	}

	fprintf(stderr, "chargerState %d %d %s %d\n", chargerShutdown, chargerState,
			chargeAlgorithm_getStateChangeReasonString(chargerStateChangeReason), shuntingDelay);
	monitorCan_sendChargerState(chargerShutdown, chargerState, chargerStateChangeReason, shuntingDelay);
}

static void voltageListener(unsigned char batteryIndex, unsigned short cellIndex, unsigned char isValid, unsigned short voltage) {
	fprintf(stderr, "charger voltageListener %d %d %d %d\n", batteryIndex, cellIndex, isValid, voltage);
	// we only control the charger in one battery
	if (batteryIndex != CHARGER_CONTROL_BATTERY_INDEX) {
		return;
	}
	if (!isValid) {
		invalidCount++;
	} else {
		validCount++;
		if (voltage > maxVoltage) {
			maxVoltage = voltage;
		}
		if (voltage < minVoltage) {
			minVoltage = voltage;
		}
	}
	if (cellIndex == config->batteries[CHARGER_CONTROL_BATTERY_INDEX].cellCount - 1) {
		fprintf(stderr, "doing charge control %d %d\n", minVoltage, maxVoltage);
		doChargerControl();
		maxVoltage = 0;
		minVoltage = 0xffff;
		validCount = 0;
		invalidCount = 0;
	}
}

static void minCurrentListener(unsigned char batteryIndex, unsigned short cellIndex,
		unsigned short minCurrent) {
	if (minCurrent > 0) {
		// have to avoid compile error by accessing BatteryIndex and CellIndex, is there a bettery way?
		fprintf(stderr, "minCurrent %d %d", batteryIndex, cellIndex);
		time(&whenLastShunting);
	}
}

static void temperatureListener(unsigned char batteryIndex, unsigned short cellIndex, unsigned short temperature) {
	if (temperature > maxShuntTemperature) {
		fprintf(stderr, "max temperature at %d %d: %d", batteryIndex, cellIndex, temperature);
		maxShuntTemperature = temperature;
	}
}

void chargeAlgorithm_init(struct config_t *_config) {
	config = _config;
	if (config->loopDelay > 20) {
		chargerShutdown = 1;
	} else {
		canEventListener_registerVoltageListener(voltageListener);
		canEventListener_registerTemperatureListener(temperatureListener);
		canEventListener_registerMinCurrentListener(minCurrentListener);
		time(&whenLastValid);
	}
}

__u8 chargeAlgorithm_isChargerOn() {
	return chargerState;
}

const char *chargeAlgorithm_getStateChangeReasonString(chargerStateChangeReason_t reason) {
	switch (reason) {
	case UNDEFINED :
		return "undefined";
	case SOC_ERROR :
		return "SOC error";
	case CONSECUTIVE_ERRORS :
		return "Consecutive Errors";
	case DATA_TIMEOUT :
		return "Data Timeout";
	case OVER_VOLTAGE :
		return "Over Voltage";
	case OVER_SHUNT_TEMPERATURE :
		return "Over Shunt Temperature";
	case OVER_BOOT_TEMPERATURE :
		return "Over Boot Temperature";
	case END_OF_CHARGE :
		return "End of Charge";
	case CHARGE_CURRENT_TOO_LOW :
		return "Current too low";
	case SHUNT_WAIT :
		return "Shunting";
	case NEED_COMPLETE_DATA_FOR_RESTART :
		return "Incomplete Data";
	case LOW_VOLTAGE :
		return "Low Voltage";
	}
	return "unknown?";
}
