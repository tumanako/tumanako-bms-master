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
#include "soc.h"
#include "canEventListener.h"
#include "monitor_can.h"
#include "chargercontrol.h"
#include "chargeAlgorithm.h"

#define CHARGER_ON_VOLTAGE 3450
#define CHARGER_OFF_VOLTAGE 3650
#define END_OF_CHARGE_VOLTAGE 3500

#define FALSE 0
#define TRUE 1

static struct config_t *config;

static unsigned short minVoltage = 0xffff;
static unsigned short maxVoltage = 0;

static time_t whenChargerOn = 0;
static char chargerShutdown = 0;
static char chargerState = 0;
static char chargerStateChangeReason = 0;
static unsigned short count;
static char errorLastTime = 0;

static void doChargerControl() {
	// do error checking stuff
	if (soc_getError()) {
		fprintf(stderr, "State of Charge error?");
		chargerShutdown = TRUE;
		chargerStateChangeReason = 5;
	}
	if (count != config->batteries[2].cellCount) {
		if (errorLastTime) {
			fprintf(stderr, "State of Charge error?");
			chargerShutdown = TRUE;
			chargerStateChangeReason = 6;
		} else {
			errorLastTime = TRUE;
		}
	}

	// do charger control stuff
	if (chargerShutdown) {
		// charging is finished or we had an error
		chargercontrol_setCharger(FALSE);
		chargerState = 0;
	} else if (chargerState == 1) {
		chargercontrol_setCharger(TRUE);
		// charger is on, find a reason to turn it off
		if (maxVoltage > CHARGER_OFF_VOLTAGE) {
			// over voltage, turn off the charger
			chargercontrol_setCharger(FALSE);
			chargerState = 0;
			chargerStateChangeReason = 1;
		} else if (minVoltage > END_OF_CHARGE_VOLTAGE && soc_getCurrent() > -4) {
			// charging is finished
			chargercontrol_setCharger(FALSE);
			chargerState = 0;
			chargerStateChangeReason = 2;
			chargerShutdown = TRUE;
		} else if (soc_getCurrent() > -3) {
			// charging current is too low
			time_t now;
			time(&now);
			// did we just just turn it on?
			if (now - whenChargerOn > 10) {
				chargercontrol_setCharger(FALSE);
				chargerState = 0;
				chargerStateChangeReason = 3;
			}
		}
	} else {
		chargercontrol_setCharger(FALSE);
		// charger is off, find a reason to turn it on
		if (maxVoltage < CHARGER_ON_VOLTAGE) {
			// battery voltage is low, charger should switch on
			chargercontrol_setCharger(TRUE);
			chargerState = 1;
			chargerStateChangeReason = 4;
			time(&whenChargerOn);
		}
	}
	fprintf(stderr, "chargerState %d %d %d\n", chargerShutdown, chargerState, chargerStateChangeReason);
	monitorCan_sendChargerState(chargerShutdown, chargerState, chargerStateChangeReason);
}

static void voltageListener(unsigned char batteryIndex, unsigned short cellIndex, unsigned short voltage) {
	fprintf(stderr, "charger voltageLisenter %d\n", batteryIndex);
	// we only control the charger in battery 2
	if (batteryIndex != 2) {
		return;
	}
	if (voltage > maxVoltage) {
		maxVoltage = voltage;
	}
	if (voltage < minVoltage) {
		minVoltage = voltage;
	}
	count++;
	if (cellIndex == config->batteries[2].cellCount - 1) {
		fprintf(stderr, "doing charge control %d %d\n", minVoltage, maxVoltage);
		doChargerControl();
		maxVoltage = 0;
		minVoltage = 0xffff;
		count = 0;
	}
}

void chargeAlgorithm_init(struct config_t *_config) {
	config = _config;
	canEventListener_registerVoltageListener(voltageListener);
}
