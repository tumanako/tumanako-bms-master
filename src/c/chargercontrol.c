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
#include <linux/types.h>
#include "chargercontrol.h"
#include "chargercontrol_labjack.h"

static __u8 useLabjack;

int chargercontrol_init() {
	if (chargercontrol_labjack_init() == 0) {
		useLabjack = 1;
		printf("Using labjack to control charger\n");
	}
	return 0;
}

int buscontrol_init() {
	// don't need to do any additional setup
	return 0;
}

double chargercontrol_getChargeCurrent() {
	if (useLabjack) {
		return chargercontrol_labjack_getChargeCurrent();
	}
	return 0;
}

void chargercontrol_setCharger(char on) {
	if (useLabjack) {
		chargercontrol_labjack_setCharger(on);
	}
}

void chargercontrol_shutdown() {
	if (useLabjack) {
		chargercontrol_labjack_shutdown();
	}
}

void buscontrol_setBus(char on) {
	if (useLabjack) {
		buscontrol_labjack_setBus(on);
	}
}
