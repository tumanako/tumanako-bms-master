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
#include <stdio.h>
#include <stdlib.h>

#include "monitor.h"
#include "shuntAlgorithm.h"

#define FALSE 0
#define TRUE 1

static __u8 isCellVoltageSocRelevant(__u16 voltage, double current) {
	if (
			(voltage < 3450) ||
			(voltage < 3500 && current <= -5) ||
			(voltage < 3600 && current <= -10) ||
			(current < -10)
			) {
		return FALSE;
	}
	return TRUE;
}

__u8 shuntAlgorithm_shouldCellShunt(struct status_t *cell, __u16 minVoltage, double current,
		monitor_mode_t mode, __u8 isChargerOn) {
	if (mode != MONITOR_MODE_CHARGING) {
		return FALSE;
	}
	if (!cell->isResistorShunt && !cell->isKelvinConnection && isChargerOn) {
		// transistor shunts with non-kelvin connection are slow to respond
		// this reduces our ability to monitor voltage so we do not want them on
		// while charging
		return FALSE;
	}
	if (!isCellVoltageSocRelevant(cell->vCell, current)) {
		return FALSE;
	}
	__u16 difference = cell->vCell - minVoltage;
	if (difference < 25 ||
			(difference < 50 && current < -3) ||
			(difference < 75 && current < -5) ||
			(difference < 125 && current < -7) ||
			(difference < 150 && current < -10)
			) {
		return FALSE;
	}
	return TRUE;
}

static void assertCellVoltageSocRelevant(__u8 expected, __u16 voltage, double current) {
	__u8 actual = isCellVoltageSocRelevant(voltage, current);
	if (expected != actual) {
		printf("%dV %fA expected %d actual %d\n", voltage, current, expected, actual);
		abort();
	}
}

void testIsCellVoltageRelevant() {
	assertCellVoltageSocRelevant(FALSE, 3300, 0);
	assertCellVoltageSocRelevant(FALSE, 3300, -1);
	assertCellVoltageSocRelevant(FALSE, 3300, -2);
	assertCellVoltageSocRelevant(FALSE, 3300, -4.9);
	assertCellVoltageSocRelevant(FALSE, 3300, -5);
	assertCellVoltageSocRelevant(FALSE, 3300, -5.1);
	assertCellVoltageSocRelevant(FALSE, 3300, -9.9);
	assertCellVoltageSocRelevant(FALSE, 3300, -10);

	assertCellVoltageSocRelevant(FALSE, 3449, 0);
	assertCellVoltageSocRelevant(FALSE, 3449, -1);
	assertCellVoltageSocRelevant(FALSE, 3449, -2);
	assertCellVoltageSocRelevant(FALSE, 3449, -4.9);
	assertCellVoltageSocRelevant(FALSE, 3449, -5);
	assertCellVoltageSocRelevant(FALSE, 3449, -5.1);
	assertCellVoltageSocRelevant(FALSE, 3449, -9.9);
	assertCellVoltageSocRelevant(FALSE, 3449, -10);

	assertCellVoltageSocRelevant(TRUE, 3450, 0);
	assertCellVoltageSocRelevant(TRUE, 3450, -1);
	assertCellVoltageSocRelevant(TRUE, 3450, -2);
	assertCellVoltageSocRelevant(TRUE, 3450, -4.9);
	assertCellVoltageSocRelevant(FALSE, 3450, -5);
	assertCellVoltageSocRelevant(FALSE, 3450, -5.1);
	assertCellVoltageSocRelevant(FALSE, 3450, -9.9);
	assertCellVoltageSocRelevant(FALSE, 3450, -10);

	assertCellVoltageSocRelevant(TRUE, 3500, 0);
	assertCellVoltageSocRelevant(TRUE, 3500, -1);
	assertCellVoltageSocRelevant(TRUE, 3500, -2);
	assertCellVoltageSocRelevant(TRUE, 3500, -4.9);
	assertCellVoltageSocRelevant(TRUE, 3500, -5);
	assertCellVoltageSocRelevant(TRUE, 3500, -5.1);
	assertCellVoltageSocRelevant(TRUE, 3500, -9.9);
	assertCellVoltageSocRelevant(FALSE, 3500, -10);

	assertCellVoltageSocRelevant(TRUE, 3600, 0);
	assertCellVoltageSocRelevant(TRUE, 3600, -1);
	assertCellVoltageSocRelevant(TRUE, 3600, -2);
	assertCellVoltageSocRelevant(TRUE, 3600, -4.9);
	assertCellVoltageSocRelevant(TRUE, 3600, -5);
	assertCellVoltageSocRelevant(TRUE, 3600, -5.1);
	assertCellVoltageSocRelevant(TRUE, 3600, -9.9);
	assertCellVoltageSocRelevant(TRUE, 3600, -10);
	assertCellVoltageSocRelevant(FALSE, 3600, -11);
}
