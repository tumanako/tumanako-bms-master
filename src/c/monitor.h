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

#ifndef TUMANAKO_MONITOR_H_
#define TUMANAKO_MONITOR_H_

struct status_t {
	struct battery_t *battery;
	unsigned short cellIndex;
	unsigned short cellId;
	unsigned short iShunt;
	unsigned short vCell;
	unsigned short vShunt;
	unsigned short temperature;
	unsigned short minCurrent;
	unsigned char sequenceNumber;
	// lower numbers == less gain
	char gainPot;
	// lower numbers == less voltage
	char vShuntPot;
	// true if we have received a character since the last time loopCounter overflowed
	unsigned char hasRx;
	// true if we are doing software addressing
	unsigned char softwareAddressing;
	// true if we are controlling the shunt current automatically
	unsigned char automatic;
	unsigned short crc;
	// target current (what we last sent to the cell)
	unsigned short targetShuntCurrent;
	// microseconds required to acquire last reading
	unsigned long latency;
	char version;
	unsigned char isKelvinConnection;
	unsigned char isResistorShunt;
	unsigned char isHardSwitchedShunt;
	unsigned char hasTemperatureSensor;
	unsigned short revision;
	unsigned char isClean;
	unsigned long whenProgrammed;
	unsigned short errorCount;
	// true if we have current data for this cell
	unsigned short isDataCurrent;
};

struct battery_t {
	unsigned char batteryIndex;
	const char *name;
	unsigned short cellCount;
	struct status_t *cells;
};

struct monitor_t {
	unsigned char batteryCount;
	struct battery_t *batteries;
};

typedef enum {
	START,
	SLEEPING,
	WAKE_SLAVE,
	TURN_OFF_NON_KELVIN_TRANSISTOR,
	WAIT_FOR_VOLTAGE_READING,
	READ_VOLTAGE,
	TURN_ON_SHUNTS,
	WAIT_FOR_SHUNT_CURRENT,
	READ_CURRENT,
} monitor_state_t;

const char *monitor_getStateString(monitor_state_t reason);

#endif
