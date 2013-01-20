/*
 Copyright 2011 Tom Parker

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
#ifndef TUMANAKO_CAN_H_
#define TUMANAKO_CAN_H_

#include <net/if.h>
#include <linux/can.h>
#include "monitor.h"

int monitorCan_init();

void montiorCan_sendCellVoltage(const unsigned char batteryIndex, const short cellIndex, const unsigned char isValid, const short vCell);
void monitorCan_sendShuntCurrent(const unsigned char batteryIndex, const short cellIndex, const short iShunt);
void monitorCan_sendMinCurrent(const unsigned char batteryIndex, const short cellIndex, const short minCurrent);
void monitorCan_sendTemperature(const unsigned char batteryIndex, const short cellIndex, const short temperature);
void monitorCan_sendHardware(const unsigned char batteryIndex, const short cellIndex,
		const unsigned char hasKelvinConnection, const unsigned char hasResistorShunt,
		const unsigned char hasTemperatureSensor, const unsigned short revision, const unsigned char isClean);
void monitorCan_sendError(const unsigned char batteryIndex, const short cellIndex, const short errorCount);
void monitorCan_sendLatency(const unsigned char batteryIndex, const short cellIndex, const unsigned char latency);
void monitorCan_sendChargerState(const unsigned char shutdown, const unsigned char state, const unsigned char reason);
void monitorCan_sendMonitorState(const monitor_state_t state, const __u16 delay, const __u8 loopsBeforeVoltage);

#endif /* TUMANAKO_CAN_H_ */
