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

#ifndef CAN_EVENT_LISTENER_H
#define CAN_EVENT_LISTENER_H

#include <sys/socket.h>
#include <linux/can.h>

#include "config.h"
#include "monitor.h"

extern void canEventListener_init(struct config_t *_config);
extern void canEventListener_registerVoltageListener(void (*voltageListener)(unsigned char, unsigned short, unsigned char, unsigned short));
extern void canEventListener_registerShuntCurrentListener(void (*shuntCurrentListener)(unsigned char, unsigned short, unsigned short));
extern void canEventListener_registerMinCurrentListener(void (*minCurrentListener)(unsigned char, unsigned short, unsigned short));
extern void canEventListener_registerTemperatureListener(void (*temperatureListener)(unsigned char, unsigned short, unsigned short));
extern void canEventListener_registerCellConfigListener(void (*cellConfigListener)(unsigned char, unsigned short, unsigned short, unsigned char));
extern void canEventListener_registerErrorListener(void (*cellConfigListener)(unsigned char, unsigned short, unsigned short));
extern void canEventListener_registerLatencyListener(void (*cellConfigListener)(unsigned char, unsigned short, unsigned char));
extern void canEventListener_registerChargerStateListener(void (*chargerStateListener)(unsigned char, unsigned char, unsigned char, __u16));
extern void canEventListener_registerMonitorStateListener(void (*monitorStateListener)(monitor_state_t, __u16, __u8));
extern void canEventListener_registerRawCanListener(void (*rawCanListener)(struct can_frame *frame));

extern volatile char canEventListener_error;

#endif
