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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/if.h>
#include <linux/can/raw.h>

#include <pthread.h>

#include "util.h"
#include "monitor_can.h"

void monitorCan_sendChar2Shorts(const short frameId, const char c, const short s1, const short s2);
void monitorCan_send2Shorts(const short frameId, const short s1, const short s2);
char monitorCan_send(struct can_frame *frame);

/* CAN BUS socket */
int s;

/* Initialisation function, return 0 if successful */
int monitorCan_init() {
	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	struct ifreq ifr;
	strcpy(ifr.ifr_name, "slcan0");
	ioctl(s, SIOCGIFINDEX, &ifr);

	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	bind(s, (struct sockaddr *) &addr, sizeof(addr));

	return 0;
}

void montiorCan_sendCellVoltage(const unsigned char batteryIndex, const short cellIndex, const short vCell) {
	monitorCan_sendChar2Shorts(0x3f0, batteryIndex, cellIndex, vCell);
}

void monitorCan_sendShuntCurrent(const unsigned char batteryIndex, const short cellIndex, const short iShunt) {
	monitorCan_sendChar2Shorts(0x3f1, batteryIndex, cellIndex, iShunt);
}

void monitorCan_sendMinCurrent(const unsigned char batteryIndex, const short cellIndex, const short minCurrent) {
	monitorCan_sendChar2Shorts(0x3f2, batteryIndex, cellIndex, minCurrent);
}

void monitorCan_sendTemperature(const unsigned char batteryIndex, const short cellIndex, const short temperature) {
	monitorCan_sendChar2Shorts(0x3f3, batteryIndex, cellIndex, temperature);
}

void monitorCan_sendChar2Shorts(const short frameId, const char c, const short s1, const short s2) {
	struct can_frame frame;
	memset(&frame, 0, sizeof(struct can_frame)); /* init CAN frame, e.g. DLC = 0 */
	frame.can_id = frameId;
	frame.can_dlc = 5;
	charToBuf(c, frame.data);
	shortToBuf(s1, frame.data + 1);
	shortToBuf(s2, frame.data + 3);
	monitorCan_send(&frame);
}

void monitorCan_send2Shorts(const short frameId, const short s1, const short s2) {
	struct can_frame frame;
	memset(&frame, 0, sizeof(struct can_frame)); /* init CAN frame, e.g. DLC = 0 */
	frame.can_id = frameId;
	frame.can_dlc = 4;
	shortToBuf(s1, frame.data);
	shortToBuf(s2, frame.data + 2);
	monitorCan_send(&frame);
}

/* Returns true if there is an error */
char monitorCan_send(struct can_frame *frame) {
//	fprintf(stderr, "\n");
//	fprint_long_canframe(stderr, frame, "\n", 0);
	int nbytes = write(s, frame, sizeof(struct can_frame));
	if (nbytes != sizeof(struct can_frame)) {
		printf("error writing can frame %d", nbytes);
		fprintf(stderr, "error writing can frame %d", nbytes);
		return 1;
	}
	return 0;
}

