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

#include "soc.h"
#include "util.h"
#include "config.h"

extern int readFrame(int s, struct can_frame *frame);
void *console_backgroundThread(void *ptr);

int console_init(struct config_t *config) {
	pthread_t consoleThread;
	pthread_create(&consoleThread, NULL, console_backgroundThread, (void *) config);

	return 0;
}

void moveCursor(unsigned char x, unsigned char y) {
	char buf[20];
	sprintf(buf, "\E[%d;%df", y, x);
	write(2, buf, strlen(buf));
}

void moveToCell(unsigned char batteryIndex, unsigned short cellIndex, unsigned char offset) {
	unsigned char x;
	if (cellIndex % 2) {
		x = 45;
	} else {
		x = 1;
	}
	moveCursor(x + offset, cellIndex / 2 + 1);
}

/* Decode a voltage frame. */
void console_decode3f0(struct can_frame *frame, struct config_t *config) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	moveToCell(batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3d ", cellIndex);
	fflush(stdout);
	unsigned short voltage = bufToShort(frame->data + 3);
	moveToCell(batteryIndex, cellIndex, 4);
	fprintf(stdout, "Vc=%.3f ", milliToDouble(voltage));
	fflush(stdout);
}

/* Decode a shunt current frame. */
void console_decode3f1(struct can_frame *frame, struct config_t *config) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	moveToCell(batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3d ", cellIndex);
	fflush(stdout);
	unsigned short shuntCurrent = bufToShort(frame->data + 3);
	moveToCell(batteryIndex, cellIndex, 13);
	printf("Is=%.3f ", milliToDouble(shuntCurrent));
	fflush(stdout);
}

/* Decode a minCurrent frame. */
void console_decode3f2(struct can_frame *frame, struct config_t *config) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	moveToCell(batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3d ", cellIndex);
	fflush(stdout);
	unsigned short minCurrent = bufToShort(frame->data + 3);
	moveToCell(batteryIndex, cellIndex, 22);
	fprintf(stdout, "It=%.3f ", milliToDouble(minCurrent));
	fflush(stdout);
}

/* Decode a temperature frame. */
void console_decode3f3(struct can_frame *frame, struct config_t *config) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	moveToCell(batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3d ", cellIndex);
	fflush(stdout);
	unsigned char temperature = bufToShort(frame->data + 3);
	moveToCell(batteryIndex, cellIndex, 31);
	fprintf(stdout, "t=%4.1f ", milliToDouble(temperature) * 100);
	fflush(stdout);
}

void *console_backgroundThread(void *ptr) {
	struct config_t *config = (struct config_t *) ptr;

	struct can_frame frame;

	int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	struct ifreq ifr;
	strcpy(ifr.ifr_name, "slcan0");
	ioctl(s, SIOCGIFINDEX, &ifr);

	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	bind(s, (struct sockaddr *) &addr, sizeof(addr));

	while (1) {
		if (readFrame(s, &frame)) {
			return NULL;
		}
		if (frame.can_id == 0x3f0) {
			console_decode3f0(&frame, config);
		} else if (frame.can_id == 0x3f1) {
			console_decode3f1(&frame, config);
		} else if (frame.can_id == 0x3f2) {
			console_decode3f2(&frame, config);
		} else if (frame.can_id == 0x3f3) {
			console_decode3f3(&frame, config);
		} else {
			continue;
		}
	}
	return NULL;
}
