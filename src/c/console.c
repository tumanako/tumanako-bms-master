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

static unsigned short maxVoltage = 0;
static unsigned short maxVoltageCell;
static unsigned short minVoltage = 0xffff;
static unsigned short minVoltageCell;
static unsigned long totalVoltage = 0;

int console_init(struct config_t *config) {
	pthread_t consoleThread;
	pthread_create(&consoleThread, NULL, console_backgroundThread, (void *) config);

	return 0;
}

static double asDouble(int s) {
	return ((double) s) / 1000;
}

void moveCursor(unsigned char x, unsigned char y) {
	char buf[20];
	sprintf(buf, "\E[%d;%df", y, x);
	write(1, buf, strlen(buf));
}

void moveToCell(struct config_t *config, unsigned char batteryIndex, unsigned short cellIndex, unsigned char offset) {
	unsigned char x;
	if (cellIndex % 2) {
		x = 88;
	} else {
		x = 1;
	}
	unsigned char batteryOffset = 1;
	for (int i = 0; i < batteryIndex; i++) {
		unsigned short cellCount = config->batteries[i].cellCount;
		unsigned char lines = cellCount / 2;
		if (cellCount % 2) {
			lines++;
		}
		batteryOffset += lines + 1;
	}
	moveCursor(x + offset, batteryOffset + cellIndex / 2);
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
	moveToCell(config, batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3d ", cellIndex);
	fflush(stdout);
	unsigned short voltage = bufToShort(frame->data + 3);
	moveToCell(config, batteryIndex, cellIndex, 4);
	fprintf(stdout, "Vc=%.3f ", milliToDouble(voltage));
	fflush(stdout);
	if (voltage > maxVoltage) {
		maxVoltage = voltage;
		maxVoltageCell = cellIndex;
	}
	if (voltage < minVoltage) {
		minVoltage = voltage;
		minVoltageCell = cellIndex;
	}
	totalVoltage += voltage;
	if (cellIndex == config->batteries[batteryIndex].cellCount - 1) {
		moveToCell(config, batteryIndex, battery->cellCount, 0);
		printf("%20s %.3f@%02d %.3f %.3f@%02d %7.3fV %6.2fV %7.2fA %7.2fAh", battery->name,
				asDouble(minVoltage), minVoltageCell, asDouble(totalVoltage / battery->cellCount),
				asDouble(maxVoltage), maxVoltageCell, asDouble(totalVoltage),
				soc_getVoltage(), soc_getCurrent(), soc_getAh());
		fflush(stdout);
		minVoltage = 0xffff;
		maxVoltage = 0;
		totalVoltage = 0;
	}

	unsigned short maxVoltageHundreds = maxVoltage / 100 * 100;
	unsigned short barMin = 3000;
	unsigned char tens;
	unsigned char hundreds;
	if (voltage < barMin) {
		tens = 0;
		hundreds = 0;
	} else {
		tens = (voltage / 10) % 10;
		hundreds = ((voltage / 100 * 100) - barMin) / 100;
	}
	fprintf(stderr, "%d %d %d %d %d %d\n", voltage, maxVoltage, maxVoltageHundreds, barMin, tens, hundreds);
	moveToCell(config, batteryIndex, cellIndex, 54);
	for (int i = 0; i < hundreds; i++) {
		if (i % 2) {
			fprintf(stdout, "**********");
		} else {
			fprintf(stdout, "##########");
		}
	}
	for (int i = 0; i < 10; i++) {
		if (i < tens) {
			fprintf(stdout, "-");
		} else {
			fprintf(stdout, " ");
		}
	}
	if (hundreds < 2) {
		printf("         ");
	}
	if (hundreds < 1) {
		printf("         ");
	}
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
	moveToCell(config, batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3d ", cellIndex);
	fflush(stdout);
	unsigned short shuntCurrent = bufToShort(frame->data + 3);
	moveToCell(config, batteryIndex, cellIndex, 13);
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
	moveToCell(config, batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3d ", cellIndex);
	fflush(stdout);
	unsigned short minCurrent = bufToShort(frame->data + 3);
	moveToCell(config, batteryIndex, cellIndex, 22);
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
	moveToCell(config, batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3d ", cellIndex);
	fflush(stdout);
	unsigned short temperature = bufToShort(frame->data + 3);
	moveToCell(config, batteryIndex, cellIndex, 31);
	fprintf(stdout, "t=%4.1f ", milliToDouble(temperature) * 10);
	fflush(stdout);
}

/* Decode a hardware config frame. */
void console_decode3f4(struct can_frame *frame, struct config_t *config) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	moveToCell(config, batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3d ", cellIndex);
	fflush(stdout);
	unsigned short revision = bufToShort(frame->data + 3);
	moveToCell(config, batteryIndex, cellIndex, 38);
	unsigned char value = bufToChar(frame->data + 5);
	char isClean = value & 0x8 ? ' ' : '*';
	char shuntType = value & 0x2 ? 'r' : ' ';
	char hardSwitched = value & 0x4 ? 'h' : ' ';
	char kelvin = value & 0x1 ? 'k' : ' ';
	printf("%4hd%c%c%c%c", revision, isClean, shuntType, hardSwitched, kelvin);
	fflush(stdout);
}

/* Decode an error frame. */
void console_decode3f5(struct can_frame *frame, struct config_t *config) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	moveToCell(config, batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3d ", cellIndex);
	fflush(stdout);
	unsigned short errorCount = bufToShort(frame->data + 3);
	moveToCell(config, batteryIndex, cellIndex, 46);
	fprintf(stdout, "%4d", errorCount);
	fflush(stdout);
}

/* Decode a latency frame. */
void console_decode3f6(struct can_frame *frame, struct config_t *config) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	moveToCell(config, batteryIndex, cellIndex, 0);
	fprintf(stdout, "%3hu ", cellIndex);
	fflush(stdout);
	unsigned char latency = bufToChar(frame->data + 3);
	moveToCell(config, batteryIndex, cellIndex, 51);
	fprintf(stdout, "%2hhu", latency);
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
		} else if (frame.can_id == 0x3f4) {
			console_decode3f4(&frame, config);
		} else if (frame.can_id == 0x3f5) {
			console_decode3f5(&frame, config);
		} else if (frame.can_id == 0x3f6) {
			console_decode3f6(&frame, config);
		} else {
			continue;
		}
	}
	return NULL;
}
