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

#include "soc.h"
#include "util.h"

struct logger_status_t {
	int index;
	char valued;
	short voltage;
	short shuntCurrent;
	short minCurrent;
	short temperature;
};

struct threadArguments_t {
	char *filename;
	short (*cellIdMapper)(short cellIndex);
	short cellCount;
};

extern int readFrame(int s, struct can_frame *frame);
void logger_decode3f0(struct can_frame *frame, struct logger_status_t cells[], short(*cellIdMapper)(short));
void logger_decode3f1(struct can_frame *frame, struct logger_status_t cells[], short(*cellIdMapper)(short));
void logger_decode3f2(struct can_frame *frame, struct logger_status_t cells[], short(*cellIdMapper)(short));
void logger_decode3f3(struct can_frame *frame, struct logger_status_t cells[], short(*cellIdMapper)(short));
void *logger_backgroundThread(void *ptr);
time_t logger_writeLogLine(time_t last, struct logger_status_t cells[], short cellCount, FILE *out);

short logger_tractionMapper(short cellIndex);
short logger_accessoryMapper(short cellIndex);

int logger_init() {
	struct threadArguments_t *tractionArgs = malloc(sizeof(struct threadArguments_t));
	tractionArgs->filename = "traction.txt";
	tractionArgs->cellIdMapper = logger_tractionMapper;
	tractionArgs->cellCount = 36;
	pthread_t tractionThread;
	pthread_create(&tractionThread, NULL, logger_backgroundThread, (void *) tractionArgs);

	struct threadArguments_t *accessoryArgs = malloc(sizeof(struct threadArguments_t));
	accessoryArgs->filename = "accessory.txt";
	accessoryArgs->cellIdMapper = logger_accessoryMapper;
	accessoryArgs->cellCount = 4;
	pthread_t accessoryThread;
	pthread_create(&accessoryThread, NULL, logger_backgroundThread, (void *) accessoryArgs);

	return 0;
}

void *logger_backgroundThread(void *ptr) {
	struct threadArguments_t *args = (struct threadArguments_t *) ptr;
	struct logger_status_t cells[args->cellCount];
	for (int i = 0; i < args->cellCount; i++) {
		cells[i].index = i;
		cells[i].valued = 0;
	}
	FILE *out = fopen(args->filename, "a");

	struct can_frame frame;

	int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	struct ifreq ifr;
	strcpy(ifr.ifr_name, "slcan0");
	ioctl(s, SIOCGIFINDEX, &ifr);

	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	bind(s, (struct sockaddr *) &addr, sizeof(addr));

	fprintf(out, "\n");
	fflush(out);

	time_t last = 0;
	while (1) {
		if (readFrame(s, &frame)) {
			return NULL;
		}
		if (frame.can_id == 0x3f0) {
			logger_decode3f0(&frame, cells, args->cellIdMapper);
		} else if (frame.can_id == 0x3f1) {
			logger_decode3f1(&frame, cells, args->cellIdMapper);
		} else if (frame.can_id == 0x3f2) {
			logger_decode3f2(&frame, cells, args->cellIdMapper);
		} else if (frame.can_id == 0x3f3) {
			logger_decode3f3(&frame, cells, args->cellIdMapper);
		} else {
			continue;
		}
		last = logger_writeLogLine(last, cells, args->cellCount, out);
	}
	return NULL;
}

/*
 * Write a line to the log if:
 * The appropriate amount of time has passed (so we don't log too quickly)
 * All the data has been refreshed since the last line was written
 */
time_t logger_writeLogLine(time_t last, struct logger_status_t cells[], short cellCount, FILE *out) {
	time_t now;
	time(&now);
	if (now - last < 10) {
		// we wrote a line recently, wait some more
		return last;
	}
	for (int i = 0; i < cellCount; i++) {
		if (!(cells[i].valued && 0x0f)) {
			// no data from at least one cell since we last wrote a line, wait some more
			return last;
		}
	}

	fprintf(out, "%d %.1f %.2f ", (int) now, soc_getCurrent(), soc_getAh());
	for (int i = 0; i < cellCount; i++) {
		struct logger_status_t cell = cells[i];
		fprintf(out, " %.3f %.3f", milliToDouble(cell.voltage), milliToDouble(cell.shuntCurrent));
	}
	fprintf(out, "\n");
	fflush(out);

	for (int i = 0; i < cellCount; i++) {
		cells[i].valued = 0;
	}

	return now;
}

/* Decode a voltage frame. */
void logger_decode3f0(struct can_frame *frame, struct logger_status_t cells[], short(*cellIdMapper)(short)) {
	short cellId = bufToShort(frame->data);
	short cellIndex = cellIdMapper(cellId);
	if (cellIndex != -1) {
		cells[cellIndex].voltage = bufToShort(frame->data + 2);
		cells[cellIndex].valued |= 0x01;
	}
}

/* Decode a shunt current frame. */
void logger_decode3f1(struct can_frame *frame, struct logger_status_t cells[], short(*cellIdMapper)(short)) {
	short cellId = bufToShort(frame->data);
	short cellIndex = cellIdMapper(cellId);
	if (cellIndex != -1) {
		cells[cellIndex].shuntCurrent = bufToShort(frame->data + 2);
		cells[cellIndex].valued |= 0x02;
	}
}

/* Decode a minCurrent frame. */
void logger_decode3f2(struct can_frame *frame, struct logger_status_t cells[], short(*cellIdMapper)(short)) {
	short cellId = bufToShort(frame->data);
	short cellIndex = cellIdMapper(cellId);
	if (cellIndex != -1) {
		cells[cellIndex].minCurrent = bufToShort(frame->data + 2);
		cells[cellIndex].valued |= 0x04;
	}
}

/* Decode a temperature frame. */
void logger_decode3f3(struct can_frame *frame, struct logger_status_t cells[], short(*cellIdMapper)(short)) {
	short cellId = bufToShort(frame->data);
	short cellIndex = cellIdMapper(cellId);
	if (cellIndex != -1) {
		cells[cellIndex].temperature = bufToShort(frame->data + 2);
		cells[cellIndex].valued |= 0x08;
	}
}

/* Cell index mapper for the traction battery. */
short logger_tractionMapper(short cellIndex) {
	// TODO make configurable
	if (cellIndex >= 0 && cellIndex < 36) {
		return cellIndex;
	}
	return -1;
}

/* Cell index mapper for the accessory battery. */
short logger_accessoryMapper(short cellIndex) {
	// TODO make configurable
	if (cellIndex >= 36 && cellIndex < 40) {
		return cellIndex - 36;
	}
	return -1;
}
