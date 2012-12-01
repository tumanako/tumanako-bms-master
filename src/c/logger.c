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
#include "logger.h"

struct logger_status_t {
	int index;
	char valued;
	unsigned short voltage;
	unsigned short shuntCurrent;
	unsigned short minCurrent;
	unsigned short temperature;
};

struct threadArguments_t {
	unsigned char batteryId;
	char *filename;
	unsigned short cellCount;
	unsigned short *cellIds;
};

extern int readFrame(int s, struct can_frame *frame);
void logger_decode3f0(struct can_frame *frame, struct logger_status_t *cells, struct threadArguments_t *);
void logger_decode3f1(struct can_frame *frame, struct logger_status_t *cells, struct threadArguments_t *);
void logger_decode3f3(struct can_frame *frame, struct logger_status_t *cells, struct threadArguments_t *);
void *logger_backgroundThread(void *ptr);
time_t logger_writeLogLine(time_t last, struct logger_status_t cells[], short cellCount, FILE *out);
void logMilli(FILE *out, unsigned short value, char isValid);
int countCellsWithData(struct logger_status_t cells[], short cellCount);

unsigned char logger_init(struct config_t *config) {
	for (unsigned char i = 0; i < config->batteryCount; i++) {
		struct threadArguments_t *args = malloc(sizeof(struct threadArguments_t));
		args->batteryId = i;
		args->filename = malloc(strlen(config->batteries[i].name) + strlen(".txt") + 1);
		if (args->filename == 0) {
			return 1;
		}
		strcpy(args->filename, config->batteries[i].name);
		strcat(args->filename, ".txt");
		args->cellIds = config->batteries[i].cellIds;
		args->cellCount = config->batteries[i].cellCount;
		pthread_t thread;
		pthread_create(&thread, NULL, logger_backgroundThread, (void *) args);
	}
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
		fd_set rfds;
		struct timeval tv;
		FD_ZERO(&rfds);
		FD_SET(s, &rfds);
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		if (select(s + 1, &rfds, NULL, NULL, &tv) > 0) {
			if (!readFrame(s, &frame)) {
				if (frame.can_id == 0x3f0) {
					logger_decode3f0(&frame, cells, args);
				} else if (frame.can_id == 0x3f1) {
					logger_decode3f1(&frame, cells, args);
				} else if (frame.can_id == 0x3f3) {
					logger_decode3f3(&frame, cells, args);
				}
			}
		}
		last = logger_writeLogLine(last, cells, args->cellCount, out);
	}
	return NULL;
}

/*
 * Write a line to the log if:
 * The appropriate amount of time has passed (so we don't log too quickly)
 * (All the data has been refreshed since the last line was written
 * or
 * the timeout has expired and we log a line with incomplete data)
 */
time_t logger_writeLogLine(time_t last, struct logger_status_t cells[], short cellCount, FILE *out) {
	time_t now;
	time(&now);
	if (now - last < 1) {
		// we wrote a line recently, wait some more
		return last;
	}
	for (int i = 0; i < cellCount; i++) {
		int thisCount = countCellsWithData(cells, cellCount);
		if (thisCount != cellCount) {
			// no data from at least one cell since we last wrote a line
			if (thisCount == 0) {
				// we haven't had any data, make sure we don't log anything until we do
				return now;
			}
			if (now - last < 20) {
				// we had complete data less than 5 seconds ago, wait some more
				return last;
			}
		}
	}

	fprintf(out, "%d %.1f %.2f ", (int) now, soc_getCurrent(), soc_getAh());
	for (int i = 0; i < cellCount; i++) {
		struct logger_status_t *cell = cells + i;
		logMilli(out, cell->voltage, cell->valued & 0x01);
		logMilli(out, cell->shuntCurrent, cell->valued & 0x02);
		cell->valued = 0;
	}
	fprintf(out, "\n");
	fflush(out);

	return now;
}

int countCellsWithData(struct logger_status_t cells[], short cellCount) {
	int result = 0;
	for (unsigned short i = 0; i < cellCount; i++) {
		if (cells[i].valued == 0x07) {
			result++;
		}
	}
	return result;
}
void logMilli(FILE *out, unsigned short value, char isValid) {
	if (isValid) {
		fprintf(out, " %.3f", milliToDouble(value));
	} else {
		fprintf(out, " -");
	}
}

/* Decode a voltage frame. */
void logger_decode3f0(struct can_frame *frame, struct logger_status_t cells[], struct threadArguments_t *args) {
	unsigned char batteryId = bufToChar(frame->data);
	if (batteryId != args->batteryId) {
		return;
	}
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > args->cellCount) {
		return;
	}
	cells[cellIndex].voltage = bufToShort(frame->data + 3);
	cells[cellIndex].valued |= 0x01;
}

/* Decode a shunt current frame. */
void logger_decode3f1(struct can_frame *frame, struct logger_status_t cells[], struct threadArguments_t *args) {
	unsigned char batteryId = bufToChar(frame->data);
	if (batteryId != args->batteryId) {
		return;
	}
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > args->cellCount) {
		return;
	}
	cells[cellIndex].shuntCurrent = bufToShort(frame->data + 3);
	cells[cellIndex].valued |= 0x02;
}

/* Decode a temperature frame. */
void logger_decode3f3(struct can_frame *frame, struct logger_status_t cells[], struct threadArguments_t *args) {
	unsigned char batteryId = bufToChar(frame->data);
	if (batteryId != args->batteryId) {
		return;
	}
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > args->cellCount) {
		return;
	}
	cells[cellIndex].temperature = bufToShort(frame->data + 3);
	cells[cellIndex].valued |= 0x04;
}

