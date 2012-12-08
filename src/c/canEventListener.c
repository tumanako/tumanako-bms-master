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

/** Listen to CAN Bus and dispatch events */

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
#include "config.h"
#include "canEventListener.h"

static pthread_t thread;
static struct config_t *config;

static void (*voltageListeners[10])(unsigned char, unsigned short, unsigned short);
static void (*shuntCurrentListeners[10])(unsigned char, unsigned short, unsigned short);
static void (*minCurrentListeners[10])(unsigned char, unsigned short, unsigned short);

volatile char canEventListener_error = 1;

static void decodeVoltage(struct can_frame *frame) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	unsigned short voltage = bufToShort(frame->data + 3);

	for (int i = 0; voltageListeners[i]; i++) {
		voltageListeners[i](batteryIndex, cellIndex, voltage);
	}
}

static void decodeCurrent(struct can_frame *frame) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	unsigned short shuntCurrent = bufToShort(frame->data + 3);

	for (int i = 0; voltageListeners[i]; i++) {
		shuntCurrentListeners[i](batteryIndex, cellIndex, shuntCurrent);
	}
}

static void decodeMinCurrent(struct can_frame *frame) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	unsigned short minCurrent = bufToShort(frame->data + 3);

	for (int i = 0; voltageListeners[i]; i++) {
		minCurrentListeners[i](batteryIndex, cellIndex, minCurrent);
	}
}

static void decodeFrame(struct can_frame *frame) {
	switch (frame->can_id) {
	case 0x3f0:
		decodeVoltage(frame);
		break;
	case 0x3f1:
		decodeCurrent(frame);
		break;
	case 0x3f2:
		decodeMinCurrent(frame);
		break;
//	case 0x3f3:
//		console_decode3f3(&frame, config);
//		break;
//	case 0x3f4:
//		console_decode3f4(&frame, config);
//		break;
//	case 0x3f5:
//		console_decode3f5(&frame, config);
//		break;
//	case 0x3f6:
//		console_decode3f6(&frame, config);
//		break;
//	case 0x3f8:
//		console_decode3f8(&frame, config);
//		break;
//	case 0x703:
//	case 0x705:
//	case 0x701:
//		console_printSoc(config);
//		break;
	}
}

static int readFrame(int s, struct can_frame *frame) {
	ssize_t nbytes = read(s, frame, sizeof(struct can_frame));

	if (nbytes < 0) {
		perror("can raw socket read");
		return 1;
	}

	/* paranoid check ... */
	if (nbytes < (int) sizeof(struct can_frame)) {
		fprintf(stderr, "read: incomplete CAN frame\n");
		return 1;
	}
	return 0;
}

void *backgroundThread(void *unused __attribute__ ((unused))) {
	while (1) {
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
				canEventListener_error = 1;
				break;
			}
			canEventListener_error = 0;
			decodeFrame(&frame);
		}
		// there was an error, wait for CAN bus to settle
		canEventListener_error = 1;
		sleep(1);
	}
	return NULL;
}

void canEventListener_init(struct config_t *_config) {
	canEventListener_error = 0;
	config = _config;
	pthread_create(&thread, NULL, backgroundThread, "unused");
}

void canEventListener_registerVoltageListener(void (*voltageListener)(unsigned char, unsigned short, unsigned short)) {
	int i = 0;
	while (voltageListeners[i] != NULL) {
		i++;
	}
	voltageListeners[i] = voltageListener;
}

void canEventListener_registerShuntCurrentListener(void (*shuntCurrentListener)(unsigned char, unsigned short, unsigned short)) {
	int i = 0;
	while (shuntCurrentListeners[i] != NULL) {
		i++;
	}
	shuntCurrentListeners[i] = shuntCurrentListener;
}

void canEventListener_registerMinCurrentListener(void (*minCurrentListener)(unsigned char, unsigned short, unsigned short)) {
	int i = 0;
	while (minCurrentListeners[i] != NULL) {
		i++;
	}
	minCurrentListeners[i] = minCurrentListener;
}
