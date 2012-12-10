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

static void (*voltageListeners[10])(unsigned char, unsigned short, unsigned char, unsigned short);
static void (*shuntCurrentListeners[10])(unsigned char, unsigned short, unsigned short);
static void (*minCurrentListeners[10])(unsigned char, unsigned short, unsigned short);
static void (*temperatureListeners[10])(unsigned char, unsigned short, unsigned short);
static void (*cellConfigListeners[10])(unsigned char, unsigned short, unsigned short, unsigned char);
static void (*errorListeners[10])(unsigned char, unsigned short, unsigned short);
static void (*latencyListeners[10])(unsigned char, unsigned short, unsigned char);
static void (*chargerStateListeners[10])(unsigned char, unsigned char, unsigned char);
static void (*rawCanListeners[10])(struct can_frame *frame);

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
	unsigned char isValid = bufToChar(frame->data + 3);
	unsigned short voltage = bufToShort(frame->data + 4);


	for (int i = 0; voltageListeners[i]; i++) {
		voltageListeners[i](batteryIndex, cellIndex, isValid, voltage);
	}
}

static void decodeBatteryCellShort(struct can_frame *frame, void (*listeners[])(unsigned char, unsigned short, unsigned short)) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	unsigned short value = bufToShort(frame->data + 3);

	for (int i = 0; listeners[i]; i++) {
		listeners[i](batteryIndex, cellIndex, value);
	}
}

static void decodeCellConfig(struct can_frame *frame) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	unsigned short revision = bufToShort(frame->data + 3);
	unsigned short cellConfig = bufToChar(frame->data + 5);

	for (int i = 0; cellConfigListeners[i]; i++) {
		cellConfigListeners[i](batteryIndex, cellIndex, revision, cellConfig);
	}
}

static void decodeLatency(struct can_frame *frame) {
	unsigned char batteryIndex = bufToChar(frame->data);
	if (batteryIndex > config->batteryCount) {
		return;
	}
	struct config_battery_t *battery = config->batteries + batteryIndex;
	unsigned short cellIndex = bufToShort(frame->data + 1);
	if (cellIndex > battery->cellCount) {
		return;
	}
	unsigned char latency = bufToChar(frame->data + 3);

	for (int i = 0; latencyListeners[i]; i++) {
		latencyListeners[i](batteryIndex, cellIndex, latency);
	}
}

static void decodeChargerState(struct can_frame *frame) {
	unsigned char shutdown = bufToChar(frame->data);
	unsigned char state = bufToChar(frame->data + 1);
	unsigned char reason = bufToChar(frame->data + 2);

	for (int i = 0; chargerStateListeners[i]; i++) {
		chargerStateListeners[i](shutdown, state, reason);
	}
}

static void decodeFrame(struct can_frame *frame) {
	switch (frame->can_id) {
	case 0x3f0:
		decodeVoltage(frame);
		break;
	case 0x3f1:
		// shunt current frame
		decodeBatteryCellShort(frame, shuntCurrentListeners);
		break;
	case 0x3f2:
		// min current frame
		decodeBatteryCellShort(frame, minCurrentListeners);
		break;
	case 0x3f3:
		// temperature frame
		decodeBatteryCellShort(frame, temperatureListeners);
		break;
	case 0x3f4:
		decodeCellConfig(frame);
		break;
	case 0x3f5:
		decodeBatteryCellShort(frame, errorListeners);
		break;
	case 0x3f6:
		decodeLatency(frame);
		break;
	case 0x3f8:
		decodeChargerState(frame);
		break;
	default:
		for (int i = 0; rawCanListeners[i]; i++) {
			rawCanListeners[i](frame);
		}
		break;
	}
}

int readFrame(int s, struct can_frame *frame) {
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

void registerListener(void (*listener)(unsigned char, unsigned short, unsigned short),
		void (*listeners[])(unsigned char, unsigned short, unsigned short)) {
	int i = 0;
	while (listeners[i] != NULL) {
		i++;
	}
	listeners[i] = listener;
}

void canEventListener_registerVoltageListener(void (*voltageListener)(unsigned char, unsigned short, unsigned char, unsigned short)) {
	int i = 0;
	while (voltageListeners[i] != NULL) {
		i++;
	}
	voltageListeners[i] = voltageListener;
}

void canEventListener_registerShuntCurrentListener(void (*shuntCurrentListener)(unsigned char, unsigned short, unsigned short)) {
	registerListener(shuntCurrentListener, shuntCurrentListeners);
}

void canEventListener_registerMinCurrentListener(void (*minCurrentListener)(unsigned char, unsigned short, unsigned short)) {
	registerListener(minCurrentListener, minCurrentListeners);
}

void canEventListener_registerTemperatureListener(void (*temperatureListener)(unsigned char, unsigned short, unsigned short)) {
	registerListener(temperatureListener, temperatureListeners);
}

void canEventListener_registerCellConfigListener(void (*cellConfigListener)(unsigned char, unsigned short, unsigned short, unsigned char)) {
	int i = 0;
	while (cellConfigListeners[i] != NULL) {
		i++;
	}
	cellConfigListeners[i] = cellConfigListener;
}

void canEventListener_registerErrorListener(void (*errorListener)(unsigned char, unsigned short, unsigned short)) {
	registerListener(errorListener, errorListeners);
}

void canEventListener_registerLatencyListener(void (*latencyListener)(unsigned char, unsigned short, unsigned char)) {
	int i = 0;
	while (latencyListeners[i] != NULL) {
		i++;
	}
	latencyListeners[i] = latencyListener;
}

void canEventListener_registerChargerStateListener(void (*chargerStateListener)(unsigned char, unsigned char, unsigned char)) {
	int i = 0;
	while (chargerStateListeners[i] != NULL) {
		i++;
	}
	chargerStateListeners[i] = chargerStateListener;
}

void canEventListener_registerRawCanListener(void (*rawCanListener)(struct can_frame *frame)) {
	int i = 0;
	while (rawCanListeners[i] != NULL) {
		i++;
	}
	rawCanListeners[i] = rawCanListener;
}
