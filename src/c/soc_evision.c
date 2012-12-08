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

/** State of Charge implementation using SocketCAN and an EVision */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>

#include "soc.h"
#include "canEventListener.h"

volatile unsigned short volts = 0;
volatile long chargeCurrent = 0;
volatile long dischargeCurrent = 0;
volatile short aH = 0;
volatile short halfVoltage = 0;
volatile char error = 0;

/**
 * Make a short from the 16 bits starting at c
 *
 * TODO deal with endian
 */
static unsigned short makeShort(__u8 *c) {
	unsigned short result = *c;
	result = result << 8;
	result = result | *(c + 1);
	return result;
}

/**
 * Make a long from the 24 bits starting at c
 *
 * TODO deal with endian
 */
static unsigned long make24BitLong(__u8 *c) {
	unsigned long result = c[0];
	result = result << 8;
	result = result | c[1];
	result = result << 8;
	result = result | c[2];
	return result;
}

double soc_getCurrent() {
	if (chargeCurrent != 0) {
		return chargeCurrent / (double) -100;
	}
	if (dischargeCurrent != 0) {
		return dischargeCurrent / (double) 100;
	}
	return 0;
}

double soc_getVoltage() {
	return volts / (double) 100;
}

double soc_getAh() {
	return aH / (double) 100;
}

double soc_getHalfVoltage() {
	return halfVoltage / (double) 100;
}

char soc_getError() {
	return error;
}

static void decode701(struct can_frame *frame) {
	dischargeCurrent = make24BitLong(frame->data + 4);
	chargeCurrent = make24BitLong(frame->data);
}

static void decode703(struct can_frame *frame) {
	volts = makeShort(frame->data + 1);
	halfVoltage = makeShort(frame->data + 4);
}

static void decode705(struct can_frame *frame) {
	aH = makeShort(frame->data + 1);
}

static void printFrame(struct can_frame *frame) {
	printf("%x %d ", frame->can_id, frame->can_dlc);
	for (int i = 0; i < frame->can_dlc; i++) {
		printf("%02x", frame->data[i]);
	}
	printf("\n");
}

void rawCanListener(struct can_frame *frame) {
	if (frame->can_id == 0x703) {
		decode703(frame);
	} else if (frame->can_id == 0x705) {
		decode705(frame);
	} else if (frame->can_id == 0x701) {
		decode701(frame);
	}
}

int soc_init() {
	canEventListener_registerRawCanListener(rawCanListener);
	return 0;
}
