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

static void (*socEventListeners[10])();
static void (*instVoltageListeners[10])();

volatile unsigned short volts = 0;
volatile long chargeCurrent = 0;
volatile long dischargeCurrent = 0;
volatile short aH = 0;
volatile short halfVoltage = 0;
volatile long wH = 0;
volatile short t1 = 0;
volatile short t2 = 0;
volatile short speed = 0;

volatile unsigned short instVolts = 0;
volatile unsigned short instHalfVoltage = 0;
volatile long instChargeCurrent = 0;
volatile long instDischargeCurrent = 0;

time_t lastValidCurrent;
time_t lastValidVoltage;

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

/**
 * Make a long from the 32 bits starting at c
 *
 * TODO deal with endian
 */
static long makeLong(__u8 *c) {
	long result = c[0];
	result = result << 8;
	result = result | c[1];
	result = result << 8;
	result = result | c[2];
	result = result << 8;
	result = result | c[3];
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

double soc_getInstCurrent() {
	if (instChargeCurrent != 0) {
		return instChargeCurrent / (double) -100;
	}
	if (instDischargeCurrent != 0) {
		return instDischargeCurrent / (double) 100;
	}
	return 0;
}

double soc_getInstVoltage() {
	return instVolts / (double) 100;
}

double soc_getInstHalfVoltage() {
	return instHalfVoltage / (double) 100;
}

double soc_getWh() {
	return wH / (double) 100;
}

double soc_getT1() {
	return t1 / (double) 100;
}

double soc_getT2() {
	return t2 / (double) 100;
}

double soc_getSpeed() {
	return speed / (double) 100;
}

char soc_getError() {
	time_t now;
	time(&now);
	return now - lastValidVoltage > 5 || now - lastValidCurrent > 5;
}

static void decode700(struct can_frame *frame) {
	instDischargeCurrent = make24BitLong(frame->data + 4);
	instChargeCurrent = make24BitLong(frame->data);
}

static void decode701(struct can_frame *frame) {
	dischargeCurrent = make24BitLong(frame->data + 4);
	chargeCurrent = make24BitLong(frame->data);
	time(&lastValidCurrent);
}

static void decode702(struct can_frame *frame) {
	instVolts = makeShort(frame->data + 1);
	instHalfVoltage = makeShort(frame->data + 4);
	for (int i = 0; instVoltageListeners[i]; i++) {
		instVoltageListeners[i]();
	}
}

static void decode703(struct can_frame *frame) {
	volts = makeShort(frame->data + 1);
	halfVoltage = makeShort(frame->data + 4);
	time(&lastValidVoltage);
}

static void decode705(struct can_frame *frame) {
	aH = makeShort(frame->data + 1);
}

static void decode706(struct can_frame *frame) {
	wH = makeLong(frame->data);
}

static void decode704(struct can_frame *frame) {
	t1 = makeShort(frame->data + 2);
	t2 = makeShort(frame->data + 4);
}

static void decode708(struct can_frame *frame) {
	speed = makeShort(frame->data);
}

void rawCanListener(struct can_frame *frame) {
	if (frame->can_id == 0x703) {
		decode703(frame);
	} else if (frame->can_id == 0x705) {
		decode705(frame);
	} else if (frame->can_id == 0x706) {
		decode706(frame);
	} else if (frame->can_id == 0x701) {
		decode701(frame);
	} else if (frame->can_id == 0x704) {
		decode704(frame);
	} else if (frame->can_id == 0x708) {
		decode708(frame);
	} else if (frame->can_id == 0x700) {
		decode700(frame);
	} else if (frame->can_id == 0x702) {
		decode702(frame);
	} else {
		return;
	}
	for (int i = 0; socEventListeners[i]; i++) {
		socEventListeners[i]();
	}
}

int soc_init() {
	time(&lastValidCurrent);
	time(&lastValidVoltage);
	canEventListener_registerRawCanListener(rawCanListener);
	return 0;
}

void soc_registerSocEventListener(void (*socEventListener)()) {
	int i = 0;
	while (socEventListeners[i] != NULL) {
		i++;
	}
	socEventListeners[i] = socEventListener;
}

void soc_registerInstVoltageListener(void (*instVoltageListener)()) {
	int i = 0;
	while (instVoltageListeners[i] != NULL) {
		i++;
	}
	instVoltageListeners[i] = instVoltageListener;
}
