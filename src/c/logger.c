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

#include <pthread.h>

#include "soc.h"
#include "util.h"
#include "canEventListener.h"
#include "logger.h"

struct logger_battery_t {
	FILE *out;
	time_t whenLastLogged;
	struct logger_status_t *cells;
};

struct logger_status_t {
	int index;
	char valued;
	unsigned short voltage;
	unsigned short shuntCurrent;
	unsigned short minCurrent;
	unsigned short temperature;
};

static void voltageListener(unsigned char batteryId, unsigned short cellIndex, unsigned char isValid, unsigned short voltage);
static void shuntCurrentListener(unsigned char batteryId, unsigned short cellIndex, unsigned short shuntCurrent);
static void temperatureListener(unsigned char batteryId, unsigned short cellIndex, unsigned short temperature);
void *logger_backgroundThread(void *ptr);
void logger_writeLogLine(unsigned char i);
void logMilli(FILE *out, unsigned short value, char isValid);
int countCellsWithData(struct logger_status_t cells[], short cellCount);

static pthread_t thread;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static struct config_t *config;
static struct logger_battery_t *loggerBatteries;

unsigned char logger_init(struct config_t *_config) {
	config = _config;
	loggerBatteries = calloc(config->batteryCount, sizeof(struct logger_battery_t));
	if (!loggerBatteries) {
		goto error;
	}
	for (unsigned char i = 0; i < config->batteryCount; i++) {
		struct config_battery_t *configBattery = config->batteries + i;
		struct logger_battery_t *loggerBattery = loggerBatteries + i;
		loggerBattery->cells = calloc(configBattery->cellCount, sizeof(struct logger_status_t));
		if (!loggerBattery->cells) {
			goto error;
		}
		struct logger_status_t *cells = loggerBattery->cells;
		for (int j = 0; j < configBattery->cellCount; j++) {
			(cells + i)->index = j;
			(cells + i)->valued = 0;
		}
		char *filename = malloc(strlen(configBattery->name) + strlen(".txt") + 1);
		if (!filename) {
			goto error;
		}
		strcpy(filename, config->batteries[i].name);
		strcat(filename, ".txt");
		loggerBattery->out = fopen(filename, "a");
		free(filename);
		if (!loggerBattery->out) {
			goto error;
		}
		fprintf(loggerBattery->out, "\n");
		fflush(loggerBattery->out);
	}
	pthread_create(&thread, NULL, logger_backgroundThread, (void *) "unused");
	canEventListener_registerVoltageListener(voltageListener);
	canEventListener_registerShuntCurrentListener(shuntCurrentListener);
	canEventListener_registerTemperatureListener(temperatureListener);
	return 0;
error:
	// todo cleanup
	return 1;
}

void *logger_backgroundThread(void *unused __attribute__ ((unused))) {
	while (1) {
		sleep(1);
		for (unsigned char i = 0; i < config->batteryCount; i++) {
			logger_writeLogLine(i);
		}
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
void logger_writeLogLine(unsigned char i) {
	pthread_mutex_lock(&mutex);
	struct logger_battery_t *loggerBattery = loggerBatteries + i;
	struct config_battery_t *configBattery = config->batteries + i;
	time_t now;
	time(&now);
	if (now - loggerBattery->whenLastLogged < 1) {
		// we wrote a line recently, wait some more
		pthread_mutex_unlock(&mutex);
		return;
	}
	int thisCount = countCellsWithData(loggerBattery->cells, configBattery->cellCount);
	if (thisCount != configBattery->cellCount) {
		// no data from at least one cell since we last wrote a line
		if (thisCount == 0) {
			// we haven't had any data, make sure we don't log anything until we do
			loggerBattery->whenLastLogged = now;
			pthread_mutex_unlock(&mutex);
			return;
		}
		// todo, this is too long during motoring, but expected during charging with
		// non kelvin connected transistor shunts. Either choose a short delay for
		// motoring or make the transistor shunts faster
		if (now - loggerBattery->whenLastLogged < 120) {
			// we had complete data less than 2 minutes ago, wait some more
			pthread_mutex_unlock(&mutex);
			return;
		}
	}

	fprintf(loggerBattery->out, "%d %.1f %.2f %.2f %.2f %.2f %.1f - %.1f", (int) now, soc_getCurrent(), soc_getAh(),
			soc_getVoltage(), soc_getHalfVoltage(), soc_getWh(), soc_getT1(), soc_getSpeed());
	for (int i = 0; i < configBattery->cellCount; i++) {
		struct logger_status_t *cell = loggerBattery->cells + i;
		logMilli(loggerBattery->out, cell->voltage, cell->valued & 0x01);
		logMilli(loggerBattery->out, cell->shuntCurrent, cell->valued & 0x02);
		cell->valued = 0;
	}
	fprintf(loggerBattery->out, "\n");
	fflush(loggerBattery->out);
	loggerBattery->whenLastLogged = now;
	pthread_mutex_unlock(&mutex);
}

int countCellsWithData(struct logger_status_t cells[], short cellCount) {
	int result = 0;
	for (unsigned short i = 0; i < cellCount; i++) {
		if ((cells[i].valued & 0x01) && (cells[i].valued & 0x2)) {
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

static void voltageListener(unsigned char batteryId, unsigned short cellIndex, unsigned char isValid, unsigned short voltage) {
	fprintf(stderr, "v %d %d %d %d\n", batteryId, cellIndex, isValid, voltage);
	if (!isValid) {
		return;
	}
	struct logger_status_t *cells = (loggerBatteries + batteryId)->cells;
	cells[cellIndex].voltage = voltage;
	cells[cellIndex].valued |= 0x01;
	logger_writeLogLine(batteryId);
}

static void shuntCurrentListener(unsigned char batteryId, unsigned short cellIndex, unsigned short shuntCurrent) {
	fprintf(stderr, "s %d %d %d\n", batteryId, cellIndex, shuntCurrent);
	struct logger_status_t *cells = (loggerBatteries + batteryId)->cells;
	cells[cellIndex].shuntCurrent = shuntCurrent;
	cells[cellIndex].valued |= 0x02;
	logger_writeLogLine(batteryId);
}

static void temperatureListener(unsigned char batteryId, unsigned short cellIndex, unsigned short temperature) {
	fprintf(stderr, "t %d %d %d\n", batteryId, cellIndex, temperature);
	struct logger_status_t *cells = (loggerBatteries + batteryId)->cells;
	cells[cellIndex].temperature = temperature;
	cells[cellIndex].valued |= 0x04;
	logger_writeLogLine(batteryId);
}

