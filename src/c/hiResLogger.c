/*
 Copyright 2013 Tom Parker

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

#include <linux/types.h>
#include <sys/time.h>
#include <stdio.h>
#include <pthread.h>

#include "soc.h"

FILE *logFile;

static volatile __u8 logging = 0;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

static void voltageListener() {
	if (logging) {
		struct timeval t;
		gettimeofday(&t, NULL);
		double now = t.tv_sec + t.tv_usec / (double) 1000000;
		pthread_mutex_lock(&mutex);
		fprintf(logFile, "%.3f %.2f %.2f %.1f\n", now, soc_getInstVoltage(), soc_getInstCurrent(), soc_getSpeed());
		pthread_mutex_unlock(&mutex);
	}
}

void hiResLogger_init() {
	logFile = fopen("hiRes.txt", "a");
	soc_registerInstVoltageListener(voltageListener);
}

void hiResLogger_start() {
	logging = 1;
}

void hiResLogger_stop() {
	logging = 0;
	pthread_mutex_lock(&mutex);
	fprintf(logFile, "\n");
	fflush(logFile);
	pthread_mutex_unlock(&mutex);
}
