#define _GNU_SOURCE

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <unistd.h>
#include <confuse.h>

#include "../../../slave/src/c/evd5.h"

#include "monitor.h"
#include "config.h"
#include "crc.h"
#include "chargercontrol.h"
#include "buscontrol.h"
#include "soc.h"
#include "monitor_can.h"
#include "logger.h"
#include "util.h"

#define BAUDRATE B9600
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define CELL_ID_FILE "cells.txt"
#define DEBUG 0
#define HAS_KELVIN_CONNECTION 0
#define CHARGER_RELAY_PORT 7
#define CHARGER_ON_VOLTAGE 3550
#define CHARGER_OFF_VOLTAGE 3650
#define SHUNT_ON_VOLTAGE 3500
#define SHUNT_MAX_CURRENT 300
#define FORCED_SHUNT_OFF_VOLTAGE 3530
#define CHARGE_CURRENT_OVERSAMPLING 5

void initData(struct config_t *config);
void sendCommand(unsigned char version, unsigned short address, char sequence, char command);
void sendCommandV0(unsigned short address, char sequenceNumber, char command);
void sendCommandV1(unsigned short address, char sequenceNumber, char command);
void getCellStates();
char getCellState(struct status_t *cell);
char _getCellState(struct status_t *status, int attempts);
void writeSlowly(int fd, char *s, int length);
int readEnough(int fd, unsigned char *buf, int length);
unsigned short maxVoltageInAnyBattery();
unsigned short maxVoltage(struct battery_t *battery);
unsigned short maxVoltageCell(struct battery_t *battery);
unsigned short minVoltage(struct battery_t *battery);
unsigned short minVoltageCell(struct battery_t *battery);
unsigned short avgVoltage(struct battery_t *battery);
unsigned int totalVoltage(struct battery_t *battery);
void setShuntCurrent(struct battery_t *battery);
void setMinCurrent(struct status_t *cell, unsigned short minCurrent);
void dumpBuffer(unsigned char *buf, int length);
void findCells();
void evd5ToStatus(struct evd5_status_t *from, struct status_t *to);
void printSummary();
void printCellDetail(struct status_t *status);
double asDouble(int s);
void turnOffAllShunts();
char isAnyCellShunting();
char isCellShunting(struct status_t *cell);
void flushInputBuffer();
void getSlaveVersions();

int fd;

struct monitor_t data;

char chargerState = 0;
int main() {
	struct termios oldtio, newtio;

	struct config_t *config = getConfig();
	if (!config) {
		printf("error reading configuration file\n");
		return 1;
	}
	initData(config);

	if (chargercontrol_init()) {
		return 1;
	}

	if (buscontrol_init()) {
		return 1;
	}

	if (soc_init()) {
		return 1;
	}

	if (monitorCan_init()) {
		return 1;
	}

	if (logger_init(config)) {
		return 1;
	}

	buscontrol_setBus(TRUE);
	fd = open(config->serialPort, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		perror(config->serialPort);
		return -1;
	}

	tcgetattr(fd, &oldtio); /* save current port settings */

	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;

	/* set input mode (non-canonical, no echo,...) */
	newtio.c_lflag = 0;

	newtio.c_cc[VTIME] = 1;
	newtio.c_cc[VMIN] = 10;

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	char seq = '0';

	// send some bytes to wake up the slaves
	writeSlowly(fd, "garbage", 7);

	for (unsigned char i = 0; i < data.batteryCount; i++) {
		struct battery_t *battery = data.batteries + i;
		for (unsigned short j = 0; j < battery->cellCount; j++) {
			sendCommand(0, battery->cells[j].cellId, seq++, 'r');
			sendCommand(1, battery->cells[j].cellId, seq++, 'r');
		}
	}

	sleep(1);

	// send some bytes to wake up the slaves (they drop characters while flashing the light)
	writeSlowly(fd, "garbage", 7);

	// findCells();

	getSlaveVersions();

	// clear the screen
	write(1, "\E[H\E[2J", 7);

	chargerState = 0;
	time_t last = 0;
	char shutdown = 0;
	while (1) {
		time_t t;
		time(&t);
		if (t < last + config->loopDelay) {
			sleep(1);
			continue;
		}
		if (config->loopDelay > 30) {
			// if the slaves have gone to sleep, send some characters to wake them up
			writeSlowly(fd, "garbage", 7);
			// wait for slaves to wake up and take a measurement
			sleep(2);
		}
		last = t;
		if (!HAS_KELVIN_CONNECTION) {
			turnOffAllShunts();
		}
		getCellStates();
		if (maxVoltageInAnyBattery() > CHARGER_OFF_VOLTAGE) {
			chargercontrol_setCharger(FALSE);
			chargerState = 0;
			shutdown = 1;
		}
		if (maxVoltageInAnyBattery() < CHARGER_ON_VOLTAGE || chargerState) {
			if (!shutdown) {
				chargercontrol_setCharger(TRUE);
				chargerState = 1;
			}
		}
		if (soc_getError()) {
			fprintf(stderr, "State of Charge error?");
			chargercontrol_shutdown();
			shutdown = 1;
		}
		printSummary();
		for (unsigned char i = 0; i < data.batteryCount; i++) {
			setShuntCurrent(&data.batteries[i]);
		}
		fflush(NULL);

		// If we don't have a kelvin connection then we will be turning off the shunts to read the voltages.
		// This causes problems because the cells are quite slow to turn on the shunts. Here we read the
		// shunt current a few times without turning off the shunts to let them have time to do their work
		//
		// TODO implement "suspend" command to tell slaves to stop shunting and store their shunt config for
		// later fast re-enabling
		for (int j = 0; !HAS_KELVIN_CONNECTION && isAnyCellShunting() && j < 5; j++) {
			sleep(10);
			getCellStates();
		}
	}
	tcsetattr(fd, TCSANOW, &oldtio);
}

void getCellStates() {
	// move to the top of the screen
	write(1, "\E[H", 3);
	for (unsigned char i = 0; i < data.batteryCount; i++) {
		struct battery_t *battery = data.batteries + i;
		for (unsigned short j = 0; j < battery->cellCount; j++) {
			struct status_t *cell = battery->cells + j;
			char success = getCellState(cell);
			printCellDetail(cell);
			if (!success) {
				cell->errorCount++;
				continue;
			}
			if (!isCellShunting(cell)) {
				// the voltage doesn't mean much when we are drawing current
				montiorCan_sendCellVoltage(i, j, cell->vCell);
			}
			monitorCan_sendShuntCurrent(i, j, cell->iShunt);
			monitorCan_sendMinCurrent(i, j, cell->minCurrent);
			monitorCan_sendTemperature(i, j, cell->temperature);
		}
	}
}

unsigned char sequenceNumber = 0;

char getCellState(struct status_t *cell) {
	char success = _getCellState(cell, 1);
	if (!success) {
		fprintf(stderr, "bus errors talking to cell %d (id %d) in %s, exiting\n", cell->cellIndex, cell->cellId,
				cell->battery->name);
		chargercontrol_shutdown();
		return FALSE;
	}
	return TRUE;
}

char _getCellState(struct status_t *status, int maxAttempts) {
	int actualLength = 0;
	for (int attempt = 0; TRUE; attempt++) {
		if (attempt >= maxAttempts) {
			return 0;
			fprintf(stderr, "%d bus errors, exiting\n", attempt);
			chargercontrol_shutdown();
			exit(1);
		}
		if (attempt > 0 && actualLength == 0) {
			fprintf(stderr, "no response from %d (id %d) in %s, resetting\n", status->cellIndex, status->cellId,
					status->battery->name);
			buscontrol_setBus(FALSE);
			buscontrol_setBus(TRUE);
		}
		unsigned char buf[EVD5_STATUS_LENGTH];
		unsigned char sentSequenceNumber;
		sentSequenceNumber = sequenceNumber++;
		struct timeval start;
		gettimeofday(&start, NULL);
		sendCommand(status->version, status->cellId, sentSequenceNumber, '/');
		actualLength = readEnough(fd, buf, EVD5_STATUS_LENGTH);
		struct timeval end;
		gettimeofday(&end, NULL);
		if (actualLength != EVD5_STATUS_LENGTH) {
			fprintf(stderr, "read %d, expected %d from cell %d (id %2d) in %s\n", actualLength, EVD5_STATUS_LENGTH,
					status->cellIndex, status->cellId, status->battery->name);
			dumpBuffer(buf, actualLength);
			flushInputBuffer();
			continue;
		}
		unsigned short *actualCRC = (unsigned short *) (buf + EVD5_STATUS_LENGTH - sizeof(crc_t));
		crc_t expectedCRC = crc_init();
		expectedCRC = crc_update(expectedCRC, buf, EVD5_STATUS_LENGTH - sizeof(crc_t));
		expectedCRC = crc_finalize(expectedCRC);
		if (expectedCRC != *actualCRC) {
			fprintf(stderr, "\nSent message to %2d (id %2d) in %s, expected CRC 0x%04x got 0x%04x\n", status->cellIndex,
					status->cellId, status->battery->name, expectedCRC, *actualCRC);
			dumpBuffer(buf, actualLength);
			flushInputBuffer();
			continue;
		}
		struct evd5_status_t evd5Status;
		memcpy(&evd5Status, buf, EVD5_STATUS_LENGTH);
		// have to copy this one separately because of padding
		evd5Status.crc = *actualCRC;
		if (evd5Status.cellAddress != status->cellId) {
			fprintf(stderr, "\nSent message to %2d (id %2d) in %s but received response from 0x%x\n", status->cellIndex,
					status->cellId, status->battery->name, evd5Status.cellAddress);
			dumpBuffer(buf, actualLength);
			flushInputBuffer();
			continue;
		}
		if (evd5Status.sequenceNumber != sentSequenceNumber) {
			fprintf(stderr, "\nSent message to %2d (id %2d) in %s with seq 0x%02x but received seq 0x%02hhx\n",
					status->cellIndex, status->cellId, status->battery->name, sentSequenceNumber,
					evd5Status.sequenceNumber);
			dumpBuffer(buf, actualLength);
			flushInputBuffer();
			continue;
		}
		evd5ToStatus(&evd5Status, status);
		status->latency = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
		break;
	}
	return 1;
}

void evd5ToStatus(struct evd5_status_t* from, struct status_t* to) {
	to->automatic = from->automatic;
	to->gainPot = from->gainPot;
	to->hasRx = from->hasRx;
	to->iShunt = from->iShunt;
	to->minCurrent = from->minCurrent;
	to->sequenceNumber = from->sequenceNumber;
	to->softwareAddressing = from->softwareAddressing;
	to->temperature = from->temperature;
	if (!isCellShunting(to)) {
		to->vCell = from->vCell;
	}
	to->vShunt = from->vShunt;
	to->vShuntPot = from->vShuntPot;
	to->crc = from->crc;
}

/** turn shunting off on any cells that are shunting */
void turnOffAllShunts() {
	char changed = 0;
	for (unsigned char i = 0; i < data.batteryCount; i++) {
		struct battery_t *battery = data.batteries + i;
		for (unsigned short j = 0; j < battery->cellCount; j++) {
			struct status_t *cell = battery->cells + j;
			if (cell->minCurrent != 0 || cell->targetShuntCurrent != 0) {
				setMinCurrent(cell, 0);
				changed = 1;
			}
		}
	}
	if (changed) {
		// give slaves time to process
		sleep(2);
	}
}

void setShuntCurrent(struct battery_t *battery) {
	for (unsigned short i = 0; i < battery->cellCount; i++) {
		struct status_t *cell = battery->cells + i;
		unsigned short target;
		if (cell->vCell > SHUNT_ON_VOLTAGE) {
			short difference = cell->vCell - minVoltage(battery);
			if (difference < 30) {
				target = 0;
			} else {
				target = (difference / 5) * 50 + 50;
			}
			if (target > SHUNT_MAX_CURRENT) {
				target = SHUNT_MAX_CURRENT;
			} else if (target > 0 && target < 150) {
				// TODO the slaves don't respond to shunt demands less than 150 :(
				target = 150;
			}
		} else {
			target = 0;
		}
		setMinCurrent(cell, target);
	}
}

void setMinCurrent(struct status_t *cell, unsigned short minCurrent) {
	if (cell->minCurrent == minCurrent) {
		return;
	}
	char command;
	unsigned char buf[7];
	if (minCurrent > SHUNT_MAX_CURRENT) {
		minCurrent = SHUNT_MAX_CURRENT;
	}
	cell->targetShuntCurrent = minCurrent;
	int actual = SHUNT_MAX_CURRENT + 1;
	for (int i = 0; i < 20; i++) {
		if (actual > minCurrent) {
			command = '<';
		} else if (actual < minCurrent) {
			command = '>';
		} else {
			return;
		}
		sendCommand(cell->version, cell->cellId, '0', command);
		readEnough(fd, buf, 7);
		buf[6] = 0;
		char *endPtr;
		actual = strtol((char *) buf, &endPtr, 10);
	}
	// couldn't get to desired current after 10 attempts???
	chargercontrol_shutdown();
	getCellState(cell);
	fprintf(stderr, "%2d (id %2d) in %s trying to get to %d but had %d actual = %d\n", cell->cellIndex, cell->cellId,
			cell->battery->name, minCurrent, cell->minCurrent, actual);
	exit(1);
}

unsigned short minVoltage(struct battery_t *battery) {
	unsigned short result = 0xffff;
	for (unsigned short i = 0; i < battery->cellCount; i++) {
		if (battery->cells[i].vCell < result) {
			result = battery->cells[i].vCell;
		}
	}
	return result;
}

unsigned short minVoltageCell(struct battery_t *battery) {
	unsigned short min = 0xffff;
	unsigned short result = 0;
	for (unsigned short i = 0; i < battery->cellCount; i++) {
		if (battery->cells[i].vCell < min) {
			min = battery->cells[i].vCell;
			result = i;
		}
	}
	return result;
}

unsigned short maxVoltageInAnyBattery() {
	unsigned short result = 0;
	for (unsigned char i = 0; i < data.batteryCount; i++) {
		unsigned short max = maxVoltage(&data.batteries[i]);
		if (result < max) {
			result = max;
		}
	}
	return result;
}

unsigned short maxVoltage(struct battery_t *battery) {
	unsigned short result = 0;
	for (unsigned short i = 0; i < battery->cellCount; i++) {
		if (battery->cells[i].vCell > result) {
			result = battery->cells[i].vCell;
		}
	}
	return result;
}

unsigned short maxVoltageCell(struct battery_t *battery) {
	unsigned short max = 0;
	unsigned short result = 0;
	for (unsigned short i = 0; i < battery->cellCount; i++) {
		if (battery->cells[i].vCell > max) {
			max = battery->cells[i].vCell;
			result = i;
		}
	}
	return result;
}

unsigned int totalVoltage(struct battery_t *battery) {
	int result = 0;
	for (unsigned short i = 0; i < battery->cellCount; i++) {
		result += battery->cells[i].vCell;
	}
	return result;
}

unsigned short avgVoltage(struct battery_t *battery) {
	return totalVoltage(battery) / (battery->cellCount);
}

/**
 * @return true if any cell is shunting
 */
char isAnyCellShunting() {
	for (unsigned char i = 0; i < data.batteryCount; i++) {
		struct battery_t *battery = data.batteries + i;
		for (unsigned short j = 0; j < battery->cellCount; j++) {
			if (battery->cells[j].targetShuntCurrent > 0) {
				return 1;
			}
		}
	}
	return 0;
}

/**
 * Returns true if the passed cell or an adjacent cell is shunting current. This works
 * even when cells are out of order because each board has it's own end connections and we are careful
 * not to reorder cells within a board.
 */
char isCellShunting(struct status_t *cell) {
	if (HAS_KELVIN_CONNECTION) {
		return 0;
	}
	if (cell->targetShuntCurrent != 0) {
		return 1;
	}
	if (cell->cellIndex != 0 && (cell - 1)->targetShuntCurrent != 0) {
		return 1;
	}
	if (cell->cellIndex + 1 < cell->battery->cellCount && (cell + 1)->targetShuntCurrent != 0) {
		return 1;
	}
	return 0;
}

void sendCommand(unsigned char version, unsigned short address, char sequence, char command) {
	if (version == 0) {
		sendCommandV0(address, sequence, command);
	} else {
		sendCommandV1(address, sequence, command);
	}
}

void sendCommandV0(unsigned short address, char sequence, char command) {
	char buf[] = "heloXXYX";
	// little endian
	buf[4] = (char) address & 0x00FF;
	buf[5] = (char) ((address & 0xFF00) >> 8);
	buf[6] = sequence;
	buf[7] = command;
	if (DEBUG) {
		fprintf(stderr, "sending command '%c' to 0x%02x%02x with seq %02x\n", buf[7], buf[4], buf[5], sequence);
	}
	writeSlowly(fd, buf, 8);
}

void sendCommandV1(unsigned short address, char sequence, char command) {
	char buf[6] = "XXYZCC";
	// little endian
	buf[0] = (char) address & 0x00FF;
	buf[1] = (char) ((address & 0xFF00) >> 8);
	buf[2] = sequence;
	buf[3] = command;
	crc_t crc = crc_init();
	crc = crc_update(crc, (unsigned char *) buf, 4);
	crc = crc_finalize(crc);
	buf[4] = (char) crc & 0x00FF;
	buf[5] = (char) ((crc & 0xFF00) >> 8);
	if (DEBUG) {
		fprintf(stderr, "sending command '%c' to 0x%02x%02x with CRC 0x%02x%02x\n", buf[3], buf[0], buf[1], buf[4],
				buf[5]);
	}
	writeSlowly(fd, buf, 6);
}

void writeSlowly(int fd, char *s, int length) {
	//printf("%s\n", s);
	for (int i = 0; i < length; i++) {
		write(fd, s + i, 1);
		//usleep(1000);
	}
}

int readEnough(int fd, unsigned char *buf, int length) {
	fd_set rfds;
	struct timeval tv;

	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);

	tv.tv_sec = 0;
	tv.tv_usec = 200000;

	int actual = 0;
	for (int i = 0; i < 5; i++) {
		int retval = select(fd + 1, &rfds, NULL, NULL, &tv);
		if (retval == -1) {
			fflush(NULL);
			return actual;
		}
		if (!retval) {
			fflush(NULL);
			continue;
		}
		actual += read(fd, buf + actual, length - actual);
		if (DEBUG) {
			fprintf(stderr, "read %d expecting %d: ", actual, length);
			for (int j = 0; j < actual; j++) {
				fprintf(stderr, "%02x ", buf[j]);
			}
			fprintf(stderr, "\n");
		}
		if (actual >= length) {
			break;
		}
	}
	return actual;
}

/** read all the data in the input buffers, used instead of a start of message byte to re-sync */
void flushInputBuffer() {
	unsigned char buf[255];
	int length = readEnough(fd, buf, 255);
	do {
		length = readEnough(fd, buf, 255);
		fprintf(stderr, "read %d more\n", length);
		dumpBuffer(buf, length);
	} while (length > 0);
}

void dumpBuffer(unsigned char *buf, int length) {
	if (DEBUG) {
		for (int i = 0; i < length; i++) {
			fprintf(stderr, "%02d %02x\n", i, buf[i]);
		}
	}
}

void printSummary() {
	write(1, "\E[0J", 4);
	printf("\n");
	for (unsigned char i = 0; i < data.batteryCount; i++) {
		struct battery_t *battery = data.batteries + i;
		write(1, "\E[0J", 4);
		printf("%20s %.3f@%02d %.3f %.3f@%02d %7.3fV %6.2fV %7.2fA %7.2fAh %s\n", battery->name,
				asDouble(minVoltage(battery)), minVoltageCell(battery), asDouble(avgVoltage(battery)),
				asDouble(maxVoltage(battery)), maxVoltageCell(battery), asDouble(totalVoltage(battery)),
				soc_getVoltage(), soc_getCurrent(), soc_getAh(), chargerState ? "on" : "off");
	}
	// TODO clear to the bottom of the screen
}

void printCellDetail(struct status_t *status) {
	if (status->minCurrent > 0) {
		write(1, "\E[31m", 5);
	} else {
		write(1, "\E[m", 3);
	}
	char isClean = !status->isClean ? '*' : ' ';
	printf("%02d %02d Vc=%.3f Vs=%.3f Is=%.3f It=%5.3f t=%5.1f s=%02d g=%02d %2ld %4d%c %5d ", status->cellIndex,
			status->cellId, asDouble(status->vCell), asDouble(status->vShunt), asDouble(status->iShunt),
			asDouble(status->minCurrent), asDouble(status->temperature) * 10, status->vShuntPot, status->gainPot,
			status->latency / 1000, status->revision, isClean, status->errorCount);
	unsigned char tens;
	unsigned char hundreds;
	if (status->vCell < 3000) {
		tens = 0;
		hundreds = 0;
	} else {
		tens = (status->vCell / 10) % 10;
		hundreds = (status->vCell / 100) % 10;
	}
	for (int j = 0; j < hundreds; j++) {
		for (int k = 0; k < 10; k++) {
			printf("*");
		}
	}
	for (int j = 0; j < tens; j++) {
		printf("-");
	}
	write(1, "\E[K", 3);
	printf("\n");
	fflush(NULL);
}

double asDouble(int s) {
	return ((double) s) / 1000;
}

/**
 * Obtain version information about the specified cell and store it in the passed cell structure
 * @return true if version information was successfully obtained
 */
unsigned char getCellVersion(struct status_t *cell) {
	sendCommandV1(cell->cellId, 0, '?');
	unsigned char buf[10];
	int actualRead = readEnough(fd, buf, 10);
	if (actualRead != 10) {
		fprintf(stderr, "Expected 10, read %d while getting version for %d (%d)", actualRead, cell->cellIndex,
				cell->cellId);
		return 0;
	}
	crc_t expectedCrc = crc_init();
	expectedCrc = crc_update(expectedCrc, buf, 8);
	expectedCrc = crc_finalize(expectedCrc);
	crc_t actualCrc = bufToShortLE(buf + 8);
	if (actualCrc != expectedCrc) {
		fprintf(stderr, "crc missmatch %d != %d while getting version for %d (%d)", expectedCrc, actualCrc,
				cell->cellIndex, cell->cellId);
		return 0;
	}
	cell->version = buf[0];
	cell->revision = bufToShortLE(buf + 1);
	cell->isClean = buf[3];
	cell->whenProgrammed = bufToLongLE(buf + 4);
	return 1;
}

/** Interrogate cells and discover their version number */
void getSlaveVersions() {
	for (unsigned char i = 0; i < data.batteryCount; i++) {
		struct battery_t *battery = data.batteries + i;
		for (unsigned short j = 0; j < battery->cellCount; j++) {
			struct status_t *cell = battery->cells + j;
			printf("Checking cell %d (id %d) ...", j, cell->cellId);
			cell->errorCount = 0;
			if (!getCellVersion(cell)) {
				// some cells don't support getCellVersion() so we try both protocols and see which works
				cell->version = 0;
				printf("... trying version 0 ...");
				if (!_getCellState(cell, 2)) {
					printf("... trying version 1 ...");
					cell->version = 1;
					if (!_getCellState(cell, 2)) {
						printf("error getting version for cell %d (id %d)\n", i, cell->cellId);
						cell->errorCount = 1;
					}
				}
			}
			printf("... version %d r%d %s whenProgrammed %ld\n", cell->version, cell->revision,
					cell->isClean ? "clean" : "modified", cell->whenProgrammed);
		}
	}
}

void findCells() {
	struct status_t status;
	for (unsigned short i = 0; i < 255; i++) {
		status.cellId = i;
		status.version = 0;
		unsigned char found = 0;
		if (_getCellState(&status, 1)) {
			found = TRUE;
		} else {
			status.version = 1;
			if (!_getCellState(&status, 1)) {
				found = TRUE;
			}
		}
		if (found) {
			printf("found cell at %d\n", i);
		} else {
			printf("found nothing at %d\n", i);
		}
	}
}

void initData(struct config_t *config) {
	data.batteryCount = config->batteryCount;
	data.batteries = malloc(sizeof(struct battery_t) * data.batteryCount);
	for (unsigned char j = 0; j < data.batteryCount; j++) {
		struct battery_t *battery = data.batteries + j;
		battery->name = config->batteries[j].name;
		battery->cellCount = config->batteries[j].cellCount;
		battery->cells = calloc(sizeof(struct status_t), battery->cellCount);
		for (unsigned short k = 0; k < battery->cellCount; k++) {
			struct status_t *cell = battery->cells + k;
			cell->cellIndex = k;
			cell->cellId = config->batteries[j].cellIds[k];
			cell->battery = battery;
		}
	}
}
