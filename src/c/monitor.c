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

#define BAUDRATE B9600
#define MODEMDEVICE "/dev/ttyS1"
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
#define SHUNT_MAX_CURRENT 500
#define FORCED_SHUNT_OFF_VOLTAGE 3530
#define CHARGE_CURRENT_OVERSAMPLING 5
#define LOOP_DELAY 10

void initData(struct config_t *config);
void sendCommand(unsigned char version, unsigned short address, char sequence, char command);
void sendCommandV0(unsigned short address, char sequenceNumber, char command);
void sendCommandV1(unsigned short address, char sequenceNumber, char command);
void getCellStates();
void getCellState(unsigned short cellIndex);
char _getCellState(unsigned short cellIndex, struct status_t *status, int attempts);
void writeSlowly(int fd, char *s, int length);
int readEnough(int fd, unsigned char *buf, int length);
unsigned short maxVoltage();
unsigned short maxVoltageCell();
unsigned short minVoltage();
unsigned short minVoltageCell();
unsigned short avgVoltage();
unsigned int totalVoltage();
void setShuntCurrent();
void setMinCurrent(unsigned short cellIndex, unsigned short minCurrent);
void dumpBuffer(unsigned char *buf, int length);
void findCells();
void evd5ToStatus(struct evd5_status_t *from, struct status_t *to);
void printSummary();
void printCellDetail(unsigned short cellIndex, struct status_t *status);
double asDouble(int s);
void turnOffAllShunts();
char isAnyCellShunting();
char isCellShunting(short cellIndex);
void flushInputBuffer();
void getSlaveVersions();

int fd;

struct status_t *cells;
unsigned short cellCount;

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
	
	if (logger_init()) {
		return 1;
	}

	buscontrol_setBus(TRUE);
	fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		perror(MODEMDEVICE);
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

	for (int i = 0; i < cellCount; i++) {
		sendCommand(0, cells[i].cellId, seq++, 'r');
		sendCommand(1, cells[i].cellId, seq++, 'r');
	}

	sleep(1);

	// send some bytes to wake up the slaves (they drop characters while flashing the light)
	writeSlowly(fd, "garbage", 7);

	// findCells();

	getSlaveVersions();

	// clear the screen
	write(2, "\E[H\E[2J", 7);

	chargerState = 0;
	time_t last = 0;
	char shutdown = 0;
	while (1) {
		time_t t;
		time(&t);
		if (t < last + LOOP_DELAY) {
			sleep(1);
			continue;
		}
		last = t;
		if (!HAS_KELVIN_CONNECTION) {
			turnOffAllShunts();
		}
		getCellStates();
		printf("\n");
		if (maxVoltage() > CHARGER_OFF_VOLTAGE) {
			chargercontrol_setCharger(FALSE);
			chargerState = 0;
			shutdown = 1;
		}
		if (maxVoltage() < CHARGER_ON_VOLTAGE || chargerState) {
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
		setShuntCurrent();
		fflush(NULL);

		// If we don't have a kelvin connection then we will be turning off the shunts to read the voltages.
		// This causes problems because the cells are quite slow to turn on the shunts. Here we read the
		// shunt current a few times without turning off the shunts to let them have time to do their work
		//
		// TODO implement "suspend" command to tell slaves to stop shunting and store their shunt config for
		// later fast re-enabling
		for (int j = 0; !HAS_KELVIN_CONNECTION && isAnyCellShunting() && j < 5; j++) {
			sleep(2);
			getCellStates();
		}
	}
	tcsetattr(fd, TCSANOW, &oldtio);
}

void getCellStates() {
	// move to the top of the screen
	write(2, "\E[H", 3);
	for (int i = 0; i < cellCount; i++) {
		getCellState(i);
		if (!isCellShunting(i)) {
			// the voltage doesn't mean much when we are drawing current
			montiorCan_sendCellVoltage(i, cells[i].vCell);
		}
		monitorCan_sendShuntCurrent(i, cells[i].iShunt);
		monitorCan_sendMinCurrent(i, cells[i].minCurrent);
		monitorCan_sendTemperature(i, cells[i].temperature);
		struct status_t *status = &cells[i];
		printCellDetail(i, status);
	}
}

unsigned char sequenceNumber = 0;

void getCellState(unsigned short cellIndex) {
	char success = _getCellState(cellIndex, &cells[cellIndex], 4);
	if (!success) {
		printf("bus errors talking to cell %d (id %d), exiting\n", cellIndex, cells[cellIndex].cellId);
		chargercontrol_shutdown();
		exit(1);
	}
}

char _getCellState(unsigned short cellIndex, struct status_t *status, int maxAttempts) {
	int actualLength = 0;
	for (int attempt = 0; TRUE; attempt++) {
		if (attempt >= maxAttempts) {
			return 0;
			fprintf(stderr, "%d bus errors, exiting\n", attempt);
			chargercontrol_shutdown();
			exit(1);
		}
		if (attempt > 0 && actualLength == 0) {
			fprintf(stderr, "no response from %d (id %d), resetting\n", cellIndex, status->cellId);
			buscontrol_setBus(FALSE);
			sleep(1);
			buscontrol_setBus(TRUE);
			sleep(1);
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
			fprintf(stderr, "read %d, expected %d from cell %d\n", actualLength, EVD5_STATUS_LENGTH, status->cellId);
			dumpBuffer(buf, actualLength);
			flushInputBuffer();
			continue;
		}
		unsigned short *actualCRC = (unsigned short *) (buf + EVD5_STATUS_LENGTH - sizeof(crc_t));
		crc_t expectedCRC = crc_init();
		expectedCRC = crc_update(expectedCRC, buf, EVD5_STATUS_LENGTH - sizeof(crc_t));
		expectedCRC = crc_finalize(expectedCRC);
		if (expectedCRC != *actualCRC) {
			fprintf(stderr, "\nSent message to %2d (id %2d) expected CRC 0x%04x got 0x%04x\n", cellIndex,
					cells[cellIndex].cellId, expectedCRC, *actualCRC);
			dumpBuffer(buf, actualLength);
			flushInputBuffer();
			continue;
		}
		struct evd5_status_t evd5Status;
		memcpy(&evd5Status, buf, EVD5_STATUS_LENGTH);
		// have to copy this one separately because of padding
		evd5Status.crc = *actualCRC;
		if (evd5Status.cellAddress != status->cellId) {
			fprintf(stderr, "\nSent message to %2d (id %2d) but recieved response from %x\n", cellIndex, status->cellId,
					evd5Status.cellAddress);
			dumpBuffer(buf, actualLength);
			flushInputBuffer();
			continue;
		}
		if (evd5Status.sequenceNumber != sentSequenceNumber) {
			fprintf(stderr, "\nSent message to %2d (id %2d) with seq 0x%02x but received seq 0x%02hhx\n", cellIndex,
					status->cellId, sentSequenceNumber, evd5Status.sequenceNumber);
			dumpBuffer(buf, actualLength);
			flushInputBuffer();
			continue;
		}
		evd5ToStatus(&evd5Status, &cells[cellIndex]);
		cells[cellIndex].latency = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
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
	if (!isCellShunting(to->cellIndex)) {
		to->vCell = from->vCell;
	}
	to->vShunt = from->vShunt;
	to->vShuntPot = from->vShuntPot;
	to->crc = from->crc;
}

/** turn shunting off on any cells that are shunting */
void turnOffAllShunts() {
	char changed = 0;
	for (int i = 0; i < cellCount; i++) {
		struct status_t cell = cells[i];
		if (cell.minCurrent != 0 || cell.targetShuntCurrent != 0) {
			setMinCurrent(i, 0);
			changed = 1;
		}
	}
	if (changed) {
		// give slaves time to process
		sleep(2);
	}
}

void setShuntCurrent() {
	for (int i = 0; i < cellCount; i++) {
		unsigned short target;
		if (cells[i].vCell > SHUNT_ON_VOLTAGE) {
			short difference = cells[i].vCell - minVoltage();
			if (difference < 50) {
				target = 0;
			} else {
				target = (difference / 20) * 50 + 50;
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
		setMinCurrent(i, target);
	}
}

void setMinCurrent(unsigned short cellIndex, unsigned short minCurrent) {
	char command;
	unsigned char buf[7];
	if (minCurrent > SHUNT_MAX_CURRENT) {
		minCurrent = SHUNT_MAX_CURRENT;
	}
	cells[cellIndex].targetShuntCurrent = minCurrent;
	int actual = SHUNT_MAX_CURRENT + 1;
	for (int i = 0; i < 20; i++) {
		if (actual > minCurrent) {
			command = '<';
		} else if (actual < minCurrent) {
			command = '>';
		} else {
			return;
		}
		sendCommand(cells[cellIndex].version, cells[cellIndex].cellId, '0', command);
		readEnough(fd, buf, 7);
		buf[6] = 0;
		char *endPtr;
		actual = strtol((char *) buf, &endPtr, 10);
	}
	// couldn't get to desired current after 10 attempts???
	chargercontrol_shutdown();
	getCellState(cellIndex);
	fprintf(stderr, "%2d (id %2d) trying to get to %d but had %d actual = %d\n", cellIndex, cells[cellIndex].cellId,
			minCurrent, cells[cellIndex].minCurrent, actual);
	exit(1);
}

unsigned short minVoltage() {
	int result = 999999;
	for (int i = 0; i < cellCount; i++) {
		if (cells[i].vCell < result) {
			result = cells[i].vCell;
		}
	}
	return result;
}

unsigned short minVoltageCell() {
	int min = 999999;
	int result = 0;
	for (int i = 0; i < cellCount; i++) {
		if (cells[i].vCell < min) {
			min = cells[i].vCell;
			result = i;
		}
	}
	return result;
}

unsigned short maxVoltage() {
	unsigned result = 0;
	for (int i = 0; i < cellCount; i++) {
		if (cells[i].vCell > result) {
			result = cells[i].vCell;
		}
	}
	return result;
}

unsigned short maxVoltageCell() {
	int max = 0;
	unsigned short result = 0;
	for (int i = 0; i < cellCount; i++) {
		if (cells[i].vCell > max) {
			max = cells[i].vCell;
			result = i;
		}
	}
	return result;
}

unsigned int totalVoltage() {
	int result = 0;
	for (int i = 0; i < cellCount; i++) {
		result += cells[i].vCell;
	}
	return result;
}

unsigned short avgVoltage() {
	return totalVoltage() / cellCount;
}

/**
 * @return true if any cell is shunting
 */
char isAnyCellShunting() {
	for (int j = 0; j < cellCount; j++) {
		if (cells[j].targetShuntCurrent > 0) {
			return 1;
		}
	}
	return 0;
}

/**
 * Returns true if the cell at index cellIndex or an adjacent cell is shunting current. This works
 * even when cells are out of order because each board has it's own end connections and we are careful
 * not to reorder cells within a board.
 */
char isCellShunting(short cellIndex) {
	if (HAS_KELVIN_CONNECTION) {
		return 0;
	}
	if (cells[cellIndex].targetShuntCurrent != 0) {
		return 1;
	}
	if (cellIndex != 0 && cells[cellIndex - 1].targetShuntCurrent != 0) {
		return 1;
	}
	if (cellIndex + 1 < cellCount && cells[cellIndex + 1].targetShuntCurrent != 0) {
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
	unsigned char buf[6] = "XXYZCC";
	// little endian
	buf[0] = (unsigned char) address & 0x00FF;
	buf[1] = (unsigned char) ((address & 0xFF00) >> 8);
	buf[2] = sequence;
	buf[3] = command;
	crc_t crc = crc_init();
	crc = crc_update(crc, buf, 4);
	crc = crc_finalize(crc);
	buf[4] = (unsigned char) crc & 0x00FF;
	buf[5] = (unsigned char) ((crc & 0xFF00) >> 8);
	if (DEBUG) {
		fprintf(stderr, "sending command '%c' to 0x%02x%02x with CRC 0x%02x%02x\n", buf[3], buf[0], buf[1], buf[4], buf[5]);
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
	fprintf(stderr, "%.3f@%02d %.3f %.3f@%02d %6.3fV %6.2fV %7.2fA %7.2fAh %s\n", asDouble(minVoltage()),
			minVoltageCell(), asDouble(avgVoltage()), asDouble(maxVoltage()), maxVoltageCell(),
			asDouble(totalVoltage()), soc_getVoltage(), soc_getCurrent(), soc_getAh(), chargerState ? "on" : "off");
}

void printCellDetail(unsigned short cellIndex, struct status_t *status) {
	if (status->minCurrent > 0) {
		write(2, "\E[31m", 5);
	} else {
		write(2, "\E[m", 3);
	}
	fprintf(
			stderr,
			"%02d %02d Vc=%.3f Vs=%.3f Is=%.3f It=%5.3f t=%5.1f s=%02d g=%02d hasRx=%d sa=%d auto=%d seq=%02hhx crc=%04hx %ld ",
			cellIndex, status->cellId, asDouble(status->vCell), asDouble(status->vShunt), asDouble(status->iShunt),
			asDouble(status->minCurrent), asDouble(status->temperature) * 10, status->vShuntPot, status->gainPot,
			status->hasRx, status->softwareAddressing, status->automatic, status->sequenceNumber, status->crc, status->latency / 1000);
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
			fprintf(stderr, "*");
		}
	}
	for (int j = 0; j < tens; j++) {
		fprintf(stderr, "-");
	}
	write(2, "\E[K", 3);
	fprintf(stderr, "\n");
	fflush(NULL);

}

double asDouble(int s) {
	return ((double) s) / 1000;
}

/** Interrogate cells and discover their version number */
void getSlaveVersions() {
	// the cells don't support getVersion() so we try both protocols and see which works
	for (unsigned short i = 0; i < cellCount; i++) {
		struct status_t cell;
		cell.cellId = cells[i].cellId;
		cell.cellIndex = i;
		printf("Checking cell %d (id %d) ...", i, cell.cellId);
		cell.version = 0;
		if (!_getCellState(i, &cell, 20)) {
			printf("... trying version 1 ...");
			cell.version = 1;
			if (!_getCellState(i, &cell, 20)) {
				printf("error getting version for cell %d (id %d)\n", i, cell.cellId);
				exit(1);
			}
		}
		cells[i].version = cell.version;
		printf("... version %d\n", cell.version);
	}
}

void findCells() {
	struct status_t status;
	for (unsigned short i = 0; i < 255; i++) {
		status.cellId = i;
		status.version = 0;
		unsigned char found = 0;
		if (_getCellState(0, &status, 1)) {
			found = TRUE;
		} else {
			status.version = 1;
			if (!_getCellState(0, &status, 1)) {
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
	cellCount = config->cellCount;
	cells = calloc(sizeof(struct status_t), cellCount);
	for (unsigned short i = 0; i < cellCount; i++) {
		cells[i].cellId = config->cellIds[i];
	}
}
