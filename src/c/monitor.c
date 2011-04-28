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
#include "../../../slave/src/c/evd5.h"
#include "crc.h"
#include "chargercontrol.h"
#include "buscontrol.h"

#define BAUDRATE B9600
#define MODEMDEVICE "/dev/ttyS0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define CELL_ID_FILE "cells.txt"
#define DEBUG 0
#define CHARGER_RELAY_PORT 7
#define CHARGER_ON_VOLTAGE 3550
#define CHARGER_OFF_VOLTAGE 3650
#define SHUNT_ON_VOLTAGE 3500
#define SHUNT_MAX_CURRENT 500
#define FORCED_SHUNT_OFF_VOLTAGE 3530
#define CHARGE_CURRENT_OVERSAMPLING 5

void initCellIDArray();
void sendCommand(int address, char sequenceNumber, char command);
void getCellState(int cellIndex);
char _getCellState(int cellID, struct evd5_status_t* status, int attempts);
void writeSlowly(int fd, char *s, int length);
int readEnough(int fd, unsigned char *buf, int length);
int maxVoltage();
int maxVoltageCell();
int minVoltage();
int minVoltageCell();
int avgVoltage();
int totalVoltage();
void setShuntCurrent();
void setMinCurrent(int cellIndex, unsigned short minCurrent);
void dumpBuffer(unsigned char * buf, int length);
void findCells();

int fd;

struct evd5_status_t *cells;
int *cellIDs;
int cellCount;

char chargerState = 0;
int main()
{
    struct termios oldtio,newtio;

	if (chargercontrol_init()) {
		return 1;
	}

	if (buscontrol_init()) {
		return 1;
	}

	buscontrol_setBus(TRUE);
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
    if (fd <0) {
        perror(MODEMDEVICE);
        return -1;
    }

    tcgetattr(fd,&oldtio); /* save current port settings */

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 1;
    newtio.c_cc[VMIN]     = 10;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);

    initCellIDArray();

	char seq = '0';

	// send some bytes to wake up the slaves
	writeSlowly(fd, "garbage", 7);

	for (int i = 0; i < cellCount; i++) {
		sendCommand(cellIDs[i], seq++, 'r');
	}

	sleep(1);

	// send some bytes to wake up the slaves (they drop characters while flashing the light)
	writeSlowly(fd, "garbage", 7);

	// findCells();

	// clear the screen
	write(2,"\E[H\E[2J",7);

	chargerState = 0;
	printf("\n");
	time_t last = 0;
	while (1) {
		// move to the top of the screen
		write(2,"\E[H",3);
		time_t t;
		time(&t);
		if (t == last) {
			// TODO sleep instead of spinning
			continue;
		}
		last = t;
		printf("%d ", (int) t);
		printf("%lf ", chargercontrol_getChargeCurrent());
		for (int i = 0; i < cellCount; i++) {
			getCellState(i);
			printf("%5d %5d ", cells[i].vCell, cells[i].iShunt);
			struct evd5_status_t *status = &cells[i];
			fprintf(stderr, "%02d %02d Vc=%04d Vs=%04d Is=%04d It=%03d Q=? Vt=%05d Vg=%02d g=%02d hasRx=%d sa=%d auto=%d seq=%02x crc=%04x ", i, cellIDs[i], status->vCell, status->vShunt, status->iShunt, status->minCurrent, status->temperature, status->vShuntPot, status->gainPot, status->hasRx, status->softwareAddressing, status->automatic, status->sequenceNumber, status->crc);
			unsigned char tens = (status->vCell / 10) % 10;
			unsigned char hundreds = (status->vCell / 100) % 10;
			for (int j = 0; j < hundreds; j++) {
				fprintf(stderr, "*");
			}
			for (int j = 0; j < tens; j++) {
				fprintf(stderr, "-");
			}
			write(2,"\E[K",3);
			fprintf(stderr, "\n");
			fflush(NULL);
		}
		printf("\n");
		if (maxVoltage() > CHARGER_OFF_VOLTAGE) {
			chargercontrol_setCharger(FALSE);
			chargerState = 0;
 		} if (maxVoltage() < CHARGER_ON_VOLTAGE || chargerState) {
			chargercontrol_setCharger(TRUE);
			chargerState = 1;
		}
		fprintf(stderr, "%d@%02d %d %d@%02d %d %f %s\n", minVoltage(), minVoltageCell(), avgVoltage(), maxVoltage(), maxVoltageCell(), 
				totalVoltage(), chargercontrol_getChargeCurrent(), chargerState ? "on" : "off");
		setShuntCurrent();
		fflush(NULL);
	}
    	tcsetattr(fd,TCSANOW,&oldtio);
}

unsigned char sequenceNumber = 0;

void getCellState(int cellIndex) {
	char success = _getCellState(cellIDs[cellIndex], &cells[cellIndex], 4);
	if (!success) {
		printf("bus errors, exiting\n");
		chargercontrol_shutdown();
		exit(1);
	}
}

char _getCellState(int cellID, struct evd5_status_t* status, int maxAttempts) {
	int actualLength = 0;
	for (int attempt = 0; TRUE; attempt++) {
		if (attempt >= maxAttempts) {
			return 0;
			printf("%d bus errors, exiting\n", attempt);
			chargercontrol_shutdown();
			exit(1);
		}
		if (attempt > 0 && actualLength == 0) {
			fprintf(stderr, "no response, resetting\n");
			buscontrol_setBus(FALSE);
			sleep(1);
			buscontrol_setBus(TRUE);
			sleep(1);
		}
		unsigned char buf[255];
		unsigned char sentSequenceNumber;
		sentSequenceNumber = sequenceNumber++;
		sendCommand(cellID, sentSequenceNumber, '/');
		actualLength = readEnough(fd, buf, EVD5_STATUS_LENGTH);
		if (actualLength != EVD5_STATUS_LENGTH) {
			// fprintf(stderr, "read %d, expected %d from cell %d\n", actualLength, EVD5_STATUS_LENGTH, cellID);
			dumpBuffer(buf, actualLength);
			int secondLength = readEnough(fd, buf, 255);
			// fprintf(stderr, "read %d more\n", secondLength);
			dumpBuffer(buf, secondLength);
			continue;
		}
		unsigned short *actualCRC = (unsigned short *) (buf + EVD5_STATUS_LENGTH - sizeof(crc_t));
		crc_t expectedCRC = crc_init();
		expectedCRC = crc_update(expectedCRC, buf, EVD5_STATUS_LENGTH - sizeof(crc_t));
		expectedCRC = crc_finalize(expectedCRC);
		if (expectedCRC != *actualCRC) {
			fprintf(stderr, "\nSent message to %2d expected CRC 0x%04x got 0x%04x\n", cellID, expectedCRC, *actualCRC);
			dumpBuffer(buf, actualLength);
			continue;
		}
		memcpy(status, buf, EVD5_STATUS_LENGTH);
		// have to copy this one separately because of padding
		status->crc = *actualCRC;
		if (status->cellAddress != cellID) {
			fprintf(stderr, "\nSent message to %2d but recieved response from %x\n", cellID, status->cellAddress);
			dumpBuffer(buf, actualLength);
			continue;
		}
		if (status->sequenceNumber != sentSequenceNumber) {
			fprintf(stderr, "\nSent message to %2d with seq 0x%02x but recieved seq 0x%02x\n", cellID, sentSequenceNumber, status->sequenceNumber);
			dumpBuffer(buf, actualLength);
			continue;
		}
		break;
	}
	return 1;
}

void setShuntCurrent() {
	for (int i = 0; i < cellCount; i++) {
		unsigned short target;
		if (cells[i].vCell > SHUNT_ON_VOLTAGE) {
			short difference = cells[i].vCell - minVoltage();
			if (difference < 20) {
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

void setMinCurrent(int cellIndex, unsigned short minCurrent) {
	char command;
	unsigned char buf[255];
	if (minCurrent > SHUNT_MAX_CURRENT) {
			minCurrent = SHUNT_MAX_CURRENT;
	}
	for (int i = 0; i < 10; i++) {
		if (cells[cellIndex].minCurrent > minCurrent) {
			command = '<';
		} else if (cells[cellIndex].minCurrent < minCurrent) {
			command = '>';
		} else {
			return;
		}
		sendCommand(cellIDs[cellIndex], '0', command);
		readEnough(fd, buf, 5);
	}
	// couldn't get to desired current after 10 attempts???
	chargercontrol_shutdown();
	buf[6] = 0;
	char *endPtr;
	int actual = strtol(buf, &endPtr, 10);
	fprintf(stderr, "%2d trying to get to %d but had %d actual = %d\n", cellIDs[cellIndex], minCurrent, cells[cellIndex].minCurrent, actual);
	exit(1);
}

int minVoltage() {
	int result = 999999;
	for (int i = 0; i < cellCount; i++) {
		if (cells[i].vCell < result) {
			result = cells[i].vCell;
		}
	}
	return result;
}

int minVoltageCell() {
	int min = 999999;
	int result = 0;
	for (int i = 0; i < cellCount; i++) {
		if (cells[i].vCell < min) {
			min = cells[i].vCell;
			result = cellIDs[i];
		}
	}
	return result;
}

int maxVoltage() {
	int result = 0;
	for (int i = 0; i < cellCount; i++) {
		if (cells[i].vCell > result) {
			result = cells[i].vCell;
		}
	}
	return result;
}

int maxVoltageCell() {
	int max = 0;
	int result = 0;
	for (int i = 0; i < cellCount; i++) {
		if (cells[i].vCell > max) {
			max = cells[i].vCell;
			result = cellIDs[i];
		}
	}
	return result;
}

int totalVoltage() {
	int result = 0;
	for (int i = 0; i < cellCount; i++) {
		result += cells[i].vCell;
	}
	return result;
}

int avgVoltage() {
	return totalVoltage() / cellCount;
}

void sendCommand(int address, char sequence, char command) {
	char buf[] = "heloXXYX";
	// little endian
	buf[4] = (char) address & 0x00FF;
	buf[5] = (char) ((address & 0xFF00) >> 8);
	buf[6] = sequence;
	buf[7] = command;
	if (DEBUG) {
		fprintf(stderr, "sending command '%c' to 0x%x%x\n", buf[7], buf[4], buf[5]);
	}
	writeSlowly(fd, buf, 8);
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
		actual += read(fd, buf + actual, 255 - actual);
		buf[actual] = 0;
		if (DEBUG) {
			fprintf(stderr, "read %d expecting %d '%s' ", actual, length, buf);
			for (int j = 0; j < actual; j++) {
				fprintf(stderr, "%x ", buf[j]);
			}
			fprintf(stderr, "\n");
		}
		if (actual >= length) {
			break;
		}
	}
	return actual;
}

void dumpBuffer(unsigned char *buf, int length) {
	if (DEBUG) {
		for (int i = 0; i < length; i++) {
			fprintf(stderr, "%d %x\n", i,  buf[i]);
		}
	}
}

void initCellIDArray() {
	int id;
	int count = 0;
	FILE *in = fopen(CELL_ID_FILE, "r");
	while (EOF != fscanf(in, "%d\n", &id)) {
		count++;
	}
	fclose(in);
	fprintf(stderr, "Have %d cells\n", count);
	
	cellIDs = malloc(count * sizeof(int));
	cells = malloc(count * sizeof(struct evd5_status_t));
	cellCount = count;
	
	count = 0;
	in = fopen(CELL_ID_FILE, "r");
	while (EOF != fscanf(in, "%d\n", &id)) {
		cellIDs[count++] = id;
	}
	fclose(in);
}

void findCells() {
	struct evd5_status_t status;
	for (int i = 0; i < 255; i++) {
		_getCellState(i, &status, 1);
		if (status.cellAddress == i) {
			printf("found cell at %d\n", i);
		} else {
			printf("found nothing at %d\n", i);
		}
	}
}
