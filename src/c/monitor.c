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

#define DEBUG 0
#define CELL_COUNT 10
#define CELL_ID_BASE 0
#define CHARGER_RELAY_PORT 7
#define CHARGER_ON_VOLTAGE 3550
#define CHARGER_OFF_VOLTAGE 3650
#define FORCED_SHUNT_OFF_VOLTAGE 3530
#define LJ_ID -1
#define CHARGE_CURRENT_OVERSAMPLING 5
#define CHARGE_CURRENT_CHANNEL 4

void initCellIDArray();
void sendCommand(int address, char sequenceNumber, char command);
void getCellState(int cellIndex);
void writeSlowly(int fd, char *s, int length);
int readEnough(int fd, unsigned char *buf, int length);
int maxVoltage();
int maxVoltageCell();
int minVoltage();
int minVoltageCell();
int avgVoltage();
int totalVoltage();
void turnUpHighCells();
void turnDownCells();
void setMinCurrent(int cellIndex, short minCurrent);

int fd;

struct evd5_status_t cells[CELL_COUNT];

//int cellIDs[CELL_COUNT] = { 0x3030, 0x3032, 0x3033 };
//int cellIDs[CELL_COUNT] = { 0x3035, 0x3038, 0x3039 };
//int cellIDs[CELL_COUNT] = { 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9 };
int cellIDs[CELL_COUNT];

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

	for (int i = 0; i < CELL_COUNT; i++) {
		sendCommand(cellIDs[i], seq++, 'r');
	}

	sleep(1);

	// send some bytes to wake up the slaves (they drop characters while flashing the light)
	writeSlowly(fd, "garbage", 7);

	chargerState = 0;
	printf("\n");
	time_t last = 0;
	while (1) {
		time_t t;
		time(&t);
		if (t == last) {
			// TODO sleep instead of spinning
			continue;
		}
		last = t;
		printf("%d ", (int) t);
		printf("%lf ", chargercontrol_getChargeCurrent());
		for (int i = 0; i < CELL_COUNT; i++) {
			getCellState(i);
			printf("%5d %5d ", cells[i].vCell, cells[i].iShunt);
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
		turnDownCells();
		fflush(NULL);
	}
    	tcsetattr(fd,TCSANOW,&oldtio);
}

unsigned char sequenceNumber = 0;

void getCellState(int cellIndex) {
	int actualLength = 0;
	for (int attempt = 0; TRUE; attempt++) {
		if (attempt > 4) {
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
		struct evd5_status_t *status = &cells[cellIndex];
		sentSequenceNumber = sequenceNumber++;
		sendCommand(cellIDs[cellIndex], sentSequenceNumber, '/');
		actualLength = readEnough(fd, buf, EVD5_STATUS_LENGTH);
		if (actualLength != EVD5_STATUS_LENGTH) {
			fprintf(stderr, "read %d, expected %d from cell %d\n", actualLength, EVD5_STATUS_LENGTH, cellIDs[cellIndex]);
			for (int i = 0; i < actualLength; i++) {
				fprintf(stderr, "%d %x\n", i, (unsigned char) buf[i]);
			}
			int secondLength = readEnough(fd, buf, 255);
			fprintf(stderr, "read %d more\n", secondLength);
			for (int i = 0; i < secondLength; i++) {
				fprintf(stderr, "%d %x\n", i, (unsigned char) buf[i]);
			}
			continue;
		}
		unsigned short *actualCRC = (unsigned short *) (buf + EVD5_STATUS_LENGTH - sizeof(crc_t));
		crc_t expectedCRC = crc_init();
		expectedCRC = crc_update(expectedCRC, buf, EVD5_STATUS_LENGTH - sizeof(crc_t));
		expectedCRC = crc_finalize(expectedCRC);
		if (expectedCRC != *actualCRC) {
			fprintf(stderr, "\nSent message to %2d expected CRC 0x%04x got 0x%04x\n", cellIDs[cellIndex], expectedCRC, *actualCRC);
				for (int i = 0; i < actualLength; i++) {
					fprintf(stderr, "%d %x\n", i, (unsigned char) buf[i]);
			}
			continue;
		}
		memcpy(status, buf, EVD5_STATUS_LENGTH);
		// have to copy this one separately because of padding
		status->crc = *actualCRC;
		if (status->cellAddress != cellIDs[cellIndex]) {
			fprintf(stderr, "\nSent message to %2d but recieved response from %x\n", cellIDs[cellIndex], status->cellAddress);
			for (int i = 0; i < actualLength; i++) {
				fprintf(stderr, "%d %x\n", i, (unsigned char) buf[i]);
			}
			continue;
		}
		if (status->sequenceNumber != sentSequenceNumber) {
			fprintf(stderr, "\nSent message to %2d with seq 0x%02x but recieved seq 0x%02x\n", cellIDs[cellIndex], sentSequenceNumber, status->sequenceNumber);
			for (int i = 0; i < actualLength; i++) {
				fprintf(stderr, "%d %x\n", i, (unsigned char) buf[i]);
			}
			continue;
		}
		//fprintf(stderr, "\nVc=%d Vs=%d Is=%d Q=? Vt=%d Vg=%d g=%d hasRx=%d sa=%d auto=%d crc=%x\n", status->vCell, status->vShunt, status->iShunt, status->temperature, status->vShuntPot, status->gainPot, status->hasRx, status->softwareAddressing, status->automatic, status->crc);
		break;
	}
}

void turnUpHighCells() {
	for (int i = 0; i < CELL_COUNT; i++) {
		if (cells[i].vCell > CHARGER_ON_VOLTAGE) {
			setMinCurrent(i, 500);
		}
	}
}

void turnDownCells() {
	for (int i = 0; i < CELL_COUNT; i++) {
		if (cells[i].vCell > CHARGER_ON_VOLTAGE - 10 && !chargerState) {
			setMinCurrent(i, 500);
		} else {
			setMinCurrent(i, 0);
		}
	}
}

void setMinCurrent(int cellIndex, short minCurrent) {
	char command;
	unsigned char buf[255];
	while(1) {
		if (cells[cellIndex].minCurrent > 32000) {
			command = '>';
		} else if (cells[cellIndex].minCurrent > minCurrent) {
			command = '<';
		} else if (cells[cellIndex].minCurrent < minCurrent) {
			command = '>';
		} else {
			return;
		}
		sendCommand(cellIDs[cellIndex], '0', command);
		readEnough(fd, buf, 5);
		buf[6] = 0;
		char *endPtr;
		int actual = strtol(buf, &endPtr, 10);
		fprintf(stderr, "%2d actual = %d\n", cellIDs[cellIndex], actual);
		getCellState(cellIndex);
	}
}

int minVoltage() {
	int result = 999999;
	for (int i = 0; i < CELL_COUNT; i++) {
		if (cells[i].vCell < result) {
			result = cells[i].vCell;
		}
	}
	return result;
}

int minVoltageCell() {
	int min = 999999;
	int result = 0;
	for (int i = 0; i < CELL_COUNT; i++) {
		if (cells[i].vCell < min) {
			min = cells[i].vCell;
			result = i;
		}
	}
	return result;
}

int maxVoltage() {
	int result = 0;
	for (int i = 0; i < CELL_COUNT; i++) {
		if (cells[i].vCell > result) {
			result = cells[i].vCell;
		}
	}
	return result;
}

int maxVoltageCell() {
	int max = 0;
	int result = 0;
	for (int i = 0; i < CELL_COUNT; i++) {
		if (cells[i].vCell > max) {
			max = cells[i].vCell;
			result = i;
		}
	}
	return result;
}

int totalVoltage() {
	int result = 0;
	for (int i = 0; i < CELL_COUNT; i++) {
		result += cells[i].vCell;
	}
	return result;
}

int avgVoltage() {
	return totalVoltage() / CELL_COUNT;
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

	tv.tv_sec = 5;
	tv.tv_usec = 0;

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

void initCellIDArray() {
	for (int i = 0; i < CELL_COUNT; i++) {
		cellIDs[i] = CELL_ID_BASE + i;
	}
}
