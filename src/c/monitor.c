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
#include "chargercontrol.h"

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
int readEnough(int fd, char *buf, int length);
int maxVoltage();
int minVoltage();
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
    int res;
    struct termios oldtio,newtio;
    char buf[255];

	if (chargercontrol_init()) {
		return 1;
	}

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
		sendCommand(cellIDs[i], seq++, 'g');
		cells[i].minCurrent = 500;
		setMinCurrent(i, 50);
	}
	sleep(3);
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
			sleep(1);
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
		fprintf(stderr, "%d %d %d %d %s\n", minVoltage(), avgVoltage(), maxVoltage(), totalVoltage(), chargerState ? "on" : "off");
		turnDownCells();
		fflush(NULL);
	}
    	tcsetattr(fd,TCSANOW,&oldtio);
}

unsigned char sequenceNumber = 0;

void getCellState(int cellIndex) {
	char buf[255];
	unsigned char sentSequenceNumber, recSequenceNumber;
	int actualLength;
	struct evd5_status_t *status = &cells[cellIndex];
	sentSequenceNumber = sequenceNumber++;
	sendCommand(cellIDs[cellIndex], sentSequenceNumber, '/');
	actualLength = readEnough(fd, buf, EVD5_STATUS_LENGTH);
	if (actualLength > 12) {
		recSequenceNumber = (unsigned char) buf[12];
	}
	if (actualLength != EVD5_STATUS_LENGTH || sentSequenceNumber != recSequenceNumber) {
		int i;
		printf("read %d, expected %d from cell %d sent seq %x, rec seq %x\n", actualLength, EVD5_STATUS_LENGTH, cellIndex, 
				sentSequenceNumber, recSequenceNumber);
		for (i = 0; i < actualLength; i++) {
			printf("%d %x\n", i, (unsigned char) buf[i]);
		}
		actualLength = readEnough(fd, buf, 255);
		printf("read %d more\n", actualLength);
		for (i = 0; i < actualLength; i++) {
			printf("%d %x\n", i, (unsigned char) buf[i]);
		}
		exit(1);
		return;
	}
	memcpy(status, buf, EVD5_STATUS_LENGTH);
	char *sc = (char *) status;
	sc[0] = buf[1];
	sc[1] = buf[0];
	if (status->cellAddress != cellIDs[cellIndex]) {
		printf("\nSent message to %x but recieved response from %x\n", cellIDs[cellIndex], status->cellAddress);
		for (int i = 0; i < actualLength; i++) {
                        printf("%d %x\n", i, (unsigned char) buf[i]);
                }
		exit(1);
	}
	//printf("Vc=%d Vs=%d Is=%d Q=? Vt=%d Vg=%d g=%d hasRx=%d sa=%d auto=%d\n", status->vCell, status->vShunt, status->iShunt, status->temperature, status->vShuntPot, status->gainPot, status->hasRx, status->softwareAddressing, status->automatic);
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
	char buf[255];
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
		fprintf(stderr, "%c%c actual = %d\n", (char) ((cellIDs[cellIndex] & 0xFF00) >> 8), (char) cellIDs[cellIndex], actual);
		cells[cellIndex].minCurrent = actual;
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

int maxVoltage() {
	int result = 0;
	for (int i = 0; i < CELL_COUNT; i++) {
		if (cells[i].vCell > result) {
			result = cells[i].vCell;
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
	buf[4] = (char) ((address & 0xFF00) >> 8);
	buf[5] = (char) address & 0x00FF;
	buf[6] = sequence;
	buf[7] = command;
	if (DEBUG) {
		printf("sending command '%c' to 0x%x%x\n", buf[7], buf[4], buf[5]);
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

int readEnough(int fd, char *buf, int length) {

	fd_set rfds;
	struct timeval tv;
	int retval;
	
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
			printf("read %d expecting %d '%s' ", actual, length, buf);
			for (int j = 0; j < actual; j++) {
				printf("%x ", buf[j]);
			}
			printf("\n");
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