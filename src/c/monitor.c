#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include "u3.h"
#include "../../../slave/src/c/evd5.h"

#define BAUDRATE B9600
#define MODEMDEVICE "/dev/ttyS0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define CELL_COUNT 10
#define CHARGER_RELAY_PORT 7
#define CHARGER_ON_VOLTAGE 3550
#define CHARGER_OFF_VOLTAGE 3650
#define FORCED_SHUNT_OFF_VOLTAGE 3530
#define LJ_ID -1
#define CHARGE_CURRENT_OVERSAMPLING 5
#define CHARGE_CURRENT_CHANNEL 4

void sendCommand(int address, char sequenceNumber, char command);
void writeSlowly(int fd, char *s, int length);
int readEnough(int fd, char *buf, int length);
int maxVoltage();
int minVoltage();
int avgVoltage();
int totalVoltage();
int setWatchdog(HANDLE hDevice);
void turnUpHighCells();
void turnDownCells();
void setMinCurrent(int cellIndex, short minCurrent);
double getReading(int channel);
double getChargeCurrent();

int fd;

struct evd5_status_t cells[CELL_COUNT];

//int cellIDs[CELL_COUNT] = { 0x3030, 0x3032, 0x3033 };
//int cellIDs[CELL_COUNT] = { 0x3035, 0x3038, 0x3039 };
int cellIDs[10] = { 0x3030, 0x3031, 0x3032, 0x3033, 0x3034, 0x3035, 0x3036, 0x3037, 0x3038, 0x3039 };

char chargerState = 0;

HANDLE hDevice;
double chargeCurrentZero;
u3CalibrationInfo caliInfo;
HANDLE hDevice;
long DAC1Enable;

int main()
{
    int res;
    struct termios oldtio,newtio;
    char buf[255];

	if( (hDevice = openUSBConnection(LJ_ID)) == NULL) {
		fprintf(stderr, "no labjack?\n");
    		return 1;
	}
	
	//Get calibration information from UE9
	if(getCalibrationInfo(hDevice, &caliInfo) < 0)
		return(1);

	// turn off charger
	eDO(hDevice, 1, CHARGER_RELAY_PORT, 0);
	setWatchdog(hDevice);

	chargeCurrentZero = getReading(4);
	fprintf(stderr, "voltage at zero current is %lf\n", chargeCurrentZero);
	
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
		printf("%lf ", getChargeCurrent());
		for (int i = 0; i < CELL_COUNT; i++) {
			int cellID = cellIDs[i];
			sendCommand(cellID, seq++, 'c');
			res = readEnough(fd, buf, 10);
			if (res != 10) {
				buf[res] = 0;
				printf("not enough characters? %d %s\n", res, buf);
				return -1;
			}
			char *endptr;
			cells[i].vCell = strtol(buf + 3, &endptr, 10);
			if (endptr != buf + 8) {
				cells[i].vCell = -1;
			}
			printf("%5d ", cells[i].vCell);
			fflush(NULL);
 			sendCommand(cellID, seq++, 'i');
			res = readEnough(fd, buf, 10);
			if (res != 10) {
				printf("not enough characters? %d %s\n", res, buf);
				return -1;
			}
			cells[i].iShunt = strtol(buf + 3, &endptr, 10);
			if (endptr != buf + 8) {
				cells[i].iShunt = -1;
			}
			printf("%5d ", cells[i].iShunt);
			fflush(NULL);
		}
		printf("\n");
		if (maxVoltage() > CHARGER_OFF_VOLTAGE) {
			eDO(hDevice, 1, CHARGER_RELAY_PORT, 0);
			chargerState = 0;
 		} if (maxVoltage() < CHARGER_ON_VOLTAGE || chargerState) {
			eDO(hDevice, 1, CHARGER_RELAY_PORT, 1);
			chargerState = 1;
		}
		fprintf(stderr, "%d %d %d %d %s\n", minVoltage(), avgVoltage(), maxVoltage(), totalVoltage(), chargerState ? "on" : "off");
		turnDownCells();
		fflush(NULL);
	}
    	tcsetattr(fd,TCSANOW,&oldtio);
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
//		buf[actual] = 0;
//		printf("read %d expecting %d %s\n", actual, length, buf);
		if (actual >= length) {
			break;
		}
	}
	return actual;
}

int setWatchdog(HANDLE hDevice) {
  
  uint8 sendBuff[26];
  uint8 recBuff[38];
  uint16 checksumTotal;
  int sendChars, recChars;

  //Setting all bytes to zero since we only want to read back the U3
  //configuration settings
  for(int i = 0; i < 26; i++)
    sendBuff[i] = 0;

  sendBuff[1] = (uint8)(0xF8);  //Command byte
  sendBuff[2] = (uint8)(0x05);  //Number of data words
  sendBuff[3] = (uint8)(0x09);  //Extended command number
  sendBuff[6] = 0x01;           // write?
//  sendBuff[7] = 0x18;           // set DO and reset
  sendBuff[7] = 0x10;           // reset
  sendBuff[8] = 0x14;           // timeout MSB
  sendBuff[9] = 0x00;           // timeout LSB
  sendBuff[10] = CHARGER_RELAY_PORT;          // set DO 4 to low
  sendBuff[0] = extendedChecksum8(sendBuff);
  extendedChecksum(sendBuff, 16);

  //Sending command to U3
  if( (sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, 16)) < 16)
  {
    if(sendChars == 0)
      printf("ConfigU3 error : write failed\n");
    else
      printf("ConfigU3 error : did not write all of the buffer\n");
    return -1;
  }

  //Reading response from U3
  if( (recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, recBuff, 16)) < 16)
  {
    if(recChars == 0)
      printf("ConfigU3 error : read failed\n");
    else
      printf("ConfigU3 error : did not read all of the buffer\n");
    return -1;
  }
  
  checksumTotal = extendedChecksum16(recBuff, 16);
  if( (uint8)((checksumTotal / 256) & 0xff) != recBuff[5])
  {
    printf("ConfigU3 error : read buffer has bad checksum16(MSB)\n");
    return -1;
  }
  if( (uint8)(checksumTotal & 0xff) != recBuff[4])
  {
    printf("ConfigU3 error : read buffer has bad checksum16(LBS)\n");
    return -1;
  }
 
  if( recBuff[6] != 0)
  {
    printf("ConfigU3 error : read buffer received errorcode %d\n", recBuff[6]);
    return -1;
  }
  /*for (int i = 0; i < 16; i++) {
     printf("%x ", recBuff[i]);
  }
  printf("\n"); */
  return 0;
}

double getReading(int channel) {
	double value = 0;
	for (int j = 0; j < CHARGE_CURRENT_OVERSAMPLING; j++) {
		double singleValue;
		if((eAIN(hDevice, &caliInfo, 1, &DAC1Enable, channel, 31, &singleValue, 0, 0, 0, 0, 0, 0)) != 0)
			exit(1);
		value += singleValue;
	}
	return value / CHARGE_CURRENT_OVERSAMPLING;
}

double getChargeCurrent() {
	double value = getReading(CHARGE_CURRENT_CHANNEL);
	double result = (chargeCurrentZero - value) / 0.020 * (3 / 2.7);
	//printf("%lf %lf %lf\n", result, value, zero);
	return result;
}
