/*
 Copyright 2009-2013 Tom Parker

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

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <strings.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <unistd.h>

#include "config.h"

#define BAUDRATE B9600

#define SYS_CLASS_TTY "/sys/class/tty/"
#define DEVICE_UEVENT "/device/uevent"
#define DEV "/dev/"
#define DRIVER "DRIVER=pl2303"

static int fd = -1;

static struct config_t *staticConfig;

static void getDriver(char *deviceName, char *destination, int length) {
	char ueventFileName[strlen(SYS_CLASS_TTY) + strlen(deviceName) + strlen(DEVICE_UEVENT)];
	strcpy(ueventFileName, SYS_CLASS_TTY);
	strcpy(ueventFileName + strlen(SYS_CLASS_TTY), deviceName);
	strcpy(ueventFileName + strlen(SYS_CLASS_TTY) + strlen(deviceName), DEVICE_UEVENT);
	FILE *ueventFile = fopen(ueventFileName, "r");
	while (fread(destination, 1, length, ueventFile) != 0) {
		;
	}
	fclose(ueventFile);
}

static void findSerialPort(char *result, int length) {
	   DIR *dirp = opendir("/sys/class/tty");
	   struct dirent *dp;
	   char *deviceShortName = NULL;
	   while ((dp = readdir(dirp)) != NULL) {
		   if (!strncmp(dp->d_name, "ttyUSB", strlen("ttyUSB"))) {
			   char driver[20];
			   getDriver(dp->d_name, driver, 20);
			   if (strncmp(driver, DRIVER, strlen(DRIVER)) == 0) {
			   	   deviceShortName = dp->d_name;
			   }
		   }
	   }
	   if (deviceShortName == NULL) {
		   *result = 0;
	   } else {
		   strncpy(result, DEV, length - strlen(DEV));
		   strncpy(result + strlen(DEV), deviceShortName, length - strlen(deviceShortName) - strlen(DEV));
	   }
	   (void)closedir(dirp);
}


int serial_openSerialPort(struct config_t *config) {
	staticConfig = config;
	char serialPort[20];
	if (config->serialPort) {
		strncpy(serialPort, config->serialPort, 20);
	} else {
		serialPort[0] = 0;
		findSerialPort(serialPort, 20);
		if (strlen(serialPort) == 0) {
			fprintf(stderr, "could not find serial port\n");
			return 1;
		}
	}
	fprintf(stderr, "opening serial port %s\n", serialPort);
	if (fd != -1) {
		close(fd);
	}
	fd = open(serialPort, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		perror(serialPort);
		return -1;
	}
	struct termios oldtio, newtio;

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
	return 0;
}

void serial_writeSlowly(unsigned char *s, int length) {
	//printf("%s\n", s);
	for (int i = 0; i < length; i++) {
		write(fd, s + i, 1);
		//usleep(100000);
	}
}

int serial_readEnough(unsigned char *buf, int length) {
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
		if (0) {
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
	if (actual != length) {
		sleep(1);
		serial_openSerialPort(staticConfig);
	}
	return actual;
}
