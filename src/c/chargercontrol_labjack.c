/*
    Copyright 2009, 2010 Tom Parker

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

#include "u3.h"
#include "chargercontrol.h"
#include "buscontrol.h"

#define CHARGER_RELAY_PORT 7
#define BUS_RELAY_PORT 6
#define LJ_ID -1
#define CHARGE_CURRENT_OVERSAMPLING 5
#define CHARGE_CURRENT_CHANNEL 4

int setWatchdog(HANDLE hDevice, char reset);
double getReading(int channel);

int fd;

HANDLE hDevice;
double chargeCurrentZero;
u3CalibrationInfo caliInfo;
HANDLE hDevice;
long DAC1Enable;

int chargercontrol_init()
{
	if( (hDevice = openUSBConnection(LJ_ID)) == NULL) {
		fprintf(stderr, "no labjack?\n");
    		return 1;
	}
	
	//Get calibration information from UE9
	if(getCalibrationInfo(hDevice, &caliInfo) < 0)
		return(1);

	// turn off charger
	chargercontrol_setCharger(0);
	buscontrol_setBus(0);
	
	// we have to disable and enable the watchdog to get it to work?
	setWatchdog(hDevice, 0);
	setWatchdog(hDevice, 1);

	chargeCurrentZero = getReading(4);
	fprintf(stderr, "voltage at zero current is %lf\n", chargeCurrentZero);
	return 0;
}

void chargercontrol_shutdown() {
	chargercontrol_setCharger(0);
	buscontrol_setBus(0);
}

int buscontrol_init() {
	// don't need to do any additional setup
	return 0;
}

void chargercontrol_setCharger(char on) {
	if (on) {
		eDO(hDevice, 1, CHARGER_RELAY_PORT, 1);
	} else {
		eDO(hDevice, 1, CHARGER_RELAY_PORT, 0);
	}
}

void buscontrol_setBus(char on) {
	if (on) {
		eDO(hDevice, 1, BUS_RELAY_PORT, 1);
	} else {
		eDO(hDevice, 1, BUS_RELAY_PORT, 0);
	}
}

int setWatchdog(HANDLE hDevice, char reset) {
  
  uint8 sendBuff[26];
  uint8 recBuff[38];
  uint16 checksumTotal;
  int sendChars, recChars;

  //Setting all bytes to zero since we only want to read back the U3
  //configuration settings
  for(int i = 0; i < 26; i++)
    sendBuff[i] = 0;

//Disable 0x44, 0xf8, 0x5, 0x9, 0x3d, 0x0, 0x1, 0x0, 0x3c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
//Enable  0x32, 0xf8, 0x5, 0x9, 0x2b, 0x0, 0x1, 0x20, 0xa, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0

  sendBuff[1] = (uint8)(0xF8);  //Command byte
  sendBuff[2] = (uint8)(0x05);  //Number of data words
  sendBuff[3] = (uint8)(0x09);  //Extended command number
  
  sendBuff[6] = 0x01;           // write?
//  sendBuff[7] = 0x18;           // set DO and reset
  if (reset) {
    sendBuff[7] = 0x20;           // reset
  } else {
    sendBuff[7] = 0x00;           // don't reset
  }
  if (reset) {
    sendBuff[8] = 0x0A;           // timeout MSB
    sendBuff[9] = 0x00;           // timeout LSB
  } else {
    sendBuff[8] = 0xff;           // timeout MSB
    sendBuff[9] = 0xff;           // timeout LSB
  }
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
//  for (int i = 0; i < 16; i++) {
//     printf("%02x ", recBuff[i]);
//  }
//  printf("\n");
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

double chargercontrol_getChargeCurrent() {
	double value = getReading(CHARGE_CURRENT_CHANNEL);
	double result = (chargeCurrentZero - value) / 0.020 * (3 / 2.7);
	//fprintf(stderr, "%lf %lf %lf\n", result, value, chargeCurrentZero);
	return result;
}
