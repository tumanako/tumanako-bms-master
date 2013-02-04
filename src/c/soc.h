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

/** State of Charge interface */

/** Initialisation function, return 0 if successful */
int soc_init();

/** Returns true if there is an error */
char soc_getError();

/** Get the current, negative is charging, positive is discharging */
double soc_getCurrent();

/** Get the voltage */
double soc_getVoltage();

/** Get the half-pack voltage */
double soc_getHalfVoltage();

/** Get the state of charge. Positive is discharged, negative is over charged. */
double soc_getAh();

/** Get the Wh consumed. Positive is discharged, negative is over charged. */
double soc_getWh();

void soc_registerSocEventListener(void (*socEventListener)());

