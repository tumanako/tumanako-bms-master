/*
 Copyright 2010 Tom Parker

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

// "no charger" Charger Control Interface
int chargercontrol_init() {
	return 0;
}

double chargercontrol_getChargeCurrent() {
	return 0;
}

void chargercontrol_setCharger(char on __attribute__ ((unused))) {
	return;
}

void chargercontrol_shutdown() {
	return;
}

int buscontrol_init() {
	return 0;
}

void buscontrol_setBus(char on __attribute__ ((unused))) {
	return;
}

