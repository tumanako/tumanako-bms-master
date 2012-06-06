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
#include "util.h"

/* return a double representation of the passed value divided by 1000. */
double milliToDouble(short s) {
	return ((double) s) / 1000;
}

/* return a double representation of the passed value divided by 100. */
double centiToDouble(short s) {
	return ((double) s) / 100;
}

/** Copy the passed char into the passed buffer. */
void charToBuf(const unsigned char c, __u8* buf) {
	buf[0] = (__u8) c;
}

/** Copy the passed short into the passed buffer. */
void shortToBuf(short s, __u8* buf) {
	buf[0] = (__u8) (s >> 8);
	buf[1] = (__u8) (s & 0x00ff);
}

/**
 * Make a char from the 8 bits starting at c
 */
unsigned char bufToChar(__u8 *c) {
	return *c;
}

/**
 * Make a short from the 16 bits starting at c
 *
 * TODO deal with endian
 */
unsigned short bufToShort(__u8 *c) {
	unsigned short result = *c;
	result = result << 8;
	result = result | *(c + 1);
	return result;
}
