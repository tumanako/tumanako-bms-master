#! /usr/bin/python

# Copyright 2013 Tom Parker
#
# This file is part of the Tumanako EVD5 BMS.
#
# The Tumanako EVD5 BMS is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# The Tumanako EVD5 BMS is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with the Tumanako EVD5 BMS.  If not, see
# <http://www.gnu.org/licenses/>.

import os
from subprocess import call
import time

ttys = os.listdir("/sys/class/tty/")

def getSlcanTty():
	for tty in ttys:
		if tty.startswith("ttyUSB"):
			ueventFile = open("/sys/class/tty/" + tty + "/device/uevent")
			uevent = ueventFile.readline().strip()
			if uevent == "DRIVER=ftdi_sio":
				return tty
		
		
tty = getSlcanTty()

print "killing old slcan"
call(["sudo", "killall", "slcan_attach"])
print "starting new slcan"
call(["screen", "-d", "-m", "-S", "slcan", "sudo", "/home/olpc/develop/can-utils/slcan_attach", "-w", "-o", "-f", "-s6", "-c", "/dev/" + tty])
time.sleep(1)
print "bringing up interface"
call(["sudo", "ifconfig", "slcan0", "up"])
