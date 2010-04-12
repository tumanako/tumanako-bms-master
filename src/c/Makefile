#
# Makefile for U3 examples
#

# CHARGER_CONTROL_SRC=chargercontrol_labjack.c /home/tparker/labjack2/Linux_C_NativeUSB/Examples/U3/u3.c
# CHARGER_CONTROL_CFLAGS=-I/home/tparker/labjack2/Linux_C_NativeUSB/Examples/U3
# CHARGER_CONTROL_LDFLAGS=-L/home/tparker/labjack2/Linux_C_NativeUSB/liblabjackusb/3
# CHARGER_CONTROL_LIBS=-llabjackusb
CHARGER_CONTROL_SRC=chargercontrol_none.c
CHARGER_CONTROL_CFLAGS=
CHARGER_CONTROL_LDFLAGS=
CHARGER_CONTROL_LIBS=
MONITOR_SRC=monitor.c $(CHARGER_CONTROL_SRC)
MONITOR_OBJ=$(MONITOR_SRC:.c=.o)

SRCS=$(wildcard *.c)
HDRS=$(wildcard *.h)

CFLAGS=-std=c99 -Wall -g $(CHARGER_CONTROL_CFLAGS)
CC=gcc $(CFLAGS)
LDFLAGS=$(CHARGER_CONTROL_LDFLAGS)
LIBS=-lm $(CHARGER_CONTROL_LIBS)

all: cycler

cycler: $(MONITOR_OBJ)
	$(CC) -o monitor $(MONITOR_OBJ) $(LDFLAGS) $(LIBS)

clean:
	rm -f *.o monitor
