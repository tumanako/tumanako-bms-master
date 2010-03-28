#
# Makefile for U3 examples
#

# CHARGER_CONTROL_SRC=chargercontrol_labjack.c /home/tparker/labjack2/Linux_C_NativeUSB/Examples/U3/u3.c
CHARGER_CONTROL_SRC=chargercontrol_none.c
MONITOR_SRC=monitor.c $(CHARGER_CONTROL_SRC)
MONITOR_OBJ=$(MONITOR_SRC:.c=.o)

SRCS=$(wildcard *.c)
HDRS=$(wildcard *.h)

CFLAGS=-std=c99 -Wall -g -I/home/tparker/labjack2/Linux_C_NativeUSB/Examples/U3
CC=gcc $(CFLAGS)
LDFLAGS=-L/home/tparker/labjack2/Linux_C_NativeUSB/liblabjackusb/
LIBS=-lm -llabjackusb

all: cycler

cycler: $(MONITOR_OBJ)
	$(CC) -o monitor $(MONITOR_OBJ) $(LDFLAGS) $(LIBS)

clean:
	rm -f *.o monitor
