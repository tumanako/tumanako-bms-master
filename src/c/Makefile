#
# Makefile for U3 examples
#

LIB_LABJACK_USB = ../../../../../exodriver

CHARGER_CONTROL_SRC=chargercontrol_labjack.c $(LIB_LABJACK_USB)/examples/U3/u3.c
CHARGER_CONTROL_CFLAGS=-I$(LIB_LABJACK_USB)/liblabjackusb -I$(LIB_LABJACK_USB)/examples/U3
#CHARGER_CONTROL_LDFLAGS=-L$(LIB_LABJACK_USB)/liblabjackusb 
CHARGER_CONTROL_LDFLAGS=-lusb-1.0 $(LIB_LABJACK_USB)/liblabjackusb/labjackusb.o
#CHARGER_CONTROL_LIBS=-llabjackusb
#CHARGER_CONTROL_SRC=chargercontrol_none.c
#CHARGER_CONTROL_CFLAGS=
#CHARGER_CONTROL_LDFLAGS=
#CHARGER_CONTROL_LIBS=
MONITOR_SRC=monitor.c crc.c soc_evision.c monitor_can.c logger.c util.c config.c console.c canEventListener.c $(CHARGER_CONTROL_SRC)
MONITOR_OBJ=$(MONITOR_SRC:.c=.o)

SRCS=$(wildcard *.c)
HDRS=$(wildcard *.h)

CFLAGS=-std=c99 -Wextra -Wall -g $(CHARGER_CONTROL_CFLAGS)
CC=gcc
LDFLAGS=$(CHARGER_CONTROL_LDFLAGS)
LIBS=-lm -lpthread -lconfuse $(CHARGER_CONTROL_LIBS)

all: cycler

cycler: $(MONITOR_OBJ)
	$(CC) -Wextra -Wall -o monitor $(MONITOR_OBJ) $(LDFLAGS) $(LIBS)

clean:
	rm -f *.o monitor
