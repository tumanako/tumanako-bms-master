#
# Makefile for U3 examples
#

LIB_LABJACK_USB = ../../../../../exodriver

MONITOR_SRC=monitor.c \
	crc.c \
	soc_evision.c \
	monitor_can.c \
	logger.c \
	util.c \
	config.c \
	console.c \
	canEventListener.c \
	chargeAlgorithm.c \
	chargercontrol.c \
	chargercontrol_labjack.c \
	serial.c \
	$(LIB_LABJACK_USB)/examples/U3/u3.c 
MONITOR_OBJ=$(MONITOR_SRC:.c=.o)

SRCS=$(wildcard *.c)
HDRS=$(wildcard *.h)

CFLAGS=-std=c99 -Werror -Wextra -Wall -g -I$(LIB_LABJACK_USB)/liblabjackusb -I$(LIB_LABJACK_USB)/examples/U3
CC=gcc
LDFLAGS=$(CHARGER_CONTROL_LDFLAGS) -lusb-1.0 $(LIB_LABJACK_USB)/liblabjackusb/labjackusb.o
LIBS=-lm -lpthread -lconfuse $(CHARGER_CONTROL_LIBS)

all: cycler

cycler: $(MONITOR_OBJ)
	$(CC) -Wextra -Wall -o monitor $(MONITOR_OBJ) $(LDFLAGS) $(LIBS)

clean:
	rm -f *.o monitor
