#
# Makefile for U3 examples
#
MONITOR_SRC=monitor.c 
MONITOR_OBJ=$(MONITOR_SRC:.c=.o)

SRCS=$(wildcard *.c)
HDRS=$(wildcard *.h)

CFLAGS=-std=c99 -Wall -g 
CC=gcc $(CFLAGS)
LDFLAGS=
LIBS=-lm 

all: cycler

cycler: $(MONITOR_OBJ)
	$(CC) -o monitor $(MONITOR_OBJ) $(LDFLAGS) $(LIBS)

clean:
	rm -f *.o monitor
