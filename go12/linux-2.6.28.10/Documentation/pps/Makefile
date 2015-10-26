TARGETS = ppstest ppsldisc

CFLAGS += -Wall -O2 -D_GNU_SOURCE
CFLAGS += -I .
CFLAGS += -ggdb
CFLAGS += -D__N_PPS=$(shell awk '/N_PPS/ {print $$3}' ../../include/linux/tty.h)

# -- Actions section --

.PHONY : all depend dep

all : .depend $(TARGETS)

.depend depend dep :
	$(CC) $(CFLAGS) -M $(TARGETS:=.c) > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif


# -- Clean section --

.PHONY : clean

clean :
	rm -f *.o *~ core .depend
	rm -f ${TARGETS}
