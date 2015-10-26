#
# Makefile for PPS clients.
#

obj-$(CONFIG_PPS_CLIENT_KTIMER)	+= ktimer.o
obj-$(CONFIG_PPS_CLIENT_LDISC)	+= pps-ldisc.o

ifeq ($(CONFIG_PPS_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
