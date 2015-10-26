#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y := pinmux.o 
obj-$(CONFIG_MACH_BCM4760_TOMTOM)	+= siena.o otavalo.o lucca.o foggia.o bcm59040-pnd.o catania_b.o messina.o lausanne.o geneva_b.o sarnen.o
obj-$(CONFIG_MACH_BCM4760_FLASHER)	+= flasher.o
obj-$(CONFIG_MACH_CATALINA)		+= arch.o vbus_usbstat.o
obj-$(CONFIG_BCM4760_STD)		+= bcm4760_battery.o

EXTRA_CFLAGS += -ggdb

# We want to save the state of the fpu in sleep476x.S
EXTRA_AFLAGS += -mfpu=vfp

