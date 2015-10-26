# arch/arm/plat-s3c6410/Makefile
#
# Copyright 2008 Openmoko, Inc.
# Copyright 2008 Simtec Electronics
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

# Core support for S3C6410 system

obj-$(CONFIG_CPU_S3C6410)	+= cpu.o
obj-$(CONFIG_CPU_S3C6410)	+= irq.o


# Helper and device support

obj-$(CONFIG_S3C6410_SETUP_SDHCI)	+= setup-sdhci.o

obj-$(CONFIG_PM)    += pm.o

# machine support

obj-$(CONFIG_MACH_SMDK6410)	+= mach-smdk6410.o
obj-$(CONFIG_MACH_SEOUL)	+= seoul.o seoul_spi.o
obj-$(CONFIG_MACH_LIMA)		+= lima.o lima_spi.o
obj-$(CONFIG_MACH_TORINOS)	+= torinos.o
obj-$(CONFIG_MACH_BEAUMONT_11)	+= beaumont_11.o
obj-$(CONFIG_MACH_VENICE)	+= venice.o venice_spi.o
