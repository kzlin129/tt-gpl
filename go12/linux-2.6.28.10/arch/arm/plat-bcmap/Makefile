#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y := clock_fw.o
obj-$(CONFIG_BCM2820_CPUFREQ_DRV) += cpufreq_drv.o
obj-$(CONFIG_BCM2153_CPUFREQ_DRV) += cpufreq_drv.o
# obj-$(CONFIG_BCM4760_CPUFREQ_DRV) += cpufreq_drv.o

EXTRA_CFLAGS += -ggdb

