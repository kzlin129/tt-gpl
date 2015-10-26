#
# Makefile for PMU drivers.
#


obj-$(CONFIG_PMU_DEVICE) += pmu_bus.o
obj-$(CONFIG_PMU_DEVICE_BCM59040) += bcm59040_core.o \
				bcm59040_control.o \
				bcm59040_adc.o \
				bcm59040_regulator.o \
				bcm59040_gpio.o
obj-$(CONFIG_BCM4760_STD) += bcm59040_batt.o
ccflags-$(CONFIG_PMU_DEVICE_DEBUG) += -DDEBUG
