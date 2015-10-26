EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/pmu_pcf506xx/

.PHONY: all

else

obj-$(CONFIG_BCM_PMU_PCF506XX) += \
	headset.o \
	led.o \
	rtc.o \
	pcf506xx.o \
	pcf50603.o \
	pcf50611.o

endif
