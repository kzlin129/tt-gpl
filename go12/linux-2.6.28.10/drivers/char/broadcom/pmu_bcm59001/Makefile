EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/pmu_bcm59001/

.PHONY: all

else

obj-$(CONFIG_BCM_PMU_BCM59001) += \
	headset.o \
	led.o \
	rtc.o \
	bcm59001.o \

endif
