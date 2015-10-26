EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/pmu_bcm59040/

.PHONY: all

else

obj-$(CONFIG_BCM_PMU_BCM59040) += \
        rtc.o \
	bcm59040.o \

endif
