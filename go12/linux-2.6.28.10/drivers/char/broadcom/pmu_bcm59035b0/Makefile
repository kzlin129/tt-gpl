EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/pmu_bcm59035b0/

.PHONY: all

else

obj-$(CONFIG_BCM_PMU_BCM59035_B0) += \
	bcm59035_b0.o \

endif
