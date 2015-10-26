# EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/hal_bcm911xxapmhss/

.PHONY: all

else

obj-$(CONFIG_BCM_HALAUDIO_BCM911XXAPMHSS) += \
	halaudio_external_bcm911xxapmhss.o \
	halaudio_bcm911xxhssaddon.o

endif
