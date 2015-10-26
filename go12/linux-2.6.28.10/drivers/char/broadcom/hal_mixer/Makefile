EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/hal_mixer/

.PHONY: all

else

obj-$(CONFIG_BCM_HALAUDIO_MIXER) += \
	halmixer_swb.o \
   halmixer_api.o \
   halmixer_resamp.o \
   halmixer_adder.o

endif
