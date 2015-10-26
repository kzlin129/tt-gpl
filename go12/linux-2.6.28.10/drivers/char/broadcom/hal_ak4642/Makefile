EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/hal_ak4642/

.PHONY: all

else

obj-$(CONFIG_BCM_HALAUDIO_AK4642) += \
	ext_codec_ak4642_i2c.o \
	halaudio_external_ak4642.o

endif
