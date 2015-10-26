EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/hal_custom/

.PHONY: all

else

obj-$(CONFIG_BCM_HALAUDIO_CUSTOM) += \
	halaudio_external_custom.o

endif
