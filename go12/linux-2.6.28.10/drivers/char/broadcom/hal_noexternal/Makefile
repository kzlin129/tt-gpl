EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/hal_noexternal/

.PHONY: all

else

obj-$(CONFIG_BCM_HALAUDIO_NOEXTERNAL) += \
	halaudio_external_stub.o

endif
