ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/vc/

.PHONY: all

else
$(warning CONFIG_BCM_VIDEOCORE_TARGET = $(CONFIG_BCM_VIDEOCORE_TARGET))

ifneq (Makefile.clean,$(findstring Makefile.clean,$(MAKEFILE_LIST)))

ifeq (,$(CONFIG_BCM_VIDEOCORE_TARGET))
$(error CONFIG_BCM_VIDEOCORE_TARGET need to be defined for video builds)
endif

endif

EXTRA_CFLAGS += -I$(ALPHA_VIDEO_DIR)/build/$(CONFIG_BCM_VIDEOCORE_TARGET)

#
# IMPORTANT: The order of these is important.
#
#	Within this directory, the order that objects are added to vc-objs
#	will determine the order that the init functions are called.
#
#	So, vc_drv.o must come before vclcd. The VC02 must be initialized before
#  the LCD driver (which needs to talk to the VC02).
#
obj-y += vc.o

vc-objs = \
	vc_dispman2.o	\
	vcfrmfwd.o	\
	vcgencmd.o	\
	vchost-port.o	\
	vchost-kernel.o	\
	vciface.o	\
	vcmsgfifo.o	\
	vcstate.o	\
	vc_drv.o	\
	vcos.o		\
	vcutil.o		\
	vclcd.o	\
	vchostreq.o	\
	vchostmem.o	\
	vchr.o	\
	vclcd_test.o \
	vclogging.o \
   vcassert_notify.o \
   vc_gpio.o
	
ifeq ($(CONFIG_BCM_HALAUDIO_MIXER),y)
vc-objs += vchost_audio.o
endif

endif
