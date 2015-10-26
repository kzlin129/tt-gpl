EXTRA_CFLAGS := -Werror

ifeq ($(srctree),)

#
# We weren't called from the linux make environment, redirect
#

all:
	$(MAKE) -C ../../../.. drivers/char/broadcom/dd/

.PHONY: all

else

obj-$(CONFIG_BCM_DD) += bcm_dd.o
bcm_dd-objs          += dd_util.o
bcm_dd-objs          += dd_dma.o
bcm_dd-objs          += dd_proc.o
bcm_dd-objs          += dd_ge.o
bcm_dd-objs          += dd_lcd.o
bcm_dd-objs          += dd_proc.o
bcm_dd-objs          += dd_misc.o
bcm_dd-objs          += dd.o

endif
