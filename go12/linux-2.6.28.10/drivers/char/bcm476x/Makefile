#
# Makefile which builds BCM476X drivers
#

#
# IMPORTANT: The order of these is important.
#	Within this directory, the order that objects are added to obj-y
#	will determine the order that the init functions are called.
#
EXTRA_CFLAGS := -Werror
obj-$(CONFIG_BACKLIGHT_LCDFB)			+= lcd_backlight.o
obj-$(CONFIG_BCM_BATTERY_MANAGER)		+= pm_battmgr.o
obj-$(CONFIG_BCM4760_GPS_DRIVER)    		+= gps.o 
obj-$(CONFIG_TOMTOM_NASHVILLE_SCENARI_BCM4760)	+= alc5627.o

#
# If the kernel make is called from the top level make, then CONVBMP will
# already be defined. If we call the kernel make directly, then it won't. So
# we define it for this case
#

CONVBMP ?= $(TOPDIR)/../bin/convbmp

%.c: %.bmp
	@echo Converting $< ...
	@$(CONVBMP) $(CONVBMP_FLAGS) -o $@ $<
