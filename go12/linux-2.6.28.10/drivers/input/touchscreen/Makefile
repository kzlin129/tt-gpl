#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

wm97xx-ts-y := wm97xx-core.o

obj-$(CONFIG_TOUCHSCREEN_ADS7846)			+= ads7846.o
obj-$(CONFIG_TOUCHSCREEN_ATMEL_TSADCC)			+= atmel_tsadcc.o
obj-$(CONFIG_TOUCHSCREEN_ATMEL_MAXTOUCH)		+= atmel_maxtouch.o
obj-$(CONFIG_TOUCHSCREEN_BITSY)				+= h3600_ts_input.o
obj-$(CONFIG_TOUCHSCREEN_CORGI)				+= corgi_ts.o
obj-$(CONFIG_TOUCHSCREEN_GUNZE)				+= gunze.o
obj-$(CONFIG_TOUCHSCREEN_ELO)				+= elo.o
obj-$(CONFIG_TOUCHSCREEN_FUJITSU)			+= fujitsu_ts.o
obj-$(CONFIG_TOUCHSCREEN_INEXIO)			+= inexio.o
obj-$(CONFIG_TOUCHSCREEN_MIGOR)				+= migor_ts.o
obj-$(CONFIG_TOUCHSCREEN_MTOUCH)			+= mtouch.o
obj-$(CONFIG_TOUCHSCREEN_MK712)				+= mk712.o
obj-$(CONFIG_TOUCHSCREEN_HP600)				+= hp680_ts_input.o
obj-$(CONFIG_TOUCHSCREEN_HP7XX)				+= jornada720_ts.o
obj-$(CONFIG_TOUCHSCREEN_HTCPEN)			+= htcpen.o
obj-$(CONFIG_TOUCHSCREEN_USB_COMPOSITE)			+= usbtouchscreen.o
obj-$(CONFIG_TOUCHSCREEN_PENMOUNT)			+= penmount.o
obj-$(CONFIG_TOUCHSCREEN_TOUCHIT213)			+= touchit213.o
obj-$(CONFIG_TOUCHSCREEN_TOUCHRIGHT)			+= touchright.o
obj-$(CONFIG_TOUCHSCREEN_TOUCHWIN)			+= touchwin.o
obj-$(CONFIG_TOUCHSCREEN_UCB1400)			+= ucb1400_ts.o
obj-$(CONFIG_TOUCHSCREEN_WM97XX)			+= wm97xx-ts.o
wm97xx-ts-$(CONFIG_TOUCHSCREEN_WM9705)			+= wm9705.o
wm97xx-ts-$(CONFIG_TOUCHSCREEN_WM9712)			+= wm9712.o
wm97xx-ts-$(CONFIG_TOUCHSCREEN_WM9713)			+= wm9713.o
obj-$(CONFIG_TOUCHSCREEN_WM97XX_MAINSTONE)		+= mainstone-wm97xx.o
obj-$(CONFIG_BCM4760_TOUCHSCREEN)			+= bcm4760_ts.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_TTSP)	+= psoc-ctsic-tma340-fw-3-2-or-above.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_TTSP)	+= psoc-bootloader-fw-3-2-or-above.o
psoc-ctsic-tma340-fw-3-2-or-above-objs := cyttsp-lib.o cyttsp.o
psoc-bootloader-fw-3-2-or-above-objs := cyttsp-lib.o cyttsp-bl.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_HX8520)                  += himax-hx8520.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_HX8526)                  += himax-hx8526.o
ifeq ($(CONFIG_TOUCHSCREEN_S3C), y)
	obj-m			+= s3c-ts.o
endif
ifeq ($(CONFIG_TOUCHSCREEN_SYNAPTIC), y)
obj-m				+= rmi4/
endif
