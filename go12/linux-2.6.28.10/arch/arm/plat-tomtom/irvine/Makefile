obj-y	:= irvine.o at24-eeprom.o irvine-usbmode.o irvine-pmu.o dock_pdata.o sd_pwr_en.o irvine-low-dc-vcc.o
obj-y	+= panel/
#obj-m	:=
#obj-n	:=
obj-$(CONFIG_TOMTOM_PB)					+= powerbutton.o
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_TTSP
obj-y	+= cyttsp.o
#endif
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MAXTOUCH
obj-y	+= atmel_ts.o
#endif
obj-$(CONFIG_INPUT_TOUCHSCREEN)				+= irvine-ts-resistive.o
obj-$(CONFIG_SOUND)					+= irvine-sound.o
obj-$(CONFIG_SOUND)					+= alc5627.o
obj-$(CONFIG_BACKLIGHT_PWM)				+= irvine-backlight.o
obj-$(CONFIG_FB_BCM476X_CLCD)				+= irvine-lcd.o
obj-$(CONFIG_TOMTOM_FDT)				+= fdtexport-setup.o
obj-$(CONFIG_TOMTOM_BLUETOOTH_MODE)			+= bcm4325.o bcm2070.o
obj-$(CONFIG_TOMTOM_GPRS)				+= faro.o
obj-$(CONFIG_TOMTOM_GPS)				+= gps.o
obj-$(CONFIG_TOMTOM_FLIPFLOP)				+= irvine-flipflop.o
obj-$(CONFIG_TOMTOM_TILT_TS0001_L)			+= tilt-ts0001-l.o
obj-$(CONFIG_TOMTOM_TILT_TS1003_AL)			+= tilt-ts1003-al.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX)				+= hx852x.o
