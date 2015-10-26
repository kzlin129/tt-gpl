# Mendoza carrier board support.
# Shared between multiple S* platforms,
# so please edit carefully...

obj-y		:= dock_pdata.o
obj-m		:=
obj-n		:=

obj-$(CONFIG_SND_SOC_ADAU1761)		+= adau1761_pdata.o
obj-$(CONFIG_SND_SOC_WM8990)		+= wm8990_pdata.o
obj-$(CONFIG_PLAT_MENDOZA)		+= bcm4750_pdata.o
obj-$(CONFIG_PLAT_MENDOZA)		+= lms480_pdata.o
obj-$(CONFIG_A043FW05V4_SPI)		+= a043fw05v4_pdata.o
obj-$(CONFIG_PLAT_MENDOZA)		+= pwm_bl.o
obj-$(CONFIG_PLAT_MENDOZA)		+= mendoza-i2c.o
obj-$(CONFIG_PLAT_MENDOZA)		+= mendoza-fb.o
obj-$(CONFIG_PLAT_MENDOZA)		+= mendoza-sdhci.o
obj-$(CONFIG_TOMTOM_GPRS)		+= gprs_faro.o
obj-$(CONFIG_TOMTOM_VBUSMON)		+= vbus.o
obj-$(CONFIG_TOMTOM_IDPINMON)		+= idpinmon.o
obj-$(CONFIG_TOMTOM_BATTERY)		+= battery.o
obj-$(CONFIG_TOMTOM_PB)			+= powerbutton.o
obj-$(CONFIG_TOMTOM_HAVE_SERIAL_BARRACUDA)	+= barracuda.o
obj-$(CONFIG_TOMTOM_HAVE_SPI_BARRACUDA)	+= barracuda.o
obj-$(CONFIG_TOMTOM_HAVE_SERIAL_AR15XX)	+= ar15xx.o
obj-$(CONFIG_TOMTOM_USBMODE)		+= usbmode_s3c64xx.o tt_usbmode.o
obj-$(CONFIG_TOMTOM_USBMODE_DUMMY)	+= tt_dummy_usbmode.o
obj-$(CONFIG_TOMTOM_PMIC_LTC3555)	+= ltc3555.o
obj-$(CONFIG_TOMTOM_PMIC_LTC3577)	+= ltc3577.o
obj-$(CONFIG_TOMTOM_BLUETOOTH_MODE)	+= bluetooth.o
obj-$(CONFIG_TOMTOM_FDT)		+= fdtexport_setup.o
obj-$(CONFIG_TOMTOM_FLIPFLOP)		+= tomtom_flipflop.o
obj-$(CONFIG_MFD_S5M8751_I2C)		+= mendoza_s5m8751.o s5m8751_bl.o
obj-$(CONFIG_MFD_S5M8751_I2C)		+= s5m8751_audio.o s5m8751_pmic.o
obj-$(CONFIG_MFD_LTC3577)		+= mendoza-ltc3577.o
obj-$(CONFIG_MFD_LTC3577)		+= ltc3577-pmic.o
obj-$(CONFIG_MFD_LTC3577)		+= ltc3577-bl.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS)	+= psoc_ctsic.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTIC)	+= synaptic_ctsic.o
obj-$(CONFIG_SND_SOC_TLV320ADC3101)	+= tlv320adc3101_pdata.o
obj-$(CONFIG_MACH_VENICE)		+= kxr94_pdata.o
obj-$(CONFIG_MACH_VENICE)   += suicide_pdata.o
obj-$(CONFIG_TOMTOM_TILT_TS0001_L)   += tilt-ts0001-l.o
