# arch/arm/plat-s3c/Makefile
#
# Copyright 2008 Simtec Electronics
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

# Core support for all Samsung SoCs

obj-y				+= init.o
ifndef CONFIG_ARCH_S5PC1XX

ifdef CONFIG_NO_HZ 
obj-y				+= hr-time.o

else

ifndef CONFIG_HIGH_RES_TIMERS 
obj-y				+= time.o
else
obj-y				+= hr-time.o
endif 

endif
endif

# DMA support

obj-$(CONFIG_S3C64XX_DMA)	+= dma.o
obj-$(CONFIG_S3C_DMA_PL330)	+= dma-pl330.o

obj-y				+= clock.o
obj-y				+= pwm-clock.o
obj-y				+= gpio.o
obj-y				+= gpio-config.o
obj-$(CONFIG_S3C_ADC)		+= adc.o

# devices
obj-$(CONFIG_S3C_DEV_HSMMC)	+= dev-hsmmc.o
obj-$(CONFIG_S3C_DEV_HSMMC1)	+= dev-hsmmc1.o
obj-$(CONFIG_S3C_DEV_HSMMC2)	+= dev-hsmmc2.o
obj-y				+= dev-i2c0.o
obj-$(CONFIG_S3C_DEV_I2C1)	+= dev-i2c1.o
obj-$(CONFIG_TOUCHSCREEN_S3C)	+= dev-ts.o
obj-$(CONFIG_TOUCHSCREEN_S3C)	+= s3c-ts.o
obj-$(CONFIG_S3C_PWM) 		+= pwm.o

