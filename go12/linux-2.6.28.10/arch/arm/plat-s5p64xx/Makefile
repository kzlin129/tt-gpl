# arch/arm/plat-s5p64xx/Makefile
#
# Copyright 2008 Openmoko, Inc.
# Copyright 2008 Simtec Electronics
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:= dummy.o
obj-				:=

# Core files

obj-y				+= dev-uart.o devs.o
obj-y				+= cpu.o
obj-y				+= irq.o
obj-y				+= irq-eint.o irq-eint-group.o
obj-y				+= clock.o
obj-y				+= gpiolib.o
obj-y				+= bootmem.o

# CPU support

obj-$(CONFIG_CPU_S5P6440_INIT)	+= s5p6440-init.o
obj-$(CONFIG_CPU_S5P6440_CLOCK)	+= s5p6440-clock.o
obj-$(CONFIG_CPU_FREQ)		+= s5p64xx-cpufreq.o
obj-$(CONFIG_PM)                += pm.o
obj-$(CONFIG_PM)                += sleep.o

# Device setup

obj-$(CONFIG_S5P64XX_SETUP_I2C0) += setup-i2c0.o
obj-$(CONFIG_S5P64XX_SETUP_I2C1) += setup-i2c1.o
obj-$(CONFIG_S5P64XX_DEV_POST)   += dev-post.o
obj-$(CONFIG_S5P64XX_SETUP_POST) += setup-post.o
obj-$(CONFIG_S3C_PWM) 	 	 += dev-pwm.o

obj-$(CONFIG_SPI_S5P6440)   += dev-spi.o

# I2S
obj-$(CONFIG_SND_S3C24XX_SOC) += dev-audio.o

# Device setup - FIMD
obj-$(CONFIG_S5P64XX_DEV_FB)	  += dev-fb.o
obj-$(CONFIG_S5P64XX_SETUP_FB)    += setup-fb.o

# ltc3714 voltage regulator
obj-$(CONFIG_S5P64XX_LTC3714)	+=  ltc3714.o

# S5M8751
obj-$(CONFIG_S5P6440_S5M8751)	+= s5p6440-s5m8751.o
