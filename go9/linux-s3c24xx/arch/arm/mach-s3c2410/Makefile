
#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y			:= cpu.o irq.o time.o gpio.o clock.o devs.o system.o
obj-m			:=
obj-n			:=
obj-			:=

# S3C2410 support files

obj-$(CONFIG_CPU_S3C2410)  += s3c2410.o
obj-$(CONFIG_CPU_S3C2410)  += s3c2410-clock.o
obj-$(CONFIG_CPU_S3C2410)  += s3c2410-bus.o
obj-$(CONFIG_S3C2410_DMA)  += dma.o

# Power Management support

obj-$(CONFIG_PM)	   += pm.o sleep.o
obj-$(CONFIG_PM_SIMTEC)	   += pm-simtec.o
obj-$(CONFIG_CPU_FREQ)     += cpufreq/

# S3C2412 support

obj-$(CONFIG_CPU_S3C2412)  += regs-dyn.o
obj-$(CONFIG_CPU_S3C2412)  += s3c2412.o s3c2412-dsc.o
obj-$(CONFIG_CPU_S3C2412)  += s3c2412-irq.o
obj-$(CONFIG_CPU_S3C2412)  += s3c2412-clock.o

# S3C2440 support

obj-$(CONFIG_CPU_S3C2440)  += s3c2440.o s3c2440-dsc.o
obj-$(CONFIG_CPU_S3C2440)  += s3c2440-irq.o
obj-$(CONFIG_CPU_S3C2440)  += s3c2440-clock.o

# S3C2443 support

obj-$(CONFIG_CPU_S3C2443)  += s3c2443.o
obj-$(CONFIG_CPU_S3C2443)  += s3c2443-irq.o
obj-$(CONFIG_CPU_S3C2443)  += s3c2443-clock.o
obj-$(CONFIG_CPU_S3C2443)  += s3c2443-bugs.o

# S3C2450 support

obj-$(CONFIG_CPU_S3C2443)  += s3c2450.o
obj-$(CONFIG_CPU_S3C2443)  += s3c2450-irq.o
obj-$(CONFIG_CPU_S3C2443)  += s3c2450-clock.o

# machine specific support

obj-$(CONFIG_ARCH_BAST)		+= mach-bast.o usb-simtec.o
obj-$(CONFIG_ARCH_H1940)	+= mach-h1940.o
obj-$(CONFIG_MACH_N30)		+= mach-n30.o
obj-$(CONFIG_ARCH_SMDK2410)	+= mach-smdk2410.o
obj-$(CONFIG_ARCH_S3C2440)	+= mach-smdk2440.o
obj-$(CONFIG_MACH_TOMTOMGO)	+= mach-tomtomgo.o tomtomgo-devs.o tomtomgo-type.o tomtomgo-wake.o tomtomgo-iic.o tomtomgo-ioexp.o tomtomgo-clocks.o
obj-$(CONFIG_MACH_VR1000)	+= mach-vr1000.o usb-simtec.o
obj-$(CONFIG_MACH_RX3715)	+= mach-rx3715.o
obj-$(CONFIG_MACH_OTOM)		+= mach-otom.o
obj-$(CONFIG_MACH_NEXCODER_2440) += mach-nexcoder.o
