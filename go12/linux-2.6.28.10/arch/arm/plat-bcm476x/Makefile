# BCM476X platform support
# Shared between multiple bcm476x platforms 

obj-y	:= mm.o dma.o reset.o clkmgr_bcm4760.o irq.o lm.o pll_reinit4760.o sleep476x.o rtc_cpuapi476x.o spuHw.o hw_timer.o bcm4760_gfx_pm.o core.o
obj-y	+= clk_bcm4760/ ../plat-bcmap/
obj-m	:=
obj-n	:=

obj-$(CONFIG_USB_GADGET_DWC_OTG)	+= lm.o
obj-$(CONFIG_FB_BCM476X_CLCD)		+= clcd.o
obj-$(CONFIG_BCM_POWER_MANAGER)		+= pm.o
obj-$(CONFIG_BCM_POWER_MANAGER)		+= pm_platform.o
obj-$(CONFIG_BCM_POWER_MANAGER)		+= pwr_seq.o
obj-$(CONFIG_PMU_DEVICE_BCM59040)	+= pm.o pwr_seq.o
obj-$(CONFIG_BCM_POWER_MANAGER)		+= pll_reinit4760.o
obj-$(CONFIG_BCM_BATTERY_MANAGER)	+= bcm4760_battery.o
obj-$(CONFIG_BCM4760_PWM)		+= pwm_bcm4760.o
obj-$(CONFIG_PCI)			+= pci_emu.o
# We want to save the state of the fpu in sleep476x.S
EXTRA_AFLAGS += -mfpu=vfp

