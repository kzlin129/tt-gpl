#
# Makefile which builds Broadcom drivers
#

EXTRA_CFLAGS := -Werror

#obj-y += spi_load.o
obj-$(CONFIG_BCM_HAVE_PROC_GPIO) += proc_gpio.o

obj-$(CONFIG_BCM_PERFCNT_SUPPORT) += perfcnt.o
ifeq ($(CFG_GLOBAL_PERFTEST_SUPPORT),1)
obj-$(CONFIG_BCM_PERFCNT_SUPPORT) += mymemcpy.o _mymemcpy.o perftest.o
endif
obj-$(CONFIG_BCM_BUS_ARBITER) += hpm.o
obj-$(CONFIG_BCM_KNLLOG_SUPPORT) += knllog.o

obj-$(CONFIG_BCM_POWER_MANAGER)         += PowerManager.o
obj-$(CONFIG_BCM_IDLE_PROFILER_SUPPORT)                         += idle_profiler.o
obj-$(CONFIG_BCM_HALAUDIO_HOOK)                                 += halaudio_hook.o
obj-$(CONFIG_BCM_HALAUDIO_MIXER)                                += hal_mixer/
obj-$(CONFIG_BCM_EPTDRIVER_SUPPORT)             += sock.o
obj-$(CONFIG_BCM_EPTDRIVER_SUPPORT)             += ept_hook.o
obj-$(CONFIG_BCM_PMU)                     += pmu_chip.o
obj-$(CONFIG_BCM_PMU_BCM59001)            += pmu_bcm59001/
obj-$(CONFIG_BCM_PMU_BCM59035_B0)         += pmu_bcm59035b0/
obj-$(CONFIG_BCM_PMU_BCM59040)		  += pmu_bcm59040/
obj-$(CONFIG_BCM_PMU_PCF506XX)            += pmu_pcf506xx/
obj-$(CONFIG_BCM_HALAUDIO_AK4642)                       += hal_ak4642/
obj-$(CONFIG_BCM_HALAUDIO_BCM911XXAPMHSS) += hal_bcm911xxapmhss/
obj-$(CONFIG_BCM_HALAUDIO_NOEXTERNAL)           += hal_noexternal/
obj-$(CONFIG_BCM_HALAUDIO_CUSTOM)                       += hal_custom/
obj-$(CONFIG_BCM_HALAUDIO_PLATFORM)       += halaudio_platform/
obj-$(CONFIG_BCM_DEBUG_PAUSE)             += debug_pause.o
obj-$(CONFIG_BCM_RELTIME)                 += reltime.o
obj-$(CONFIG_BCM_OTP)                     += otp.o
obj-$(CONFIG_BCM_GPIO_DRIVER)		  += user-gpio.o
obj-$(CONFIG_BCM_4325_DRIVER)		  += bcm4325.o

obj-$(CONFIG_BCM_ADC_DRIVER)          += bcm_adc.o
obj-$(CONFIG_BCM4760_ADC_DRIVER)      += bcm4760_adc.o

obj-$(CONFIG_BCM_DD)                      += dd/

ifeq ($(CONFIG_BCM_VC02),y)
obj-$(CONFIG_BCM_VC02)                    += vc/
endif
ifeq ($(CONFIG_BCM_VC03),y)
obj-$(CONFIG_BCM_VC03)                    += vc03/
obj-$(CONFIG_BCM_VC03B0)                  += vc03b0/
endif

ifneq ($(CONFIG_BCM_VC02),y)
ifneq ($(CONFIG_BCM_VC03),y)
ifneq ($(CONFIG_BCM_VC03B0),y)
obj-y                                     += vc_stub.o
endif
endif
endif

ifeq ($(CONFIG_BCM_VC02),y)
ifeq ($(CONFIG_BCM_VC03),y)
ifeq ($(CONFIG_BCM_VC03B0),y)
$(error Enable only one of the following: CONFIG_BCM_VC02, CONFIG_BCM_VC03, CONFIG_BCM_VC03B0 )
endif
endif
endif

obj-$(CONFIG_BCM_IODUMP_SUPPORT)              += iodump.o
ifneq ($(CONFIG_BCM_IODUMP_SUPPORT),y)
obj-y                                   += iodump_stub.o
endif

obj-$(CONFIG_BCM_USERDMA)              += userdma/
#obj-y += trample-test.o
