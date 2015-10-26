#
# Makefile which builds BCM116x drivers
#

#
# IMPORTANT: The order of these is important.
#
#	Withint this directory, the order that objects are added to obj-y
#	will determine the order that the init functions are called.
#
EXTRA_CFLAGS := -fno-inline -Wno-multichar -O0
EXTRA_CFLAGS += -I$(LINUX_DIR)/include/linux/broadcom/omx
EXTRA_CFLAGS += -I$(LINUX_DIR)/include/linux/broadcom/vc03

ifeq (1,$(strip $(CFG_GLOBAL_GFX_SERVICE_SUPPORT)))
EXTRA_CFLAGS += -DKHRONOS_GFX
endif

obj-$(CONFIG_BCM_VC03B0) 		+= vc_vchi_gencmd.o 
obj-$(CONFIG_BCM_VC03B0) 		+= vc_vchi_dispmanx.o 
obj-$(CONFIG_BCM_VC03B0) 		+= vc_vchi_hostreq.o 
obj-$(CONFIG_BCM_VC03B0) 		+= vc_vchi_filesys.o 
ifeq (1,$(strip $(CFG_GLOBAL_GFX_SERVICE_SUPPORT)))
obj-$(CONFIG_BCM_VC03B0) 		+= vc_vchi_khronos.o 
endif
obj-$(CONFIG_BCM_VC03B0)		+= vcilcs_vchi.o
obj-$(CONFIG_BCM_VC03B0) 		+= vc_drv.o
obj-$(CONFIG_BCM_VC03B0) 		+= vcos.o
#obj-$(CONFIG_BCM_VC03B0) 		+= vciface.o vcmsgfifo.o vc_drv.o vcos.o vcutil.o 
#obj-$(CONFIG_BCM_VC03B0) 		+= vchostmem.o vchr.o 
#obj-$(CONFIG_BCM_VC03B0) 		+= vchost-kernel.o
obj-$(CONFIG_BCM_VC03B0) 		+= vc_hostport_common.o
obj-$(CONFIG_BCM_VC03B0) 		+= vclcd.o
obj-$(CONFIG_BCM_VC03B0)		+= vcilcs_in.o vcilcs_out.o vcilcore.o vcilclient.o vcilplatform.o
obj-$(CONFIG_BCM_VC03B0)		+= vcstripes.o 
obj-$(CONFIG_BCM_VC03B0) 		+= vcplay.o
obj-$(CONFIG_BCM_VC03B0) 		+= vccam.o
obj-$(CONFIG_BCM_VC03B0) 		+= vcplaycam.o
obj-$(CONFIG_BCM_VC03B0) 		+= vc_vidconftest.o

ifdef CONFIG_ARCH_BCMRING
obj-$(CONFIG_BCM_VC03B0)		+= vc_hostport_pif.o
endif

#VESPER_EP3 B0 board has a 27Mhz XTAL
ifeq (BCM9_VESPER_EP3_WVGA, $(BOARD))
obj-$(CONFIG_BCM_VC03B0)		+= vcstage2_vesper.o
endif

#JAZZ board has a 19.2Mhz XTAL
ifeq (BCM91103EVM_JAZZ, $(BOARD))
obj-$(CONFIG_BCM_VC03B0)		+= vcstage2_jazz.o
endif

ifeq (BCM911107EVM_JAZZ_WVGA, $(BOARD))
obj-$(CONFIG_BCM_VC03B0)		+= vcstage2_jazz.o
endif

obj-$(CONFIG_BCM_VC03B0)		+= vcfile-kernel.o
#obj-$(CONFIG_BCM_VC03B0)	        += vcdigicam.o vcrecorder.o
obj-$(CONFIG_BCM_VC03B0)        	+= vcdispmanxtest.o
obj-$(CONFIG_BCM_VC03B0)	        += vc_omxdec.o
obj-$(CONFIG_BCM_VC03B0)	        += vc03_frmfwd.o
obj-$(CONFIG_BCM_VC03B0)	        += vchitest.o vchitest_dc4.o vchi/

# If the kernel make is called from the top level make, then CONVBMP will
# already be defined. If we call the kernel make directly, then it won't. So
# we define it for this case
#

CONVBMP ?= $(TOPDIR)/../bin/convbmp

%.c: %.bmp
	@echo Converting $< ...
	@$(CONVBMP) $(CONVBMP_FLAGS) -o $@ $<

