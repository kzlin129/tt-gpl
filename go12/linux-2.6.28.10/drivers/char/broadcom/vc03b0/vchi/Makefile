#
EXTRA_CFLAGS := -fno-inline -Wno-multichar -O0
EXTRA_CFLAGS += -I$(LINUX_DIR)/include/linux/broadcom/omx
EXTRA_CFLAGS += -I$(LINUX_DIR)/include/linux/broadcom/vc03

### Include this only when message logging is needed
##EXTRA_CFLAGS += -DLOGGING_VCHI 

EXTRA_CFLAGS += -DVCHI_MAX_MSG_SIZE=4096

obj-$(CONFIG_BCM_VC03B0)          += vchi.o mphi.o control_service.o non_wrap_fifo.o endian.o vcmphidriver.o single.o multiqueue.o
#obj-$(CONFIG_BCM_VC03B0)          += host_msgfifo_wrapper.o vcih.o

obj-$(CONFIG_BCM_VC03B0)          += vcos.o bulk_aux_service.o
