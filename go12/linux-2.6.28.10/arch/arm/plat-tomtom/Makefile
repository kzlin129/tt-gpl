#
# Makefile for the linux kernel.
#

# Common support
# !!!!Don't remove the obj-y line!!!!
obj-$(CONFIG_STRICT_DEVMEM)		+= mem.o
obj-$(CONFIG_TOMTOM_GADC)		+= gadc_generic.o
obj-$(CONFIG_TOMTOM_FDT)		+= libfdt.o
obj-$(CONFIG_TOMTOM_TT_SETUP)		+= tt_setup_handler.o
ifeq ($(CONFIG_TOMTOM_FDT),y)
obj-$(CONFIG_PLAT_TOMTOM)		+= tt_brick.o tt_crypto.o
else
obj-$(CONFIG_PLAT_TOMTOM)		+= tt_brick.o
endif
obj-m :=
obj-n :=
obj-  :=

# Structure that are to be reused , not necessarily fully initialized
obj-$(CONFIG_TOMTOM_TT_SETUP)		+= tt_setup_dev.o
obj-$(CONFIG_TOMTOM_FDT)		+= libfdt_dev.o fdtexport_dev.o

