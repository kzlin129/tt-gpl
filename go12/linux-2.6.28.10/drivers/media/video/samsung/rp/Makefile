#################################################
# Makefile for S3C6410 Renderer Pipeline
# 2009 (C) Samsung Electronics 
# Author : Jonghun Han <jonghun.han@samsung.com>
#################################################

EXTRA_CFLAGS += -Idrivers/media/video

obj-$(CONFIG_VIDEO_RP)			+= s3c_rp.o

s3c_rp-objs	:= s3c_rp_core.o s3c_rp_v4l2.o s3c_rp_cfg.o s3c_rp_pp_regs.o s3c_rp_rot_regs.o

ifeq ($(CONFIG_VIDEO_RP_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

