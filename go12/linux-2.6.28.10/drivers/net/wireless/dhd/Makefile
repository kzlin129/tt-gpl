#
# Makefile fragment for Linux 2.6
# Broadcom DHD Driver
#
# Copyright (C) 1999-2008, Broadcom Corporation
# 
#      Unless you and Broadcom execute a separate written software license
# agreement governing use of this software, this software is licensed to you
# under the terms of the GNU General Public License version 2 (the “GPL”),
# available at http://www.broadcom.com/licenses/GPLv2.php, with the
# following added to such license:
# 
#      As a special exception, the copyright holders of this software give you
# permission to link this software with independent modules, and to copy and
# distribute the resulting executable under terms of your choice, provided that
# you also meet, for each linked independent module, the terms and conditions of
# the license of that module.  An independent module is a module which is not
# derived from this software.  The special exception does not apply to any
# modifications of the software.
# 
#      Notwithstanding the above, under no circumstances may you combine this
# software in any way with any other Broadcom software provided under a license
# other than the GPL, without Broadcom's express prior written consent.
#
# $Id: makefile.26,v 1.1.144.1 2008/05/07 22:53:44 Exp $

DHDOFILES=dhd_linux.o		\
	  linux_osl.o		\
	  bcmutils.o		\
	  dhd_common.o		\
	  siutils.o		\
	  sbutils.o		\
	  aiutils.o		\
	  hndpmu.o		\
	  dhd_linux_sched.o	\
	  wl_iw.o		\
	  bcmwifi.o		\
	  dhd_cdc.o		\
	  dhd_sdio.o		\
	  bcmsdh_sdmmc.o	\
	  bcmsdh.o		\
	  bcmsdh_linux.o	\
	  bcmsdh_sdmmc_linux.o

DHDCFLAGS=-DLINUX		\
	  -DSRCBASE=.		\
	  -DBCMDRIVER		\
	  -DBCMDONGLEHOST	\
	  -DDHDTHREAD		\
	  -DBCMWPA2		\
	  -DUNRELEASEDCHIP	\
	  -DDHD_GPL		\
	  -DDHD_SCHED		\
	  -DBDC			\
	  -DTOE			\
	  -DDHD_BCMEVENTS	\
	  -DSHOW_EVENTS		\
	  -DBCMSDIO		\
	  -DDHD_GPL		\
	  -DBCMLXSDMMC		\
	  -DBCMPLATFORM_BUS	\
	  -DSDIO_ISR_THREAD	\
	  -D__KERNEL__		\
	  -Wall			\
	  -Wstrict-prototypes	\
	  -Werror		\
	  -Iinclude/linux/broadcom/dhd/include	\
	  -Iinclude/linux/broadcom/dhd/shared	\
	  -Iinclude/linux/broadcom/dhd/dhd/sys	\
	  -Iinclude/linux/broadcom/dhd/dongle	\
	  -Iinclude/linux/broadcom/dhd/wl/sys

obj-$(CONFIG_DHD)	+= dhd.o
dhd-objs		= $(DHDOFILES)
EXTRA_CFLAGS 		= $(DHDCFLAGS)
