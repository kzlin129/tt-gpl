/*
 * linux/drivers/userdma.h
 *
 * Broadcom BCM476X user dma driver header file
 *
 * Copyright (C) 2008-2009 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __USERDMA_H
#define __USERDMA_H
/* ---- Include Files ---------------------------------------------------- */
#include <linux/ioctl.h>

#define __DEBUG_USERDMA__

#ifdef  __DEBUG_USERDMA__
# define dprintk(x...) printk(x)
#else
# define dprintk(x...) do { } while(0)
#endif

#define USERDMA_MAGIC   'D'

enum {
    USERDMA_ALLOC_CHANNEL=1,
    USERDMA_RELEASE_CHANNEL,
    USERDMA_ALLOC_BUF,
    USERDMA_RELEASE_BUF,
    USERDMA_DO_DMA,
};

typedef struct userdma_mem_s
{
    void       *virt;
    void       *phys;
    size_t     alloc_size;
} userdma_mem_t;

typedef struct userdma_xfer_desc {
    int  n_addrs;                   /* number of addresses in addrs & sizes */
    userdma_mem_t **addrs;          /* xfer addresses */
    int  *sizes;                    /* xfer sizes */
    int  width;                     /* transfer width, 0=8-bits, 1=16-bits, 2=32-bits */
    int  incr;                      /* increment address when transfer ? */
} userdma_xfer_desc_t;

typedef struct userdma_ioctl_s {
    userdma_xfer_desc_t src;        /* source description. */
    userdma_xfer_desc_t dest;       /* destination description. */
    int dma_ch;                     /* dma channel to use or -1 for any channel */
    int loop;                       /* number of repeat transfers */
    int burst;                      /* burst size */
    int verify;                     /* verify after done ? */
    int verbose;                    /* verbose enable */
} userdma_ioctl_t;

#define USERDMA_IOCTL_ALLOC_CHANNEL     _IOR(USERDMA_MAGIC, USERDMA_ALLOC_CHANNEL, int)     // arg is int*
#define USERDMA_IOCTL_RELEASE_CHANNEL   _IOW(USERDMA_MAGIC, USERDMA_RELEASE_CHANNEL, int)   // arg is int
#define USERDMA_IOCTL_ALLOC_BUF         _IOWR(USERDMA_MAGIC, USERDMA_ALLOC_CHANNEL, userdma_mem_t)     // arg is userdma_mem_t*
#define USERDMA_IOCTL_RELEASE_BUF       _IOW(USERDMA_MAGIC, USERDMA_RELEASE_CHANNEL, userdma_mem_t)   // arg is userdma_mem_t*
#define USERDMA_IOCTL_DMA               _IOW(USERDMA_MAGIC, USERDMA_DO_DMA, userdma_ioctl_t)// arg is userdma_ioctl_t*


#endif /* __USERDMA_H */
