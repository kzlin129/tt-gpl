/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/
/*
 * linux/drivers/userdma.h
 *
 * Broadcom BCM476X user dma driver header file
 */

#ifndef __BCM476X_USERDMA_H
#define __BCM476X_USERDMA_H

int bcm476x_userdma_do_dma( userdma_ioctl_t *arg );
int bcm476x_userdma_alloc_channel(int *ch);
int bcm476x_userdma_release_channel(int ch);

#define userdma_alloc_channel(pch)      bcm476x_userdma_alloc_channel(pch)
#define userdma_release_channel(ch)     bcm476x_userdma_release_channel(ch)
#define userdma_do_dma(arg)             bcm476x_userdma_do_dma(arg)

#endif /* __BCM476X_USERDMA_H */
