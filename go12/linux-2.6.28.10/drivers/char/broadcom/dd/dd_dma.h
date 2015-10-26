/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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
 * Description: Header of the Display Director DMA mechanism
 */

#ifndef DD_DMA_H
#define DD_DMA_H

#include <linux/broadcom/dd/dd.h>

/*
 * Initialize the DMA module and register its ISR
 *
 * This routine needs to be called once (and only once) before DMA can
 * be used
 */
extern DD_STATUS_T dd_dma_init(void);

/*
 * Terminate the DMA
 */
extern DD_STATUS_T dd_dma_term(void);

#endif /* DD_DMA_H */
