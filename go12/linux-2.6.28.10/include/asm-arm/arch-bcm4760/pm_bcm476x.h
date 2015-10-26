/*****************************************************************************
* Copyright 2009 Broadcom Corporation.  All rights reserved.
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
/*  pm_bcm476x.h - low level PM functions and definitions for BCM476x architecture.
 */

#ifndef PM_BCM476X_H
#define PM_BCM476X_H

#include <linux/types.h>

extern uint32_t bcm476x_cpu_enter_sleep(void*);
extern void bcm476x_cpu_exit_sleep(void);

#endif
