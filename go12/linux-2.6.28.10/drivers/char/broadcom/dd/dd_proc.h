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
 * Description: Header of the Display Director procfs
 */

#ifndef DD_PROC_H
#define DD_PROC_H

#include <linux/broadcom/dd/dd.h>

DD_STATUS_T dd_proc_init(void);
DD_STATUS_T dd_proc_term(void);

#endif /* DD_PROC_H */
