/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
 * Description: Header of the BCM476X Random Number Generator (RNG) driver.
 */ 

#ifndef _BCM476X_RNG_H
#define _BCM476X_RNG_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/hw_random.h>

/*
 * Read a randomly generated data. If success, data read in bytes will be returned
 */
extern int rng_data_read(struct hwrng *rng, u32 *data);
#endif /* __KERNEL__ */

#endif /* _BCM476X_RNG_H */
