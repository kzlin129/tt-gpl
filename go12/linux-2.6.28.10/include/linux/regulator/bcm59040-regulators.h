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


/*
*
*****************************************************************************
*
*  PURPOSE:
*     This file contains definitions for the BCM59040 regulators.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( __LINUX_REGULATORS_BCM59040_REGULATORS_H )
#define __LINUX_REGULATORS_BCM59040_REGULATORS_H

/* ---- Include Files ---------------------------------------------------- */
#include <asm/arch/hardware.h>

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

enum bcm59040_regulator_ids
{
    BCM59040_LDO1_ID=0,
    BCM59040_LDO2_ID,
    BCM59040_LDO3_ID,
    BCM59040_LDO4_ID,
    BCM59040_LDO5_ID,
    BCM59040_LDO6_ID,
    BCM59040_CSR_ID,
    BCM59040_IOSR_ID,
};


#endif /* __LINUX_REGULATORS_BCM59040_REGULATORS_H */
