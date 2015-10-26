/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
/** 
 * 
 *   @file   bcm_amba_io.h 
 * 
 *   @brief  Low level read write functions. 
 * 
 ****************************************************************************/

#ifndef _BCM_AMBA_IO_H
#define _BCM_AMBA_IO_H

#include "bcm_os_support.h"
#include "bcm_basedefs.h"

BCM_INLINE uint64_t amba_read64(volatile uint64_t *addr)
{
	return bcm_amba_read64(addr);
}

BCM_INLINE void amba_write64(volatile uint64_t *addr, uint64_t value)
{
	bcm_amba_write64(addr, value);
}

BCM_INLINE uint32_t amba_read32(volatile uint32_t *addr)
{
	return bcm_amba_read32(addr);
}

BCM_INLINE void amba_write32(volatile uint32_t *addr, uint32_t value)
{
	bcm_amba_write32(addr, value);
}

BCM_INLINE uint16_t amba_read16(volatile uint16_t *addr)
{
	return bcm_amba_read16(addr);
}

BCM_INLINE void amba_write16(volatile uint16_t *addr, uint16_t value)
{
	bcm_amba_write16(addr, value);
}

BCM_INLINE uint8_t amba_read8(volatile uint8_t *addr)
{
	return bcm_amba_read8(addr);
}

BCM_INLINE void amba_write8(volatile uint8_t *addr, uint8_t value)
{
	bcm_amba_write8(addr, value);
}

#endif /* #ifndef _BCM_AMBA_IO_H */

