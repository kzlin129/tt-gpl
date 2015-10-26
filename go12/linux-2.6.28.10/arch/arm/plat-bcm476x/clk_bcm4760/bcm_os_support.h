/*****************************************************************************
* Copyright 2007 - 2009 Broadcom Corporation.  All rights reserved.
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
 *   @file   bcm_os_support.h
 * 
 *   @brief  This file contains linux os support functions.
 * 
 ****************************************************************************/

#ifndef DOXYGEN_IGNORE_EXTERNAL
//----------------------------------------------------------
/**

 @file bcm_os_support.h

    This file contains linux os support functions.

 @todo 

 @author Seetharam Samptur

    Copyright 2004 - 2007 Broadcom Corporation.  All rights reserved.

    Unless you and Broadcom execute a separate written software license
    agreement governing use of this software, this software is licensed to you
    under the terms of the GNU General Public License version 2, available at
    http://www.gnu.org/copyleft/gpl.html (the "GPL").

    Notwithstanding the above, under no circumstances may you combine this
    software in any way with any other Broadcom software provided under a
    license other than the GPL, without Broadcom's express prior written
    consent.


*/
//----------------------------------------------------------
#endif

#ifndef _BCM_OS_SUPPORT_H_
#define _BCM_OS_SUPPORT_H_

/* bcm_os_support.h, version for linux (kernel mode) */

#if !defined __KERNEL__
#error "Error: This file is meant for Linux kernel mode only"
#endif

/* be extremely careful about what files are included here
 *  or else "os-dependency" will "leak" into other parts of tahoe
 */
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/spinlock.h>
#include <linux/syscalls.h>

// Linux kernel already has these defined...
//typedef unsigned long long    uint64_t;
typedef int				int_t;
//typedef volatile 	uint8_t		reg8_t;
//typedef volatile 	uint16_t	reg16_t;
//typedef volatile 	uint32_t	reg32_t;
//typedef unsigned long       uint_ptr_t;
//typedef char *				char_ptr_t;

int bcm_vprintf(const char *fmt, va_list ap);

uint32_t	bcm_xlate_to_phys_addr(uint32_t addr);

uint32_t	bcm_xlate_to_virt_addr(uint32_t addr);

void bcm_delay_usec(uint32_t usecs_delay);

static inline void bcm_nop(void)
{
    __asm("mov\tr0,r0\t@ nop\n\t");  //from asm-arm/system.h  
}

static inline uint64_t bcm_amba_read64(volatile uint64_t *addr)
{
	/* linux does not have an equivalent 64-bit function */
	return *addr;
}

static inline void bcm_amba_write64(volatile uint64_t *addr, uint64_t value)
{
	/* linux does not have an equivalent 64-bit function */
	*addr = value;
}

static inline uint32_t bcm_amba_read32(volatile uint32_t *addr)
{
	return *addr;
}

static inline void bcm_amba_write32(volatile uint32_t *addr, uint32_t value)
{
	*addr = value;
}

static inline uint16_t bcm_amba_read16(volatile uint16_t *addr)
{
	return *addr;
}

static inline void bcm_amba_write16(volatile uint16_t *addr, uint16_t value)
{
	*addr = value;
}

static inline uint8_t bcm_amba_read8(volatile uint8_t *addr)
{
	/* linux does not have an equivalent 8-bit function */
	return *addr;
}

static inline void bcm_amba_write8(volatile uint8_t *addr, uint8_t value)
{
	/* linux does not have an equivalent 8-bit function */
	*addr = value;
}


/**
    Kernel Synchronization Methods
*/

typedef spinlock_t bcm_lock_t;

static inline void bcm_lock_init(bcm_lock_t *plock)
{
    BUG_ON(!plock);
    spin_lock_init(plock);
}

static inline void bcm_lock(bcm_lock_t *plock)
{
    BUG_ON(!plock);
    spin_lock(plock);
}

static inline void bcm_unlock(bcm_lock_t *plock)
{
    BUG_ON(!plock);
    spin_unlock(plock);
}

static inline void bcm_lock_irq(bcm_lock_t *plock, unsigned long *pflags)
{
    BUG_ON(!plock);
	BUG_ON(!pflags);
    spin_lock_irqsave(plock, *pflags);
}

static inline void bcm_unlock_irq(bcm_lock_t *plock, unsigned long *pflags)
{
    BUG_ON(!plock);
    spin_unlock_irqrestore(plock, *pflags);
}

static inline long bcm_sys_sync(void)
{
    return sys_sync();
}


/**
    CPU frequency related functions
      Because of timing constrainsts, OS-support is being used to provide an
      entry into the linux cpufreq driver.

      In the next phase, this will be moved into separate event handling code with
      provisions for multiple clients to register for different events. 
*/
void bcm_cpufreq_prechange_handler(unsigned long old_freq, unsigned long new_freq);

void bcm_cpufreq_postchange_handler(unsigned long old_freq, unsigned long new_freq);


/*
    string related functions
 */
char *bcm_strcpy(char *dest, const char *src);

char *bcm_strncpy(char *dest, const char *src, size_t count);

int bcm_strcmp(const char *str1, const char *str2);

int bcm_strlen(const char *str);

int bcm_memcmp(const void *buf1, const void *buf2, int count);

void *bcm_memset(void *s, int c, size_t count);

long bcm_strtol(const char *str, char **end_ptr, unsigned int base);

char *bcm_strsep(char **stringp, const char *delim);

char *bcm_strstr(const char *cs, const char *ct);


#endif /* #ifndef _BCM_OS_SUPPORT_H_ */

