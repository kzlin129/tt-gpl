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
 *   @file   bcm_os_support.c
 * 
 *   @brief  Linux os level functions.
 * 
 ****************************************************************************/


/* bcm_os_support.h, version for linux (kernel mode) */

#if !defined __KERNEL__  
#error "Error: This file is meant for Linux kernel mode only"
#endif


#include "bcm_os_support.h"
#include <linux/kernel.h>
#include <linux/cpufreq.h> //ARM Frequency change transitions..
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/delay.h>

// static void bcm_cpufreq_notify_transition(struct cpufreq_freqs *freqs, unsigned int state);
// static void adjust_jiffies(unsigned long val, struct cpufreq_freqs *ci);

int bcm_vprintf(const char *fmt, va_list ap)
{
	return vprintk(fmt, ap);
}

uint32_t	bcm_xlate_to_phys_addr(uint32_t addr)
{
	return IO_ADDRESS_TO_PHY(addr);
}

uint32_t	bcm_xlate_to_virt_addr(uint32_t addr)
{
	return IO_ADDRESS(addr);
}

void bcm_delay_usec(uint32_t usecs_delay)
{
	udelay(usecs_delay);
}

int bcm_strcmp(const char *str1, const char *str2)
{
	return strcmp(str1, str2);
}

int bcm_strlen(const char *str)
{
	return strlen(str);
}

char *bcm_strcpy(char *dest, const char *src)
{
    return strcpy(dest, src);
}

char *bcm_strncpy(char *dest, const char *src, size_t count)
{
    return strncpy(dest, src, count);
}

int bcm_memcmp(const void *buf1, const void *buf2, int count)
{
	return memcmp(buf1, buf2, count);
}

void *bcm_memset(void *s, int c, size_t count)
{
    return memset(s, c, count);
}

long bcm_strtol(const char *str, char **end_ptr, unsigned int base)
{
    return(simple_strtol(str, end_ptr, base));
}

char *bcm_strsep(char **stringp, const char *delim)
{
    return(strsep(stringp, delim));
}

char *bcm_strstr(const char *cs, const char *ct)
{
    return(strstr(cs, ct));

}

#if 0
void bcm_cpufreq_prechange_handler(unsigned long old_freq, unsigned long new_freq)
{
#if 0
	struct cpufreq_freqs cpu_freqs = { 0 };

    cpu_freqs.old = old_freq/1000; //in KHz
    cpu_freqs.new = new_freq/1000; //in KHz
    cpu_freqs.flags = 0; //TODO: is this the correct value?

    //note: should we invoke get_cpu() here??
    //get_cpu disables preemption until put_cpu is called though...
    cpu_freqs.cpu = 0; 

	bcm_cpufreq_notify_transition(&cpu_freqs, CPUFREQ_PRECHANGE);
#endif
    return;
}

void bcm_cpufreq_postchange_handler(unsigned long old_freq, unsigned long new_freq)
{
#if 0
	struct cpufreq_freqs cpu_freqs = { 0 };

    cpu_freqs.old = old_freq/1000; //in KHz
    cpu_freqs.new = new_freq/1000; //in KHz
    cpu_freqs.flags = 0; //TODO: is this the correct value?

    //note: should we invoke get_cpu() here??
    //get_cpu disables preemption until put_cpu is called though...
    cpu_freqs.cpu = 0; 

	bcm_cpufreq_notify_transition(&cpu_freqs, CPUFREQ_POSTCHANGE);
#endif
    return;
}


static void bcm_cpufreq_notify_transition(struct cpufreq_freqs *freqs, unsigned int state)
{
    return(adjust_jiffies(state, freqs));
}

/**
    Copied from cpufreq.c under drivers/cpufreq.

    The cpufreq driver uses a semaphore in the notify_transistion function.
    The Tahoe library can be called from interrupt context and from other
    modules which disable interrupts.
     
    Hence, this driver will not call into the cpufreq driver about frequency
    change.
*/
extern unsigned long loops_per_jiffy;

static void adjust_jiffies(unsigned long val, struct cpufreq_freqs *ci)
{
#if 0
    static unsigned long l_p_j_ref = 0;
    static unsigned int  l_p_j_ref_freq = 0;

	if (ci->flags & CPUFREQ_CONST_LOOPS)
		return;

	if (!l_p_j_ref_freq) 
	{
		l_p_j_ref = loops_per_jiffy;
		l_p_j_ref_freq = ci->old;
		//printk("saving %lu as reference value for loops_per_jiffy; freq is %u kHz\n", l_p_j_ref, l_p_j_ref_freq);
	}

	if ((val == CPUFREQ_PRECHANGE  && ci->old < ci->new) ||
	    (val == CPUFREQ_POSTCHANGE && ci->old > ci->new) ||
	    (val == CPUFREQ_RESUMECHANGE || val == CPUFREQ_SUSPENDCHANGE)) 
	{
		loops_per_jiffy = cpufreq_scale(l_p_j_ref, l_p_j_ref_freq, ci->new);
		//printk("scaling loops_per_jiffy to %lu for frequency %u kHz\n", loops_per_jiffy, ci->new);
	}
#endif
    return;
}

#endif
