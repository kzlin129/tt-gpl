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



/****************************************************************************
*
*  idle_prof.h
*
*  PURPOSE:
*
*     This file contains code to profile idle cycles.
*
*  NOTES:
*
****************************************************************************/

#if !defined( __IDLE_PROF_H )
#define __IDLE_PROF_H

#if defined( CONFIG_BCM_IDLE_PROFILER_SUPPORT )

/* ---- Include Files ---------------------------------------------------- */

#if defined( __KERNEL__ )

#include <asm/types.h>
#include <linux/spinlock.h>

/* ---- Constants and Types ---------------------------------------------- */

typedef struct
{
   u32 last_smtclk;
   u32 last_idle_count;
} idle_handle_t;

extern spinlock_t idle_lock;
extern u32 idle_count;

extern u32 idle_prof_get_tick_count(void);
extern u32 idle_prof_get_tick_rate(void);

/*
 * arch_init_idle_profile - initialize profiling handle
 *
 * parameters:
 *   handle [in/out] - pointer to the profiling handle
 */
static inline void arch_init_idle_profile(idle_handle_t *handle)
{
   unsigned long flags;

   spin_lock_irqsave( &idle_lock, flags );

   handle->last_smtclk = idle_prof_get_tick_count();
   handle->last_idle_count = idle_count;

   spin_unlock_irqrestore( &idle_lock, flags );
}

/*
 * arch_get_idle_profile - get the idle profiling results for the provided handle
 *
 * parameters:
 *   handle [in] - pointer to the profiling handle, must be previously initialized
 *   idle [out]  - returns the number of idle cycles since last init or get call
 *   total [out] - returns the number of total cycles since last init or get call
 *
 * return:
 *   The number of cycles per second
 *
 * note:
 *   To prevent overflowing the cycle counters, the get call must be made no
 *   later than (2^32 / ticks_per_second) seconds from the last init or get call.
 *   For 1024 ticks per second, the time is 4,194,304 seconds = 48.54 days;
 *   For 812500 ticks per second, the time is 5286 seconds = 1 hour 28 minutes.
 */
static inline int arch_get_idle_profile(idle_handle_t *handle, u32 *idle, u32 *total)
{
   unsigned long flags;
   u32 now;

   spin_lock_irqsave( &idle_lock, flags );

   now = idle_prof_get_tick_count();
   *idle = idle_count - handle->last_idle_count;
   *total = now - handle->last_smtclk;

   handle->last_idle_count = idle_count;
   handle->last_smtclk = now;

   spin_unlock_irqrestore( &idle_lock, flags );
	return idle_prof_get_tick_rate();
}

#endif  /* CONFIG_BCM_IDLE_PROFILER_SUPPORT */
#endif  /* __KERNEL__ */
#endif  /* __IDLE_PROF_H */
