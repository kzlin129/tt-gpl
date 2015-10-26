/*****************************************************************************
* Copyright 2005 - 2008 Broadcom Corporation.  All rights reserved.
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
 * linux/include/asm-arm/relay.h
 *
 * arm definitions for relayfs
 *
*****************************************************************************/


#ifndef _ASM_ARM_RELAY_H
#define _ASM_ARM_RELAY_H

#include <linux/relayfs_fs.h>

#ifdef (CONFIG_ARCH_BCM116X)
#include <asm/arch/reg_smt.h>

/**
 *	get_time_delta - utility function for getting time delta
 *	@now: pointer to a timeval struct that may be given current time
 *	@rchan: the channel
 *
 *	Returns either the TSC if TSCs are being used, or the time and the
 *	time difference between the current time and the buffer start time
 *	if TSCs are not being used.
 */
static inline u32
get_time_delta(struct timeval *now, struct rchan *rchan)
{
	u32 time_delta;

	if (using_tsc(rchan) == 1)
		time_delta = REG_SMT_CLK;
	else {
		do_gettimeofday(now);
		time_delta = calc_time_delta(now, &rchan->buf_start_time);
	}

	return time_delta;
}

/**
 *	get_timestamp - utility function for getting a time and TSC pair
 *	@now: current time
 *	@tsc: the TSC associated with now
 *	@rchan: the channel
 *
 *	Sets the value pointed to by now to the current time and the value
 *	pointed to by tsc to the tsc associated with that time, if the
 *	platform supports TSC.
 */
static inline void
get_timestamp(struct timeval *now,
	      u32 *tsc,
	      struct rchan *rchan)
{
	do_gettimeofday(now);

	if (using_tsc(rchan) == 1)
		*tsc = REG_SMT_CLK;
}

/**
 *	get_time_or_tsc - utility function for getting a time or a TSC
 *	@now: current time
 *	@tsc: current TSC
 *	@rchan: the channel
 *
 *	Sets the value pointed to by now to the current time or the value
 *	pointed to by tsc to the current tsc, depending on whether we're
 *	using TSCs or not.
 */
static inline void
get_time_or_tsc(struct timeval *now,
		u32 *tsc,
		struct rchan *rchan)
{
	if (using_tsc(rchan) == 1)
		*tsc = REG_SMT_CLK;
	else
		do_gettimeofday(now);
}

/**
 *	have_tsc - does this platform have a useable TSC?
 *
 *	Returns 1 if this platform has a useable TSC counter for
 *	timestamping purposes, 0 otherwise.
 */
static inline int
have_tsc(void)
{
	return 1;
}

#else /* No TSC support  */
#include <asm-generic/relay.h>
#endif
#endif
