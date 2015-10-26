/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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
*  perfcnt.h
*
*  Usage:
*****************************************************************************/
#ifndef LINUX_BROADCOM_PERFCNT_H
#define LINUX_BROADCOM_PERFCNT_H

#include <cfg_global.h>


#if !defined ( CFG_GLOBAL_CPU )
#error    CFG_GLOBAL_CPU must be defined
#endif

#if (CFG_GLOBAL_CPU != ARM11) && (CFG_GLOBAL_CPU != MIPS32)
#error CFG_GLOBAL_CPU must be ARM11 or MIPS32 to be using this file
#endif


#if ( CFG_GLOBAL_CPU == ARM11 )
#include <asm/arch/csp/arm_perf.h>
#elif ( CFG_GLOBAL_CPU == MIPS32 )
#include <asm/broadcom/bcm1103/mips_perf.h>
#endif


#if ( CFG_GLOBAL_CPU == ARM11 )
typedef ARM_PERF_CNTRS PERFCNT_CNTRS;
#elif ( CFG_GLOBAL_CPU == MIPS32 )
typedef MIPS_PERF_CNTRS PERFCNT_CNTRS;
#endif

void perfcnt_start(void);
void perfcnt_stop(PERFCNT_CNTRS *cntrs);
void perfcnt_clear(void);
void perfcnt_read(PERFCNT_CNTRS *cntrs);
const char *perfcnt_get_evtstr0(void);
const char *perfcnt_get_evtstr1(void);


#endif // !defined( LINUX_BROADCOM_PERFCNT_H )

