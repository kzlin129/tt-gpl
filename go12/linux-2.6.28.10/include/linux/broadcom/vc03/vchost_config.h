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



#ifndef VCHOST_CONFIG_H
#define VCHOST_CONFIG_H

/*****************************************************************************
*
*   Building as part of the kernel
*
*****************************************************************************/

#if defined( __KERNEL__ )

#include <linux/types.h>
#include <linux/string.h>

/* Some environments may want different versions of the assert macro. If so, they can be
   customised here. */

#include <linux/kernel.h>
#include <linux/sched.h>

static inline void assert_delay( void )
{
   set_current_state( TASK_INTERRUPTIBLE );

   schedule_timeout( HZ * 5 );
}

#define vc_assert( expr )												\
  ((void)((expr) || ( printk( KERN_ERR "k[%s:%d] ***** Assertion '%s' failed\n", __FUNCTION__, __LINE__, #expr), 0 )))



/*****************************************************************************
*
*   Building as part of the bootloader
*
*****************************************************************************/

#elif defined( BUILDING_BOOTLOADER )

#define printk  printf
#define KERN_ERR

#define vc_assert( expr ) \
  ((void)((expr) || ( fprintf(stdout,"b[%s:%d]***** Assertion '%s' Failed\n", __FUNCTION__, __LINE__, #expr), 0 )))
    
/*****************************************************************************
*
*   Building as part of a user-mode program (i.e. vc-filesys or vc-fuse)
*
*****************************************************************************/

#else   // User Mode

#define USE_FILESYS 1

extern  int gVcFd;

#include <assert.h>
#include <stdio.h>
#include <inttypes.h>

#define printk      printf
#define KERN_ERR

#define vc_assert( expr ) \
  ((void)((expr) || ( fprintf(stdout,"u[%s:%d]***** Assertion '%s' Failed\n", __FUNCTION__, __LINE__, #expr), 0 )))


#define min( a, b )     (( (a) < (b) ) ? (a) : (b) )
#define max( a, b )     (( (a) > (b) ) ? (a) : (b) )

#endif

// The following is also defined in include/linux/broadcom/vchost-port.h
//#define VC_HOST_IS_BIG_ENDIAN

/* "Host to VideoCore and VideoCore to host long/short". */

//#define VC_HTOV32(val) ((val<<24) | ((val&0xff00)<<8) | ((val>>8)&0xff00) | ((val>>24)&0xff))
//#define VC_HTOV16(val) ((uint16_t)((val<< 8) | ((val>>8)&0xff)))

#define VC_HTOV32(val) (val)
#define VC_HTOV16(val) (val)
#define VC_VTOH32(val) VC_HTOV32(val)
#define VC_VTOH16(val) VC_HTOV16(val)

#define VCHPRE_    extern
#define VCHPOST_

#endif
