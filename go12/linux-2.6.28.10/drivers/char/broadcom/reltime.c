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
*
*****************************************************************************
*
*  reltime.c
*
*  PURPOSE:
*
*     This implements a relative time driver used to retrieve a time
*     value not affected by changes to the system time (settime()).
*     Basically, this driver used used in the same way a call to
*     clock_gettime() with clock type CLOCK_MONOTONIC would work if
*     that clock type was available on our system.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/ioctl.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/reltime.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

#define ONE_SEC_IN_NSEC 1000000000

/* ---- Private Variables ------------------------------------------------ */

static char banner[] __initdata = KERN_INFO "BCM116x Relative Time Driver: 0.02\n";

/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */

/****************************************************************************
*
*  reltime_open
*
***************************************************************************/

static int reltime_open( struct inode *inode, struct file *file )
{
    return 0;

} /* reltime_open */

/****************************************************************************
*
*  reltime_read
*
***************************************************************************/

static ssize_t reltime_read( struct file *file, char *buffer, size_t count, loff_t *ppos )
{
    return -EINVAL;

} /* reltime_read */

/****************************************************************************
*
*  reltime_release
*
***************************************************************************/

static int reltime_release( struct inode *inode, struct file *file )
{
    return 0;

} /* reltime_release */

/****************************************************************************
*
*  reltime_write
*
***************************************************************************/

static ssize_t reltime_write( struct file *file, const char *buffer, size_t count, loff_t *ppos )
{
    return -EINVAL;

} /* reltime_write */

/****************************************************************************
*
*  reltime_write
*
***************************************************************************/

static int reltime_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
    struct timespec relativeTime;
    struct timespec diffTime;
    u64 j64;
    u64 nsec;

    //  Get the current jiffies counter as a timespec, using the full 64-bit jiffies
    //  counter to derive the full 32-bit seconds counter.  A full 32-bit seconds
    //  counter allows wrap conditions to be handled cleanly using signed subtract
    //  comparisons.
    //
    j64 = get_jiffies_64();
    nsec = j64 * TICK_NSEC;
    relativeTime.tv_nsec = do_div(nsec, NSEC_PER_SEC);
    relativeTime.tv_sec = nsec;

    switch (cmd)
    {
        case RELTIME_IOCTL_GET:
            //  Just return back the current jiffie counter as a timespec
            //
            if (copy_to_user((void *) arg, &relativeTime, sizeof(relativeTime)))
                return -EFAULT;
            break;
        case RELTIME_IOCTL_DIFF:
            //  Get from the users their initial time value
            //
            if (copy_from_user(&diffTime, (void *) arg, sizeof(diffTime)))
                return -EFAULT;

            //  Find the difference between the two timespecs
            //
            if ((diffTime.tv_sec > relativeTime.tv_sec) ||
                ((diffTime.tv_sec == relativeTime.tv_sec) && (diffTime.tv_nsec > diffTime.tv_nsec)))
            {
                //  The user's time value is greater than current time.  Just
                //  return their time value back to them.
            }
            else
            {
                if (diffTime.tv_nsec > relativeTime.tv_nsec)
                {
                    relativeTime.tv_sec--;
                    relativeTime.tv_nsec += ONE_SEC_IN_NSEC;
                }
                relativeTime.tv_sec -= diffTime.tv_sec;
                relativeTime.tv_nsec -= diffTime.tv_nsec;
            }

            //  Return the difference to the user.
            //
            if (copy_to_user((void *) arg, &relativeTime, sizeof(relativeTime)))
                return -EFAULT;
            break;
        default:
            return -EINVAL;
    }

	return 0;

} /* reltime_ioctl */

/****************************************************************************
*
*   File Operations (these are the device driver entry points)
*
***************************************************************************/

struct file_operations reltime_fops =
{
    owner:      THIS_MODULE,
    open:       reltime_open,
    release:    reltime_release,
    read:       reltime_read,
    write:      reltime_write,
    ioctl:      reltime_ioctl
};

/****************************************************************************
*
*  reltime_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init reltime_init( void )
{
    int rc;

    printk( banner );

    if (( rc = register_chrdev( BCM_RELTIME_MAJOR, "reltime", &reltime_fops )) < 0 )
    {
        printk( KERN_WARNING "reltime: register_chrdev failed for major %d\n", BCM_RELTIME_MAJOR );
        return rc;
    }

    return 0;

} /* reltime_init */

/****************************************************************************
*
*  reltime_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit reltime_exit( void )
{
} /* reltime_exit */

/****************************************************************************/


/* Changed from module_init to fs_initcall. Not sure if it was reqired.
 * PMU drivers were changed to fs_initcall so that they are loaded before
 * most of the other drivers. THis was done because the host has to
 * read the PMU interrupts in time (< 8sec) or else the PMU
 * timeout timer (of 8sec) could expire causing the phone to shut off.
 * This was observed in cases where a battery was removed and then re inserted.
 * This action would cause a LOWBAT interrupt generated and the host has 8sec
 * to clear it before PMU goes into standby mode. If VC02 driver was loaded
 * before PMU driver, the PMU driver was getting loaded well past 8sec window
 */

fs_initcall(reltime_init);
module_exit(reltime_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Relative Time Driver");

