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
*  gpio.c
*
*  PURPOSE:
*
*     This implements the gpio driver.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#include <linux/broadcom/gpio.h>
#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/gpio_irq.h>

#if defined( CONFIG_BCM_VC02 )
#include <linux/broadcom/vc_gpio.h>
#endif

/* ---- Private Constants and Types -------------------------------------- */

#define GPIO_DRIVER_NAME "gpio"

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO)

#if DEBUG
#	define GPIO_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define GPIO_DEBUG(level,x)
#endif

/* ---- Private Function Prototypes -------------------------------------- */

static int proc_do_gpio_intvec(ctl_table *table, int write, struct file *filp,
		     void __user *buffer, size_t *lenp, loff_t *ppos );

/* ---- Private Variables ------------------------------------------------ */
static int gLevel = DBG_DEFAULT_LEVEL;

#if defined( CONFIG_ARCH_116X )
#   define  NUM_CHIP_GPIOS  64
#else
#   define  NUM_CHIP_GPIOS  30
#endif

static int gValue[NUM_CHIP_GPIOS];

#if defined( CONFIG_BCM_VC02 )
static int gValue_vc[VC_GPIO_NUM_GPIO];
#endif

/* sysctl */
static  struct ctl_table_header    *gSysCtlHeader = NULL;

#define BCM_SYSCTL_GPIO_NODE(ID) {			            \
	.ctl_name      = ID,		   \
	.procname      = #ID,					               \
	.data          = &gValue[ID],					         \
	.maxlen        = sizeof( int ),					      \
	.mode          = 0644,					               \
   .proc_handler  = &proc_do_gpio_intvec              \
}

#define BCM_SYSCTL_GPIO_VC_NODE(ID) {			         \
	.ctl_name      = ID + GPIO_VC02_OFFSET,		\
	.procname      = "vc_"#ID,					            \
	.data          = &gValue_vc[ID],					      \
	.maxlen        = sizeof( int ),					      \
	.mode          = 0644,					               \
   .proc_handler  = &proc_do_gpio_intvec           \
}
static struct ctl_table gSysCtlLocal[] =
{
   {
      .ctl_name   = BCM_SYSCTL_GPIO_LEVEL,
      .procname   = "level",
      .data       = &gLevel,
      .maxlen     = sizeof( int ),
      .mode       = 0644,
      .proc_handler  = &proc_dointvec
   },
   BCM_SYSCTL_GPIO_NODE(0),
   BCM_SYSCTL_GPIO_NODE(1),
   BCM_SYSCTL_GPIO_NODE(2),
   BCM_SYSCTL_GPIO_NODE(3),
   BCM_SYSCTL_GPIO_NODE(4),
   BCM_SYSCTL_GPIO_NODE(5),
   BCM_SYSCTL_GPIO_NODE(6),
   BCM_SYSCTL_GPIO_NODE(7),
   BCM_SYSCTL_GPIO_NODE(8),
   BCM_SYSCTL_GPIO_NODE(9),
   BCM_SYSCTL_GPIO_NODE(10),
   BCM_SYSCTL_GPIO_NODE(11),
   BCM_SYSCTL_GPIO_NODE(12),
   BCM_SYSCTL_GPIO_NODE(13),
   BCM_SYSCTL_GPIO_NODE(14),
   BCM_SYSCTL_GPIO_NODE(15),
   BCM_SYSCTL_GPIO_NODE(16),
   BCM_SYSCTL_GPIO_NODE(17),
   BCM_SYSCTL_GPIO_NODE(18),
   BCM_SYSCTL_GPIO_NODE(19),
   BCM_SYSCTL_GPIO_NODE(20),
   BCM_SYSCTL_GPIO_NODE(21),
   BCM_SYSCTL_GPIO_NODE(22),
   BCM_SYSCTL_GPIO_NODE(23),
   BCM_SYSCTL_GPIO_NODE(24),
   BCM_SYSCTL_GPIO_NODE(25),
   BCM_SYSCTL_GPIO_NODE(26),
   BCM_SYSCTL_GPIO_NODE(27),
   BCM_SYSCTL_GPIO_NODE(28),
   BCM_SYSCTL_GPIO_NODE(29),
#if ( NUM_CHIP_GPIOS > 30 )
   BCM_SYSCTL_GPIO_NODE(30),
   BCM_SYSCTL_GPIO_NODE(31),
   BCM_SYSCTL_GPIO_NODE(32),
   BCM_SYSCTL_GPIO_NODE(33),
   BCM_SYSCTL_GPIO_NODE(34),
   BCM_SYSCTL_GPIO_NODE(35),
   BCM_SYSCTL_GPIO_NODE(36),
   BCM_SYSCTL_GPIO_NODE(37),
   BCM_SYSCTL_GPIO_NODE(38),
   BCM_SYSCTL_GPIO_NODE(39),
   BCM_SYSCTL_GPIO_NODE(40),
   BCM_SYSCTL_GPIO_NODE(41),
   BCM_SYSCTL_GPIO_NODE(42),
   BCM_SYSCTL_GPIO_NODE(43),
   BCM_SYSCTL_GPIO_NODE(44),
   BCM_SYSCTL_GPIO_NODE(45),
   BCM_SYSCTL_GPIO_NODE(46),
   BCM_SYSCTL_GPIO_NODE(47),
   BCM_SYSCTL_GPIO_NODE(48),
   BCM_SYSCTL_GPIO_NODE(49),
   BCM_SYSCTL_GPIO_NODE(50),
   BCM_SYSCTL_GPIO_NODE(51),
   BCM_SYSCTL_GPIO_NODE(52),
   BCM_SYSCTL_GPIO_NODE(53),
   BCM_SYSCTL_GPIO_NODE(54),
   BCM_SYSCTL_GPIO_NODE(55),
   BCM_SYSCTL_GPIO_NODE(56),
   BCM_SYSCTL_GPIO_NODE(57),
   BCM_SYSCTL_GPIO_NODE(58),
   BCM_SYSCTL_GPIO_NODE(59),
   BCM_SYSCTL_GPIO_NODE(60),
   BCM_SYSCTL_GPIO_NODE(61),
   BCM_SYSCTL_GPIO_NODE(62),
   BCM_SYSCTL_GPIO_NODE(63),
#endif
#if defined( CONFIG_BCM_VC02 )
   BCM_SYSCTL_GPIO_VC_NODE(0),
   BCM_SYSCTL_GPIO_VC_NODE(1),
   BCM_SYSCTL_GPIO_VC_NODE(2),
   BCM_SYSCTL_GPIO_VC_NODE(3),
   BCM_SYSCTL_GPIO_VC_NODE(4),
   BCM_SYSCTL_GPIO_VC_NODE(5),
   BCM_SYSCTL_GPIO_VC_NODE(6),
   BCM_SYSCTL_GPIO_VC_NODE(7),
   BCM_SYSCTL_GPIO_VC_NODE(8),
   BCM_SYSCTL_GPIO_VC_NODE(9),
   BCM_SYSCTL_GPIO_VC_NODE(10),
   BCM_SYSCTL_GPIO_VC_NODE(11),
   BCM_SYSCTL_GPIO_VC_NODE(12),
   BCM_SYSCTL_GPIO_VC_NODE(13),
   BCM_SYSCTL_GPIO_VC_NODE(14),
   BCM_SYSCTL_GPIO_VC_NODE(15),
   BCM_SYSCTL_GPIO_VC_NODE(16),
   BCM_SYSCTL_GPIO_VC_NODE(17),
   BCM_SYSCTL_GPIO_VC_NODE(18),
   BCM_SYSCTL_GPIO_VC_NODE(19),
   BCM_SYSCTL_GPIO_VC_NODE(20),
   BCM_SYSCTL_GPIO_VC_NODE(21),
   BCM_SYSCTL_GPIO_VC_NODE(22),
   BCM_SYSCTL_GPIO_VC_NODE(23),
   BCM_SYSCTL_GPIO_VC_NODE(24),
   BCM_SYSCTL_GPIO_VC_NODE(25),
   BCM_SYSCTL_GPIO_VC_NODE(26),
   BCM_SYSCTL_GPIO_VC_NODE(27),
   BCM_SYSCTL_GPIO_VC_NODE(28),
   BCM_SYSCTL_GPIO_VC_NODE(29),
   BCM_SYSCTL_GPIO_VC_NODE(30),
   BCM_SYSCTL_GPIO_VC_NODE(31),
   BCM_SYSCTL_GPIO_VC_NODE(32),
   BCM_SYSCTL_GPIO_VC_NODE(33),
   BCM_SYSCTL_GPIO_VC_NODE(34),
   BCM_SYSCTL_GPIO_VC_NODE(35),
   BCM_SYSCTL_GPIO_VC_NODE(36),
   BCM_SYSCTL_GPIO_VC_NODE(37),
   BCM_SYSCTL_GPIO_VC_NODE(38),
   BCM_SYSCTL_GPIO_VC_NODE(39),
   BCM_SYSCTL_GPIO_VC_NODE(40),
   BCM_SYSCTL_GPIO_VC_NODE(41),
   BCM_SYSCTL_GPIO_VC_NODE(42),
   BCM_SYSCTL_GPIO_VC_NODE(43),
   BCM_SYSCTL_GPIO_VC_NODE(44),
   BCM_SYSCTL_GPIO_VC_NODE(45),
   BCM_SYSCTL_GPIO_VC_NODE(46),
   BCM_SYSCTL_GPIO_VC_NODE(47),
   BCM_SYSCTL_GPIO_VC_NODE(48),
   BCM_SYSCTL_GPIO_VC_NODE(49),
   BCM_SYSCTL_GPIO_VC_NODE(50),
   BCM_SYSCTL_GPIO_VC_NODE(51),
#endif
   {}
};

static ctl_table gSysCtl[] = {
	{
		.ctl_name	= CTL_BCM_GPIO,
		.procname	= GPIO_DRIVER_NAME,
		.mode		   = 0555,
		.child		= gSysCtlLocal
	},
	{}
};

static int proc_do_gpio_intvec(ctl_table *table, int write, struct file *filp,
		     void __user *buffer, size_t *lenp, loff_t *ppos )
{
	int rc;
	int *dataPtr;
    int gpio;
 
	if ( !table || !table->data )
		return -EINVAL;

	/* get GPIO number */
	dataPtr = (int *)table->data;
    gpio = table->ctl_name;

	if ( write )
	{
		/* get value from buffer */
		rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );

		if (rc < 0)
			return rc;

		if ( gpio_direction_is_output( gpio ))
        {
            /* output value to GPIO */
            gpio_set_value( gpio, *dataPtr );
        }
        else
		{
            /* Switch GPIO pin to being an output and set its value */

			GPIO_DEBUG(DBG_INFO,("Warning, GPIO %d previously configured as Input\n", gpio ));
			gpio_direction_output( gpio, *dataPtr );
		}
	}
	else
	{
		/* read value from GPIO once per access to sysctl entry */
		if (filp->f_pos == 0)
		{
			/* save GPIO type and switch to input if necessary */
			if ( gpio_direction_is_output( gpio ))
			{
				int value;

				value = ( gpio_get_value( gpio ) != 0 );
				GPIO_DEBUG(DBG_INFO,("Warning, GPIO %d previously configured as Output of %d\n", gpio, value));
				gpio_direction_input( gpio );
			}

			/* read GPIO value */
			*dataPtr = ( gpio_get_value( gpio ) != 0 );
		}

		/* return GPIO value to buffer */
		rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
	}

	return rc;
}

/****************************************************************************
*
*  gpio_cleanup
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void proc_gpio_cleanup( void )
{
	GPIO_DEBUG(DBG_TRACE,("COMCTL - gpio_cleanup()\n"));

	/* unregister sysctl table */
	if ( gSysCtlHeader != NULL )
	{
		unregister_sysctl_table( gSysCtlHeader );
	}
}

/****************************************************************************
*
*  gpio_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/

static int __init proc_gpio_init( void )
{
	/* Initialize debug level */
	gLevel = DBG_DEFAULT_LEVEL;

	/* register sysctl table */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
	gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
#else
	gSysCtlHeader = register_sysctl_table( gSysCtl );
#endif
	if ( gSysCtlHeader == NULL )
	{
		goto fail;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
	gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
#endif

   return 0;

fail:
	proc_gpio_cleanup();
	return -EINVAL;

} /* gpio_init */

/****************************************************************************
*
*  gpio_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit proc_gpio_exit( void )
{
	GPIO_DEBUG(DBG_INFO,( "proc_gpio_exit called\n" ));

	proc_gpio_cleanup();
} /* gpio_exit */


/* Changed from module_init to fs_initcall so that GPIO driver
 * is loaded before the any of the PMU drivers is loaded. PMU drivers
 * were also changed to fs_initcall so that they are loaded before
 * VC02 and USB drivers are loaded. THis was done because the host has to
 * read the PMU interrupts in time (< 8sec) or else the PMU
 * timeout timer (of 8sec) could expire causing the phone to shut off.
 * This was observed in cases where a battery was removed and then re inserted.
 * This action would cause a LOWBAT interrupt generated and the host has 8sec
 * to clear it before PMU goes into standby mode. If VC02 driver was loaded
 * before PMU driver, the PMU driver was getting loaded well past 8sec window
 */

fs_initcall(proc_gpio_init);
module_exit(proc_gpio_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("GPIO Control Proc Driver");


