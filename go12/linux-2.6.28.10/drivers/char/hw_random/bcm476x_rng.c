/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
 * DESCRIPTION: The BCM476X random number generator (RNG) driver
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/hw_random.h>
#include <linux/delay.h>
#include <linux/broadcom/bcm476x_rng.h>

#include <asm/arch/rngHw.h>
#include <asm/arch/rngHw_reg.h>

static char banner[] __initdata = KERN_INFO "Broadcom RNG Driver\n";
static struct semaphore lock; /* lock for data access */

static void rngHw_init ( uint32_t warmUp )
{
   rngHw_REG_SET_RANDOM_BIT_GEN_ENABLE ();
   rngHw_REG_SET_RANDOM_BIT_SPEED_DOUBLE ();
   rngHw_REG_SET_INT_DISABLE ();
   rngHw_REG_SET_WARMUP_CYCLE ( warmUp );
   while ( ( pRngHw->Status & rngHw_REG_WARMUP_MASK ) != rngHw_REG_WARMUP_MASK );
}


static void rngHw_interruptDisable ( void )
{
   rngHw_REG_SET_INT_DISABLE(); 
}

/**
static void rngHw_interruptEnable ( void )
{
   rngHw_REG_SET_INT_ENABLE(); 
}
**/

static uint32_t rngHw_getRandomNumber ( void )
{

   while ( rngHw_REG_GET_VALID_WORDS() == 0 );
   
   return pRngHw->Data;
}


static int rng_data_present(struct hwrng *rng, int wait)
{
   int data, i;
   
   for (i = 0; i < 20; i++) {
      data = rngHw_REG_GET_VALID_WORDS() ? 1 : 0;
      if (data || !wait)
         break;
      /*
       * RNG produces data fast enough.  We *could* use the RNG IRQ, but
       * that'd be higher overhead ... so why bother?
       */
      udelay(10);
   }
   return data;
}

int rng_data_read(struct hwrng *rng, u32 *data)
{
/* Lock it here since other kernel driver can access it */
   down(&lock);
   *data = rngHw_getRandomNumber();
	 up(&lock);
   return 4;
}
EXPORT_SYMBOL(rng_data_read);


static struct hwrng rng_ops = {
   .name = "bcm476x",
   .data_present = rng_data_present,
   .data_read = rng_data_read,
};


static int __init rng_probe(struct platform_device *pdev)
{
   int ret;

   /* init the RNG block and make sure interrupt is disabled */
   rngHw_init(0xFFF);
   rngHw_interruptDisable();
	 
	 init_MUTEX(&lock); /* init unlock */

   /* register to the Linux RNG framework */
   ret = hwrng_register(&rng_ops);
   if (ret)
      goto err_register;
   
   printk(KERN_INFO "RNG: Driver initialized\n");
   
   return 0;

err_register:
   return ret;
}


static int rng_remove(struct platform_device *pdev)
{
   hwrng_unregister(&rng_ops);
   return 0;
}


static struct platform_driver rng_driver = {
   .driver = {
      .name = "bcm476x-rng",
      .owner = THIS_MODULE,
   },
   .probe = rng_probe,
   .remove = rng_remove,
};


static int __init rng_init(void)
{
   printk(banner);
   return platform_driver_register(&rng_driver);
}


static void __exit rng_exit(void)
{
   platform_driver_unregister(&rng_driver);
}


module_init(rng_init);
module_exit(rng_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("RNG Driver");
MODULE_LICENSE("GPL");
