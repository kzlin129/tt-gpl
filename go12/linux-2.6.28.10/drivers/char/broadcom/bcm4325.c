/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/ioctls.h>
#include <asm/arch/reg_gpio.h>
#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/wlan_settings.h>
#include <linux/broadcom/gpio.h>
#include <asm/gpio.h>
#include <linux/broadcom/bcm4325.h>

/* BCMRING has its own gpiomux library */
#ifdef CONFIG_ARCH_BCMRING
#include <linux/broadcom/bcmring/gpio_defs.h>
#include <asm/arch/csp/gpiomux.h>
#include <csp/sdioHw.h>
#endif

static void bcm4325_cfg(int wl, int bt);

/* previous wireless setting, enable=1 disable=0 */
static int prev_wl = 0;
/* previous bluetooth setting, enable=1 disable=0 */
static int prev_bt = 0;

#define BCM4325_DRV_DEV_NAME   "bcm4325"

static dev_t gBCM4325DrvDevNum = MKDEV(BCM_WLAN_DRV_MAJOR, 0);
static struct cdev gBCM4325DrvCDev;

static BCM4325_GPIO_PIN_MAP bcm4325_wl_gpio_map[] = HW_DEFAULT_WLAN_PIN_CFG;
static BCM4325_GPIO_PIN_MAP bcm4325_bt_gpio_map[] = HW_DEFAULT_BT_PIN_CFG;
static BCM4325_GPIO_PIN_MAP bcm4325_common_gpio_map[] = HW_DEFAULT_COMMON_PIN_CFG;

#define BCM4325_WL_CFG_SIZE (sizeof(bcm4325_wl_gpio_map) / sizeof(bcm4325_wl_gpio_map[0]))
#define BCM4325_BT_CFG_SIZE (sizeof(bcm4325_bt_gpio_map) / sizeof(bcm4325_bt_gpio_map[0]))
#define BCM4325_COMMON_CFG_SIZE (sizeof(bcm4325_common_gpio_map) / sizeof(bcm4325_common_gpio_map[0]))

void bcm4325_enable_wl(int wl)
{
	bcm4325_cfg(wl, prev_bt);
}

void bcm4325_enable_bt(int bt)
{
	bcm4325_cfg(prev_wl, bt);
}

static int gpio_init(BCM4325_GPIO_PIN_MAP *map_ptr, unsigned int map_size)
{
   int rval;
   unsigned int map_index;

   for (map_index = 0; map_index < map_size; map_index++, map_ptr++)
   {
      /* reserve GPIO pins */
      #ifdef CONFIG_ARCH_BCMRING
      rval = gpiomux_request(map_ptr->pin_num,
            chipcHw_GPIO_FUNCTION_GPIO,
            "BCM4325 GPIO");
      if (rval != gpiomux_rc_SUCCESS)
      #else
      rval = gpio_request(map_ptr->pin_num, "BCM4325 GPIO");
      if (rval < 0)
      #endif
      {
         printk(KERN_ERR "%s(): Unable to request GPIO pin %d\n",
               __FUNCTION__, map_ptr->pin_num);
         return -1;
      }
      
      /* ensure GPIO pins set to output mode */
      gpio_direction_output(map_ptr->pin_num, map_ptr->disable_val);

      /* disable by default */
      gpio_set_value(map_ptr->pin_num, map_ptr->disable_val);
      msleep(map_ptr->disable_delay);
   }

   return 0;
}

static void gpio_term(BCM4325_GPIO_PIN_MAP *map_ptr, unsigned int map_size)
{
   unsigned int map_index;

   for (map_index = 0; map_index < map_size; map_index++, map_ptr++)
   {
      /* free GPIO pins */
      #ifdef CONFIG_ARCH_BCMRING
      gpiomux_free(map_ptr->pin_num);
      #else
      gpio_free(map_ptr->pin_num);
      #endif
   }
}

static void gpio_cfg(BCM4325_GPIO_PIN_MAP *map_ptr, unsigned int map_size,
      unsigned int enable)
{
   unsigned int map_index;

   for (map_index = 0; map_index < map_size; map_index++, map_ptr++)
   {
      if (enable)
      {
         gpio_set_value(map_ptr->pin_num, map_ptr->enable_val);
         msleep(map_ptr->enable_delay);
      }
      else /* disable */
      {
         gpio_set_value(map_ptr->pin_num, map_ptr->disable_val);
         msleep(map_ptr->disable_delay);
      }
   }
}

/****************************************************************************
*
*   Called when the GPIO driver ioctl is made
*
***************************************************************************/
static int bcm4325_ioctl(struct inode *inode, struct file *file,
      unsigned int cmd, unsigned long arg)
{
   int rc = 0;
   int value;

   switch (cmd)
   {
      case BCM4325_WIRELESSLAN:
      {
         if (copy_from_user(&value, (int *)arg, sizeof(int)) != 0)
         {
            printk(KERN_ERR "%s(): copy_from_user failed for cmd: 0x%x\n",
                  __FUNCTION__, cmd);
            return -EFAULT;
         }

         bcm4325_enable_wl(value);
         break;
      }

      case BCM4325_BLUETOOTH:
      {
         if (copy_from_user( &value, (int *)arg, sizeof(int)) != 0)
         {
            printk(KERN_ERR "%s(): copy_from_user failed for cmd: 0x%x\n",
                  __FUNCTION__, cmd);
            return -EFAULT;
         }

         bcm4325_enable_bt(value);
         break;
      }

      default:
      {
         printk(KERN_ERR "%s(): Unrecognized ioctl: 0x%x\n",
               __FUNCTION__, cmd);
         return -ENOTTY;
      }
   }

   return rc;
}

static void bcm4325_cfg(int wl, int bt)
{
   int old_rtc_state, new_rtc_state;
	BCM4325_GPIO_PIN_MAP *map_ptr = NULL;

   old_rtc_state = (prev_wl || prev_bt);
   new_rtc_state = (wl || bt);

	printk(KERN_NOTICE "%s(): wl=%d, bt=%d\n", __FUNCTION__, wl, bt);

   /* configure wireless */
   if (BCM4325_WL_CFG_SIZE)
   {
      if (wl != prev_wl)
      {
         map_ptr = bcm4325_wl_gpio_map;
         gpio_cfg(map_ptr, BCM4325_WL_CFG_SIZE, wl);
         prev_wl = wl;
      }
   }

   /* configure bluetooth */
   if (BCM4325_BT_CFG_SIZE)
   {
      if (bt != prev_bt)
      {
         map_ptr = bcm4325_bt_gpio_map;
         gpio_cfg(map_ptr, BCM4325_BT_CFG_SIZE, bt);
         prev_bt = bt;
      }
   }

   if (BCM4325_COMMON_CFG_SIZE)
   {
      if (new_rtc_state != old_rtc_state)
      {
         map_ptr = bcm4325_common_gpio_map;
         gpio_cfg(map_ptr, BCM4325_COMMON_CFG_SIZE, new_rtc_state);
      }
   }
}

static int bcm4325_init(void)
{
   int rval;
   BCM4325_GPIO_PIN_MAP *map_ptr = NULL;

   if (BCM4325_WL_CFG_SIZE)
   {
      #ifdef CONFIG_ARCH_BCMRING
      /* Initialize SDIO1 clock */
      SdioHw_Init1();
      
      /* set GPIO MUX to SDIO1 4-bit mode */
      rval = gpiomux_requestGroup(gpiomux_group_sdio1_4,
            "SDIO1 WL");
      if (rval != gpiomux_rc_SUCCESS) {
         printk(KERN_ERR "%s(): Unable to request GPIO SDIO1_4 group, "
               "error code = %d\n", __FUNCTION__, rval);
         return -1;
      }
      #endif

      prev_wl = 0;
      map_ptr = bcm4325_wl_gpio_map;
      rval = gpio_init(map_ptr, BCM4325_WL_CFG_SIZE);
      if (rval != 0)
         return rval;
   }
   
   if (BCM4325_BT_CFG_SIZE)
   {
      prev_bt = 0;
      map_ptr = bcm4325_bt_gpio_map;
      rval = gpio_init(map_ptr, BCM4325_BT_CFG_SIZE);
      if (rval != 0)
         return rval;
   }
   
   if (BCM4325_COMMON_CFG_SIZE)
   {
      map_ptr = bcm4325_common_gpio_map;
      rval = gpio_init(map_ptr, BCM4325_COMMON_CFG_SIZE);
      if (rval != 0)
         return rval;
   }

   return 0;
}

static void bcm4325_term(void)
{
   BCM4325_GPIO_PIN_MAP *map_ptr = NULL;

   if (BCM4325_WL_CFG_SIZE)
   {
      #ifdef CONFIG_ARCH_BCMRING
      gpiomux_freeGroup(gpiomux_group_sdio1_4);
      /* Shutdown SDIO1 clock */
      SdioHw_Exit1();      
      #endif
            
      prev_wl = 0;
      map_ptr = bcm4325_wl_gpio_map;
      gpio_term(map_ptr, BCM4325_WL_CFG_SIZE);
   }
   
   if (BCM4325_BT_CFG_SIZE)
   {
      prev_bt = 0;
      map_ptr = bcm4325_bt_gpio_map;
      gpio_term(map_ptr, BCM4325_WL_CFG_SIZE);
   }
   
   if (BCM4325_COMMON_CFG_SIZE)
   {
      map_ptr = bcm4325_common_gpio_map;
      gpio_term(map_ptr, BCM4325_WL_CFG_SIZE);
   }
}

struct file_operations bcm4325_fops =
{
   owner: THIS_MODULE,
   ioctl: bcm4325_ioctl,
};

/****************************************************************************
*
*   WLAN module init
*
***************************************************************************/
static int __devinit bcm4325_drv_init(void)
{
   int rc;
   
   printk(KERN_NOTICE "%s().\n", __FUNCTION__);
   
   if (MAJOR(gBCM4325DrvDevNum) == 0)
   {
      /* allocate a major number dynamically */
      if ((rc = alloc_chrdev_region(&gBCM4325DrvDevNum, 0, 1, BCM4325_DRV_DEV_NAME)) < 0)
      {
         printk(KERN_WARNING "BCM4325: alloc_chrdev_region failed; err: %d\n", rc);
         return rc;
      }
   }
   else
   {
      /* use the statically assigned major number */
      if ((rc = register_chrdev_region(gBCM4325DrvDevNum, 1, BCM4325_DRV_DEV_NAME)) < 0)
      {
         printk(KERN_WARNING "BCM4325: register_chrdev failed for major %d; err: %d\n",
               BCM_WLAN_DRV_MAJOR, rc);
         return rc;
      }
   }
   
   cdev_init(&gBCM4325DrvCDev, &bcm4325_fops);
   gBCM4325DrvCDev.owner = THIS_MODULE;

   if ((rc = cdev_add(&gBCM4325DrvCDev, gBCM4325DrvDevNum, 1)) != 0)
   {
      printk( KERN_WARNING "BCM4325: cdev_add failed: %d\n", rc);
      goto err_unreg_drv;
   }

   /* initialize GPIO pins for both WL and BT */
   rc = bcm4325_init();
   if (rc != 0)
      goto err_free_gpio;

   goto success;

err_free_gpio:
   bcm4325_term();
err_unreg_drv:
   unregister_chrdev_region(gBCM4325DrvDevNum, 1);

success:
    return rc;
}

/****************************************************************************
*
*   WLAN module cleanup
*
***************************************************************************/
static void __exit bcm4325_drv_exit(void)
{
   bcm4325_term();
   cdev_del(&gBCM4325DrvCDev);
   unregister_chrdev_region(gBCM4325DrvDevNum, 1);
   return;
}

module_init(bcm4325_drv_init);
module_exit(bcm4325_drv_exit);

EXPORT_SYMBOL(bcm4325_enable_wl);
EXPORT_SYMBOL(bcm4325_enable_bt);
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM4325 Driver");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");
