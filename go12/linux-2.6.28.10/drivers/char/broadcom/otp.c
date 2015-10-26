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

#include <asm/uaccess.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/otp.h>

#ifdef CONFIG_BROADCOM_BCM1103
#include <asm/broadcom/bcm1103/bcm1103.h>
#include <asm/broadcom/bcm1103/otp1103.h>
#endif

#ifdef CONFIG_ARCH_BCMRING
#include <asm/arch/csp/chipcHw_inline.h>
#include <asm/arch/csp/otpHw_inline.h>
#include <asm/arch/csp/otpHw_reg.h>
#endif

#undef OTP_DBG
#define OTP_DBG 0

#if (OTP_DBG == 1)
#define OTP_DEBUG( fmt, args... ) printk( KERN_NOTICE "OTP: " fmt, ## args )
#else
#define OTP_DEBUG( fmt, args... ) 
#endif



static char banner[] __initdata = KERN_INFO "BCM OTP Driver\n";

static void otp_info_get(OTP_INFO *info)
{
   uint32_t data;
   
#ifdef CONFIG_BROADCOM_BCM1103
   data = bcm1103mmr->bSafe.prodCfg;
#elif defined(CONFIG_ARCH_BCMRING)
   data = otpHw_readData(7); /* row 7 */
#else
   data = 0;
#endif

   info->eth_phy[0] = (data & otpHw_MASK_ETH_PHY0) >> otpHw_SHIFT_ETH_PHY0;
   info->eth_phy[1] = (data & otpHw_MASK_ETH_PHY1) >> otpHw_SHIFT_ETH_PHY1;
#ifdef CONFIG_ARCH_BCMRING
   info->eth_gmii[0] = (data & otpHw_MASK_ETH_GMII0) >> otpHw_SHIFT_ETH_GMII0;
   info->eth_gmii[1] = (data & otpHw_MASK_ETH_GMII1) >> otpHw_SHIFT_ETH_GMII1;
   info->eth_sgmii[0] = (data & otpHw_MASK_ETH_SGMII0) >> otpHw_SHIFT_ETH_SGMII0;
   info->eth_sgmii[1] = (data & otpHw_MASK_ETH_SGMII1) >> otpHw_SHIFT_ETH_SGMII1;
#else
   info->eth_gmii[0] = OTP_NOT_SUPPORTED;
   info->eth_gmii[1] = OTP_NOT_SUPPORTED;
   info->eth_sgmii[0] = OTP_NOT_SUPPORTED;
   info->eth_sgmii[1] = OTP_NOT_SUPPORTED;
#endif
   
#ifdef CONFIG_BROADCOM_BCM1103
   info->ext_rgmii = (data & otpHw_MASK_ETH_RGMII) >> otpHw_SHIFT_ETH_RGMII;
#else
   info->ext_rgmii = OTP_NOT_SUPPORTED;
#endif

   info->vpm = (data & otpHw_MASK_VPM) >> otpHw_SHIFT_VPM;
   info->ehss[0] = (data & otpHw_MASK_EHSS0) >> otpHw_SHIFT_EHSS0;
   info->ehss[1] = (data & otpHw_MASK_EHSS1) >> otpHw_SHIFT_EHSS1;
   info->uart = (data & otpHw_MASK_UART) >> otpHw_SHIFT_UART;
   info->touch_screen = (data & otpHw_MASK_TOUCH_SCRN) >> otpHw_SHIFT_TOUCH_SCRN;
   info->keypad = (data & otpHw_MASK_KEYPAD) >> otpHw_SHIFT_KEYPAD;
   info->led_matrix = (data & otpHw_MASK_LED_MTX) >> otpHw_SHIFT_LED_MTX;
   info->usb[0] = (data & otpHw_MASK_USB0) >> otpHw_SHIFT_USB0;
#ifdef CONFIG_ARCH_BCMRING
   info->usb[1] = (data & otpHw_MASK_USB1) >> otpHw_SHIFT_USB1;
#else
   info->usb[1] = OTP_NOT_SUPPORTED;
#endif

#ifdef CONFIG_ARCH_BCMRING
   info->sdio[0] = (data & otpHw_MASK_SDIO0) >> otpHw_SHIFT_SDIO0;
   info->sdio[1] = (data & otpHw_MASK_SDIO1) >> otpHw_SHIFT_SDIO1;
#else
   info->sdio[0] = OTP_NOT_SUPPORTED;
   info->sdio[1] = OTP_NOT_SUPPORTED;
#endif

#ifdef CONFIG_BROADCOM_BCM1103
   info->crypto = (data & otpHw_MASK_CRYPTO) >> otpHw_SHIFT_CRYPTO;
   info->hvg_rfg_apm = (data & otpHw_MASK_HVG_APM) >> otpHw_SHIFT_HVG_APM;
#else
   info->crypto = OTP_NOT_SUPPORTED;
   info->hvg_rfg_apm = OTP_NOT_SUPPORTED; 
#endif

#ifdef CONFIG_ARCH_BCMRING
   info->lcd = (data & otpHw_MASK_LCD) >> otpHw_SHIFT_LCD;
   info->ge = (data & otpHw_MASK_GE) >> otpHw_SHIFT_GE;
   info->vdec = (data & otpHw_MASK_VDEC) >> otpHw_SHIFT_VDEC;
   info->pif = (data & otpHw_MASK_PIF) >> otpHw_SHIFT_PIF;
   info->bbl = (data & otpHw_MASK_BBL) >> otpHw_SHIFT_BBL;
#else
   info->lcd = OTP_NOT_SUPPORTED;
   info->ge = OTP_NOT_SUPPORTED;
   info->vdec = OTP_NOT_SUPPORTED;
   info->pif = OTP_NOT_SUPPORTED;
   info->bbl = OTP_NOT_SUPPORTED;
#endif

#ifdef CONFIG_ARCH_BCMRING
   data = otpHw_readData(8); /* row 8 */
#endif

   info->apm_ch[0] = (data & otpHw_MASK_APM_CH0) >> otpHw_SHIFT_APM_CH0;
   info->apm_ch[1] = (data & otpHw_MASK_APM_CH1) >> otpHw_SHIFT_APM_CH1;
#ifdef CONFIG_ARCH_BCMRING
   info->apm_ch[2] = (data & otpHw_MASK_APM_CH2) >> otpHw_SHIFT_APM_CH2;
#else
   info->apm_ch[2] = OTP_NOT_SUPPORTED;
#endif

   info->max_vpm_clk = (data & otpHw_MASK_MAX_VPM_CLK) >> otpHw_SHIFT_MAX_VPM_CLK;
   info->max_cpu_clk = (data & otpHw_MASK_MAX_CPU_CLK) >> otpHw_SHIFT_MAX_CPU_CLK;

#ifdef CONFIG_ARCH_BCMRING
   info->lcd_res = (data & otpHw_MASK_MAX_LCD_RES) >> otpHw_SHIFT_MAX_LCD_RES;
#else
   info->lcd_res = 0;
#endif

#ifdef CONFIG_ARCH_BCMRING
   data = otpHw_readData(10); /* row 10 */
#endif

   info->chip_id = (data & otpHw_MASK_CHIP_ID) >> otpHw_SHIFT_CHIP_ID;
}

static int otp_open(struct inode *inode, struct file *filp)
{
   OTP_DEBUG("Device opened\n");
   return 0;
}

static int otp_release(struct inode *inode, struct file *filp)
{
   OTP_DEBUG("Device closed\n");
   return 0;
}

static int otp_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
      unsigned long arg)
{
   switch (cmd) {
      case OTP_IOCTL_INFO_GET:
      {
         OTP_INFO info;

         memset(&info, 0, sizeof(info));

         otp_info_get(&info);

         if (copy_to_user( (void *)arg, &info, sizeof(info)))
            return -EFAULT;
      }
      break;

      default:
         return -EINVAL;
   }

   return 0;
}

struct file_operations otp_fops =
{
   owner: THIS_MODULE,
   open: otp_open,
   release: otp_release,
   ioctl: otp_ioctl
};

static int __init otp_init(void)
{
   int rc;

   printk(banner);

   rc = register_chrdev(BCM_OTP_MAJOR, "otp", &otp_fops);
   if (rc < 0) {
      printk(KERN_WARNING "OTP: register_chrdev failed for major %d\n",
            BCM_OTP_MAJOR);
      return rc;
   }

#ifdef CONFIG_ARCH_BCMRING
   chipcHw_busInterfaceClockEnable(chipcHw_REG_BUS_CLOCK_OTP);
   otpHw_init();
#endif

   return 0;
}

static void __exit otp_exit(void)
{
   unregister_chrdev(BCM_OTP_MAJOR, "otp");

#ifdef CONFIG_ARCH_BCMRING
   chipcHw_busInterfaceClockDisable(chipcHw_REG_BUS_CLOCK_OTP);
#endif
}

module_init(otp_init);
module_exit(otp_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("OTP Driver");
MODULE_LICENSE("GPL");
