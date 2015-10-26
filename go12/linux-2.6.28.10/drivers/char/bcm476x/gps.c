/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
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
/* linux/drivers/char/gps.c
 *
 * Implementation of the GPS driver.
 *
 * Changelog:
 *
 * 19-Feb-2009  KOM  Initial version for refs #811:
 *                 
 * 22-Feb-2009  KOM  GPS initialization modified.
 *                 ADDED::    
 *                    - 02.001: Call gps_reset() and gps_power(0) added in gps_mod_init()
 *                 FIXME:: 
 *                    - 03.001: No print if KERN_WARNING is being used in printk()
 *                    - 03.002: Using gps_power(0) instead of gps_power(1) because serial driver 
 *                              puts extra 0 during identification interruput in amba-pl011.c::pl011_startup()
 * 25-Mar-2009  KOM  Startup BUGFIXED in gps_mod_init() and gps_mod_exit(). FIXME Need fix in in amba-pl011.c.
 *                   ADDED in gps_mod_init()::    
 *                    - gps_reset() added firsttime after power on otherwise nRESET deasserted
 *                    - After delay udelay(156) (5 RTS clks) nSTANDBY deasserted gps_power(1) for 80ms added
 *                    - nSTANDBY asserted gps_power(0) added because ser.driver puts extra 0 during identification interruput
 *                   ADDED in gps_mod_exit()::    
 *                    - nSTANDBY asserted gps_power(0) added 
 * 25-Mar-2009  KOM  FIXME: Temporary fix. Need fix Startup bug in amba-pl011.c. 
 *                   Apparently optimization is wrong therefore I am using external IO functions __read...() and __write...() from gps.c.  
 *                   I wil debug it.
 * 27-Apr-2009  KOM  - Startup BUGFIXED: mdelay(80) added IOW_GPS_ON:gps_power(1) and IOW_GPS_OFF:after gps_power(0)
 *                   - Unlock added before Deactivating GPS_RESET in gps_mod_init()
 * 05-May-2009  KOM  - udelay(156) added instead of mdelay(80) in IOW_GPS_OFF:after gps_power(0).
 *                   - PK_WARN and PK_DBG are nothing now.
 *                   - Using volatile xxx_save variables for I/O to/from CMU and PML blocks in gps_power(), gps_reset() and 
 *                     gps_passthrough_mode() added. Startup problem should be fixed in gps_power(). I think so.
 *
 * 12-08-2009   JLH  Modified name of pl011_serial_get_port_info to bcm4760_uart_serial_get_port_info for new serial driver.        
 */                  

/* Includes */ 
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/fs.h>
//#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/version.h>

#include <linux/major.h>
#include <asm/uaccess.h>
#include <asm/ioctls.h>

#include <linux/delay.h>     
//#include <asm/arch/gps.h>    
#include <linux/broadcom/bcm_major.h>
//#include <linux/broadcom/gpio.h>
#include <asm/arch/gps.h>  

#include <asm/io.h>
#include <asm/sizes.h>

#include <asm/arch/hw_cfg.h>

/* Defines */
#define PFX "gps: "

#define PK_DBG_NONE(fmt, arg...)	do {} while (0)

//#define PK_DBG(fmt, arg...)       printk(KERN_DEBUG PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_DBG(fmt, arg...)         do {} while (0)
#define PK_ERR(fmt, arg...)			printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
//KOM FIXME 03.001:: No print if KERN_WARNING is being used
//#define PK_WARN(fmt, arg...)		printk(KERN_WARNING PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)		do {} while (0)
#define PK_NOTICE(fmt, arg...)		printk(KERN_NOTICE PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)		printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)

//#define dbg(x...) printk(KERN_EMERG PFX x)
//#define dbg(x...) printk(KERN_DEBUG PFX x)
//#define dbg(fmt, arg...) printk(KERN_EMERG PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define dbg(fmt, arg...) do {} while (0)

#define CMU_BLOCK_ADDR              0xD00B0000
#define PML_BLOCK_ADDR              0xD00D2000
#define CMU_BLOCK_RESET1__OFS       0x0A0
#define CMU_BLOCK_BUS_CLK_STOP__OFS 0x0B0
#define PML_BLOCK_SW_ISOLATE__OFS   0x124

#define CMU_BLOCK_RESET1       ((volatile unsigned long *)(CMU_BLOCK_ADDR+CMU_BLOCK_RESET1__OFS))
#define CMU_BLOCK_BUS_CLK_STOP ((volatile unsigned long *)(CMU_BLOCK_ADDR+CMU_BLOCK_BUS_CLK_STOP__OFS))
#define PML_BLOCK_SW_ISOLATE   ((volatile unsigned long *)(PML_BLOCK_ADDR+PML_BLOCK_SW_ISOLATE__OFS))

#define UNLOCK_GPS             ((volatile unsigned long *)(0xD00B0100))     


#define CONFIGURE_GPS_UART0_1 ((volatile unsigned long *)(0xD00B0420))
#define CONFIGURE_GPS_UART0_2 ((volatile unsigned long *)(0xD00B0480))

extern int bcm4760_uart_serial_get_port_info(int line,char *szBuf);


//KOM FIXME: __attribute__ ((noinline)) forces compilation error 
//void gps_reset(void) __attribute__ ((noinline));
//static void gps_passthrough_mode(unsigned on) __attribute__ ((noinline));
//void gps_power(unsigned on) __attribute__ ((noinline));
extern void gps_power(unsigned on);
extern void gps_reset(void);


/*   This function turns Standby GPS device on/off
 * Input:
 *       unsigned on - 1 is Standby GPS device off, 0 - on 
 * Output:
 *       None
 * References:
 *       gps_ioctl, gps_mod_init
 * Global:
 *       pml_block_sw_isolate_save, cmu_block_bus_clk_stop_save
 * History :
 *       KOM::19-Feb-2009 Initial Version
 *       KOM::05-May-2009 Using volatile xxx_save for I/O CMU and PML blocks
 *                        Startup problem should be fixed in gps_power(). I think so. 
 *                        Look at explanation below.                        
 */
volatile unsigned long pml_block_sw_isolate_save = 0;
volatile unsigned long cmu_block_bus_clk_stop_save = 0;

void gps_power(unsigned on)
{
    PK_DBG("Standby GPS device %s\n", on==0 ? "on" : "off");

    if (on) {
        //*PML_BLOCK_SW_ISOLATE |= (1<<11);   // 1'b0 = enable isolation between gps rf and gps bb
        pml_block_sw_isolate_save = (*PML_BLOCK_SW_ISOLATE) | (1<<11);
        *PML_BLOCK_SW_ISOLATE = pml_block_sw_isolate_save;
    } 
    else { 
        // Startup problem explanation 
        //*CMU_BLOCK_BUS_CLK_STOP |= (1<<6);   // To enable first clear the mask bits
        //*PML_BLOCK_SW_ISOLATE &= ~(1<<11);   // 0'b0 = disable isolation between gps rf and gps bb
        //01   ldr	r1, .L15+20                // load addr CMU_BLOCK_ADDR (0xD00B0000)    
        //02   ldr	r0, .L15+16                // load addr PML_BLOCK_ADDR (0xD00D2000)
        //03   ldr	r3, [r1, #176]             // load from *CMU_BLOCK_BUS_CLK_STOP (0xD00B0000 + 0x0B0)         
        //04   ldr	r2, [r0, #292]             // read from *PML_BLOCK_SW_ISOLATE   (0xD00D2000 + 0x124)     
        //05   orr	r3, r3, #64                // (*CMU_BLOCK_BUS_CLK_STOP) | (1<<6)
        //06   bic	r2, r2, #2048              // (*PML_BLOCK_SW_ISOLATE) & ~(1<<11)
        //07   str	r3, [r1, #176]             // store to *CMU_BLOCK_BUS_CLK_STOP  (0xD00B0000 + 0x0B0)          
        //08   str	r2, [r0, #292]             // store to *PML_BLOCK_SW_ISOLATE    (0xD00D2000 + 0x124)
        // Problem during optimization code ???
        // intruction line 07 enables first clear the mask bits and then at once
        // following line 08 disables isolation between gps rf and gps bb
        // I think problem is here. We need little bit delay or ticks 
        // between enable mask (CMU block) and turn Standby on (PML block), i.e.
        // between storing to difference blocks 

        cmu_block_bus_clk_stop_save = (*CMU_BLOCK_BUS_CLK_STOP) | (1<<6);
        *CMU_BLOCK_BUS_CLK_STOP = cmu_block_bus_clk_stop_save;
        
        pml_block_sw_isolate_save = (*PML_BLOCK_SW_ISOLATE) & ~(1<<11);
        *PML_BLOCK_SW_ISOLATE = pml_block_sw_isolate_save;
   }
}

/*   This function resets GPS device
 * Input:
 *       None
 * Output:
 *       None
 * References:
 *       gps_ioctl, gps_mod_init
 * Global:
 *       cmu_block_reset1_save, unlock_gps_save
 * History :
 *       KOM::19-Feb-2009 Initial Version
 *       KOM::05-May-2009 Using volatile xxx_save for I/O CMU and PML blocks
 *                        Startup problem should be fixed in gps_power(). Look at explanation above. 
 */
volatile unsigned long cmu_block_reset1_save = 0;
volatile unsigned long unlock_gps_save = 0;

void gps_reset(void)
{
    PK_DBG("Resetting GPS device\n");

    //*UNLOCK_GPS = 0xbcbc4760;       // unlock 
    unlock_gps_save = 0xbcbc4760;
    *UNLOCK_GPS = unlock_gps_save; 

    //*CMU_BLOCK_RESET1 &= ~(1<<18);   // Activate GPS_RESET	
    cmu_block_reset1_save = (*CMU_BLOCK_RESET1) & ~(1<<18);
    *CMU_BLOCK_RESET1 = cmu_block_reset1_save;
    
    mdelay(50);

    //*CMU_BLOCK_RESET1 |= (1<<18);    // Deactivate GPS_RESET	
    cmu_block_reset1_save = (*CMU_BLOCK_RESET1) | (1<<18);
    *CMU_BLOCK_RESET1 = cmu_block_reset1_save;
}


/*   This function turns passthrough mode on/off for GPS device
 * Input:
 *       unsigned on - 1 is turn on, 0 - off 
 * Output:
 *       None
 * References:
 *       gps_ioctl
 * Global:
 *       configure_gps_uart_save
 * History :
 *       KOM::19-Feb-2009 Initial Version
 *       KOM::05-May-2009 Using volatile xxx_save for I/O CMU and PML blocks
 *                        Startup problem should be fixed in gps_power(). Look at explanation above. 
 */
volatile unsigned long configure_gps_uart_save = 0;

static void gps_passthrough_mode(unsigned on)        
{
    PK_DBG("Passthrough GPS/UART1 %s\n", on ? "on" : "off");
    if (on) {
        //*CONFIGURE_GPS_UART0_1 = 0x09240000;
        configure_gps_uart_save = 0x09240000;
        *CONFIGURE_GPS_UART0_1 = configure_gps_uart_save;

        //*CONFIGURE_GPS_UART0_2 = 0x4;
        configure_gps_uart_save = 0x4;
        *CONFIGURE_GPS_UART0_2 = configure_gps_uart_save;
    } else {
        //*CONFIGURE_GPS_UART0_1 = 0;
        configure_gps_uart_save = 0;
        *CONFIGURE_GPS_UART0_1 = configure_gps_uart_save;

        //*CONFIGURE_GPS_UART0_2 = 0x2;
        configure_gps_uart_save = 0x2;
        *CONFIGURE_GPS_UART0_2 = configure_gps_uart_save;
    }
}

static int gps_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    //unsigned long flags;

    switch (cmd) {
    case IOW_GPS_ON:
        PK_WARN("WARNING: Not Ignoring IOW_GPS_ON\n");
        gps_power(1);
        mdelay(80);
        break;
    case IOW_GPS_OFF:
        PK_WARN("WARNING: Not Ignoring IOW_GPS_OFF\n");
        gps_power(0);
        udelay(156);
        break;
    case IOW_GPS_RESET:
        PK_WARN("WARNING: Not Ignoring IOW_GPS_RESET\n");
        gps_reset();
        break;
    case IOW_GPS_PASSTHROUGH_MODE_ON:
        PK_WARN("WARNING: Not Ignoring IOW_GPS_PASSTHROUGH_MODE_ON\n");
        gps_passthrough_mode(1);        
        break;
    case IOW_GPS_PASSTHROUGH_MODE_OFF:
        PK_WARN("WARNING: Not Ignoring IOW_GPS_PASSTHROUGH_MODE_OFF\n");
        gps_passthrough_mode(0);        
        break;
    case IOR_GET_SERIAL_PORT_INFO:
        {
            char szBuf[4096] = {0};
		    unsigned int line = 0;
		    int sz;
		    ret = copy_from_user(&line, (void __user *) arg, 1) ? -EFAULT: 0;
            sz = bcm4760_uart_serial_get_port_info((int)line,szBuf); //GPS_PORT,
            szBuf[sz] = 0;
            if ( sz ) 
            {  
	           sz++;
               ret = copy_to_user((void __user *) arg, szBuf, sz) ? -EFAULT: 0;
			   if ( szBuf[0] == 0 ) return 0;
            }
            //else
            //     PK_WARN("WARNING: Fail IOR_GET_SERIAL_PORT_INFO\n");
            ret = sz; 
        } 
        break;

    default:
        PK_WARN("Invalid ioctl command %u\n", cmd);
        ret = -EINVAL;
        break;
    }

    return ret;
}

static int gps_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int gps_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* Kernel interface */
static struct file_operations gps_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= gps_ioctl,
	.open		= gps_open,
	.release	= gps_release,
};

static char     gBanner[] __initdata = KERN_INFO "Broadcom GPS Driver: 1.02, Feb 22,2009 11:07\n";

static  dev_t           gGpsDrvDevNum = MKDEV( BCM_GPS_MAJOR, 0 );
static  struct class   *gGpsDrvClass = NULL;
static  struct  cdev    gGpsDrvCDev;

static int __init gps_mod_init(void)
{
    int     rc = 0;
    static int firsttime = 1;

    printk( gBanner );

    if (firsttime) 
    {
       firsttime = 0;

       /* bootloader cold boot, reset GPS */
       gps_reset();
    } else {
       *UNLOCK_GPS = 0xbcbc4760;        // unlock 
       *CMU_BLOCK_RESET1 |= (1<<18);    // Deactivate GPS_RESET	
    } 
    udelay(156);                        // t2 (5 RTS clks) = Delay from the time when nRESET is deasserted 
                                        //     to the time when nSTANDBY is deasserted = 1/6 ms
    gps_power(1);
    mdelay(80);                         // allow 80ms for ASIC to come up 

    //KOM FIXME 03.002:: Using gps_power(0) instead of gps_power(1) because serial driver 
    //                   puts extra 0 during identification interruput in amba-pl011.c::pl011_startup()
    gps_power(0);
    {
        // Use the statically assigned major number

        if (( rc = register_chrdev_region( gGpsDrvDevNum, 1, GPS_DEVNAME )) < 0 )
        {
           PK_WARN("Unable to register driver for major %d; err: %d\n", BCM_GPS_MAJOR, rc );
           return rc;
        }
    }

    cdev_init( &gGpsDrvCDev, &gps_fops );
    gGpsDrvCDev.owner = THIS_MODULE;

    if (( rc = cdev_add( &gGpsDrvCDev, gGpsDrvDevNum, 1 )) != 0 )
    {
        PK_WARN("Unable to add driver %d\n", rc );
        goto out_unregister;
    }

    // Now that we've added the device, create a class, so that udev will make the /dev entry

    gGpsDrvClass = class_create( THIS_MODULE, GPS_DEVNAME );
    if ( IS_ERR( gGpsDrvClass ))
    {
        PK_WARN("Unable to create class\n" );
        rc = -1;
        goto out_cdev_del;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
    device_create( gGpsDrvClass, NULL, gGpsDrvDevNum, NULL, GPS_DEVNAME);
#else
    class_device_create( gGpsDrvClass, NULL, gGpsDrvDevNum, NULL, GPS_DEVNAME );
#endif

    goto done;

out_cdev_del:
    cdev_del( &gGpsDrvCDev );

out_unregister:
    unregister_chrdev_region( gGpsDrvDevNum, 1 );

done:
    return rc;
}


static void __exit gps_mod_exit(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
    device_destroy( gGpsDrvClass, gGpsDrvDevNum );
#else
    class_device_destroy( gGpsDrvClass, gGpsDrvDevNum );
#endif
    class_destroy( gGpsDrvClass );

    gps_power(0);

    cdev_del( &gGpsDrvCDev );

    unregister_chrdev_region( gGpsDrvDevNum, 1 );
}


/* KOM::Mar 25,2005
 *
 * FIXME: Temporary fix. Need fix Startup bug in amba-pl011.c. 
 *        Apparently optimization is wrong therefore I am using external IO functions __read...() and __write...() from gps.c.  
 *        I wil debug it.
 */
extern unsigned short  ___readw( void *a)
{
  return (*(volatile unsigned short  *)(a));
}

extern void ___writew(unsigned short v,void *a)
{
  (*(volatile unsigned short __force *)(a) = (v));
}



module_init(gps_mod_init);
module_exit(gps_mod_exit);

MODULE_AUTHOR("BROADCOM");
MODULE_DESCRIPTION("BROADCOM BCM4760 GPS Driver");
MODULE_LICENSE("GPL");

/* EOF */
