/*
 * drivers/video/samsung/a043fw05v4.c
 *
 * Copyright (C) 2008 Travis Kuo <travis.kuo@tomtom.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */


#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/a043fw05v4_pdata.h>

#define RGB_BRIGHTNESS_DEFAULT_VAL 0x40
#define RGB_BRIGHTNESS_REG 0x3
#define MAX_RGB_BRIGHTNESS_VALUE 0xff

static atomic_t mg_rgb_brightness_val = ATOMIC_INIT(0);

struct spi_device *mg_spi;

static int a043fw05v4_spi_remove( struct spi_device *spi );
static void a043fw05v4_set_bl(unsigned short val);

static ssize_t a043fw05v4_rgb_brightness_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t count);
ssize_t a043fw05v4_rgb_brightness_show(struct device *dev, struct device_attribute *attr, char *buf);

//provide interface /sys/bus/spi/devices/spi0.0/a043fw05v4_rgb_brightness_val to set panel backight
static DEVICE_ATTR(a043fw05v4_rgb_brightness_val, 0644, a043fw05v4_rgb_brightness_show, a043fw05v4_rgb_brightness_store);


static ssize_t a043fw05v4_rgb_brightness_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t count)
{
	int val;
	int ret;

	ret = sscanf(buf, "%d",&val);
	if (ret != 1 || val >MAX_RGB_BRIGHTNESS_VALUE ) {
		printk(KERN_ERR "%s: wrong arguments\n", __func__);
		return count;
	}
	if(val!=atomic_read(&mg_rgb_brightness_val))
	{
		atomic_set(&mg_rgb_brightness_val, val);
		a043fw05v4_set_bl(val);
	}	
     	
  return count;
}

ssize_t a043fw05v4_rgb_brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val;

	if( dev == NULL){
		printk("DEV is null!");
		return -EINVAL;
	} 

	val = atomic_read(&mg_rgb_brightness_val);
	snprintf (buf, PAGE_SIZE, "%d", val);

	/* return number of characters +1 for trailing '\0'*/
	return strlen(buf)+1; 
}


static int a043fw05v4_reboot(struct notifier_block *block,
			     unsigned long cause, void* dummy)
{
	a043fw05v4_spi_remove(mg_spi);
	return 0;
}

static struct notifier_block a043fw05v4_reboot_block = {
	.notifier_call	= a043fw05v4_reboot,
};


static void a043fw05v4_write_reg( struct spi_device *spi, unsigned short index,unsigned short data )
{
	unsigned char cmd[2];
	cmd[0]=index<<1;cmd[1]=data;
	spi_write(spi, cmd, sizeof(cmd));
}

static void a043fw05v4_set_bl(unsigned short val)
{
	a043fw05v4_write_reg(mg_spi, 0x3, val);
} 

static int a043fw05v4_spi_probe( struct spi_device *spi )
{	 
	 struct a043fw05v4_platform_data 	*pdata=spi->dev.platform_data;	
	 int ret;
	 
	 mg_spi = spi;
	 atomic_set(&mg_rgb_brightness_val, RGB_BRIGHTNESS_DEFAULT_VAL);
	        	 
	 if(pdata->request_gpio)
	     pdata->request_gpio();
	 if(pdata->config_gpio)
	     pdata->config_gpio();	     
       	    	     
   if(pdata->power_on)
	     pdata->power_on();	 
	     
	 mdelay(5); //wait for power ramp up
	     
	 a043fw05v4_write_reg(spi, 0x5, 0x1e);
	 a043fw05v4_write_reg(spi, 0x5, 0x5e);
	 a043fw05v4_write_reg(spi, 0x5, 0x5f);
	 a043fw05v4_write_reg(spi, RGB_BRIGHTNESS_REG, atomic_read(&mg_rgb_brightness_val));
	 
	 mdelay(20);
	 
	 if(pdata->disp_on)
	     pdata->disp_on();   
	     
   mdelay(A043FW05V4_FRAME_PERIOD_MS*11);
   
   register_reboot_notifier(&a043fw05v4_reboot_block);
   
	 if ( (ret = device_create_file(&spi->dev, &dev_attr_a043fw05v4_rgb_brightness_val)) < 0) {
		dev_err(&spi->dev, "failed to create a043fw05v4_bl file\n");
		return ret;		 
	 }   
   return 0;
}

static int a043fw05v4_spi_remove( struct spi_device *spi )
{
	 struct a043fw05v4_platform_data 	*pdata=spi->dev.platform_data;	
	
	 device_remove_file(&spi->dev, &dev_attr_a043fw05v4_rgb_brightness_val);
	 
	 a043fw05v4_write_reg(spi, 0x5, 0x5e);
   
   if(pdata->disp_off)
	     pdata->disp_off();
	     
	 mdelay(A043FW05V4_FRAME_PERIOD_MS*10);
   
   if(pdata->power_off)
       pdata->power_off();
   
   if(pdata->free_gpio)
	     pdata->free_gpio();
	 
	 return 0;
}


static int a043fw05v4_spi_suspend( struct spi_device *spi, pm_message_t msg)
{
   struct a043fw05v4_platform_data 	*pdata=spi->dev.platform_data;	
  
	 a043fw05v4_write_reg(spi, 0x5, 0x5e);
	 
	 if(pdata->disp_off)
	     pdata->disp_off();
	     
	 mdelay(A043FW05V4_FRAME_PERIOD_MS*10);
	 
	 /* power off by GPxSLPCON. keep GPXDAT the value to early power on LCM by pm.c when pm.c restores it in
	   the beginning of resume.
	    this is special for AUO. Otherwise spi is resumed first and spi bus made LCM semi-powered on before it's
	    powered on by gpio
	   
	  be careful not to use gpio without slpcon 
	 if(pdata->power_off)
       pdata->power_off();
   */    
	 return 0;
}

static int a043fw05v4_spi_resume( struct spi_device *spi )
{	 
	 struct a043fw05v4_platform_data 	*pdata=spi->dev.platform_data;	
	     		
	 if(pdata->power_on)	 	
	     pdata->power_on();	 
	 
	 mdelay(5); //wait for power ramp up
	      
	 a043fw05v4_write_reg(spi, 0x5, 0x1e);
	 a043fw05v4_write_reg(spi, 0x5, 0x5e);
	 a043fw05v4_write_reg(spi, 0x5, 0x5f);
	 a043fw05v4_write_reg(spi, RGB_BRIGHTNESS_REG, atomic_read(&mg_rgb_brightness_val));
	 
	 mdelay(20);
	 
	 if(pdata->disp_on)
	     pdata->disp_on();   
	        
   mdelay(A043FW05V4_FRAME_PERIOD_MS*11);
   
   return 0;
}

static struct spi_driver a043fw05v4_spi_driver=
{
	.driver=
	{
		.name		= "a043fw05v4_spi",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},
	.probe			= a043fw05v4_spi_probe,
	.remove			= a043fw05v4_spi_remove,
	.suspend		= a043fw05v4_spi_suspend,
	.resume			= a043fw05v4_spi_resume,
};

static int __init a043fw05v4_spi_init( void )
{
	return spi_register_driver( &a043fw05v4_spi_driver );
}

static void __exit a043fw05v4_spi_exit( void )
{
	spi_unregister_driver( &a043fw05v4_spi_driver );
	return;
}

module_init( a043fw05v4_spi_init );
module_exit( a043fw05v4_spi_exit );

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("a043fw05v4 SPI configuration driver");
