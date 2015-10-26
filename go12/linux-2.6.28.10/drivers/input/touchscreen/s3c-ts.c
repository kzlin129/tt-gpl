/* linux/drivers/input/touchscreen/s3c-ts.c
 *
 * $Id: s3c-ts2.c,v 1.2 2008/11/12 05:50:51 ihlee215 Exp $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 * iPAQ H1940 touchscreen support
 *
 * ChangeLog
 *
 * 2004-09-05: Herbert Potzl <herbert@13thfloor.at>
 *	- added clock (de-)allocation code
 *
 * 2005-03-06: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - h1940_ -> s3c24xx (this driver is now also used on the n30
 *        machines :P)
 *      - Debug messages are now enabled with the config option
 *        TOUCHSCREEN_S3C_DEBUG
 *      - Changed the way the value are read
 *      - Input subsystem should now work
 *      - Use ioremap and readl/writel
 *
 * 2005-03-23: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Make use of some undocumented features of the touchscreen
 *        controller
 *
 * 2006-09-05: Ryu Euiyoul <ryu.real@gmail.com>
 *      - added power management suspend and resume code
 *
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>


#include <plat/regs-adc.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <mach/irqs.h>

#define CONFIG_TOUCHSCREEN_S3C_DEBUG
#undef CONFIG_TOUCHSCREEN_S3C_DEBUG

/* For ts->dev.id.version */
#define S3C_TSVERSION	0x0101

#define DETECT_STYLUS_UP  (1<<8)

#define WAIT4INT(x)  (((x)<<8) | \
		     S3C_ADCTSC_YM_SEN | S3C_ADCTSC_YP_SEN | S3C_ADCTSC_XP_SEN | \
		     S3C_ADCTSC_XY_PST(3))

#define AUTOPST	     (S3C_ADCTSC_YM_SEN | S3C_ADCTSC_YP_SEN | S3C_ADCTSC_XP_SEN | \
		     S3C_ADCTSC_AUTO_PST | S3C_ADCTSC_XY_PST(0) | DETECT_STYLUS_UP)

#define DEBUG_LVL    KERN_DEBUG

#define TS_POLL_TIME	HZ/40
#define TS_PEN_UP       1
#define TS_PEN_DOWN     0

/*
 Definitions & global arrays.
 */
static char 			*s3c_ts_name = "S3C TouchScreen";
static struct s3c_ts_info 	*ts;
static struct s3c_adc_client 	*ts_adc_client;

static void ts_poll(unsigned long data)
{
        /* We were called from the updown irq */
        s3c_adc_start (ts_adc_client, 0, 1<<ts->shift);
}

static struct timer_list touch_timer = TIMER_INITIALIZER(ts_poll, 0, 0);

static void ts_select (unsigned int selected, void *data)
{
	switch (selected) {
		case INIT:
			s3c_adc_set_ts_control(ts_adc_client, S3C_ADCTSC, WAIT4INT(1));
			return;
		break;
		case SELECTED:
	                s3c_adc_set_ts_control(ts_adc_client, S3C_ADCTSC, 
					       S3C_ADCTSC_PULL_UP_DISABLE | AUTOPST);
			return;
		break;
		case FINISHED:
	                s3c_adc_set_ts_control(ts_adc_client , S3C_ADCTSC, WAIT4INT(1));

               	        mod_timer(&touch_timer, jiffies+TS_POLL_TIME);
		break;
		default:
			printk(KERN_ERR "s3c-ts.c Unsupported value in ts_select!\n");
			return;
		break;
	}

	/* We have enough samples */
        input_report_abs(ts->dev, ABS_X, ts->xp);
        input_report_abs(ts->dev, ABS_Y, ts->yp);

        input_report_key(ts->dev, BTN_TOUCH, 1);
        input_report_abs(ts->dev, ABS_PRESSURE, 1);
        input_sync(ts->dev); 

        /* Restart the counters */
        ts->xp = 0;
        ts->yp = 0;
}


static void ts_convert (unsigned data0, unsigned data1, void *data)
{
        int state;

	state  = (!(data0 & S3C_ADCDAT0_UPDOWN)  && (!(data1 & S3C_ADCDAT1_UPDOWN)));

	if (state) { /* Down */
                #if defined(CONFIG_TOUCHSCREEN_NEW)
                        ts->yp += S3C_ADCDAT0_XPDATA_MASK_12BIT - (data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT);
                        ts->xp += S3C_ADCDAT1_YPDATA_MASK_12BIT - (data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT);
                #elif defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416) || \
		      defined(CONFIG_CPU_S3C6410) || defined(CONFIG_CPU_S5P6440)
                        ts->xp += data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT;
                        ts->yp += data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT;
                #else
                        ts->xp += data0 & S3C_ADCDAT0_XPDATA_MASK;
                        ts->yp += data1 & S3C_ADCDAT1_YPDATA_MASK;
                #endif
	}
}

static irqreturn_t stylus_updown(int irqno, void *param)
{
	unsigned long val, up;
        unsigned long flags;

        local_irq_save(flags);

	val = s3c_adc_get_ts_control(ts_adc_client, S3C_ADCTSC);
	up = (val & 0x100);

	if (up) {
		del_timer_sync (&touch_timer);

		s3c_adc_set_ts_control (ts_adc_client, S3C_ADCTSC, WAIT4INT(0));

	        s3c_adc_set_ts_control (ts_adc_client, S3C_ADCCLRWK, 0x0);	// Acknowledge

	        input_report_key(ts->dev, BTN_TOUCH, 0);
                input_report_abs(ts->dev, ABS_PRESSURE, 0);
                input_sync(ts->dev);
		ts->xp = 0;
		ts->yp = 0;
	} else {

	        s3c_adc_set_ts_control (ts_adc_client, S3C_ADCCLRWK, 0x0);	//Acknowledge

	        mod_timer (&touch_timer, jiffies+1);
	}
	local_irq_restore(flags);

	return IRQ_HANDLED;
}

/*
 * The functions for inserting/removing us as a module.
 */
static int __init s3c_ts_probe(struct platform_device *pdev)
{
	struct input_dev *input_dev;
	int ret;

	ts = kzalloc(sizeof(struct s3c_ts_info), GFP_KERNEL);
	if (!ts) {
		ret = -ENOMEM;
		goto err_alloc_ts;
	}
	
	input_dev = input_allocate_device();

	if (!input_dev) {
		ret = -ENOMEM;
		goto err_alloc_input_dev;
	}
	
	ts->dev = input_dev;

	ts->dev->evbit[0] = ts->dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	
#if defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C6410) || defined(CONFIG_CPU_S3C2416) || defined(CONFIG_CPU_S5P6440)
	input_set_abs_params(ts->dev, ABS_X, 0, 0xFFF, 0, 0);
	input_set_abs_params(ts->dev, ABS_Y, 0, 0xFFF, 0, 0);
#else
	input_set_abs_params(ts->dev, ABS_X, 0, 0x3FF, 0, 0);
	input_set_abs_params(ts->dev, ABS_Y, 0, 0x3FF, 0, 0);
#endif
	input_set_abs_params(ts->dev, ABS_PRESSURE, 0, 1, 0, 0);

	sprintf(ts->phys, "input(ts)");

	ts->dev->name = s3c_ts_name;
	ts->dev->phys = ts->phys;
	ts->dev->id.bustype = BUS_RS232;
	ts->dev->id.vendor = 0xDEAD;
	ts->dev->id.product = 0xBEEF;
	ts->dev->id.version = S3C_TSVERSION;

	ts->shift = 2; 
	
	/* Get irq */	
	ts->irq = platform_get_irq(pdev, 0);
	if (ts->irq <= 0) {
		printk(KERN_ERR "no irq resource specified\n");
		ret = -ENOENT;
		goto err_get_irqs;
	}

	ret = request_irq(ts->irq, stylus_updown, IRQF_SAMPLE_RANDOM, "s3c-updown", ts);
	if (ret != 0) {
		printk(KERN_ERR "s3c_ts.c: Could not allocate ts IRQ_TC !\n");
		ret = -EIO;
		goto err_get_irqs;
	}

	/* Register the ADC client */
	ts_adc_client = s3c_adc_register (pdev, ts_select, ts_convert, NULL, 1);
	if (ts_adc_client == NULL) {
		printk (KERN_ERR "s3c_ts.c: Unable to get adc client\n");
		ret = -ENOENT;
		goto err_adc_register;
	}

	printk(KERN_INFO "%s got loaded successfully\n", s3c_ts_name);

	/* All went ok, so register to the input system */
	ret = input_register_device(ts->dev);
	if (ret) {
		printk(KERN_ERR "s3c_ts.c: Could not register input device(touchscreen)!\n");
		ret = -EIO;
		goto err_input_register;
	}

	/* Now that everything is properly registered, setup the ADCTSC to get the pen up/down intr */
	s3c_adc_set_ts_control (ts_adc_client, S3C_ADCTSC, WAIT4INT(0));

	return 0;

err_input_register:
	s3c_adc_release(ts_adc_client);

err_adc_register:
	free_irq(ts->irq, ts->dev);

err_get_irqs:
	input_free_device(input_dev);
	
err_alloc_input_dev:
	kfree(ts);

err_alloc_ts:
	return ret;
}

static int s3c_ts_remove(struct platform_device *dev)
{
	disable_irq(ts->irq);

	del_timer_sync (&touch_timer);
	free_irq(ts->irq, ts);
	s3c_adc_release (ts_adc_client);

	input_unregister_device(ts->dev);

	return 0;
}

#ifdef CONFIG_PM
static unsigned int adctsc;

static int s3c_ts_suspend(struct platform_device *dev, pm_message_t state)
{
	printk(KERN_INFO"S5P6440 TouchScreen suspending.\n");	

	disable_irq(ts->irq);

        del_timer_sync (&touch_timer);

        /* Leave the input driver in a known state */
        input_report_key(ts->dev, BTN_TOUCH, 0);
        input_report_abs(ts->dev, ABS_PRESSURE, 0);
        input_sync(ts->dev);

        ts->xp = 0;
        ts->yp = 0;
	
	return 0;
}

static int s3c_ts_resume(struct platform_device *pdev)
{
	printk(KERN_INFO"S5P6440 TouchScreen resuming.\n");	
	s3c_adc_set_ts_control (ts_adc_client, S3C_ADCTSC, WAIT4INT(0));
	enable_irq(ts->irq);

	return 0;
}
#else
#define s3c_ts_suspend NULL
#define s3c_ts_resume  NULL
#endif

static struct platform_driver s3c_ts_driver = {
       .probe          = s3c_ts_probe,
       .remove         = s3c_ts_remove,
       .suspend        = s3c_ts_suspend,
       .resume         = s3c_ts_resume,
       .driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-ts",
	},
};

static char banner[] __initdata = KERN_INFO "S3C Touchscreen driver, (c) 2008 Samsung Electronics\n";

static int __init s3c_ts_init(void)
{
	printk(banner);
	return platform_driver_register(&s3c_ts_driver);
}

static void __exit s3c_ts_exit(void)
{
	platform_driver_unregister(&s3c_ts_driver);
}

module_init(s3c_ts_init);
module_exit(s3c_ts_exit);

MODULE_AUTHOR("Samsung AP");
MODULE_DESCRIPTION("S3C touchscreen driver");
MODULE_LICENSE("GPL");
