#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <plat/map.h>
#include <plat/irqs.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio.h>

#include <plat/bcm4750_pdata.h>

static void bcm4750_suspend(void)
{
}

static void bcm4750_resume(void)
{
}

static struct bcm4750_platform_data bcm_pdata = {
	.suspend = bcm4750_suspend,
	.resume  = bcm4750_resume, 
};

struct bcm4750_platform_data * setup_bcm4750_pdata(void)
{
	return &bcm_pdata;
}

