#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <plat/irvine.h>
#include <asm/arch/bcm4760_reg.h>
#include <mach/hw_cfg.h>
#include <plat/fdt.h>

extern unsigned int bcm476x_get_boot_sdmmc_device(void);

unsigned long action_debounce_time;

static struct resource *card_detect;
static atomic_t card_in;
static struct timer_list action_debounce;

static atomic_t insert_debouncing;
static atomic_t remove_debouncing;

static void card_detect_action_debounce_timeout_func(unsigned long data_pointer)
{
	if(atomic_read(&insert_debouncing)){
		if(!(readl(card_detect->start) & SDM1_F_CARDINSERTED_MASK) ){
			atomic_set (&insert_debouncing, 0);
			return;			
		}

		atomic_set(&card_in, 1);
		gpio_set_value(TT_VGPIO_SD_PWR_EN, 1);
		atomic_set (&insert_debouncing, 0);
		
	} else {
		if (atomic_read(&remove_debouncing)){
			if((readl(card_detect->start) & SDM1_F_CARDINSERTED_MASK)){
				atomic_set (&remove_debouncing, 0);
				return;
			}

			atomic_set(&card_in, 0);
			gpio_set_value(TT_VGPIO_SD_PWR_EN, 0);
			atomic_set (&remove_debouncing, 0);
		} else {
			printk(KERN_ERR"sd_pwr_en: card_detect_action_debounce_timeout_func in error state\n");
		}
	}
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	if(card_detect == NULL)
		return IRQ_NONE;

	if(readl(card_detect->start) & SDM1_F_CARDINSERTED_MASK){
		if(atomic_read(&remove_debouncing)){
			del_timer(&action_debounce);
			atomic_set (&remove_debouncing, 0);
		}
		if((!atomic_read(&card_in))&&(!atomic_read(&insert_debouncing))){
		 	action_debounce.function = card_detect_action_debounce_timeout_func;
			mod_timer(&action_debounce, jiffies + action_debounce_time);
			atomic_set (&insert_debouncing, 1);
		}
	}

	if (! (readl(card_detect->start) & SDM1_F_CARDINSERTED_MASK) ){
		if(atomic_read(&insert_debouncing)){
			del_timer(&action_debounce);
			atomic_set (&insert_debouncing, 0);
		}
		if(atomic_read(&card_in)&&(!atomic_read(&remove_debouncing))){
			action_debounce.function = card_detect_action_debounce_timeout_func;
			mod_timer(&action_debounce, jiffies + action_debounce_time);
			atomic_set (&remove_debouncing, 1);
		}
	}

	return IRQ_HANDLED;
}

static ssize_t sd_pwr_read_debounce(struct device *dev, struct device_attribute *attr,
		                                       char *buf)
{
	snprintf (buf, PAGE_SIZE, "%d\n", jiffies_to_msecs(action_debounce_time));

	return strlen(buf)+1;
}

static ssize_t sd_pwr_write_debounce(struct device *dev, struct device_attribute *attr,
		                                        const char *buf, size_t n)
{
	int temp;

	temp = action_debounce_time;
	sscanf(buf, "%lu", &action_debounce_time);
	/* sanity check, time is in msecs */
	if ( (action_debounce_time < 1) || (action_debounce_time > 300) ){
		action_debounce_time = temp;
		return -EINVAL;
	}
  	action_debounce_time = msecs_to_jiffies(action_debounce_time);

	return n;
}

static DEVICE_ATTR(sd_debounce_t, S_IRUGO | S_IWUSR, sd_pwr_read_debounce, sd_pwr_write_debounce);

static int sd_pwr_en_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource	*card_int;

	card_detect = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	card_int = platform_get_resource(pdev, IORESOURCE_IO, 0);

	if(card_detect == NULL || card_int == NULL){
		printk(KERN_ERR"sd_pwr_en: Missing resource!\n");
		return -1;
	}

	ret = gpio_request(TT_VGPIO_SD_PWR_EN, "sd_pwr_en");
	if (ret){
		printk(KERN_ERR"sd_pwr_en: Can't get gpio: %d!\n",TT_VGPIO_SD_PWR_EN);
		return ret;
	}

	gpio_direction_output(TT_VGPIO_SD_PWR_EN, 0);

	if(readl(card_detect->start) & SDM1_F_CARDINSERTED_MASK){
		gpio_set_value(TT_VGPIO_SD_PWR_EN, 1);
	} else {
		gpio_set_value(TT_VGPIO_SD_PWR_EN, 0);
	}

	ret=request_irq(card_int->start, irq_handler, IRQF_SHARED, "sd_pwr_en", pdev);
	if (ret){
		printk(KERN_ERR"sd_pwr_en: Can't register interrupt!\n");
		gpio_free(TT_VGPIO_SD_PWR_EN);
		return ret;
	}

	init_timer(&action_debounce);
	atomic_set (&insert_debouncing, 0);
	atomic_set (&remove_debouncing, 0);
	atomic_set (&card_in, 0);

	action_debounce_time = msecs_to_jiffies(50);

	if( device_create_file(&pdev->dev, &dev_attr_sd_debounce_t))
	{
		printk(KERN_ERR "sd_pwr_en: unable to create sysfs file\n");
	}

	return ret;
}		

static int sd_pwr_en_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct resource	*card_int;

	del_timer(&action_debounce);
	atomic_set (&insert_debouncing, 0);
	atomic_set (&remove_debouncing, 0);
	atomic_set (&card_in, 0);
  
	card_int = platform_get_resource(pdev, IORESOURCE_IO, 0);
	free_irq(card_int->start, pdev);
	gpio_set_value(TT_VGPIO_SD_PWR_EN, 0);

	return gpio_direction_input(TT_VGPIO_SD_PWR_EN);
}

static int sd_pwr_en_resume(struct platform_device *pdev)
{
	struct resource	*card_int;
	int ret = 0;

	card_int = platform_get_resource(pdev, IORESOURCE_IO, 0);
	ret=request_irq(card_int->start, irq_handler, IRQF_SHARED, "sd_pwr_en", pdev);
	if (ret){
		printk(KERN_ERR"sd_pwr_en: Can't register interrupt!\n");
		gpio_free(TT_VGPIO_SD_PWR_EN);
		return ret;
	}
	gpio_direction_output(TT_VGPIO_SD_PWR_EN, 0);
	if(readl(card_detect->start) & SDM1_F_CARDINSERTED_MASK){
		atomic_set(&card_in, 1);
		gpio_set_value(TT_VGPIO_SD_PWR_EN,1);
	} else {
		atomic_set(&card_in, 0);
	}

	return ret;
}


static int sd_pwr_remove(struct platform_device *pdev)
{
	struct resource	*card_int;

	card_int = platform_get_resource(pdev, IORESOURCE_IO, 0);
	free_irq(card_int->start, pdev);
	gpio_free(TT_VGPIO_SD_PWR_EN);
	device_remove_file(&pdev->dev, &dev_attr_sd_debounce_t);

	return 0;
}

static struct platform_driver sd_pwr_en_driver = {
	.probe 	 = sd_pwr_en_probe,
	.suspend = sd_pwr_en_suspend,
	.resume  = sd_pwr_en_resume,
	.remove	 = sd_pwr_remove,
	.driver  = {
		.owner	= THIS_MODULE,
		.name	= "sd_pwr_en",
	},
};

static int __init sd_pwr_en_init(void)
{
	int ret;

	ret = platform_driver_register(&sd_pwr_en_driver);
	if (ret)
		printk(KERN_ERR"Error creating the SD_PWR_EN platform driver\n");

	return ret;
}

arch_initcall(sd_pwr_en_init);

struct resource mmc_presentstate[] =
{
	{
		.start	= BCM4760_INTR_SDM1,
		.end	= BCM4760_INTR_SDM1,
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= IO_ADDRESS( SDM1_R_PRESENTSTATE_MEMADDR ),
		.end	= IO_ADDRESS( SDM1_R_PRESENTSTATE_MEMADDR ),
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device sd_pwr_en_device = {
	.name	= "sd_pwr_en",
	.resource	= mmc_presentstate,
	.num_resources	= ARRAY_SIZE( mmc_presentstate ),
};

static int __init sd_pwr_device_init( void )
{
	/* If we have an SD card and we are not booting from SD card, add sd_pwr_enable support. */
	if( ! bcm476x_get_boot_sdmmc_device( )
#ifndef CONFIG_BCM4760_FLASHER
	    && fdt_get_ulong("/features", "sdcard", 0)
#endif
	  )
		return platform_device_register( &sd_pwr_en_device );
	else
		return 0;
}
arch_initcall( sd_pwr_device_init );
